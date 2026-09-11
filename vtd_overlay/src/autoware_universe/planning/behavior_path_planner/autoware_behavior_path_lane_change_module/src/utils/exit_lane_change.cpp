// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0

#include "autoware/behavior_path_lane_change_module/utils/path.hpp"

#include <autoware/interpolation/spline_interpolation_points_2d.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change
{
namespace
{
bool finite_pose(const Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(q.x) &&
         std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
         q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-6;
}

// Check the approach, its join to the lateral curve, and the merge. The original low-speed
// generator checks its analytic curve; these checks also cover the newly composed junction.
bool feasible_geometry(const LaneChangePath & path, const CommonDataPtr & data, double velocity)
{
  const auto & points = path.path.points;
  const auto end = path.info.shift_line.end_idx;
  if (end + 1 >= points.size()) return false;
  const auto & limits = data->lc_param_ptr->trajectory;
  const auto max_curvature = data->bpp_param_ptr->vehicle_info.calcMaxCurvature();
  const auto max_lat_acc = limits.lat_acc_map.find(velocity).second;
  if (!std::isfinite(max_curvature) || max_curvature <= 0.0) return false;
  double previous_curvature = 0.0;
  for (size_t i = 0; i <= end; ++i) {
    const auto & a = points[i].point.pose;
    const auto & b = points[i + 1].point.pose;
    if (!finite_pose(a) || !finite_pose(b)) return false;
    const auto ds = autoware_utils::calc_distance2d(a, b);
    if (!std::isfinite(ds) || ds < 1e-3) return false;
    const double travel_yaw = std::atan2(b.position.y - a.position.y, b.position.x - a.position.x);
    if (std::abs(autoware_utils::normalize_radian(travel_yaw - tf2::getYaw(a.orientation))) > 0.15)
      return false;
    const double yaw_curvature =
      autoware_utils::normalize_radian(tf2::getYaw(b.orientation) - tf2::getYaw(a.orientation)) /
      ds;
    const double curvature =
      i == 0
        ? autoware_utils::calc_curvature(a.position, b.position, points[i + 2].point.pose.position)
        : autoware_utils::calc_curvature(points[i - 1].point.pose.position, a.position, b.position);
    if (
      !std::isfinite(curvature) || !std::isfinite(yaw_curvature) ||
      std::max(std::abs(curvature), std::abs(yaw_curvature)) > max_curvature ||
      (limits.enable_lateral_acceleration_limit &&
       velocity * velocity * std::max(std::abs(curvature), std::abs(yaw_curvature)) >
         max_lat_acc) ||
      (i > 0 && limits.enable_lateral_jerk_limit &&
       velocity * velocity * velocity * std::abs(curvature - previous_curvature) / ds >
         limits.lateral_jerk))
      return false;
    previous_curvature = curvature;
  }
  return true;
}
}  // namespace

std::optional<LaneChangePath> generate_exit_lane_change_path(
  const CommonDataPtr & data, const double prepare_distance, const double target_distance,
  const double velocity)
{
  if (
    !data || !data->self_odometry_ptr || !data->bpp_param_ptr || !data->lc_param_ptr ||
    !data->lanes_ptr || !std::isfinite(prepare_distance) || prepare_distance <= 0.0 ||
    !std::isfinite(target_distance) || target_distance <= 0.0 || !std::isfinite(velocity) ||
    velocity <= 0.0 || !finite_pose(data->get_ego_pose()))
    return std::nullopt;

  const double acceleration =
    std::min(data->bpp_param_ptr->max_acc, data->lc_param_ptr->trajectory.max_longitudinal_acc);
  if (!std::isfinite(acceleration) || acceleration <= 0.0) return std::nullopt;

  // A spline follows the curved current lane; a single chord to the future lane-change start
  // would cut across the turn. Remove duplicate positions before constructing that spline.
  std::vector<PathPointWithLaneId> reference;
  for (const auto & point : data->current_lanes_path.points) {
    if (!finite_pose(point.point.pose)) return std::nullopt;
    if (reference.empty() || autoware_utils::calc_distance2d(reference.back(), point) > 1e-5)
      reference.push_back(point);
  }
  if (reference.size() < 3 || data->target_lanes_path.points.size() < 3) return std::nullopt;
  const auto & ego = data->get_ego_pose();
  const double start_arc = motion_utils::calcSignedArcLength(
    reference, reference.front().point.pose.position, ego.position);
  const double reference_length = motion_utils::calcArcLength(reference);
  if (
    !std::isfinite(start_arc) || start_arc < 0.0 ||
    start_arc + prepare_distance > reference_length - 1e-3)
    return std::nullopt;

  const autoware::interpolation::SplineInterpolationPoints2d spline(reference);
  const auto reference_pose = [&](double arc) {
    return spline.getSplineInterpolatedPose(0, std::clamp(arc, 0.0, reference_length));
  };
  const auto derivative = [&](double arc) {
    constexpr double step = 1e-3;
    const double before = std::max(0.0, arc - step);
    const double after = std::min(reference_length, arc + step);
    const auto a = reference_pose(before).position;
    const auto b = reference_pose(after).position;
    return std::array<double, 2>{(b.x - a.x) / (after - before), (b.y - a.y) / (after - before)};
  };
  const auto initial_reference = reference_pose(start_arc);
  const auto initial_derivative = derivative(start_arc);
  const double initial_speed = std::hypot(initial_derivative[0], initial_derivative[1]);
  if (!std::isfinite(initial_speed) || initial_speed < 1e-3) return std::nullopt;
  const double ego_yaw = tf2::getYaw(ego.orientation);

  // Decay the measured position/tangent mismatch while following the reference. Both the
  // correction and its first two derivatives vanish at the end of the preparation segment.
  const auto correction = [](double position, double tangent) {
    return std::array<double, 6>{
      position,
      tangent,
      0.0,
      -10.0 * position - 6.0 * tangent,
      15.0 * position + 8.0 * tangent,
      -6.0 * position - 3.0 * tangent};
  };
  const auto cx = correction(
    ego.position.x - initial_reference.position.x,
    prepare_distance * (initial_speed * std::cos(ego_yaw) - initial_derivative[0]));
  const auto cy = correction(
    ego.position.y - initial_reference.position.y,
    prepare_distance * (initial_speed * std::sin(ego_yaw) - initial_derivative[1]));
  const auto evaluate = [](const auto & c, double u) {
    return std::array<double, 2>{
      c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5])))),
      c[1] + u * (2.0 * c[2] + u * (3.0 * c[3] + u * (4.0 * c[4] + u * 5.0 * c[5])))};
  };

  PathWithLaneId prefix;
  prefix.header = data->current_lanes_path.header;
  const size_t count = std::max<size_t>(2, static_cast<size_t>(std::ceil(prepare_distance / 0.25)));
  for (size_t i = 0; i <= count; ++i) {
    const double u = static_cast<double>(i) / count;
    const double arc = start_arc + u * prepare_distance;
    const auto ref_pose = reference_pose(arc);
    const auto dx = evaluate(cx, u);
    const auto dy = evaluate(cy, u);
    const auto ref_derivative = derivative(arc);
    const double vx = ref_derivative[0] + dx[1] / prepare_distance;
    const double vy = ref_derivative[1] + dy[1] / prepare_distance;
    if (!std::isfinite(vx) || !std::isfinite(vy) || std::hypot(vx, vy) < 1e-3) return std::nullopt;
    PathPointWithLaneId point;
    point.point.pose = ref_pose;
    point.point.pose.position.x += dx[0];
    point.point.pose.position.y += dy[0];
    point.point.pose.position.z += (ego.position.z - initial_reference.position.z) *
                                   (1.0 - u * u * u * (10.0 + u * (-15.0 + 6.0 * u)));
    point.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(std::atan2(vy, vx));
    point.point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.lane_ids =
      reference[motion_utils::findNearestIndex(reference, ref_pose.position)].lane_ids;
    prefix.points.push_back(point);
  }
  prefix.points.front().point.pose = ego;
  prefix.points.back().point.pose = reference_pose(start_arc + prepare_distance);

  auto future_data = std::make_shared<behavior_path_planner::lane_change::CommonData>(*data);
  auto future_odometry = std::make_shared<nav_msgs::msg::Odometry>(*data->self_odometry_ptr);
  future_odometry->pose.pose = prefix.points.back().point.pose;
  future_data->self_odometry_ptr = future_odometry;
  auto candidate = generate_low_speed_path(future_data, target_distance, velocity);
  if (!candidate) return std::nullopt;

  const auto curve_shift = candidate->shifted_path.shift_length;
  const size_t curve_end = candidate->info.shift_line.end_idx;
  prefix.points.pop_back();  // The curve already contains the shared preparation endpoint.
  candidate->path.points.insert(
    candidate->path.points.begin(), prefix.points.begin(), prefix.points.end());
  auto & info = candidate->info;
  info.shift_line.start_idx = count;
  info.shift_line.end_idx = count + curve_end;
  info.length.prepare = motion_utils::calcSignedArcLength(candidate->path.points, size_t{0}, count);
  info.duration.prepare = info.length.prepare / velocity + velocity / acceleration;
  info.duration.lane_changing = info.length.lane_changing / velocity;
  candidate->shifted_path.path = candidate->path;
  candidate->shifted_path.path.points.resize(info.shift_line.end_idx + 1);
  candidate->shifted_path.shift_length.assign(count, curve_shift.front());
  candidate->shifted_path.shift_length.insert(
    candidate->shifted_path.shift_length.end(), curve_shift.begin(), curve_shift.end());
  if (!feasible_geometry(*candidate, data, velocity)) return std::nullopt;
  return candidate;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
