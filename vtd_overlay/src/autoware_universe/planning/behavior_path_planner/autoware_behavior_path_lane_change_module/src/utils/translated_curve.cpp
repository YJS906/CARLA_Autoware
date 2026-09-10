// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0

#include "autoware/behavior_path_lane_change_module/utils/path.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace autoware::behavior_path_planner::utils::lane_change
{
std::optional<LaneChangePath> translate_approved_curve(
  const CommonDataPtr & data, const LaneChangePath & approved, const double forward_offset,
  const double velocity)
{
  if (
    !data || !data->is_data_available() || approved.path.points.size() < 4 ||
    !std::isfinite(forward_offset) || forward_offset <= 0.0 || !std::isfinite(velocity) ||
    velocity <= 0.0)
    return std::nullopt;
  const auto & points = approved.path.points;
  auto begin = motion_utils::findNearestIndex(points, approved.info.lane_changing_start.position);
  // Do not replay the portion already driven before this stop. Preserve every remaining
  // curve point, starting with the first original sample at or ahead of ego's projection.
  const double ego_arc = motion_utils::calcSignedArcLength(
    points, points.front().point.pose.position, data->get_ego_pose().position);
  std::vector<double> arcs(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i)
    arcs[i] = arcs[i - 1] + autoware_utils::calc_distance2d(points[i - 1], points[i]);
  const auto future = std::lower_bound(arcs.begin(), arcs.end(), std::max(0.0, ego_arc - 1e-6));
  if (!std::isfinite(ego_arc) || future == arcs.end()) return std::nullopt;
  begin = std::max(begin, static_cast<size_t>(std::distance(arcs.begin(), future)));
  const auto end = motion_utils::findNearestIndex(points, approved.info.lane_changing_end.position);
  if (end <= begin + 1 || end + 1 >= points.size()) return std::nullopt;
  const auto & origin = points[begin].point.pose;
  const auto & ego = data->get_ego_pose();
  const double heading = tf2::getYaw(origin.orientation);
  // Forward means along the source lane, not along a sample already turning sideways.
  const auto & reference = data->current_lanes_path.points;
  if (reference.size() < 2) return std::nullopt;
  const double reference_arc = motion_utils::calcSignedArcLength(
    reference, reference.front().point.pose.position, ego.position);
  const auto reference_pose = motion_utils::calcInterpolatedPose(
    reference, std::clamp(reference_arc, 0.0, motion_utils::calcArcLength(reference)));
  const double forward_heading = tf2::getYaw(reference_pose.orientation);
  const double dx = forward_offset * std::cos(forward_heading);
  const double dy = forward_offset * std::sin(forward_heading);
  auto translate = [&](Pose pose) {
    pose.position.x += dx;
    pose.position.y += dy;
    return pose;
  };
  const auto goal = translate(origin);
  const double ego_yaw = tf2::getYaw(ego.orientation);
  const double gx = goal.position.x - ego.position.x;
  const double gy = goal.position.y - ego.position.y;
  const double scale = std::hypot(gx, gy);
  // Never connect backwards to a curve whose translated start is already behind ego.
  if (
    !std::isfinite(scale) || scale < 0.5 || gx * std::cos(ego_yaw) + gy * std::sin(ego_yaw) < 0.5 ||
    gx * std::cos(heading) + gy * std::sin(heading) < 0.5)
    return std::nullopt;
  const double end_curvature = autoware_utils::calc_curvature(
    points[begin].point.pose.position, points[begin + 1].point.pose.position,
    points[begin + 2].point.pose.position);
  if (!std::isfinite(end_curvature)) return std::nullopt;
  // Interpolate only the approach to the translated start. The approved curve itself is
  // copied point for point: no refitting, shortening, stretching or lateral target change.
  const auto coefficients = [](double p0, double p1, double v0, double v1, double a1) {
    const double d = p1 - p0;
    return std::array<double, 6>{
      p0,
      v0,
      0.0,
      10 * d - 6 * v0 - 4 * v1 + 0.5 * a1,
      -15 * d + 8 * v0 + 7 * v1 - a1,
      6 * d - 3 * v0 - 3 * v1 + 0.5 * a1};
  };
  const auto xs = coefficients(
    ego.position.x, goal.position.x, scale * std::cos(ego_yaw), scale * std::cos(heading),
    -scale * scale * end_curvature * std::sin(heading));
  const auto ys = coefficients(
    ego.position.y, goal.position.y, scale * std::sin(ego_yaw), scale * std::sin(heading),
    scale * scale * end_curvature * std::cos(heading));
  const auto evaluate = [](const auto & c, double u) {
    return std::array<double, 3>{
      c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5])))),
      c[1] + u * (2 * c[2] + u * (3 * c[3] + u * (4 * c[4] + u * 5 * c[5]))),
      2 * c[2] + u * (6 * c[3] + u * (12 * c[4] + u * 20 * c[5]))};
  };
  const auto & vehicle = data->bpp_param_ptr->vehicle_info;
  const double max_curvature = vehicle.calcMaxCurvature();
  if (!std::isfinite(max_curvature) || max_curvature <= 0.0) return std::nullopt;
  LaneChangePath candidate;
  candidate.path.header = approved.path.header;
  const size_t count = static_cast<size_t>(std::ceil(scale / 0.25));
  for (size_t i = 0; i < count; ++i) {
    const double u = static_cast<double>(i) / count;
    const auto x = evaluate(xs, u);
    const auto y = evaluate(ys, u);
    const double norm = std::hypot(x[1], y[1]);
    if (!std::isfinite(norm) || norm < 1e-3) return std::nullopt;
    const double curvature = (x[1] * y[2] - y[1] * x[2]) / (norm * norm * norm);
    if (!std::isfinite(curvature) || std::abs(curvature) > max_curvature) return std::nullopt;
    PathPointWithLaneId point;
    point.point.pose.position.x = x[0];
    point.point.pose.position.y = y[0];
    point.point.pose.position.z = ego.position.z + u * (goal.position.z - ego.position.z);
    point.point.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(std::atan2(y[1], x[1]));
    point.point.longitudinal_velocity_mps = velocity;
    candidate.path.points.push_back(point);
  }
  // Every pose on the curve AND its stopping continuation gets the same XY displacement.
  for (size_t i = begin; i < points.size(); ++i) {
    auto point = points[i];
    if (
      !std::isfinite(point.point.longitudinal_velocity_mps) ||
      (i <= end && point.point.longitudinal_velocity_mps <= 0.0))
      return std::nullopt;
    point.point.pose = translate(point.point.pose);
    point.point.longitudinal_velocity_mps =
      std::min<float>(velocity, point.point.longitudinal_velocity_mps);
    candidate.path.points.push_back(point);
  }
  candidate.path.points.front().point.pose = ego;
  const size_t end_index = count + end - begin;
  // Validate the connector, its junction, and the unchanged curve with the physical limits.
  const auto & p = *data->lc_param_ptr;
  double last_curvature = 0.0;
  for (size_t i = 1; i <= end_index; ++i) {
    const auto & a = candidate.path.points[i - 1].point.pose;
    const auto & b = candidate.path.points[i].point.pose;
    const double ds = autoware_utils::calc_distance2d(a, b);
    const double yaw = std::atan2(b.position.y - a.position.y, b.position.x - a.position.x);
    if (
      !std::isfinite(ds) || ds < 1e-3 ||
      std::abs(autoware_utils::normalize_radian(yaw - tf2::getYaw(a.orientation))) > 0.15)
      return std::nullopt;
    const auto & c = candidate.path.points[i + 1].point.pose;
    const double k = autoware_utils::calc_curvature(a.position, b.position, c.position);
    if (
      !std::isfinite(k) || std::abs(k) > max_curvature ||
      (p.trajectory.enable_lateral_acceleration_limit &&
       velocity * velocity * std::abs(k) > p.trajectory.lat_acc_map.find(velocity).second) ||
      (p.trajectory.enable_lateral_jerk_limit &&
       velocity * velocity * velocity * std::abs(k - last_curvature) / ds >
         p.trajectory.lateral_jerk))
      return std::nullopt;
    last_curvature = k;
  }
  candidate.type =
    autoware::behavior_path_planner::lane_change::PathType::LowSpeed;  // Existing stopped-launch
                                                                       // time prediction.
  candidate.info = approved.info;
  auto & info = candidate.info;
  info.braking_profile.reset();
  info.speed_preparation_target.reset();
  info.longitudinal_acceleration = {0.0, 0.0};
  info.velocity = {velocity, velocity};
  info.terminal_lane_changing_velocity = velocity;
  info.lane_changing_start = goal;
  info.lane_changing_end = candidate.path.points[end_index].point.pose;
  info.shift_line.start = goal;
  info.shift_line.end = info.lane_changing_end;
  info.shift_line.start_idx = count;
  info.shift_line.end_idx = end_index;
  const double prepare_length =
    motion_utils::calcSignedArcLength(candidate.path.points, size_t{0}, count);
  const double curve_length =
    motion_utils::calcSignedArcLength(candidate.path.points, count, end_index);
  const double acceleration =
    std::min(data->bpp_param_ptr->max_acc, p.trajectory.max_longitudinal_acc);
  if (!std::isfinite(acceleration) || acceleration <= 0.0) return std::nullopt;
  info.length = {prepare_length, curve_length};
  info.duration = {prepare_length / velocity + velocity / acceleration, curve_length / velocity};
  candidate.shifted_path.path = candidate.path;
  candidate.shifted_path.path.points.resize(end_index + 1);
  candidate.shifted_path.shift_length.resize(end_index + 1, info.shift_line.end_shift_length);
  for (size_t i = 0; i <= end - begin; ++i) {
    double shift =
      info.shift_line.end_shift_length * (1.0 - static_cast<double>(i) / (end - begin));
    if (
      !approved.shifted_path.path.points.empty() &&
      approved.shifted_path.path.points.size() == approved.shifted_path.shift_length.size()) {
      const size_t nearest = motion_utils::findNearestIndex(
        approved.shifted_path.path.points, points[begin + i].point.pose.position);
      shift = approved.shifted_path.shift_length[nearest];
    }
    candidate.shifted_path.shift_length[count + i] = shift;
  }
  return candidate;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
