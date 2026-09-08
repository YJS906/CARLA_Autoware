// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef SELFCAR__TRAJECTORY_SAFETY__REACHABLE_POSE_ENVELOPE_HPP_
#define SELFCAR__TRAJECTORY_SAFETY__REACHABLE_POSE_ENVELOPE_HPP_

#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace selfcar::trajectory_safety
{
struct EgoMotion
{
  double speed{};             // measured longitudinal speed, not the lane speed limit
  double max_acceleration{};  // nonnegative bound, including measured acceleration if larger
};

inline std::optional<EgoMotion> measuredEgoMotion(
  const double speed, const double acceleration_limit, const double measured_acceleration)
{
  if (!std::isfinite(speed) || !std::isfinite(acceleration_limit) ||
      !std::isfinite(measured_acceleration) || acceleration_limit < 0.0)
    return std::nullopt;
  return EgoMotion{speed, std::max(acceleration_limit, std::abs(measured_acceleration))};
}

// Only the clock used for the current-pose error envelope changes. The full nominal
// corridor, margins, trajectory coordinates and executable velocity profile stay intact.
// This is NOT a proof of a kinematically feasible recovery path.
inline std::vector<autoware_utils_geometry::Polygon2d> createReachablePosePolygons(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  const geometry_msgs::msg::Pose & ego_pose, const double margin, const bool consider_current_pose,
  const double convergence_time, const double step_length,
  const std::optional<EgoMotion> & motion = std::nullopt, const double off_track_scale = 0.0)
{
  namespace downstream = autoware::motion_velocity_planner::polygon_utils;
  const auto original = [&]() {
    return downstream::create_one_step_polygons(
      points, vehicle, ego_pose, margin, consider_current_pose, convergence_time, step_length,
      off_track_scale);
  };
  // Missing/invalid state must never silently select a relaxed envelope.
  if (
    !consider_current_pose || !motion || points.empty() ||
    !std::isfinite(motion->speed) || !std::isfinite(motion->max_acceleration) ||
    motion->max_acceleration < 0.0 || !std::isfinite(convergence_time) ||
    convergence_time <= 0.0)
    return original();

  const double initial_speed = std::abs(motion->speed);
  const double acceleration = motion->max_acceleration;
  auto clock_points = points;
  double arc = 0.0;
  for (size_t i = 0; i < clock_points.size(); ++i) {
    const double velocity = points[i].longitudinal_velocity_mps;
    if (!std::isfinite(velocity)) return original();
    double ds = 0.0;
    if (i + 1 < points.size()) {
      ds = std::hypot(
        points[i + 1].pose.position.x - points[i].pose.position.x,
        points[i + 1].pose.position.y - points[i].pose.position.y);
      if (!std::isfinite(ds)) return original();
    }
    const double before = std::sqrt(initial_speed * initial_speed + 2.0 * acceleration * arc);
    const double after = std::sqrt(initial_speed * initial_speed + 2.0 * acceleration * (arc + ds));
    // ds / average_speed is the earliest arrival time under the acceleration bound,
    // including launch from rest. A commanded zero must not erase measured motion.
    const double upper_average_speed = 0.5 * (before + after);
    if (!std::isfinite(upper_average_speed)) return original();
    const double clock_speed =
      std::min(std::max(std::abs(velocity), initial_speed), upper_average_speed);
    clock_points[i].longitudinal_velocity_mps = std::copysign(clock_speed, velocity);
    arc += ds;
  }
  auto polygons = downstream::create_one_step_polygons(
    clock_points, vehicle, ego_pose, margin, true, convergence_time, step_length, off_track_scale);

  // The original error-pose implementation drops longitudinal projection error.
  // Explicitly retain the EXACT current vehicle with full lateral margin at the
  // start/first swept step, even if the first decimated point lies behind ego.
  namespace bg = boost::geometry;
  const auto & q = ego_pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double half_width = vehicle.vehicle_width_m * 0.5 + margin;
  autoware_utils_geometry::Polygon2d footprint;
  for (const auto & [x, y] : std::vector<std::pair<double, double>>{
         {vehicle.max_longitudinal_offset_m, half_width},
         {vehicle.max_longitudinal_offset_m, -half_width},
         {-vehicle.rear_overhang_m, -half_width},
         {-vehicle.rear_overhang_m, half_width}}) {
    bg::append(footprint, autoware_utils_geometry::Point2d(
      ego_pose.position.x + c * x - s * y, ego_pose.position.y + s * x + c * y));
  }
  bg::correct(footprint);
  for (size_t i = 0; i < std::min<size_t>(2, polygons.size()); ++i) {
    auto joined = polygons[i];
    bg::append(joined, footprint.outer());
    bg::convex_hull(joined, polygons[i]);
    bg::correct(polygons[i]);
  }
  return polygons;
}
}  // namespace selfcar::trajectory_safety
#endif
