// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "autoware/behavior_path_lane_change_module/utils/path.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change
{
std::optional<LaneChangePath> generate_low_speed_path(
  const CommonDataPtr & data, const double target_distance, const double velocity)
{
  const auto & reference = data->target_lanes_path;
  const auto & ego = data->get_ego_pose();
  const auto & vehicle = data->bpp_param_ptr->vehicle_info;
  if (
    reference.points.size() < 3 || !std::isfinite(velocity) || velocity <= 0.0 ||
    !std::isfinite(target_distance) || target_distance <= 0.0 || vehicle.wheel_base_m <= 0.0 ||
    vehicle.max_steer_angle_rad <= 0.0)
    return std::nullopt;

  const auto start_arc = motion_utils::calcSignedArcLength(
    reference.points, reference.points.front().point.pose.position, ego.position);
  const auto end_arc = start_arc + target_distance;
  const auto ref_length = motion_utils::calcArcLength(reference.points);
  if (end_arc < 0.0 || end_arc + 1.0 >= ref_length) return std::nullopt;
  const auto goal = motion_utils::calcInterpolatedPose(reference.points, end_arc);
  const auto goal_before = motion_utils::calcInterpolatedPose(reference.points, end_arc - 0.5);
  const auto goal_after = motion_utils::calcInterpolatedPose(reference.points, end_arc + 0.5);
  const auto end_curvature =
    autoware_utils::calc_curvature(goal_before.position, goal.position, goal_after.position);
  const double initial_yaw = tf2::getYaw(ego.orientation);
  const double final_yaw = tf2::getYaw(goal.orientation);
  const double scale = autoware_utils::calc_distance2d(ego, goal);
  if (scale < 1.0) return std::nullopt;

  // Quintic Hermite geometry fixes position and tangent at both ends. At a standstill the
  // initial curvature is zero; the final curvature matches the target reference path.
  const auto coefficients = [&](double p0, double p1, double v0, double v1, double acc1) {
    const auto delta = p1 - p0;
    return std::array<double, 6>{
      p0,
      v0,
      0.0,
      10.0 * delta - 6.0 * v0 - 4.0 * v1 + 0.5 * acc1,
      -15.0 * delta + 8.0 * v0 + 7.0 * v1 - acc1,
      6.0 * delta - 3.0 * v0 - 3.0 * v1 + 0.5 * acc1};
  };
  const auto xs = coefficients(
    ego.position.x, goal.position.x, scale * std::cos(initial_yaw), scale * std::cos(final_yaw),
    -scale * scale * end_curvature * std::sin(final_yaw));
  const auto ys = coefficients(
    ego.position.y, goal.position.y, scale * std::sin(initial_yaw), scale * std::sin(final_yaw),
    scale * scale * end_curvature * std::cos(final_yaw));
  const auto evaluate = [](const auto & c, double u) {
    return std::array<double, 3>{
      c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5])))),
      c[1] + u * (2 * c[2] + u * (3 * c[3] + u * (4 * c[4] + u * 5 * c[5]))),
      2 * c[2] + u * (6 * c[3] + u * (12 * c[4] + u * 20 * c[5]))};
  };

  LaneChangePath candidate;
  candidate.type = autoware::behavior_path_planner::lane_change::PathType::LowSpeed;
  candidate.path.header = reference.header;
  const auto max_curvature = vehicle.calcMaxCurvature();
  const auto max_lat_acc = data->lc_param_ptr->trajectory.lat_acc_map.find(velocity).second;
  const auto max_jerk = data->lc_param_ptr->trajectory.lateral_jerk;
  const auto count = static_cast<size_t>(std::ceil(scale / 0.25));
  double length = 0.0;
  double previous_curvature = 0.0;
  for (size_t i = 0; i <= count; ++i) {
    const double u = static_cast<double>(i) / count;
    const auto x = evaluate(xs, u);
    const auto y = evaluate(ys, u);
    const auto norm = std::hypot(x[1], y[1]);
    if (!std::isfinite(norm) || norm < 1e-3) return std::nullopt;
    const auto curvature = (x[1] * y[2] - y[1] * x[2]) / (norm * norm * norm);
    if (
      !std::isfinite(curvature) || std::abs(curvature) > max_curvature ||
      (data->lc_param_ptr->trajectory.enable_lateral_acceleration_limit &&
       velocity * velocity * std::abs(curvature) > max_lat_acc))
      return std::nullopt;
    PathPointWithLaneId point;
    point.point.pose.position.x = x[0];
    point.point.pose.position.y = y[0];
    point.point.pose.position.z = ego.position.z + u * (goal.position.z - ego.position.z);
    point.point.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(std::atan2(y[1], x[1]));
    point.point.longitudinal_velocity_mps = static_cast<float>(velocity);
    if (!candidate.path.points.empty()) {
      const auto & previous = candidate.path.points.back().point.pose;
      const auto ds = autoware_utils::calc_distance2d(previous, point.point.pose);
      if (ds < 1e-3) return std::nullopt;
      const auto travel_yaw = std::atan2(y[0] - previous.position.y, x[0] - previous.position.x);
      if (
        std::abs(autoware_utils::normalize_radian(travel_yaw - tf2::getYaw(previous.orientation))) >
        0.15) {
        return std::nullopt;  // No cusp or reverse segment.
      }
      if (
        data->lc_param_ptr->trajectory.enable_lateral_jerk_limit &&
        velocity * velocity * velocity * std::abs(curvature - previous_curvature) / ds > max_jerk) {
        return std::nullopt;
      }
      length += ds;
    }
    previous_curvature = curvature;
    for (const auto & lane : data->lanes_ptr->current) point.lane_ids.push_back(lane.id());
    for (const auto & lane : data->lanes_ptr->target) point.lane_ids.push_back(lane.id());
    candidate.path.points.push_back(point);
  }
  candidate.path.points.front().point.pose = ego;
  candidate.path.points.back().point.pose = goal;
  candidate.shifted_path.path = candidate.path;
  candidate.shifted_path.shift_length.resize(candidate.path.points.size());
  auto & info = candidate.info;
  info.length = {0.0, length};
  info.duration = {0.0, length / velocity + velocity / std::max(0.1, data->bpp_param_ptr->max_acc)};
  info.velocity = {velocity, velocity};
  info.terminal_lane_changing_velocity = velocity;
  info.lane_changing_start = ego;
  info.lane_changing_end = goal;
  info.shift_line.start = ego;
  info.shift_line.end = goal;
  info.shift_line.end_idx = candidate.path.points.size() - 1;
  const auto reference_start =
    motion_utils::calcInterpolatedPose(reference.points, std::max(0.0, start_arc));
  info.shift_line.end_shift_length = -std::sin(tf2::getYaw(reference_start.orientation)) *
                                       (ego.position.x - reference_start.position.x) +
                                     std::cos(tf2::getYaw(reference_start.orientation)) *
                                       (ego.position.y - reference_start.position.y);
  for (size_t i = 0; i < candidate.shifted_path.shift_length.size(); ++i) {
    candidate.shifted_path.shift_length[i] =
      info.shift_line.end_shift_length *
      (1.0 - static_cast<double>(i) / (candidate.shifted_path.shift_length.size() - 1));
  }

  // Keep the continuation available for the shared stopping-envelope check and downstream stop.
  for (double arc = end_arc + 0.5; arc <= ref_length; arc += 0.5) {
    auto point = reference.points[motion_utils::findNearestIndex(
      reference.points, motion_utils::calcInterpolatedPose(reference.points, arc).position)];
    point.point.pose = motion_utils::calcInterpolatedPose(reference.points, arc);
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, static_cast<float>(velocity));
    candidate.path.points.push_back(point);
    if (arc >= end_arc + data->bpp_param_ptr->forward_path_length) break;
  }
  return candidate;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
