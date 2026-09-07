// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/difference.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::behavior_path_planner
{
utils::path_safety_checker::TrajectoryCollisionResult NormalLaneChange::check_static_path(
  const LaneChangePath & candidate) const
{
  if (!planner_data_->dynamic_object || candidate.path.points.size() < 2) return {};
  const auto & path = candidate.path;
  const auto & parameters = lane_change_parameters_->trajectory_safety;
  const auto start = motion_utils::calcSignedArcLength(
    path.points, path.points.front().point.pose.position, getEgoPosition());
  const auto end = motion_utils::calcSignedArcLength(
    path.points, path.points.front().point.pose.position,
    candidate.info.lane_changing_end.position);
  const auto velocity = std::max(0.0, candidate.info.terminal_lane_changing_velocity);
  const auto deceleration = -std::max(
    planner_data_->parameters.min_acc, lane_change_parameters_->trajectory.min_longitudinal_acc);
  if (!std::isfinite(velocity) || !std::isfinite(deceleration) || deceleration <= 0.0) return {};
  // Reserve space to finish the shift and then stop behind a car in the target lane. The swept
  // footprint already includes ego's front overhang; only add the downstream stop margin here.
  const auto stop_distance = motion_utils::calcDecelDistWithJerkAndAccConstraints(
    velocity, 0.0, candidate.info.longitudinal_acceleration.lane_changing, -deceleration,
    parameters.max_jerk, parameters.min_jerk);
  if (!stop_distance || !std::isfinite(*stop_distance)) return {};
  const auto buffer = parameters.stop_margin + *stop_distance;
  const auto checked_end = std::max(start, end) + buffer;
  // A truncated path is not evidence that the remainder of the stopping corridor is clear.
  if (checked_end > motion_utils::calcArcLength(path.points) + 1e-3) return {};
  return utils::path_safety_checker::checkStaticTrajectory(
    path, *planner_data_->dynamic_object, planner_data_->parameters.vehicle_info, getEgoPose(),
    start, checked_end, parameters);
}

bool NormalLaneChange::isStaticObstaclePathSafe(const LaneChangePath & path) const
{
  return check_static_path(path).is_safe();
}

void NormalLaneChange::stop_for_static_obstacle(PathWithLaneId & path)
{
  if (path.points.size() < 2) return;
  const auto ego_arc = motion_utils::calcSignedArcLength(
    path.points, path.points.front().point.pose.position, getEgoPosition());
  utils::path_safety_checker::TrajectoryCollisionResult collision;
  if (planner_data_->dynamic_object) {
    collision = utils::path_safety_checker::checkStaticTrajectory(
      path, *planner_data_->dynamic_object, planner_data_->parameters.vehicle_info, getEgoPose(),
      ego_arc, motion_utils::calcArcLength(path.points),
      lane_change_parameters_->trajectory_safety);
  }
  if (collision.is_safe()) return;
  const auto stop_arc = collision.valid ? collision.collision_arc -
                                            lane_change_parameters_->trajectory_safety.stop_margin
                                        : ego_arc;
  set_stop_pose(std::max(ego_arc, stop_arc), path, "static obstacle on lane-change path");
}

bool NormalLaneChange::updateApprovedPath()
{
  approved_path_blocked_ = false;
  if (!is_activated_ || !status_.is_valid_path || isAbortState()) return false;
  const bool stopped =
    std::abs(getEgoVelocity()) <= std::min(0.1, lane_change_parameters_->th_stop_velocity) &&
    getStopTime() >= lane_change_parameters_->th_stop_time;
  const auto & safety = lane_change_parameters_->trajectory_safety;
  const auto clearance = common_data_ptr_->transient_data.distance_to_static_obstacle;
  // Optimization downstream can still invalidate an otherwise clear nominal path. A prolonged
  // stop close to its blocking obstacle is also a recovery trigger, once per nominal path.
  const bool downstream_stalled =
    stopped && status_.lane_change_path.type != lane_change::PathType::LowSpeed &&
    std::isfinite(clearance) && clearance <= safety.stop_margin + 2.0 * safety.decimation_step;
  approved_path_blocked_ =
    downstream_stalled || !isStaticObstaclePathSafe(status_.lane_change_path);
  if (!approved_path_blocked_ || !common_data_ptr_->is_lanes_available() || !stopped) return false;
  const auto now = clock_.now();
  if (
    last_replan_time_ && (now - *last_replan_time_).seconds() >= 0.0 &&
    (now - *last_replan_time_).seconds() < 1.0)
    return false;
  last_replan_time_ = now;

  // Approved lanes and RTC remain fixed. Failed searches retain the old path and its stop.
  const auto target_objects = get_target_objects(filtered_objects_, get_current_lanes());
  const auto maximum_velocity = std::min(
    lane_change_parameters_->stopped_replan_velocity,
    common_data_ptr_->transient_data.current_path_velocity);
  if (maximum_velocity <= 0.0) return false;
  const auto maximum_length = std::min(
    {40.0, common_data_ptr_->transient_data.dist_to_terminal_end,
     common_data_ptr_->transient_data.dist_to_target_end});
  const auto regulatory_distance =
    utils::lane_change::get_distance_to_next_regulatory_element(common_data_ptr_, false, false);
  const auto & polygons = *common_data_ptr_->lanes_polygon_ptr;
  autoware_utils::StopWatch<std::chrono::milliseconds> search_time;
  const auto footprint_in_lanes = [&](const PathWithLaneId & path) {
    for (const auto & point : path.points) {
      const auto footprint = utils::lane_change::get_ego_footprint(
        point.point.pose, planner_data_->parameters.vehicle_info);
      std::vector<autoware_utils::Polygon2d> remaining{footprint};
      for (const auto * polygon :
           {&polygons.current, &polygons.target_neighbor, &polygons.target}) {
        std::vector<autoware_utils::Polygon2d> next;
        for (const auto & part : remaining) boost::geometry::difference(part, *polygon, next);
        remaining = std::move(next);
      }
      if (std::any_of(remaining.begin(), remaining.end(), [](const auto & polygon) {
            return std::abs(boost::geometry::area(polygon)) > 1e-4;
          }))
        return false;
    }
    return true;
  };
  for (double length = 6.0; length <= maximum_length && length < regulatory_distance;
       length += 2.0) {
    for (const double scale : {1.0, 0.75, 0.5}) {
      if (search_time.toc() > lane_change_parameters_->time_limit) return false;
      const auto velocity = maximum_velocity * scale;
      auto candidate =
        utils::lane_change::generate_low_speed_path(common_data_ptr_, length, velocity);
      if (!candidate || !footprint_in_lanes(candidate->shifted_path.path)) continue;
      if (
        utils::lane_change::is_intersecting_no_lane_change_lines(
          common_data_ptr_, candidate->info.length, candidate->shifted_path.path.points))
        continue;
      if (!isStaticObstaclePathSafe(*candidate)) continue;
      if (
        utils::lane_change::has_overtaking_turn_lane_object(
          common_data_ptr_, filtered_objects_.target_lane_trailing))
        continue;
      CollisionCheckDebugMap debug;
      const auto prediction =
        utils::lane_change::convert_to_predicted_paths(common_data_ptr_, *candidate, 1);
      if (prediction.empty() || prediction.front().size() < 2) continue;
      if (!isLaneChangePathSafe(
             *candidate, prediction, target_objects, lane_change_parameters_->safety.rss_params,
             debug, true)
             .is_safe)
        continue;
      status_.lane_change_path = std::move(*candidate);
      status_.is_safe = true;
      approved_path_blocked_ = false;
      unsafe_hysteresis_count_ = 0;
      toNormalState();
      RCLCPP_INFO(
        logger_, "Replanned blocked lane change from stopped pose: length=%.2f speed=%.2f", length,
        velocity);
      return true;
    }
  }
  return false;
}
}  // namespace autoware::behavior_path_planner
