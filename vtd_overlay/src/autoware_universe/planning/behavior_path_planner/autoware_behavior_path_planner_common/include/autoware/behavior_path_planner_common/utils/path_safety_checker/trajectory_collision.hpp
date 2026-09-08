// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__TRAJECTORY_COLLISION_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__TRAJECTORY_COLLISION_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <selfcar/trajectory_safety/reachable_pose_envelope.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <limits>
#include <map>
#include <optional>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
// The names/units below are shared with motion_velocity_planner, not another RSS margin.
// The launch loads the SAME obstacle_stop and trajectory-polygon YAML files into both nodes.
struct TrajectoryCollisionParameters
{
  double lateral_margin{0.3};
  std::map<uint8_t, double> class_lateral_margins;
  double stationary_velocity{0.3};
  bool consider_current_pose{true};
  double time_to_convergence{1.5};
  double decimation_step{2.0};
  double stop_margin{5.0};
  double min_jerk{-1.0};
  double max_jerk{1.0};
  // Additional upstream clearance budget for subsequent path optimization. Never subtract it
  // from the downstream safety margin.
  double optimization_margin{0.15};
};

TrajectoryCollisionParameters loadTrajectoryCollisionParameters(rclcpp::Node & node);

struct TrajectoryCollisionResult
{
  bool valid{false};
  std::optional<unique_identifier_msgs::msg::UUID> object_id;
  double collision_arc{std::numeric_limits<double>::infinity()};
  [[nodiscard]] bool is_safe() const { return valid && !object_id; }
};

// Checks the finite maneuver [start_arc, end_arc], including the full vehicle at its endpoints.
// Uses exactly the downstream decimation and current-pose polygon implementation. Moving
// objects must ALSO be checked with the time-based predicted-path/RSS check by the caller.
TrajectoryCollisionResult checkStaticTrajectory(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  const geometry_msgs::msg::Pose & ego_pose, double start_arc, double end_arc,
  const TrajectoryCollisionParameters & parameters);

TrajectoryCollisionResult checkStaticTrajectory(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  const geometry_msgs::msg::Pose & ego_pose, double start_arc, double end_arc,
  const TrajectoryCollisionParameters & parameters,
  const std::optional<selfcar::trajectory_safety::EgoMotion> & motion);
}  // namespace autoware::behavior_path_planner::utils::path_safety_checker

#endif
