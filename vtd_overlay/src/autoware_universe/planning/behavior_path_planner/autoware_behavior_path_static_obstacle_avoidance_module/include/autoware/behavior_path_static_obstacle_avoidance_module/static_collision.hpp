// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__STATIC_COLLISION_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__STATIC_COLLISION_HPP_

#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
{
// Keep every generator/feasibility check above the shared downstream clearance budget.
void enforceTrajectorySafetyMargins(AvoidanceParameters & parameters);

std::vector<utils::path_safety_checker::PoseWithVelocityStamped> predictManeuverPath(
  const PathWithLaneId & path, const Pose & ego_pose, double velocity, double horizon,
  double resolution, double max_acceleration, double min_acceleration);

utils::path_safety_checker::TrajectoryCollisionResult checkStaticObstacleCollision(
  const PathWithLaneId & path, const PredictedObjects & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const AvoidanceParameters & parameters, double start_arc, double end_arc,
  const Pose * ego_pose = nullptr);

utils::path_safety_checker::TrajectoryCollisionResult checkStaticObstacleCollision(
  const PathWithLaneId & path, const PredictedObjects & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const AvoidanceParameters & parameters, double start_arc, double end_arc,
  const Pose * ego_pose, const std::optional<selfcar::trajectory_safety::EgoMotion> & motion);

// Arc distances are measured from path.points.front(). Uses the downstream swept footprints,
// including objects excluded by avoidance-target, parked-object or lane-centroid filters.
// Pass the actual ego pose for final maneuver checks. A side-corridor probe omits it and uses
// the pose on that virtual, already-shifted corridor. Moving prediction remains separate.
bool hasStaticObstacleCollision(
  const PathWithLaneId & path, const PredictedObjects & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const AvoidanceParameters & parameters, double start_arc, double end_arc,
  UUID * collided_object = nullptr, const Pose * ego_pose = nullptr);
}  // namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__STATIC_COLLISION_HPP_
