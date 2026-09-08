// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "autoware/behavior_path_static_obstacle_avoidance_module/static_collision.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
{
void enforceTrajectorySafetyMargins(AvoidanceParameters & parameters)
{
  const auto & safety = parameters.trajectory_collision;
  for (auto & [label, object] : parameters.object_parameters) {
    const auto found = safety.class_lateral_margins.find(label);
    const double nominal =
      found == safety.class_lateral_margins.end() ? safety.lateral_margin : found->second;
    const double clearance = nominal + safety.optimization_margin;
    if (
      !std::isfinite(clearance) || clearance < 0.0 ||
      !std::isfinite(object.envelope_buffer_margin) || !std::isfinite(object.lateral_hard_margin) ||
      !std::isfinite(object.lateral_hard_margin_for_parked_vehicle) ||
      !std::isfinite(object.lateral_soft_margin) || object.lateral_soft_margin < 0.0) {
      throw std::invalid_argument("Invalid static avoidance clearance parameters");
    }
    // Hard/soft distance, close-object feasibility and the necessity check all read these
    // fields. Clamping once prevents a best-effort branch from restoring a negative margin.
    object.envelope_buffer_margin = std::max(object.envelope_buffer_margin, nominal);
    object.lateral_hard_margin = std::max(object.lateral_hard_margin, clearance);
    object.lateral_hard_margin_for_parked_vehicle =
      std::max(object.lateral_hard_margin_for_parked_vehicle, clearance);
  }
}

utils::path_safety_checker::TrajectoryCollisionResult checkStaticObstacleCollision(
  const PathWithLaneId & path, const PredictedObjects & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const AvoidanceParameters & parameters, const double start_arc, const double end_arc,
  const Pose * ego_pose)
{
  return checkStaticObstacleCollision(
    path, objects, vehicle_info, parameters, start_arc, end_arc, ego_pose, std::nullopt);
}

utils::path_safety_checker::TrajectoryCollisionResult checkStaticObstacleCollision(
  const PathWithLaneId & path, const PredictedObjects & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const AvoidanceParameters & parameters, const double start_arc, const double end_arc,
  const Pose * ego_pose, const std::optional<selfcar::trajectory_safety::EgoMotion> & motion)
{
  if (
    path.points.size() < 2 || !std::isfinite(start_arc) || !std::isfinite(end_arc) ||
    !std::isfinite(vehicle_info.vehicle_width_m) || vehicle_info.vehicle_width_m <= 0.0) {
    return {};
  }
  if (end_arc < start_arc) {
    utils::path_safety_checker::TrajectoryCollisionResult result;
    result.valid = true;
    return result;  // The finite maneuver is already behind ego.
  }
  try {
    auto safety = parameters.trajectory_collision;
    // Retain per-class stationary filtering, never below the shared static threshold. Moving
    // objects excluded here still pass through the existing time-based safety checker.
    auto stationary = objects;
    stationary.objects.clear();
    for (const auto & object : objects.objects) {
      const auto & velocity = object.kinematics.initial_twist_with_covariance.twist.linear;
      if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y)) return {};
      const auto label = std::max_element(
        object.classification.begin(), object.classification.end(),
        [](const auto & a, const auto & b) { return a.probability < b.probability; });
      const auto parameter = parameters.object_parameters.find(
        label == object.classification.end() ? ObjectClassification::UNKNOWN : label->label);
      const double threshold = parameter == parameters.object_parameters.end()
                                 ? parameters.trajectory_collision.stationary_velocity
                                 : std::max(
                                     parameters.trajectory_collision.stationary_velocity,
                                     parameter->second.moving_speed_threshold);
      if (!std::isfinite(threshold) || threshold < 0.0) return {};
      if (std::hypot(velocity.x, velocity.y) > threshold) continue;
      safety.stationary_velocity = std::max(safety.stationary_velocity, threshold);
      stationary.objects.push_back(object);
    }

    const auto corridor_pose =
      ego_pose ? *ego_pose
               : autoware::motion_utils::calcInterpolatedPose(
                   path.points,
                   std::clamp(start_arc, 0.0, autoware::motion_utils::calcArcLength(path.points)));
    return utils::path_safety_checker::checkStaticTrajectory(
      path, stationary, vehicle_info, corridor_pose, start_arc, end_arc, safety, motion);
  } catch (const std::exception &) {
    return {};
  }
}

bool hasStaticObstacleCollision(
  const PathWithLaneId & path, const PredictedObjects & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const AvoidanceParameters & parameters, const double start_arc, const double end_arc,
  UUID * collided_object, const Pose * ego_pose)
{
  const auto result = checkStaticObstacleCollision(
    path, objects, vehicle_info, parameters, start_arc, end_arc, ego_pose);
  if (collided_object && result.object_id) *collided_object = *result.object_id;
  return !result.is_safe();
}
}  // namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
