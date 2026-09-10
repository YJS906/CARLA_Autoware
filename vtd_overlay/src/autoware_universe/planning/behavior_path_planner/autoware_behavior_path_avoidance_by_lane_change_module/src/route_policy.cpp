// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "scene.hpp"

#include <autoware/behavior_path_lane_change_module/utils/utils.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::behavior_path_planner
{
namespace
{
double arc_at(const PathWithLaneId & path, const Point & point)
{
  return motion_utils::calcSignedArcLength(
    path.points, path.points.front().point.pose.position, point);
}

}  // namespace

void AvoidanceByLaneChange::updateStationaryObservations()
{
  auto & stationary_observations_ = motion_history_->observations;
  auto & observation_stamp_ = motion_history_->stamp;
  if (!planner_data_ || !planner_data_->dynamic_object) {
    stationary_observations_.clear();
    observation_stamp_.reset();
    return;
  }
  const auto & objects = *planner_data_->dynamic_object;
  const auto stamp = rclcpp::Time(objects.header.stamp).seconds();
  // Repeated/compensated perception samples must not accrue stationary evidence.
  if (observation_stamp_ && stamp == *observation_stamp_) return;
  if (
    !std::isfinite(stamp) || stamp <= 0.0 ||
    (observation_stamp_ && (stamp < *observation_stamp_ || stamp - *observation_stamp_ > 0.5))) {
    stationary_observations_.clear();
  }
  observation_stamp_ = stamp;
  for (const auto & object : objects.objects) {
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto & v = object.kinematics.initial_twist_with_covariance.twist.linear;
    if (
      !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(pose.position.x) ||
      !std::isfinite(pose.position.y))
      continue;
    const bool moving =
      std::hypot(v.x, v.y) > lane_change_parameters_->trajectory_safety.stationary_velocity;
    auto [it, inserted] = stationary_observations_.try_emplace(
      object.object_id.uuid,
      AvoidanceMotionHistory::Observation{pose.position, stamp, stamp, !moving, moving});
    auto & observation = it->second;
    // Keep the original displacement-based motion evidence as well as the speed threshold.
    const bool displaced =
      !inserted && autoware_utils::calc_distance2d(observation.anchor, pose.position) >
                     avoidance_parameters_->route_blockage_max_position_drift;
    const bool observed_motion = moving || displaced;
    // Start the stationary window at the FIRST stopped sample after motion, not at the
    // last moving sample. A restart invalidates eligibility in this same update; there is
    // no seven-second delay before treating the object as moving again.
    if (!inserted && (observed_motion || !observation.stationary)) {
      observation.anchor = pose.position;
      observation.since = stamp;
    }
    observation.observed_moving |= observed_motion;
    observation.stationary = !observed_motion;
    observation.last_seen = stamp;
  }
  for (auto it = stationary_observations_.begin(); it != stationary_observations_.end();) {
    if (it->second.last_seen != stamp)
      it = stationary_observations_.erase(it);
    else
      ++it;
  }
}

bool AvoidanceByLaneChange::isPersistentBlocker(const unique_identifier_msgs::msg::UUID & id) const
{
  const auto & stationary_observations_ = motion_history_->observations;
  const auto & observation_stamp_ = motion_history_->stamp;
  const auto it = stationary_observations_.find(id.uuid);
  if (it == stationary_observations_.end() || !observation_stamp_) return false;
  const auto & observation = it->second;
  const auto data_age =
    rclcpp::Time(planner_data_->self_odometry->header.stamp).seconds() - *observation_stamp_;
  if (!std::isfinite(data_age) || data_age < -0.1 || data_age > 0.5) return false;
  // Preserve the original static-object confirmation. Only objects with motion evidence
  // take the separate seven-second reclassification branch; neither branch forces approval.
  const auto required_duration =
    observation.observed_moving ? avoidance_parameters_->route_stopped_dynamic_min_duration
                               : avoidance_parameters_->route_blockage_min_duration;
  return observation.stationary && observation.last_seen == *observation_stamp_ &&
         observation.last_seen - observation.since >= required_duration;
}

bool AvoidanceByLaneChange::isRouteDepartureRequired() const
{
  const auto * obstacle = getNearestAvoidanceTarget();
  if (
    !obstacle || !common_data_ptr_->is_lanes_available() ||
    !isPersistentBlocker(obstacle->object.object_id) ||
    avoidance_data_.reference_path.points.size() < 2 || !planner_data_->dynamic_object)
    return false;

  // Signal color alone must not suppress avoidance candidate planning. Downstream traffic-light
  // stop control remains responsible for obeying signals; retain the regulatory-distance,
  // physical blockage, queue and maneuver safety checks.
  const auto regulatory_distance =
    utils::lane_change::get_distance_to_next_regulatory_element(common_data_ptr_, false, false);
  if (
    std::isfinite(regulatory_distance) &&
    regulatory_distance <= obstacle->longitudinal + obstacle->length +
                             lane_change_parameters_->trajectory_safety.stop_margin)
    return false;

  const auto ego_arc = arc_at(avoidance_data_.reference_path, getEgoPosition());
  const auto collision = utils::path_safety_checker::checkStaticTrajectory(
    avoidance_data_.reference_path, *planner_data_->dynamic_object, getCommonParam().vehicle_info,
    getEgoPose(), ego_arc, motion_utils::calcArcLength(avoidance_data_.reference_path.points),
    lane_change_parameters_->trajectory_safety);
  if (
    !collision.valid || !collision.object_id ||
    *collision.object_id != obstacle->object.object_id) {
    return false;
  }
  // Do not overtake a stationary queue merely because its first vehicle has not moved recently.
  for (const auto & other : planner_data_->dynamic_object->objects) {
    if (other.object_id == obstacle->object.object_id) continue;
    const auto & pose = other.kinematics.initial_pose_with_covariance.pose;
    const auto distance = arc_at(avoidance_data_.reference_path, pose.position) - ego_arc;
    if (
      distance < obstacle->longitudinal ||
      distance > obstacle->longitudinal + obstacle->length + 20.0)
      continue;
    if (
      std::abs(
        motion_utils::calcLateralOffset(avoidance_data_.reference_path.points, pose.position)) <
      getCommonParam().vehicle_width)
      return false;
  }
  return true;
}

}  // namespace autoware::behavior_path_planner
