// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include "scene.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::behavior_path_planner
{
bool AvoidanceByLaneChange::isRecedingTarget(
  const AvoidancePlanningData & data, const ObjectData & object) const
{
  // This filter governs new/waiting requests only. Keep an approved maneuver's
  // completion and collision handling independent of whether its original
  // obstacle starts moving.
  const auto & path = data.reference_path_rough.points;
  if (is_activated_ || path.size() < 2 || !planner_data_->dynamic_object) return false;
  const auto & parameters = avoidance_parameters_->object_parameters.at(
    utils::getHighestProbLabel(object.object.classification));
  const auto & velocity = object.object.kinematics.initial_twist_with_covariance.twist.linear;
  const double speed = std::hypot(velocity.x, velocity.y);
  if (
    !std::isfinite(speed) || speed <= parameters.moving_speed_threshold ||
    !(object.move_time >= parameters.moving_time_threshold))
    return false;

  const auto stamp = rclcpp::Time(planner_data_->dynamic_object->header.stamp).seconds();
  const auto age = rclcpp::Time(planner_data_->self_odometry->header.stamp).seconds() - stamp;
  if (stamp <= 0.0 || !std::isfinite(age) || age < -0.1 || age > 0.5) return false;

  const auto & pose = object.object.kinematics.initial_pose_with_covariance.pose;
  const auto index = motion_utils::findNearestSegmentIndex(path, pose.position);
  const auto & first = path.at(index).point.pose.position;
  const auto & second = path.at(index + 1).point.pose.position;
  const double path_yaw = std::atan2(second.y - first.y, second.x - first.x);
  const double angle = tf2::getYaw(pose.orientation) - path_yaw;
  const double forward = velocity.x * std::cos(angle) - velocity.y * std::sin(angle);
  const double lateral = velocity.x * std::sin(angle) + velocity.y * std::cos(angle);
  const double ego_speed = getEgoVelocity();
  // Do not classify crossing/oncoming traffic or a slower lead vehicle as
  // departing.
  constexpr double relative_speed_margin = 0.2;  // [m/s], reject near-equal noisy speeds.
  if (
    !std::isfinite(forward) || !std::isfinite(lateral) || !std::isfinite(ego_speed) ||
    forward <= std::max(0.0, ego_speed) + relative_speed_margin || std::abs(lateral) > forward)
    return false;

  // Use the CURRENT physical footprint, not the accumulated static-object
  // envelope. Its old rear edge can stay near the ego long after the tracked
  // vehicle has driven away.
  const auto footprint = autoware_utils::to_polygon2d(object.object);
  if (footprint.outer().empty()) return false;
  double distance = std::numeric_limits<double>::infinity();
  for (const auto & vertex : footprint.outer()) {
    Point point;
    point.x = vertex.x();
    point.y = vertex.y();
    const auto arc = motion_utils::calcSignedArcLength(path, getEgoPosition(), point);
    if (!std::isfinite(arc)) return false;
    distance = std::min(distance, arc);
  }
  const auto limit = avoidance_parameters_->max_execution_distance;
  return std::isfinite(limit) && limit > 0.0 && distance > limit;
}

void AvoidanceByLaneChange::updatePendingTarget(const ObjectData * target)
{
  if (is_activated_) return;
  const auto next = target ? std::make_optional(target->object.object_id) : std::nullopt;
  if (next == pending_target_id_ && (next || !speed_preparation_target_)) return;

  // Longitudinal preparation belongs to the current triggering object. Clear it
  // before path generation so the old target cannot seed another low-speed
  // candidate in the same lane.
  speed_preparation_target_.reset();
  speed_preparation_profile_.reset();
  speed_preparation_lane_id_ = lanelet::InvalId;
  speed_preparation_start_ = Pose{};
  terminal_lane_change_path_.reset();
  status_ = LaneChangeStatus();
  lane_change_debug_.reset();
  pending_target_id_ = next;
}
}  // namespace autoware::behavior_path_planner
