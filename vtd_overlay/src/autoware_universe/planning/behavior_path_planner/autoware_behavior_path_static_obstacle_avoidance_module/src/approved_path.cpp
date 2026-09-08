// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/scene.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/utils.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::behavior_path_planner
{
ShiftedPath StaticObstacleAvoidanceModule::makeExecutionPath(const ShiftedPath & path) const
{
  if (path.path.points.size() < 2 || path.shift_length.size() != path.path.points.size()) {
    return {};
  }
  ShiftedPath output;
  output.path = utils::resamplePathWithSpline(path.path, parameters_->resample_interval_for_output);
  output.shift_length.reserve(output.path.points.size());
  for (const auto & point : output.path.points) {
    const auto index =
      autoware::motion_utils::findNearestSegmentIndex(path.path.points, point.point.pose.position);
    const auto length = autoware_utils::calc_distance2d(
      path.path.points[index].point.pose, path.path.points[index + 1].point.pose);
    const double ratio = length < 1e-6 ? 0.0
                                       : std::clamp(
                                           autoware::motion_utils::calcLongitudinalOffsetToSegment(
                                             path.path.points, index, point.point.pose.position) /
                                             length,
                                           0.0, 1.0);
    output.shift_length.push_back(
      path.shift_length[index] + ratio * (path.shift_length[index + 1] - path.shift_length[index]));
  }
  return output;
}

bool StaticObstacleAvoidanceModule::isExecutionPathAligned(const ShiftedPath & path) const
{
  if (path.path.points.size() < 2) return false;
  const double ego_arc = autoware::motion_utils::calcSignedArcLength(
    path.path.points, path.path.points.front().point.pose.position, getEgoPosition());
  const auto matched_pose =
    autoware::motion_utils::calcInterpolatedPose(path.path.points, std::max(0.0, ego_arc));
  return std::abs(autoware_utils::calc_lateral_deviation(matched_pose, getEgoPosition())) <=
           std::max(0.1, parameters_->max_deviation_from_lane) &&
         std::abs(autoware_utils::calc_yaw_deviation(matched_pose, getEgoPose())) <= 0.26;
}

double StaticObstacleAvoidanceModule::getSafetyCheckEndArc(const ShiftedPath & shifted_path) const
{
  const auto & path = shifted_path.path;
  const auto & origin = path.points.front().point.pose.position;
  const auto arc = [&](const Point & point) {
    return autoware::motion_utils::calcSignedArcLength(path.points, origin, point);
  };
  const auto & vehicle = planner_data_->parameters.vehicle_info;
  double end_arc = arc(getEgoPosition()) +
                   std::max(helper_->getFeasibleDecelDistance(0.0), vehicle.vehicle_length_m);
  if (approved_path_) {
    // Do not let a new candidate, its return line, or reclassification of the same object
    // silently change the safety horizon of an already approved maneuver.
    return std::max(end_arc, approved_path_->safety_end_arc);
  }
  auto lines = path_shifter_.getShiftLines();
  const auto new_lines =
    utils::static_obstacle_avoidance::toShiftLineArray(avoid_data_.new_shift_line);
  lines.insert(lines.end(), new_lines.begin(), new_lines.end());
  for (const auto & line : lines) {
    end_arc = std::max(end_arc, arc(line.end.position));
  }
  const auto target = std::find_if(
    avoid_data_.target_objects.begin(), avoid_data_.target_objects.end(),
    [](const auto & object) { return object.avoid_required && object.longitudinal > 0.0; });
  if (target != avoid_data_.target_objects.end()) {
    end_arc = std::max(
      end_arc, arc(target->object.kinematics.initial_pose_with_covariance.pose.position) +
                 target->length + vehicle.rear_overhang_m);
  }
  return end_arc;
}

void StaticObstacleAvoidanceModule::updateApprovedPath(
  AvoidancePlanningData & data, DebugData & debug)
{
  auto & approved = *approved_path_;
  data.candidate_path = approved.spline;
  data.new_shift_line.clear();
  data.safe_shift_line.clear();
  data.valid = true;
  data.comfortable = true;
  data.ready = true;
  data.found_avoidance_path = true;
  data.avoid_required = true;
  data.state = AvoidanceState::RUNNING;

  const bool tracking_path = isExecutionPathAligned(approved.execution);
  const auto deactivated = [&](const std::string & direction, const auto & shifts) {
    return std::any_of(shifts.begin(), shifts.end(), [&](const auto & shift) {
      return rtc_interface_ptr_map_.at(direction)->isForceDeactivated(shift.uuid);
    });
  };
  const bool operator_stop =
    deactivated("left", left_shift_array_) || deactivated("right", right_shift_array_);

  // Re-evaluate exactly the resampled geometry that will be output. Approval never bypasses
  // stationary-footprint, predicted-object/RSS, current-pose, or operator-stop checks.
  const auto upstream = getPreviousModuleOutput();
  const bool upstream_available =
    upstream.path.points.size() >= 2 && upstream.reference_path.points.size() >= 2 &&
    upstream.path.header.frame_id == approved.execution.path.header.frame_id;
  auto execution = approved.execution;
  applyUpstreamVelocityLimits(execution);
  const bool safe =
    isSafePath(execution, debug) && tracking_path && !operator_stop && upstream_available;
  if (!safe) {
    if (!approved.blocked) {
      RCLCPP_WARN(
        getLogger(), "Approved avoidance blocked: keep geometry and stop (tracking=%s operator=%s)",
        tracking_path ? "true" : "false", operator_stop ? "true" : "false");
    }
    approved.blocked = true;
    approved.clear_since.reset();
  } else if (approved.blocked) {
    const auto now = clock_->now();
    if (!helper_->isVehicleStopped()) {
      approved.clear_since.reset();
    } else if (!approved.clear_since || now < *approved.clear_since) {
      approved.clear_since = now;
    } else if ((now - *approved.clear_since).seconds() >= 1.0) {
      // Resume the same maneuver only after stopping and a continuous clear interval. This is
      // not a new RTC request and cannot restart the approval/cancel loop every planning cycle.
      approved.blocked = false;
      approved.clear_since.reset();
      RCLCPP_INFO(getLogger(), "Approved avoidance clear: resume the retained path");
    }
  }
  data.safe = safe && !approved.blocked;
  data.yield_required = !data.safe;
}

void StaticObstacleAvoidanceModule::finishApprovedPath()
{
  if (
    !approved_path_ || approved_path_->blocked ||
    !isExecutionPathAligned(approved_path_->execution))
    return;
  const auto & approved = *approved_path_;
  const auto & path = approved.execution.path;
  const double remaining = autoware::motion_utils::calcSignedArcLength(
    path.points, getEgoPosition(), approved.finish_pose.position);
  if (remaining >= -planner_data_->parameters.vehicle_info.rear_overhang_m - 1.0) return;

  const auto & reference = path_shifter_.getReferencePath();
  if (reference.points.size() < 2) return;
  // Preserve the completed offset for the next (e.g. return) maneuver. Clearing it to zero
  // would put the next reference path back in the original lane beneath a shifted vehicle.
  path_shifter_.removeBehindShiftLineAndSetBaseOffset(
    planner_data_->findEgoIndex(reference.points));
  const auto finish = [&](const std::string & direction, auto & shifts) {
    for (const auto & shift : shifts) {
      rtc_interface_ptr_map_.at(direction)->updateCooperateStatus(
        shift.uuid, true, State::SUCCEEDED, remaining, remaining, clock_->now());
    }
    shifts.clear();
  };
  finish("left", left_shift_array_);
  finish("right", right_shift_array_);
  approved_path_.reset();
  generator_.reset();
  unlockNewModuleLaunch();
  RCLCPP_INFO(getLogger(), "Approved avoidance completed: allow the next maneuver");
}

void StaticObstacleAvoidanceModule::applyUpstreamVelocityLimits(ShiftedPath & path) const
{
  if (!approved_path_) return;
  const auto upstream = getPreviousModuleOutput().path;
  // Geometry is retained, not its message timestamp or permission to ignore a new upstream stop.
  path.path.header.stamp = clock_->now();
  if (upstream.points.size() < 2 || upstream.header.frame_id != path.path.header.frame_id) {
    for (auto & point : path.path.points) point.point.longitudinal_velocity_mps = 0.0;
    return;
  }

  const auto & spline = approved_path_->spline.path.points;
  const auto & reference = approved_path_->reference.points;
  const auto & origin = upstream.points.front().point.pose.position;
  const auto ego_index = planner_data_->findEgoSegmentIndex(upstream.points);
  double stop_arc = autoware::motion_utils::calcSignedArcLength(
    upstream.points, size_t{0}, upstream.points.size() - 1);
  for (size_t i = ego_index; i < upstream.points.size(); ++i) {
    const double velocity = upstream.points[i].point.longitudinal_velocity_mps;
    if (!std::isfinite(velocity) || velocity <= 0.0) {
      stop_arc = autoware::motion_utils::calcSignedArcLength(upstream.points, size_t{0}, i);
      break;
    }
  }

  for (auto & point : path.path.points) {
    // Use the unshifted reference corresponding to this spline point. Projecting a several-metre
    // shifted point directly onto the upstream path can select a different station on a bend.
    const auto index =
      autoware::motion_utils::findNearestSegmentIndex(spline, point.point.pose.position);
    const double length =
      autoware_utils::calc_distance2d(spline[index].point.pose, spline[index + 1].point.pose);
    const double ratio = length < 1e-6 ? 0.0
                                       : std::clamp(
                                           autoware::motion_utils::calcLongitudinalOffsetToSegment(
                                             spline, index, point.point.pose.position) /
                                             length,
                                           0.0, 1.0);
    const auto reference_pose = autoware_utils::calc_interpolated_pose(
      reference[index].point.pose, reference[index + 1].point.pose, ratio);
    const auto upstream_index =
      autoware::motion_utils::findNearestSegmentIndex(upstream.points, reference_pose.position);
    const double arc =
      autoware::motion_utils::calcSignedArcLength(upstream.points, origin, reference_pose.position);
    const double first = upstream.points[upstream_index].point.longitudinal_velocity_mps;
    const double second = upstream.points[upstream_index + 1].point.longitudinal_velocity_mps;
    const double limit = arc >= stop_arc || !std::isfinite(first) || !std::isfinite(second)
                           ? 0.0
                           : std::max(0.0, std::min(first, second));
    // PathShifter only changed geometry; the cached velocities were upstream limits at approval.
    // Refresh them instead of permanently latching an old red-light stop or temporary slowdown.
    // The usual avoidance velocity constraints are applied after this mapping in updateEgoBehavior.
    point.point.longitudinal_velocity_mps =
      std::min(limit, std::max(0.0, planner_data_->parameters.max_vel));
  }
}

void StaticObstacleAvoidanceModule::stopOnApprovedPath(ShiftedPath & path)
{
  // Retain the approved lateral geometry even midway through a shift. A zero target at the
  // current projection requests braking without substituting an unvalidated return-to-lane path.
  utils::static_obstacle_avoidance::insertDecelPoint(
    getEgoPosition(), 0.0, 0.0, path.path, stop_pose_);
  for (auto & point : path.path.points) {
    point.point.longitudinal_velocity_mps = 0.0;
  }
  if (stop_pose_) stop_pose_->detail = "approved avoidance blocked: retained path";
}
}  // namespace autoware::behavior_path_planner
