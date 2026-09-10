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
#include <array>
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
    start, checked_end, parameters,
    selfcar::trajectory_safety::measuredEgoMotion(
      std::abs(getEgoVelocity()), planner_data_->parameters.max_acc,
      planner_data_->self_acceleration->accel.accel.linear.x));
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
      ego_arc, motion_utils::calcArcLength(path.points), lane_change_parameters_->trajectory_safety,
      selfcar::trajectory_safety::measuredEgoMotion(
        std::abs(getEgoVelocity()), planner_data_->parameters.max_acc,
        planner_data_->self_acceleration->accel.accel.linear.x));
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
  approved_path_blocked_ = !isStaticObstaclePathSafe(status_.lane_change_path);
  // A downstream obstacle_stop is the trigger; a local collision check alone must not
  // replace the approved curve with a newly fitted maneuver.
  if (!common_data_ptr_->is_lanes_available() || !stopped || !obstacle_stop_active_) {
    stopped_curve_cursor_ = 0;
    return false;
  }
  approved_path_blocked_ = true;
  const auto now = clock_.now();
  if (
    last_replan_time_ && (now - *last_replan_time_).seconds() >= 0.0 &&
    (now - *last_replan_time_).seconds() < 1.0)
    return false;
  last_replan_time_ = now;

  // Approved lanes and RTC remain fixed. Failed searches retain the old path and its stop.
  const auto target_objects = get_target_objects(filtered_objects_, get_current_lanes());
  auto maximum_velocity = std::min(
    lane_change_parameters_->stopped_replan_velocity,
    common_data_ptr_->transient_data.current_path_velocity);
  if (!std::isfinite(maximum_velocity) || maximum_velocity <= 0.0) return false;
  if (!stopped_curve_template_) stopped_curve_template_ = status_.lane_change_path;
  const auto & original = *stopped_curve_template_;
  if (original.path.points.size() < 4) return false;
  const auto curve_begin = motion_utils::findNearestIndex(
    original.path.points, original.info.lane_changing_start.position);
  const auto curve_end =
    motion_utils::findNearestIndex(original.path.points, original.info.lane_changing_end.position);
  if (curve_end <= curve_begin) return false;
  for (size_t i = curve_begin; i <= curve_end; ++i) {
    const auto speed = original.path.points[i].point.longitudinal_velocity_mps;
    if (!std::isfinite(speed) || speed <= 0.0) return false;
    maximum_velocity = std::min<double>(maximum_velocity, speed);
  }
  const auto maximum_length = std::min(
    common_data_ptr_->transient_data.dist_to_terminal_end,
    common_data_ptr_->transient_data.dist_to_target_end);
  const auto regulatory_distance =
    utils::lane_change::get_distance_to_next_regulatory_element(common_data_ptr_, false, false);
  const auto & polygons = *common_data_ptr_->lanes_polygon_ptr;
  std::vector<lanelet::BasicPolygon2d> corridor_polygons;
  for (const auto & lane : get_lane_change_corridor()) {
    corridor_polygons.push_back(
      utils::lane_change::create_polygon({lane}, 0.0, std::numeric_limits<double>::max()));
  }
  autoware_utils::StopWatch<std::chrono::milliseconds> search_time;
  const auto trim_to_corridor = [&](PathWithLaneId & path, size_t curve_end) {
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto & point = path.points[i];
      const auto footprint = utils::lane_change::get_ego_footprint(
        point.point.pose, planner_data_->parameters.vehicle_info);
      std::vector<autoware_utils::Polygon2d> remaining{footprint};
      for (const auto * polygon :
           {&polygons.current, &polygons.target_neighbor, &polygons.target}) {
        std::vector<autoware_utils::Polygon2d> next;
        for (const auto & part : remaining) boost::geometry::difference(part, *polygon, next);
        remaining = std::move(next);
      }
      for (const auto & polygon : corridor_polygons) {
        std::vector<autoware_utils::Polygon2d> next;
        for (const auto & part : remaining) boost::geometry::difference(part, polygon, next);
        remaining = std::move(next);
      }
      if (std::any_of(remaining.begin(), remaining.end(), [](const auto & polygon) {
            return std::abs(boost::geometry::area(polygon)) > 1e-4;
          })) {
        if (i <= curve_end) return false;
        path.points.resize(i);
        break;
      }
    }
    return true;
  };
  const auto corridor = get_lane_change_corridor();
  std::vector<std::pair<lanelet::Id, lanelet::BasicPolygon2d>> lane_polygons;
  for (const auto & lane : corridor) {
    lane_polygons.emplace_back(lane.id(), lane.polygon2d().basicPolygon());
  }
  // Search the smallest forward displacement first. A fixed original template bounds the
  // total translation to 12 m; repeated STOP feedback cannot progressively stretch the curve.
  constexpr size_t sample_count = 36;
  constexpr std::array<double, 3> scales{1.0, 0.75, 0.5};
  for (size_t checked = 0; checked < sample_count; ++checked) {
    if (search_time.toc() >= lane_change_parameters_->time_limit) return false;
    const size_t sample = stopped_curve_cursor_;
    stopped_curve_cursor_ = (stopped_curve_cursor_ + 1) % sample_count;
    const double offset = 1.0 + sample / scales.size();
    if (offset <= stopped_curve_offset_) continue;
    const double velocity = maximum_velocity * scales[sample % scales.size()];
    auto candidate =
      utils::lane_change::translate_approved_curve(common_data_ptr_, original, offset, velocity);
    if (!candidate) continue;
    const auto & reference = common_data_ptr_->current_lanes_path.points;
    if (reference.size() < 2) continue;
    const double end_distance = motion_utils::calcSignedArcLength(
      reference, getEgoPosition(), candidate->info.lane_changing_end.position);
    if (
      !std::isfinite(end_distance) || end_distance <= 0.0 || end_distance > maximum_length ||
      end_distance >= regulatory_distance)
      continue;
    if (!boost::geometry::covered_by(
          autoware_utils::Point2d(
            candidate->info.lane_changing_end.position.x,
            candidate->info.lane_changing_end.position.y),
          polygons.target))
      continue;
    // Translation can cross longitudinal lanelet boundaries, so recompute memberships.
    bool has_memberships = true;
    const size_t end_index = candidate->info.shift_line.end_idx;
    for (size_t i = 0; i < candidate->path.points.size(); ++i) {
      auto & point = candidate->path.points[i];
      point.lane_ids.clear();
      const autoware_utils::Point2d position(
        point.point.pose.position.x, point.point.pose.position.y);
      for (const auto & [id, polygon] : lane_polygons) {
        if (boost::geometry::covered_by(position, polygon)) point.lane_ids.push_back(id);
      }
      if (point.lane_ids.empty()) {
        if (i <= end_index)
          has_memberships = false;
        else
          candidate->path.points.resize(i);
        break;
      }
    }
    if (!has_memberships || !trim_to_corridor(candidate->path, end_index)) continue;
    // If the translated tail reaches the legal corridor edge, keep only the legal prefix.
    // The stopping-envelope check below rejects tails too short to stop after the merge.
    candidate->path.points.back().point.longitudinal_velocity_mps = 0.0;
    for (size_t i = 0; i < candidate->shifted_path.path.points.size(); ++i)
      candidate->shifted_path.path.points[i].lane_ids = candidate->path.points[i].lane_ids;
    if (utils::lane_change::is_intersecting_no_lane_change_lines(
          common_data_ptr_, candidate->info.length, candidate->shifted_path.path.points))
      continue;
    if (!isStaticObstaclePathSafe(*candidate)) continue;
    if (utils::lane_change::has_overtaking_turn_lane_object(
          common_data_ptr_, filtered_objects_.target_lane_trailing))
      continue;
    CollisionCheckDebugMap debug;
    const auto prediction =
      utils::lane_change::convert_to_predicted_paths(common_data_ptr_, *candidate, 1);
    if (
      prediction.empty() || prediction.front().size() < 2 ||
      autoware_utils::calc_distance2d(
        prediction.front().back().pose, candidate->info.lane_changing_end) > 0.25)
      continue;
    if (!isLaneChangePathSafe(
           *candidate, prediction, target_objects, lane_change_parameters_->safety.rss_params,
           debug, true)
           .is_safe)
      continue;
    status_.lane_change_path = std::move(*candidate);
    status_.is_safe = true;
    approved_path_blocked_ = false;
    unsafe_hysteresis_count_ = 0;
    stopped_curve_offset_ = offset;
    stopped_curve_cursor_ = 0;
    toNormalState();
    RCLCPP_INFO(
      logger_,
      "Translated approved curve after obstacle_stop: offset=%.2f connector=%.2f "
      "unchanged_curve=%.2f speed=%.2f (approved target unchanged)",
      offset, status_.lane_change_path.info.length.prepare,
      status_.lane_change_path.info.length.lane_changing, velocity);
    return true;
  }
  return false;
}
}  // namespace autoware::behavior_path_planner
