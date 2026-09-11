// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "autoware/behavior_path_lane_change_module/utils/intersection_exit.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <optional>
#include <string>
#include <system_error>

namespace autoware::behavior_path_planner::utils::lane_change
{
namespace
{
using route_handler::Direction;
constexpr double connection_tolerance = 0.1;
constexpr double rear_clearance = 0.5;

bool is_connector(const lanelet::ConstLanelet & lane)
{
  const auto turn = lane.attributeOr("turn_direction", std::string{});
  if (turn == "left" || turn == "right" || turn == "straight") return true;
  const auto area = lane.attributeOr("intersection_area", std::string{});
  lanelet::Id id = lanelet::InvalId;
  const auto result = std::from_chars(area.data(), area.data() + area.size(), id);
  return result.ec == std::errc{} && result.ptr == area.data() + area.size() && id > 0;
}

bool is_outgoing_road(const lanelet::ConstLanelet & lane)
{
  return lane.attributeOr("subtype", std::string{}) == "road" && !is_connector(lane);
}

bool joined(const lanelet::ConstPoint3d & a, const lanelet::ConstPoint3d & b)
{
  return std::hypot(a.x() - b.x(), a.y() - b.y()) <= connection_tolerance;
}

bool has_bounds(const lanelet::ConstLanelet & lane)
{
  return lane.leftBound3d().size() >= 2 && lane.rightBound3d().size() >= 2;
}

// The outgoing directions, not the connector's incoming direction or turn label, establish
// same-direction adjacency. This makes a left-turn exit followed by a right lane change valid.
bool same_outgoing_direction(
  const lanelet::ConstLanelet & current, const lanelet::ConstLanelet & target)
{
  const auto a = current.centerline2d();
  const auto b = target.centerline2d();
  if (a.size() < 2 || b.size() < 2) return false;
  const double ax = a[1].x() - a[0].x();
  const double ay = a[1].y() - a[0].y();
  const double bx = b[1].x() - b[0].x();
  const double by = b[1].y() - b[0].y();
  const double lengths = std::hypot(ax, ay) * std::hypot(bx, by);
  return lengths > 1e-6 && ax * bx + ay * by > 0.5 * lengths;
}

std::optional<geometry_msgs::msg::Point> exit_position(
  const lanelet::ConstLanelet & target, const lanelet::ConstLanelets & current_lanes,
  const Direction direction)
{
  if (
    (direction != Direction::LEFT && direction != Direction::RIGHT) || !is_outgoing_road(target) ||
    !has_bounds(target)) {
    return std::nullopt;
  }
  const bool left = direction == Direction::LEFT;
  const auto & target_inner = left ? target.rightBound3d() : target.leftBound3d();
  for (size_t i = 1; i < current_lanes.size(); ++i) {
    const auto & connector = current_lanes[i - 1];
    const auto & outgoing = current_lanes[i];
    if (
      !is_connector(connector) || !is_outgoing_road(outgoing) || !has_bounds(connector) ||
      !has_bounds(outgoing) || target.id() == outgoing.id()) {
      continue;
    }
    // Both boundaries must continue into this exact outgoing lane. A connector elsewhere in the
    // route, a nearby intersection, or a diverging/disconnected road is insufficient.
    if (
      !joined(connector.leftBound3d().back(), outgoing.leftBound3d().front()) ||
      !joined(connector.rightBound3d().back(), outgoing.rightBound3d().front())) {
      continue;
    }
    const auto & connector_outer = left ? connector.leftBound3d() : connector.rightBound3d();
    if (
      !joined(connector_outer.back(), target_inner.front()) ||
      !same_outgoing_direction(outgoing, target)) {
      continue;
    }
    // Confirm that target width lies on the requested side of the common outgoing boundary.
    const auto a = outgoing.centerline2d();
    const auto b = target.centerline2d();
    const double dx = a[1].x() - a[0].x();
    const double dy = a[1].y() - a[0].y();
    const double side = dx * (b.front().y() - a.front().y()) - dy * (b.front().x() - a.front().x());
    if ((left && side <= 0.0) || (!left && side >= 0.0)) continue;
    geometry_msgs::msg::Point position;
    position.x = a.front().x();
    position.y = a.front().y();
    position.z = outgoing.centerline3d().front().z();
    return position;
  }
  return std::nullopt;
}
}  // namespace

bool is_intersection_exit_target(
  const lanelet::ConstLanelet & real_target, const lanelet::ConstLanelets & current_lanes,
  const Direction direction)
{
  return exit_position(real_target, current_lanes, direction).has_value();
}

std::optional<double> intersection_exit_prepare_length(
  const behavior_path_planner::lane_change::CommonDataPtr & data)
{
  if (
    !data || !data->lanes_ptr || data->lanes_ptr->target.empty() || !data->self_odometry_ptr ||
    !data->bpp_param_ptr || !data->lc_param_ptr || data->current_lanes_path.points.size() < 2) {
    return std::nullopt;
  }
  auto target = data->lanes_ptr->target.front();
  if (data->route_handler_ptr) {
    const auto map = data->route_handler_ptr->getLaneletMapPtr();
    if (map && map->laneletLayer.exists(target.id())) target = map->laneletLayer.get(target.id());
  }
  // Without the actual map object a synthetic target cannot establish a real exit boundary.
  if (target.attributeOr("lane_change_backward_overlap", std::string{}) == "yes") {
    return std::nullopt;
  }
  const auto exit = exit_position(target, data->lanes_ptr->current, data->direction);
  if (!exit) return std::nullopt;
  const double distance = autoware::motion_utils::calcSignedArcLength(
    data->current_lanes_path.points, data->get_ego_pose().position, *exit);
  const auto & vehicle = data->bpp_param_ptr->vehicle_info;
  const double vicinity = std::max(
    data->lc_param_ptr->trajectory.target_lane_backward_overlap_length, vehicle.vehicle_length_m);
  const double prepare = distance + vehicle.rear_overhang_m + rear_clearance;
  if (!std::isfinite(distance) || distance > vicinity || prepare <= 0.0) return std::nullopt;
  return prepare;
}
bool starts_before_intersection_exit(
  const behavior_path_planner::lane_change::CommonDataPtr & data,
  const geometry_msgs::msg::Pose & lane_change_start)
{
  if (!data || !data->self_odometry_ptr || !data->lanes_ptr || data->lanes_ptr->target.empty())
    return false;
  auto target = data->lanes_ptr->target.front();
  if (data->route_handler_ptr) {
    const auto map = data->route_handler_ptr->getLaneletMapPtr();
    if (map && map->laneletLayer.exists(target.id())) target = map->laneletLayer.get(target.id());
  }
  if (!is_intersection_exit_target(target, data->lanes_ptr->current, data->direction)) return false;
  auto future = std::make_shared<behavior_path_planner::lane_change::CommonData>(*data);
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>(*data->self_odometry_ptr);
  odometry->pose.pose = lane_change_start;
  future->self_odometry_ptr = odometry;
  const auto distance = intersection_exit_prepare_length(future);
  return distance && *distance > 1e-3;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
