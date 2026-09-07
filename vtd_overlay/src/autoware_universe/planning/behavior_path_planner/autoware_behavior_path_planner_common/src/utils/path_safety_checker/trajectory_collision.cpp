// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/trajectory_collision.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
TrajectoryCollisionParameters loadTrajectoryCollisionParameters(rclcpp::Node & node)
{
  const auto get_with_default = [&](const std::string & name, auto fallback) {
    using T = decltype(fallback);
    return node.has_parameter(name) ? node.get_parameter(name).get_value<T>()
                                    : node.declare_parameter<T>(name, fallback);
  };
  using Label = autoware_perception_msgs::msg::ObjectClassification;
  TrajectoryCollisionParameters p;
  const std::string margin = "obstacle_stop.obstacle_filtering.lateral_margin.nominal.";
  p.lateral_margin = get_with_default(margin + "default", 0.3);
  for (const auto & [label, name] : std::vector<std::pair<uint8_t, std::string>>{
         {Label::UNKNOWN, "unknown"},
         {Label::CAR, "car"},
         {Label::TRUCK, "truck"},
         {Label::BUS, "bus"},
         {Label::TRAILER, "trailer"},
         {Label::MOTORCYCLE, "motorcycle"},
         {Label::BICYCLE, "bicycle"},
         {Label::PEDESTRIAN, "pedestrian"}}) {
    p.class_lateral_margins[label] = get_with_default(margin + name, p.lateral_margin);
  }
  const std::string ns = "trajectory_polygon_collision_check.";
  p.consider_current_pose =
    get_with_default(ns + "consider_current_pose.enable_to_consider_current_pose", true);
  p.time_to_convergence = get_with_default(ns + "consider_current_pose.time_to_convergence", 1.5);
  p.decimation_step = get_with_default(ns + "decimate_trajectory_step_length", 2.0);
  p.stop_margin = get_with_default("obstacle_stop.stop_planning.stop_margin", 5.0);
  p.optimization_margin = get_with_default("trajectory_safety.optimization_margin", 0.15);
  p.min_jerk = get_with_default("normal.min_jerk", -1.0);
  p.max_jerk = get_with_default("normal.max_jerk", 1.0);
  if (
    !std::isfinite(p.lateral_margin) || p.lateral_margin < 0.0 ||
    !std::isfinite(p.optimization_margin) || p.optimization_margin < 0.0 ||
    !std::isfinite(p.decimation_step) || p.decimation_step <= 0.0 ||
    !std::isfinite(p.time_to_convergence) || p.time_to_convergence <= 0.0 ||
    !std::isfinite(p.stop_margin) || p.stop_margin < 0.0 || !std::isfinite(p.min_jerk) ||
    p.min_jerk >= 0.0 || !std::isfinite(p.max_jerk) || p.max_jerk <= 0.0 ||
    std::any_of(
      p.class_lateral_margins.begin(), p.class_lateral_margins.end(),
      [](const auto & v) { return !std::isfinite(v.second) || v.second < 0.0; })) {
    throw std::invalid_argument("Invalid shared trajectory collision safety parameters");
  }
  return p;
}

TrajectoryCollisionResult checkStaticTrajectory(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  const geometry_msgs::msg::Pose & ego_pose, const double start_arc, const double end_arc,
  const TrajectoryCollisionParameters & p)
{
  TrajectoryCollisionResult result;
  const auto valid_pose = [](const geometry_msgs::msg::Pose & pose) {
    const auto & q = pose.orientation;
    const auto norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) && std::isfinite(norm) && norm > 1e-6;
  };
  if (
    path.points.size() < 2 || !std::isfinite(start_arc) || !std::isfinite(end_arc) ||
    !std::isfinite(vehicle.vehicle_width_m) || vehicle.vehicle_width_m <= 0.0 ||
    !std::isfinite(vehicle.max_longitudinal_offset_m) || vehicle.max_longitudinal_offset_m <= 0.0 ||
    !std::isfinite(vehicle.rear_overhang_m) || vehicle.rear_overhang_m < 0.0 ||
    !std::isfinite(p.decimation_step) || p.decimation_step <= 0.0 ||
    !std::isfinite(p.lateral_margin) || p.lateral_margin < 0.0 ||
    !std::isfinite(p.optimization_margin) || p.optimization_margin < 0.0 ||
    !std::isfinite(p.stationary_velocity) || p.stationary_velocity < 0.0 ||
    !std::isfinite(p.time_to_convergence) || p.time_to_convergence <= 0.0 ||
    !valid_pose(ego_pose) ||
    std::any_of(
      path.points.begin(), path.points.end(),
      [&](const auto & point) {
        return !valid_pose(point.point.pose) ||
               !std::isfinite(point.point.longitudinal_velocity_mps);
      }) ||
    std::any_of(
      p.class_lateral_margins.begin(), p.class_lateral_margins.end(),
      [](const auto & v) { return !std::isfinite(v.second) || v.second < 0.0; })) {
    return result;
  }
  if (end_arc < start_arc) {
    return result;
  }
  using autoware_planning_msgs::msg::TrajectoryPoint;
  std::vector<TrajectoryPoint> trajectory;
  std::vector<double> arcs{0.0};
  for (size_t i = 1; i < path.points.size(); ++i) {
    arcs.push_back(
      arcs.back() +
      autoware_utils::calc_distance2d(path.points[i - 1].point.pose, path.points[i].point.pose));
  }
  if (!std::isfinite(arcs.back())) return result;
  const double begin = std::clamp(start_arc, 0.0, arcs.back());
  const double end = std::clamp(end_arc, begin, arcs.back());
  if (end - begin < 1e-3) return result;
  // Exact endpoint interpolation avoids checking an unrelated obstacle one coarse sample past
  // the maneuver. Do not extend this finite maneuver as though it were the route's final goal.
  const auto append_at = [&](const double arc) {
    TrajectoryPoint point;
    point.pose = autoware::motion_utils::calcInterpolatedPose(path.points, arc);
    const auto it = std::lower_bound(arcs.begin(), arcs.end(), arc);
    const auto index = std::min<size_t>(std::distance(arcs.begin(), it), path.points.size() - 1);
    point.longitudinal_velocity_mps = path.points[index].point.longitudinal_velocity_mps;
    trajectory.push_back(point);
  };
  append_at(begin);
  for (size_t i = 0; i < path.points.size(); ++i) {
    if (arcs[i] <= begin || arcs[i] >= end) continue;
    TrajectoryPoint point;
    point.pose = path.points[i].point.pose;
    point.longitudinal_velocity_mps = path.points[i].point.longitudinal_velocity_mps;
    trajectory.push_back(point);
  }
  append_at(end);
  try {
    namespace downstream = autoware::motion_velocity_planner;
    auto decimated = downstream::utils::decimate_trajectory_points_from_ego(
      trajectory, trajectory.front().pose, 3.0, 1.57, p.decimation_step, 0.0);
    if (decimated.size() < 2) decimated = trajectory;
    double smallest_collision_arc = std::numeric_limits<double>::infinity();
    std::map<double, std::vector<autoware_utils::Polygon2d>> polygon_cache;
    for (const auto & object : objects.objects) {
      const auto & velocity = object.kinematics.initial_twist_with_covariance.twist.linear;
      if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y)) return result;
      if (std::hypot(velocity.x, velocity.y) > p.stationary_velocity) continue;
      const auto label_it = std::max_element(
        object.classification.begin(), object.classification.end(),
        [](const auto & a, const auto & b) { return a.probability < b.probability; });
      const uint8_t label = label_it == object.classification.end() ? 0 : label_it->label;
      const auto margin_it = p.class_lateral_margins.find(label);
      const double margin =
        (margin_it == p.class_lateral_margins.end() ? p.lateral_margin : margin_it->second) +
        p.optimization_margin;
      if (!valid_pose(object.kinematics.initial_pose_with_covariance.pose)) return result;
      const auto object_polygon = autoware_utils::to_polygon2d(
        object.kinematics.initial_pose_with_covariance.pose, object.shape);
      if (!boost::geometry::is_valid(object_polygon)) return result;
      if (!polygon_cache.count(margin)) {
        polygon_cache[margin] = downstream::polygon_utils::create_one_step_polygons(
          decimated, vehicle, ego_pose, margin, p.consider_current_pose, p.time_to_convergence,
          p.decimation_step);
      }
      const auto & polygons = polygon_cache.at(margin);
      if (polygons.size() != decimated.size()) return result;
      for (size_t i = 0; i < polygons.size(); ++i) {
        if (boost::geometry::disjoint(polygons[i], object_polygon)) continue;
        const auto arc = autoware::motion_utils::calcSignedArcLength(
          path.points, path.points.front().point.pose.position,
          decimated[i ? i - 1 : 0].pose.position);
        if (arc < smallest_collision_arc) {
          smallest_collision_arc = arc;
          result.object_id = object.object_id;
          result.collision_arc = arc;
        }
        break;
      }
    }
  } catch (const std::exception &) {
    return result;  // Fail closed; invalid geometry is not a clear corridor.
  }
  result.valid = true;
  return result;
}
}  // namespace autoware::behavior_path_planner::utils::path_safety_checker
