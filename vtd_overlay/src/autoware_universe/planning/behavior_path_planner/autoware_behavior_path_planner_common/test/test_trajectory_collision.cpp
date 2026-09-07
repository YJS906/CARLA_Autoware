// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/trajectory_collision.hpp"

#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
namespace
{
auto path()
{
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  for (const double x : {0.0, 10.0, 20.0, 50.0}) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 2.0;
    path.points.push_back(point);
  }
  return path;
}
auto vehicle()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.3, 0.2, 2.0, 1.6, 1.0, 1.0, 0.2, 0.2, 1.5, 0.6);
}
auto objects(const double x, const double y, const double speed = 0.0)
{
  autoware_perception_msgs::msg::PredictedObjects objects;
  autoware_perception_msgs::msg::PredictedObject object;
  object.object_id.uuid[0] = 1;
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  object.kinematics.initial_pose_with_covariance.pose.position.y = y;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = speed;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 0.2;
  object.shape.dimensions.y = 0.2;
  object.shape.dimensions.z = 1.0;
  objects.objects.push_back(object);
  return objects;
}
auto check(const double x, const double y, const double begin = 0.0, const double end = 20.0)
{
  auto trajectory = path();
  auto ego = trajectory.points.front().point.pose;
  ego.position.x = begin;
  return checkStaticTrajectory(trajectory, objects(x, y), vehicle(), ego, begin, end, {});
}
}  // namespace

TEST(TrajectoryCollision, CoversSparseSweepsAndFullEndpointFootprints)
{
  EXPECT_FALSE(check(5.0, 0.0).is_safe());
  EXPECT_FALSE(check(22.9, 0.0).is_safe());
  EXPECT_TRUE(check(25.0, 0.0).is_safe());
  EXPECT_TRUE(check(3.0, 0.0, 10.0).is_safe());
  EXPECT_FALSE(check(9.2, 0.0, 10.0).is_safe());
}

TEST(TrajectoryCollision, IncludesDownstreamMarginAndOptimizationBudget)
{
  EXPECT_FALSE(check(10.0, 1.5).is_safe());
  EXPECT_TRUE(check(10.0, 1.7).is_safe());
  auto parameters = TrajectoryCollisionParameters{};
  parameters.class_lateral_margins[0] = 0.8;
  const auto trajectory = path();
  EXPECT_FALSE(checkStaticTrajectory(
                 trajectory, objects(10.0, 1.9), vehicle(), trajectory.points.front().point.pose,
                 0.0, 20.0, parameters)
                 .is_safe());
}

TEST(TrajectoryCollision, MatchesDownstreamSweptPolygonForCurrentPoseErrors)
{
  const auto trajectory = path();
  auto ego = trajectory.points.front().point.pose;
  ego.position.y = 0.7;
  ego.orientation = autoware_utils::create_quaternion_from_yaw(0.15);
  TrajectoryCollisionParameters parameters;
  parameters.optimization_margin = 0.0;
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  for (const auto & point : trajectory.points) {
    autoware_planning_msgs::msg::TrajectoryPoint p;
    p.pose = point.point.pose;
    p.longitudinal_velocity_mps = point.point.longitudinal_velocity_mps;
    points.push_back(p);
  }
  namespace downstream = autoware::motion_velocity_planner;
  const auto sampled = downstream::utils::decimate_trajectory_points_from_ego(
    points, points.front().pose, 3.0, 1.57, parameters.decimation_step, 0.0);
  const auto polygons = downstream::polygon_utils::create_one_step_polygons(
    sampled, vehicle(), ego, parameters.lateral_margin, true, parameters.time_to_convergence,
    parameters.decimation_step);
  for (double x : {1.0, 5.0, 15.0, 30.0}) {
    for (double y : {0.0, 1.2, 1.6, 2.0, 2.5}) {
      const auto detected = objects(x, y);
      const auto & object = detected.objects.front();
      const auto polygon = autoware_utils::to_polygon2d(
        object.kinematics.initial_pose_with_covariance.pose, object.shape);
      const auto collision = std::any_of(polygons.begin(), polygons.end(), [&](const auto & swept) {
        return !boost::geometry::disjoint(swept, polygon);
      });
      EXPECT_EQ(
        !checkStaticTrajectory(trajectory, detected, vehicle(), ego, 0.0, 50.0, parameters)
           .is_safe(),
        collision)
        << "x=" << x << " y=" << y;
    }
  }
}

TEST(TrajectoryCollision, InvalidGeometryFailsClosed)
{
  const autoware_perception_msgs::msg::PredictedObjects empty_objects;
  auto trajectory = path();
  const auto ego = trajectory.points.front().point.pose;
  trajectory.points[1].point.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    checkStaticTrajectory(trajectory, empty_objects, vehicle(), ego, 0.0, 20.0, {}).is_safe());
  trajectory = path();
  trajectory.points[1].point.pose.position.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(
    checkStaticTrajectory(trajectory, empty_objects, vehicle(), ego, 0.0, 20.0, {}).is_safe());
  trajectory = path();
  auto parameters = TrajectoryCollisionParameters{};
  parameters.lateral_margin = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    checkStaticTrajectory(trajectory, empty_objects, vehicle(), ego, 0.0, 20.0, parameters)
      .is_safe());
  EXPECT_FALSE(
    checkStaticTrajectory(trajectory, empty_objects, vehicle(), ego, 20.0, 0.0, {}).is_safe());
}

TEST(TrajectoryCollision, LoadsSharedNonDefaultMarginsAndRejectsInvalidParameters)
{
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("obstacle_stop.obstacle_filtering.lateral_margin.nominal.default", 0.55),
       rclcpp::Parameter("obstacle_stop.obstacle_filtering.lateral_margin.nominal.unknown", 0.65),
       rclcpp::Parameter("obstacle_stop.stop_planning.stop_margin", 7.0),
       rclcpp::Parameter("trajectory_polygon_collision_check.decimate_trajectory_step_length", 1.0),
       rclcpp::Parameter(
         "trajectory_polygon_collision_check.consider_current_pose.time_to_convergence", 2.5),
       rclcpp::Parameter("normal.min_jerk", -0.8)});
    rclcpp::Node node("shared_trajectory_parameters", options);
    const auto p = loadTrajectoryCollisionParameters(node);
    EXPECT_DOUBLE_EQ(p.lateral_margin, 0.55);
    EXPECT_DOUBLE_EQ(p.class_lateral_margins.at(0), 0.65);
    EXPECT_DOUBLE_EQ(p.stop_margin, 7.0);
    EXPECT_DOUBLE_EQ(p.decimation_step, 1.0);
    EXPECT_DOUBLE_EQ(p.time_to_convergence, 2.5);
    EXPECT_DOUBLE_EQ(p.min_jerk, -0.8);
    EXPECT_NO_THROW(loadTrajectoryCollisionParameters(node));
    node.set_parameter(rclcpp::Parameter("trajectory_safety.optimization_margin", -0.1));
    EXPECT_THROW(loadTrajectoryCollisionParameters(node), std::invalid_argument);
  }
  rclcpp::shutdown();
}
}  // namespace autoware::behavior_path_planner::utils::path_safety_checker
