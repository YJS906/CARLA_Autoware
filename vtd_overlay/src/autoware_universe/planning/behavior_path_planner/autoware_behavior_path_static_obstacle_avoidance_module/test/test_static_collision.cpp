// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "autoware/behavior_path_static_obstacle_avoidance_module/static_collision.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
{
namespace
{
PathWithLaneId straightPath(const double lateral = 0.0)
{
  PathWithLaneId path;
  for (const double x : {0.0, 10.0, 20.0, 100.0}) {
    PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.position.y = lateral;
    point.point.pose.orientation.w = 1.0;
    path.points.push_back(point);
  }
  return path;
}

PredictedObjects obstacleAt(const double x, const double y, const double velocity = 0.0)
{
  PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  object.kinematics.initial_pose_with_covariance.pose.position.y = y;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = velocity;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 0.2;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.0;
  ObjectClassification classification;
  classification.label = ObjectClassification::UNKNOWN;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  PredictedObjects objects;
  objects.objects.push_back(object);
  return objects;
}

auto vehicleInfo()
{
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.vehicle_width_m = 2.0;
  vehicle.max_longitudinal_offset_m = 3.0;
  vehicle.rear_overhang_m = 1.0;
  return vehicle;
}
}  // namespace

TEST(StaticCollision, ChecksSweptFootprintBetweenSparsePoints)
{
  EXPECT_TRUE(hasStaticObstacleCollision(
    straightPath(), obstacleAt(5.0, 0.0), vehicleInfo(), AvoidanceParameters{}, 0.0, 10.0));
}

TEST(StaticCollision, RejectsRightLaneAndAcceptsLeftLane)
{
  const auto objects = obstacleAt(10.0, -3.2);
  EXPECT_TRUE(hasStaticObstacleCollision(
    straightPath(-3.2), objects, vehicleInfo(), AvoidanceParameters{}, 0.0, 20.0));
  EXPECT_FALSE(hasStaticObstacleCollision(
    straightPath(3.2), objects, vehicleInfo(), AvoidanceParameters{}, 0.0, 20.0));
}

TEST(StaticCollision, DoesNotVetoForObstacleBeyondManeuver)
{
  EXPECT_FALSE(hasStaticObstacleCollision(
    straightPath(), obstacleAt(70.0, 0.0), vehicleInfo(), AvoidanceParameters{}, 0.0, 20.0));
  EXPECT_TRUE(hasStaticObstacleCollision(
    straightPath(), obstacleAt(70.0, 0.0), vehicleInfo(), AvoidanceParameters{}, 0.0, 80.0));
}

TEST(StaticCollision, ChecksBodyOverlapNotOnlyCenterline)
{
  EXPECT_TRUE(hasStaticObstacleCollision(
    straightPath(), obstacleAt(12.0, 1.4), vehicleInfo(), AvoidanceParameters{}, 0.0, 20.0));
  EXPECT_FALSE(hasStaticObstacleCollision(
    straightPath(), obstacleAt(12.0, 2.0), vehicleInfo(), AvoidanceParameters{}, 0.0, 20.0));
}

TEST(StaticCollision, IgnoresAlreadyPassedObstacle)
{
  EXPECT_FALSE(hasStaticObstacleCollision(
    straightPath(), obstacleAt(3.0, 0.0), vehicleInfo(), AvoidanceParameters{}, 10.0, 20.0));
}

TEST(StaticCollision, MovingObjectsAreLeftToPrediction)
{
  EXPECT_FALSE(hasStaticObstacleCollision(
    straightPath(), obstacleAt(10.0, 0.0, 5.0), vehicleInfo(), AvoidanceParameters{}, 0.0, 20.0));
}

TEST(StaticCollision, AvoidanceTypeAndNegativeMarginCannotHideBodyOverlap)
{
  AvoidanceParameters parameters;
  ObjectParameter object_parameter{};
  object_parameter.is_avoidance_target = false;
  object_parameter.is_safety_check_target = false;
  object_parameter.moving_speed_threshold = 0.5;
  object_parameter.lateral_hard_margin = -0.2;
  parameters.object_parameters.emplace(ObjectClassification::UNKNOWN, object_parameter);
  EXPECT_TRUE(hasStaticObstacleCollision(
    straightPath(), obstacleAt(10.0, 0.0), vehicleInfo(), parameters, 0.0, 20.0));
}

TEST(StaticCollision, InvalidPathDoesNotPassSafetyCheck)
{
  EXPECT_TRUE(hasStaticObstacleCollision(
    PathWithLaneId{}, PredictedObjects{}, vehicleInfo(), AvoidanceParameters{}, 0.0, 20.0));
}
}  // namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
