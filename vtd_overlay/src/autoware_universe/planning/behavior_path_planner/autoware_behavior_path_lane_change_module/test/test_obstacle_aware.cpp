// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace autoware::behavior_path_planner
{
namespace
{
auto reference(const double y)
{
  PathWithLaneId path;
  for (double x = -10; x <= 100; x += 1.0) {
    PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.position.y = y;
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 5.0;
    path.points.push_back(point);
  }
  return path;
}
lanelet::Lanelet lane(const lanelet::Id id, const double y)
{
  return lanelet::Lanelet(
    id,
    lanelet::LineString3d(
      id + 10,
      {lanelet::Point3d(id + 20, -10, y + 1.75, 0), lanelet::Point3d(id + 30, 100, y + 1.75, 0)}),
    lanelet::LineString3d(
      id + 40,
      {lanelet::Point3d(id + 50, -10, y - 1.75, 0), lanelet::Point3d(id + 60, 100, y - 1.75, 0)}));
}
class ObstacleLaneChange : public NormalLaneChange
{
public:
  ObstacleLaneChange()
  : NormalLaneChange(
      std::make_shared<LaneChangeParameters>(), LaneChangeModuleType::NORMAL, Direction::LEFT)
  {
    data = std::make_shared<PlannerData>();
    odometry = std::make_shared<Odometry>();
    odometry->pose.pose.orientation.w = 1.0;
    data->self_odometry = odometry;
    data->self_acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
    data->parameters = {};
    data->parameters.min_acc = -1.0;
    data->parameters.max_acc = 1.0;
    data->parameters.max_vel = 10.0;
    data->parameters.forward_path_length = 50.0;
    data->parameters.backward_path_length = 10.0;
    data->parameters.ego_nearest_dist_threshold = 3.0;
    data->parameters.ego_nearest_yaw_threshold = 1.57;
    data->parameters.vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
      0.3, 0.2, 2.0, 1.6, 1.0, 1.0, 0.2, 0.2, 1.5, 0.6);
    data->dynamic_object = std::make_shared<PredictedObjects>();
    data->route_handler = std::make_shared<RouteHandler>();
    lane_change_parameters_->trajectory.lat_acc_map.add(0.0, 0.3, 0.5);
    lane_change_parameters_->trajectory.lat_acc_map.add(10.0, 0.3, 0.5);
    lane_change_parameters_->trajectory.min_lane_changing_velocity = 3.0;
    lane_change_parameters_->safety.enable_target_lane_bound_check = false;
    lane_change_parameters_->delay.enable = false;
    lane_change_parameters_->time_limit = 2000.0;
    setData(data);
    const auto current = lane(1, 0.0);
    const auto target = lane(101, 3.5);
    common_data_ptr_->lanes_ptr->current = {current};
    common_data_ptr_->lanes_ptr->target_neighbor = {current};
    common_data_ptr_->lanes_ptr->target = {target};
    common_data_ptr_->lanes_polygon_ptr->current = current.polygon2d().basicPolygon();
    common_data_ptr_->lanes_polygon_ptr->target_neighbor = current.polygon2d().basicPolygon();
    common_data_ptr_->lanes_polygon_ptr->target = target.polygon2d().basicPolygon();
    common_data_ptr_->current_lanes_path = reference(0.0);
    common_data_ptr_->target_lanes_path = reference(3.5);
    common_data_ptr_->requested_target_lane_id = 101;
    auto & transient = common_data_ptr_->transient_data;
    transient.dist_to_terminal_end = 100.0;
    transient.dist_to_terminal_start = 80.0;
    transient.dist_to_target_end = 100.0;
    transient.current_path_velocity = 5.0;
    transient.lane_change_prepare_duration = 4.0;
    stop_time_ = 4.0;
  }
  void block(const double x, const double y, const double width = 0.5)
  {
    auto objects = std::make_shared<PredictedObjects>();
    PredictedObject object;
    object.kinematics.initial_pose_with_covariance.pose.position.x = x;
    object.kinematics.initial_pose_with_covariance.pose.position.y = y;
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 1.0;
    object.shape.dimensions.y = width;
    object.shape.dimensions.z = 1.0;
    objects->objects.push_back(object);
    data->dynamic_object = objects;
  }
  void approve(const LaneChangePath & path)
  {
    status_.lane_change_path = path;
    status_.is_valid_path = true;
    status_.is_safe = true;
    is_activated_ = true;
  }
  using NormalLaneChange::check_candidate_path_safety;
  using NormalLaneChange::isStaticObstaclePathSafe;
  auto common() { return common_data_ptr_; }
  auto params() { return lane_change_parameters_; }
  auto current_targets()
  {
    filtered_objects_.current_lane.emplace_back();
    lane_change_parameters_->safety.collision_check.check_current_lane = false;
    return get_target_objects(filtered_objects_, get_current_lanes());
  }
  std::shared_ptr<PlannerData> data;
  std::shared_ptr<Odometry> odometry;
};
class ObstacleAware : public ::testing::Test
{
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
}  // namespace

TEST_F(ObstacleAware, SamplesPreparationSpeedAndShorterShiftsBeforeStuck)
{
  ObstacleLaneChange module;
  const auto common = module.common();
  module.odometry->twist.twist.linear.x = 4.0;
  common->transient_data.distance_to_static_obstacle = 12.0;
  namespace calculation = utils::lane_change::calculation;
  const auto metrics = calculation::calc_prepare_phase_metrics(common, 4.0, 5.0, 0.0, 12.0);
  ASSERT_GT(metrics.size(), 3u);
  EXPECT_TRUE(
    std::any_of(metrics.begin(), metrics.end(), [](const auto & m) { return m.duration < 1.0; }));
  EXPECT_TRUE(std::any_of(
    metrics.begin(), metrics.end(), [](const auto & m) { return m.actual_lon_accel < -0.1; }));
  EXPECT_TRUE(std::any_of(metrics.begin(), metrics.end(), [](const auto & m) {
    return std::abs(m.actual_lon_accel) < 1e-5;
  }));
  for (const auto & metric : metrics) {
    EXPECT_LE(metric.length, 12.0);
    EXPECT_GE(metric.actual_lon_accel, -1.001);
    EXPECT_LE(metric.actual_lon_accel, 1.001);
  }
  const auto slow = calculation::calc_shift_phase_metrics(common, 3.5, 1.5, 5.0, 0.0, 50.0);
  const auto fast = calculation::calc_shift_phase_metrics(common, 3.5, 4.0, 5.0, 0.0, 50.0);
  ASSERT_FALSE(slow.empty());
  ASSERT_FALSE(fast.empty());
  EXPECT_LT(slow.front().length, fast.front().length);
  common->transient_data.is_ego_near_current_terminal_start = true;
  const auto after_deceleration =
    calculation::calc_shift_phase_metrics(common, 3.5, 1.5, 5.0, -1.0, 50.0);
  ASSERT_FALSE(after_deceleration.empty());
  for (const auto & m : after_deceleration) {
    EXPECT_DOUBLE_EQ(m.actual_lon_accel, 0.0);
    EXPECT_NEAR(m.length, 1.5 * m.duration, 1e-5);
  }
}

TEST_F(ObstacleAware, CannotJumpFromStandstillToMinimumSpeed)
{
  ObstacleLaneChange module;
  const auto common = module.common();
  common->transient_data.distance_to_static_obstacle = 10.0;
  const auto metrics =
    utils::lane_change::calculation::calc_prepare_phase_metrics(common, 0.0, 5.0, 0.0, 10.0);
  ASSERT_FALSE(metrics.empty());
  for (const auto & m : metrics) {
    EXPECT_GT(m.duration, 0.0);
    EXPECT_NEAR(m.velocity, m.actual_lon_accel * m.duration, 1e-5);
    EXPECT_LE(m.actual_lon_accel, 1.001);
  }
}

TEST_F(ObstacleAware, NormalLaneChangeChecksCurrentLaneWithNoTargetObjects)
{
  ObstacleLaneChange module;
  EXPECT_EQ(module.current_targets().leading.size(), 1u);
  LaneChangePath candidate;
  candidate.path = reference(0.0);
  candidate.info.lane_changing_end = candidate.path.points.at(30).point.pose;
  candidate.info.terminal_lane_changing_velocity = 2.0;
  module.block(10.0, 0.0);
  EXPECT_FALSE(module.check_candidate_path_safety(candidate, {{}, {}}));
}

TEST_F(ObstacleAware, AllowsMergeBehindDistantCarButReservesStoppingSpace)
{
  ObstacleLaneChange module;
  auto candidate = utils::lane_change::generate_low_speed_path(module.common(), 16.0, 1.5);
  ASSERT_TRUE(candidate);
  module.block(40.0, 3.5);
  EXPECT_TRUE(module.isStaticObstaclePathSafe(*candidate));
  module.block(22.0, 3.5);
  EXPECT_FALSE(module.isStaticObstaclePathSafe(*candidate));
}

TEST_F(ObstacleAware, LocalCollisionAloneKeepsApprovedCurve)
{
  ObstacleLaneChange module;
  const auto old = utils::lane_change::generate_low_speed_path(module.common(), 36.0, 1.5);
  ASSERT_TRUE(old);
  module.approve(*old);
  module.block(15.0, 0.0, 1.5);
  ASSERT_FALSE(module.isStaticObstaclePathSafe(*old));
  EXPECT_FALSE(module.updateApprovedPath());
  EXPECT_TRUE(module.isApprovedPathBlocked());
  EXPECT_EQ(module.getLaneChangePath().path, old->path);
  EXPECT_EQ(module.common()->requested_target_lane_id, 101);
}

TEST_F(ObstacleAware, TruncatedContinuationCannotProveStoppingSpace)
{
  ObstacleLaneChange module;
  auto candidate = utils::lane_change::generate_low_speed_path(module.common(), 16.0, 1.5);
  ASSERT_TRUE(candidate);
  ASSERT_TRUE(module.isStaticObstaclePathSafe(*candidate));
  candidate->path = candidate->shifted_path.path;
  EXPECT_FALSE(module.isStaticObstaclePathSafe(*candidate));
}

TEST_F(ObstacleAware, RecoveryRespectsUpstreamStopAndVehicleLimits)
{
  ObstacleLaneChange module;
  const auto old = utils::lane_change::generate_low_speed_path(module.common(), 36.0, 1.5);
  ASSERT_TRUE(old);
  module.approve(*old);
  module.block(15.0, 0.0, 1.5);
  module.common()->transient_data.current_path_velocity = 0.0;
  EXPECT_FALSE(module.updateApprovedPath());
  EXPECT_TRUE(module.isApprovedPathBlocked());
  EXPECT_EQ(module.getLaneChangePath().path, old->path);
  module.data->parameters.vehicle_info.max_steer_angle_rad = 0.01;
  EXPECT_FALSE(utils::lane_change::generate_low_speed_path(module.common(), 6.0, 1.0));
}

TEST_F(ObstacleAware, FailedOrMovingReplanKeepsApprovedPath)
{
  ObstacleLaneChange module;
  const auto old = utils::lane_change::generate_low_speed_path(module.common(), 36.0, 1.5);
  ASSERT_TRUE(old);
  module.approve(*old);
  module.block(15.0, 1.75, 8.0);
  EXPECT_FALSE(module.updateApprovedPath());
  EXPECT_TRUE(module.isApprovedPathBlocked());
  EXPECT_EQ(module.getLaneChangePath().path, old->path);
  module.odometry->twist.twist.linear.x = 1.0;
  EXPECT_FALSE(module.updateApprovedPath());
  EXPECT_EQ(module.getLaneChangePath().path, old->path);
  EXPECT_EQ(module.common()->requested_target_lane_id, 101);
}

TEST_F(ObstacleAware, ObstacleDistanceAloneDoesNotTranslateCurve)
{
  ObstacleLaneChange module;
  auto old = utils::lane_change::generate_low_speed_path(module.common(), 16.0, 1.5);
  ASSERT_TRUE(old);
  old->type = lane_change::PathType::ConstantJerk;
  module.approve(*old);
  module.common()->transient_data.distance_to_static_obstacle = 5.0;
  ASSERT_TRUE(module.isStaticObstaclePathSafe(*old));
  EXPECT_FALSE(module.updateApprovedPath());
  EXPECT_EQ(module.getLaneChangePath().path, old->path);
  EXPECT_FALSE(module.isApprovedPathBlocked());
}
}  // namespace autoware::behavior_path_planner
