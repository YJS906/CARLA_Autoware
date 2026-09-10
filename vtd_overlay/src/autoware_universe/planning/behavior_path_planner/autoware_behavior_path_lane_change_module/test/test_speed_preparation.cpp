// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace autoware::behavior_path_planner
{
namespace
{
PathWithLaneId reference()
{
  PathWithLaneId path;
  for (double x = -10.0; x <= 150.0; x += 0.5) {
    PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 10.0;
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
      {lanelet::Point3d(id + 20, -10, y + 1.75, 0), lanelet::Point3d(id + 30, 150, y + 1.75, 0)}),
    lanelet::LineString3d(
      id + 40,
      {lanelet::Point3d(id + 50, -10, y - 1.75, 0), lanelet::Point3d(id + 60, 150, y - 1.75, 0)}));
}

class SpeedPreparationModule : public NormalLaneChange
{
public:
  SpeedPreparationModule()
  : NormalLaneChange(
      std::make_shared<LaneChangeParameters>(), LaneChangeModuleType::NORMAL, Direction::LEFT)
  {
    data = std::make_shared<PlannerData>();
    odometry = std::make_shared<Odometry>();
    odometry->pose.pose.orientation.w = 1.0;
    odometry->twist.twist.linear.x = 10.0;
    data->self_odometry = odometry;
    data->self_acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
    data->parameters = {};
    data->parameters.min_acc = -5.0;
    data->parameters.max_acc = 2.0;
    data->parameters.max_vel = 15.0;
    data->parameters.forward_path_length = 150.0;
    data->parameters.backward_path_length = 10.0;
    data->parameters.ego_nearest_dist_threshold = 3.0;
    data->parameters.ego_nearest_yaw_threshold = 1.57;
    data->parameters.vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
      0.3, 0.2, 2.0, 1.6, 1.0, 1.0, 0.2, 0.2, 1.5, 0.6);
    data->dynamic_object = std::make_shared<PredictedObjects>();
    data->route_handler = std::make_shared<RouteHandler>();
    auto & p = *lane_change_parameters_;
    p.trajectory.lat_acc_map.add(0.0, 0.3, 0.5);
    p.trajectory.lat_acc_map.add(15.0, 0.3, 0.5);
    p.trajectory.min_lane_changing_velocity = 1.5;
    p.trajectory.min_longitudinal_acc = -5.0;
    p.trajectory.max_longitudinal_acc = 2.0;
    p.trajectory.min_prepare_duration = 0.2;
    p.trajectory.lateral_jerk = 1.0;
    p.trajectory_safety.min_jerk = -5.0;
    p.trajectory_safety.max_jerk = 2.0;
    p.safety.enable_target_lane_bound_check = false;
    p.safety.collision_check.prediction_time_resolution = 0.2;
    p.delay.enable = false;
    p.time_limit = 2000.0;
    setData(data);
    const auto current = lane(1, 0.0);
    const auto target = lane(101, 3.5);
    common_data_ptr_->lanes_ptr->current = {current};
    common_data_ptr_->lanes_ptr->target_neighbor = {current};
    common_data_ptr_->lanes_ptr->target = {target};
    common_data_ptr_->lanes_polygon_ptr->current = current.polygon2d().basicPolygon();
    common_data_ptr_->lanes_polygon_ptr->target_neighbor = current.polygon2d().basicPolygon();
    common_data_ptr_->lanes_polygon_ptr->target = target.polygon2d().basicPolygon();
    common_data_ptr_->current_lanes_path = reference();
    auto & transient = common_data_ptr_->transient_data;
    transient.dist_to_terminal_end = 150.0;
    transient.dist_to_terminal_start = 140.0;
    transient.dist_to_target_end = 150.0;
    transient.current_path_velocity = 10.0;
    transient.is_ego_stuck = true;
    prev_module_output_.path = reference();
  }
  void request(const double speed = 2.0)
  {
    status_.is_valid_path = true;
    status_.is_safe = false;
    status_.lane_change_path.info.speed_preparation_target = speed;
  }
  void change_target()
  {
    common_data_ptr_->lanes_ptr->target = {lane(201, 7.0)};
    status_.lane_change_path.info.speed_preparation_target.reset();
  }
  void upstream_stop(const double x)
  {
    for (auto & p : prev_module_output_.path.points) {
      if (p.point.pose.position.x >= x) p.point.longitudinal_velocity_mps = 0.0;
    }
  }
  auto common() { return common_data_ptr_; }
  auto targets() { return get_target_objects(filtered_objects_, get_current_lanes()); }
  auto profile_start() { return speed_preparation_start_; }
  using NormalLaneChange::adapt_candidate_speed;
  using NormalLaneChange::braking_wait_output;
  using NormalLaneChange::check_candidate_path_safety;
  using NormalLaneChange::curvature_speed_limit;
  std::shared_ptr<PlannerData> data;
  std::shared_ptr<Odometry> odometry;
};

class SpeedPreparation : public ::testing::Test
{
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

LaneChangePath curved_candidate()
{
  LaneChangePath candidate;
  candidate.path = reference();
  for (auto & point : candidate.path.points) {
    const double x = point.point.pose.position.x;
    const double u = std::clamp((x - 35.0) / 30.0, 0.0, 1.0);
    point.point.pose.position.y =
      3.5 * (10 * std::pow(u, 3) - 15 * std::pow(u, 4) + 6 * std::pow(u, 5));
    const double slope = 3.5 / 30.0 * (30 * u * u - 60 * std::pow(u, 3) + 30 * std::pow(u, 4));
    point.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(std::atan(slope));
    if (x == 35.0) candidate.info.lane_changing_start = point.point.pose;
    if (x == 65.0) candidate.info.lane_changing_end = point.point.pose;
  }
  candidate.shifted_path.path = candidate.path;
  candidate.info.duration = {3.5, 3.0};
  candidate.info.velocity = {10.0, 10.0};
  candidate.info.terminal_lane_changing_velocity = 10.0;
  return candidate;
}
}  // namespace

TEST_F(SpeedPreparation, UnsafeLateralCandidateStillBrakesInCurrentLane)
{
  SpeedPreparationModule module;
  module.request();
  const auto output = module.braking_wait_output();
  ASSERT_TRUE(module.hasSpeedPreparationRequest());
  EXPECT_FALSE(module.isSafe());
  EXPECT_FLOAT_EQ(output.path.points.back().point.longitudinal_velocity_mps, 2.0);
  for (const auto & p : output.path.points) EXPECT_DOUBLE_EQ(p.point.pose.position.y, 0.0);
  EXPECT_GT(output.path.points[20].point.longitudinal_velocity_mps, 2.0);
}

TEST_F(SpeedPreparation, BrakingLatencyDoesNotRecedeWithEgo)
{
  SpeedPreparationModule module;
  module.request();
  module.braking_wait_output();
  EXPECT_DOUBLE_EQ(module.profile_start().position.x, 0.0);
  module.odometry->pose.pose.position.x = 1.0;
  const auto output = module.braking_wait_output();
  EXPECT_DOUBLE_EQ(module.profile_start().position.x, 0.0);
  EXPECT_FLOAT_EQ(output.path.points.back().point.longitudinal_velocity_mps, 2.0);
}

TEST_F(SpeedPreparation, InsufficientRoomRequestsStopInsteadOfOriginalSpeed)
{
  SpeedPreparationModule module;
  module.request();
  module.common()->transient_data.distance_to_static_obstacle = 3.0;
  const auto output = module.braking_wait_output();
  for (const auto & p : output.path.points) EXPECT_FLOAT_EQ(p.point.longitudinal_velocity_mps, 0.0);
  EXPECT_FALSE(module.isSafe());
}

TEST_F(SpeedPreparation, UpstreamStopIsNeverRemoved)
{
  SpeedPreparationModule module;
  module.request();
  module.upstream_stop(8.0);
  const auto output = module.braking_wait_output();
  for (const auto & p : output.path.points) EXPECT_FLOAT_EQ(p.point.longitudinal_velocity_mps, 0.0);
}

TEST_F(SpeedPreparation, ChangingTargetClearsPreparation)
{
  SpeedPreparationModule module;
  module.request();
  ASSERT_TRUE(module.hasSpeedPreparationRequest());
  module.change_target();
  EXPECT_FALSE(module.hasSpeedPreparationRequest());
}

TEST_F(SpeedPreparation, SpeedDerivedTargetCanBeBelowCruiseFloor)
{
  SpeedPreparationModule module;
  namespace calc = utils::lane_change::calculation;
  EXPECT_TRUE(calc::calc_shift_phase_metrics(module.common(), 3.5, 0.3, 10.0, 0.0, 100.0).empty());
  const auto adapted =
    calc::calc_shift_phase_metrics(module.common(), 3.5, 0.3, 10.0, 0.0, 100.0, true);
  ASSERT_FALSE(adapted.empty());
  for (const auto & m : adapted) EXPECT_NEAR(m.length, 0.3 * m.duration, 1e-5);
}

TEST_F(SpeedPreparation, RetimingPreservesGeometryAndPassesMeasuredSpeedPrediction)
{
  SpeedPreparationModule module;
  auto candidate = curved_candidate();
  const auto original = candidate.path;
  EXPECT_FALSE(module.check_candidate_path_safety(candidate, module.targets()));
  ASSERT_TRUE(module.adapt_candidate_speed(candidate, module.targets()));
  EXPECT_TRUE(module.check_candidate_path_safety(candidate, module.targets()));
  ASSERT_TRUE(candidate.info.braking_profile);
  EXPECT_DOUBLE_EQ(candidate.info.braking_profile->initial_velocity, 10.0);
  EXPECT_LT(candidate.info.braking_profile->target_velocity, 10.0);
  ASSERT_EQ(candidate.path.points.size(), original.points.size());
  for (size_t i = 0; i < original.points.size(); ++i) {
    EXPECT_EQ(candidate.path.points[i].point.pose, original.points[i].point.pose);
  }
}

TEST_F(SpeedPreparation, OccupiedGeometryDoesNotBecomeSpeedPreparation)
{
  SpeedPreparationModule module;
  auto objects = std::make_shared<PredictedObjects>();
  PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 45.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 1.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;
  objects->objects.push_back(object);
  module.data->dynamic_object = objects;
  auto candidate = curved_candidate();
  EXPECT_FALSE(module.adapt_candidate_speed(candidate, module.targets()));
  EXPECT_FALSE(candidate.info.speed_preparation_target);
}

TEST_F(SpeedPreparation, TooFastToMergeNowStillCreatesLongitudinalPreparation)
{
  SpeedPreparationModule module;
  module.odometry->twist.twist.linear.x = 30.0;
  auto candidate = curved_candidate();
  EXPECT_FALSE(module.adapt_candidate_speed(candidate, module.targets()));
  ASSERT_TRUE(candidate.info.speed_preparation_target);
  EXPECT_GT(*candidate.info.speed_preparation_target, 0.0);
  EXPECT_LT(*candidate.info.speed_preparation_target, 10.0);
  EXPECT_FALSE(candidate.info.braking_profile);  // Not certified as executable at measured speed.
}
}  // namespace autoware::behavior_path_planner
