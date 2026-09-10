// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#include "interface.hpp"

#include <gtest/gtest.h>

namespace autoware::behavior_path_planner
{
namespace
{
class PreparationPath : public AvoidanceByLaneChange
{
public:
  PreparationPath()
  : AvoidanceByLaneChange(
      std::make_shared<LaneChangeParameters>(),
      std::make_shared<AvoidanceByLCParameters>(AvoidanceParameters{}))
  {
    direction_ = Direction::LEFT;
    auto data = std::make_shared<PlannerData>();
    auto odometry = std::make_shared<Odometry>();
    odometry->pose.pose.orientation.w = 1.0;
    data->self_odometry = odometry;
    data->self_acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
    data->route_handler = std::make_shared<RouteHandler>();
    data->parameters.max_vel = 15.0;
    setData(data);
    const lanelet::Lanelet lane(
      101,
      lanelet::LineString3d(102, {lanelet::Point3d(103, -10, 2, 0), lanelet::Point3d(104, 100, 2, 0)}),
      lanelet::LineString3d(105, {lanelet::Point3d(106, -10, -2, 0), lanelet::Point3d(107, 100, -2, 0)}));
    common_data_ptr_->lanes_ptr->current = {lane};
    common_data_ptr_->lanes_ptr->target_neighbor = {lane};
    common_data_ptr_->lanes_ptr->target = {lane};
    common_data_ptr_->transient_data.is_ego_near_current_terminal_start = true;
    common_data_ptr_->transient_data.dist_to_terminal_end = 100.0;
    for (const double x : {-10.0, 0.0, 100.0}) {
      PathPointWithLaneId point;
      point.point.pose.position.x = x;
      point.point.pose.orientation.w = 1.0;
      common_data_ptr_->current_lanes_path.points.push_back(point);
    }
    status_.is_valid_path = true;
    status_.is_safe = false;
    status_.lane_change_path.info.speed_preparation_target = 2.0;
  }
  bool specialRequiredCheck() const override { return true; }
  bool isExecutionDistanceSatisfied() const override { return within_window; }
  void safe() { status_.is_safe = true; }
  void no_preparation() { status_.lane_change_path.info.speed_preparation_target.reset(); }
  bool within_window{false};
};

class PreparationInterface : public AvoidanceByLaneChangeInterface
{
public:
  PreparationInterface(
    rclcpp::Node & node, const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> & interest)
  : AvoidanceByLaneChangeInterface(
      "avoidance_by_lane_change", node, std::make_shared<LaneChangeParameters>(),
      std::make_shared<AvoidanceByLCParameters>(AvoidanceParameters{}), rtc, interest,
      std::make_shared<PlanningFactorInterface>(&node, "avoidance_by_lane_change"))
  {
    module_type_ = std::make_unique<PreparationPath>();
  }
  PreparationPath & path() { return static_cast<PreparationPath &>(*module_type_); }
  void publish()
  {
    LaneChangeInterface::updateRTCStatus(
      4.0, 34.0, isExecutionReady(), State::WAITING_FOR_EXECUTION);
  }
  UUID uuid() { return uuid_map_.at("left"); }
};

class SpeedPreparationGate : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("speed_preparation_gate_test");
    for (const auto & side : {"left", "right"}) {
      rtc[side] = std::make_shared<RTCInterface>(
        node.get(), std::string("avoidance_by_lane_change_") + side, false);
    }
    module = std::make_unique<PreparationInterface>(*node, rtc, interest);
    module->onEntry();
  }
  void TearDown() override
  {
    module.reset();
    rtc.clear();
    node.reset();
    rclcpp::shutdown();
  }
  rclcpp::Node::SharedPtr node;
  std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc;
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> interest;
  std::unique_ptr<PreparationInterface> module;
};
}  // namespace

TEST_F(SpeedPreparationGate, BrakingRequestBeforeTwentyMetresDoesNotApproveLateralMotion)
{
  EXPECT_TRUE(module->isExecutionRequested());
  EXPECT_FALSE(module->isExecutionReady());
  EXPECT_FALSE(module->path().specialExpiredCheck());
  module->publish();
  EXPECT_FALSE(rtc.at("left")->isActivated(module->uuid()));
}

TEST_F(SpeedPreparationGate, UnsafeCandidateStillWaitsInsideTwentyMetres)
{
  module->path().within_window = true;
  EXPECT_TRUE(module->isExecutionRequested());
  EXPECT_FALSE(module->isExecutionReady());
  module->publish();
  EXPECT_FALSE(rtc.at("left")->isActivated(module->uuid()));
}

TEST_F(SpeedPreparationGate, SafeAfterBrakingMustStillEnterApprovalWindow)
{
  module->path().safe();
  EXPECT_TRUE(module->isExecutionRequested());
  EXPECT_FALSE(module->isExecutionReady());
  module->publish();
  EXPECT_FALSE(rtc.at("left")->isActivated(module->uuid()));
  module->path().within_window = true;
  EXPECT_TRUE(module->isExecutionReady());
  module->publish();
  EXPECT_TRUE(rtc.at("left")->isActivated(module->uuid()));
}

TEST_F(SpeedPreparationGate, DistantCandidateWithoutPreparationDoesNotRequestExecution)
{
  module->path().no_preparation();
  EXPECT_FALSE(module->isExecutionRequested());
}
}  // namespace autoware::behavior_path_planner
