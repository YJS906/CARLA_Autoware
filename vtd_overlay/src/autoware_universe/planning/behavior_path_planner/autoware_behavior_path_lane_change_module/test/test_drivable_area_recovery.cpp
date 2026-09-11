// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#include "drivable_area_recovery.hpp"

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <chrono>
#include <thread>

namespace autoware::behavior_path_planner
{
class BoundaryFeedback : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("boundary_recovery_test");
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    DrivableAreaRecovery::declareParameters(*node);
    node->set_parameter(rclcpp::Parameter("lane_change.drivable_area_recovery.enabled", true));
    recovery = std::make_unique<DrivableAreaRecovery>(*node);
    publisher = node->create_publisher<DrivableAreaRecovery::Factors>(
      "/planning/planning_factors/path_optimizer", rclcpp::QoS(1).reliable());
    resets = node->create_subscription<std_msgs::msg::Header>(
      "/planning/path_optimizer/reset_previous", 1,
      [this](const std_msgs::msg::Header &) { ++reset_count; });
    ego.orientation.w = 1.0;
    path.header.frame_id = "map";
    for (int i = -5; i < 50; ++i) {
      autoware_internal_planning_msgs::msg::PathPointWithLaneId p;
      p.point.pose.orientation.w = 1.0;
      p.point.pose.position.x = i;
      p.point.longitudinal_velocity_mps = 1.5;
      path.points.push_back(p);
    }
    executor.add_node(node);
    for (int i = 0; i < 500 && publisher->get_subscription_count() == 0; ++i) spin();
    ASSERT_GT(publisher->get_subscription_count(), 0U);
  }
  void TearDown() override
  {
    executor.remove_node(node);
    recovery.reset();
  }
  void spin()
  {
    for (int i = 0; i < 3; ++i) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  bool tick(
    double now, bool eligible = true, double speed = 0.0, const char * source = "path_optimizer",
    double age = 0.0, double distance = 0.0)
  {
    EXPECT_EQ(
      rcl_set_ros_time_override(
        node->get_clock()->get_clock_handle(), static_cast<int64_t>(now * 1e9)),
      RCL_RET_OK);
    DrivableAreaRecovery::Factors message;
    message.header.frame_id = "map";
    message.header.stamp = rclcpp::Time(static_cast<int64_t>((now - age) * 1e9));
    autoware_internal_planning_msgs::msg::PlanningFactor factor;
    factor.module = source;
    factor.behavior = factor.STOP;
    factor.detail = "outside_drivable_area";
    factor.is_driving_forward = true;
    autoware_internal_planning_msgs::msg::ControlPoint point;
    point.pose = ego;
    point.distance = distance;
    factor.control_points.push_back(point);
    message.factors.push_back(factor);
    publisher->publish(message);
    spin();
    return recovery->shouldReplan(eligible, speed, ego, path);
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<DrivableAreaRecovery> recovery;
  rclcpp::Publisher<DrivableAreaRecovery::Factors>::SharedPtr publisher;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr resets;
  DrivableAreaRecovery::Path path;
  geometry_msgs::msg::Pose ego;
  int reset_count{0};
};

TEST_F(BoundaryFeedback, TenSecondsThenRateLimitedReplanWithoutResetUntilReplacement)
{
  for (int i = 0; i < 40; ++i) EXPECT_FALSE(tick(100.0 + i * 0.25));
  EXPECT_TRUE(tick(110.0));
  EXPECT_FALSE(tick(110.25));
  EXPECT_FALSE(tick(110.5));
  EXPECT_FALSE(tick(110.75));
  EXPECT_TRUE(tick(111.0));
  EXPECT_EQ(reset_count, 0);
  recovery->onReplanned();
  spin();
  EXPECT_EQ(reset_count, 1);
  EXPECT_FALSE(tick(111.0));  // Queued feedback from the replaced path.
  for (int i = 1; i <= 39; ++i) EXPECT_FALSE(tick(111.0 + i * 0.25));
  EXPECT_FALSE(tick(121.0));
  EXPECT_TRUE(tick(121.25));
}
TEST_F(BoundaryFeedback, OtherStopsManualMotionAndStaleOrDistantFeedbackCannotTrigger)
{
  for (int i = 0; i < 44; ++i) {
    EXPECT_FALSE(tick(100.0 + i * .25, true, 0.0, "traffic_light"));
  }
  for (int i = 0; i < 44; ++i) EXPECT_FALSE(tick(111.0 + i * .25, false));
  for (int i = 0; i < 44; ++i) EXPECT_FALSE(tick(122.0 + i * .25, true, 0.2));
  for (int i = 0; i < 44; ++i) EXPECT_FALSE(tick(133.0 + i * .25, true, 0, "path_optimizer", 1.0));
  for (int i = 0; i < 44; ++i)
    EXPECT_FALSE(tick(144.0 + i * .25, true, 0, "path_optimizer", 0, 10.0));
  EXPECT_EQ(reset_count, 0);
}
TEST_F(BoundaryFeedback, MissingStreamClockJumpAndModuleExitRestartTheWait)
{
  for (int i = 0; i < 36; ++i) EXPECT_FALSE(tick(100.0 + i * .25));
  EXPECT_FALSE(tick(112.0));
  EXPECT_FALSE(tick(10.0));
  for (int i = 1; i < 36; ++i) EXPECT_FALSE(tick(10.0 + i * .25));
  recovery->reset();
  EXPECT_FALSE(tick(19.0));
  path.header.frame_id = "another_map";
  for (int i = 1; i <= 44; ++i) EXPECT_FALSE(tick(19.0 + i * .25));
}
TEST_F(BoundaryFeedback, MatchesThePathSegmentBetweenSparsePoints)
{
  path.points = {path.points.front(), path.points[10]};
  for (int i = 0; i < 40; ++i) EXPECT_FALSE(tick(100.0 + i * .25));
  EXPECT_TRUE(tick(110.0));
}
}  // namespace autoware::behavior_path_planner
