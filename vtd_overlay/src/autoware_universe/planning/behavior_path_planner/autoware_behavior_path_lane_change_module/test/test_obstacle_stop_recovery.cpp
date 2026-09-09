// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0

#include "obstacle_stop_recovery.hpp"

#include <autoware/rtc_interface/rtc_interface.hpp>

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <chrono>
#include <limits>
#include <memory>
#include <thread>

namespace autoware::behavior_path_planner
{
TEST(ObstacleStopRecoveryTimer, RequiresFiveContinuousSeconds)
{
  ObstacleStopRecoveryTimer timer;
  for (int i = 0; i < 50; ++i) EXPECT_FALSE(timer.update(i * 0.1, true, 5.0, 0.5));
  EXPECT_TRUE(timer.update(5.0, true, 5.0, 0.5));
}

TEST(ObstacleStopRecoveryTimer, PauseResetAndClockJump)
{
  ObstacleStopRecoveryTimer timer;
  EXPECT_FALSE(timer.update(10.0, true, 5.0, 0.5));
  for (int i = 0; i < 100; ++i) EXPECT_FALSE(timer.update(10.0, true, 5.0, 0.5));
  EXPECT_FALSE(timer.update(20.0, true, 5.0, 0.5));  // Missing updates.
  EXPECT_FALSE(timer.update(1.0, true, 5.0, 0.5));   // Clock reset.
  EXPECT_FALSE(timer.update(1.1, false, 5.0, 0.5));
  EXPECT_FALSE(timer.update(1.2, true, 5.0, 0.5));
  EXPECT_FALSE(timer.update(1.3, true, 0.0, 0.5));
  EXPECT_FALSE(timer.update(1.4, true, std::numeric_limits<double>::quiet_NaN(), 0.5));
}

class ObstacleStopRecoveryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("obstacle_stop_recovery_test");
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    setTime(100.0);
    ObstacleStopRecovery::declareParameters(*node);
    recovery = std::make_unique<ObstacleStopRecovery>(*node, "lane_change_left");
    node->set_parameter(rclcpp::Parameter("lane_change.obstacle_stop_recovery.enabled", true));
    node->set_parameter(rclcpp::Parameter("lane_change.obstacle_stop_recovery.duration", 0.1));
    rtc = std::make_unique<autoware::rtc_interface::RTCInterface>(
      node.get(), "lane_change_left", false);
    uuid.uuid[0] = 1;
    rtc->updateCooperateStatus(
      uuid, false, tier4_rtc_msgs::msg::State::WAITING_FOR_EXECUTION, 0.0, 10.0, node->now());
    publisher = node->create_publisher<ObstacleStopRecovery::Factors>(
      "/planning/planning_factors/obstacle_stop", rclcpp::QoS(1).reliable());
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (publisher->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < deadline) {
      spin();
    }
    ASSERT_GT(publisher->get_subscription_count(), 0U);
  }

  void TearDown() override
  {
    executor.remove_node(node);
    recovery.reset();
    rtc.reset();
    publisher.reset();
    node.reset();
  }

  void setTime(double seconds)
  {
    ASSERT_EQ(
      rcl_set_ros_time_override(
        node->get_clock()->get_clock_handle(), static_cast<int64_t>(seconds * 1e9)),
      RCL_RET_OK);
  }

  void spin()
  {
    for (int i = 0; i < 10; ++i) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  void tick(
    double now, bool valid = true, bool signal = false, double speed = 0.016, bool stop = true,
    bool deactivated = false, double stamp_age = 0.0)
  {
    setTime(now);
    ObstacleStopRecovery::Factors factors;
    factors.header.stamp = rclcpp::Time(static_cast<int64_t>((now - stamp_age) * 1e9));
    if (stop) {
      autoware_internal_planning_msgs::msg::PlanningFactor factor;
      factor.module = "obstacle_stop";
      factor.behavior = factor.STOP;
      autoware_internal_planning_msgs::msg::ControlPoint point;
      point.velocity = 0.0;
      point.distance = -0.1;
      factor.control_points.push_back(point);
      factors.factors.push_back(factor);
    }
    publisher->publish(factors);
    spin();
    recovery->update(true, valid, signal, speed, uuid, false, deactivated);
    spin();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<ObstacleStopRecovery> recovery;
  std::unique_ptr<autoware::rtc_interface::RTCInterface> rtc;
  rclcpp::Publisher<ObstacleStopRecovery::Factors>::SharedPtr publisher;
  ObstacleStopRecovery::UUID uuid;
};

TEST_F(ObstacleStopRecoveryTest, ActivatesUnsafeValidCandidateThroughActualRtc)
{
  EXPECT_FALSE(rtc->isActivated(uuid));
  tick(100.0);
  tick(100.05);
  EXPECT_FALSE(rtc->isActivated(uuid));
  tick(100.11);
  EXPECT_TRUE(rtc->isActivated(uuid));
  EXPECT_TRUE(rtc->isForceActivated(uuid));
}

TEST_F(ObstacleStopRecoveryTest, InvalidCandidateWaitsUntilValid)
{
  tick(100.0, false);
  tick(100.2, false);
  EXPECT_FALSE(rtc->isActivated(uuid));
  tick(100.3, true);
  EXPECT_TRUE(rtc->isActivated(uuid));
}

TEST_F(ObstacleStopRecoveryTest, SignalQueueResetsDuration)
{
  tick(100.0);
  tick(100.2, true, true);
  EXPECT_FALSE(rtc->isActivated(uuid));
  tick(100.3);
  EXPECT_FALSE(rtc->isActivated(uuid));
  tick(100.41);
  EXPECT_TRUE(rtc->isActivated(uuid));
}

TEST_F(ObstacleStopRecoveryTest, EmptyStopAndMessageGapResetDuration)
{
  tick(100.0);
  tick(100.2, true, false, 0.016, false);
  tick(100.3);
  EXPECT_FALSE(rtc->isActivated(uuid));
  tick(101.0);
  EXPECT_FALSE(rtc->isActivated(uuid));
  tick(101.11);
  EXPECT_TRUE(rtc->isActivated(uuid));
}

TEST_F(ObstacleStopRecoveryTest, DisableSwitchAndExplicitDeactivationBlockApproval)
{
  tick(100.0);
  node->set_parameter(rclcpp::Parameter("lane_change.obstacle_stop_recovery.enabled", false));
  tick(100.2);
  EXPECT_FALSE(rtc->isActivated(uuid));
  node->set_parameter(rclcpp::Parameter("lane_change.obstacle_stop_recovery.enabled", true));
  tick(100.3, true, false, 0.016, true, true);
  tick(100.5, true, false, 0.016, true, true);
  EXPECT_FALSE(rtc->isActivated(uuid));
}

TEST_F(ObstacleStopRecoveryTest, MovingEgoAndStaleFactorsDoNotApprove)
{
  tick(100.0, true, false, 1.0);
  tick(100.2, true, false, 1.0);
  EXPECT_FALSE(rtc->isActivated(uuid));
  tick(100.3, true, false, 0.016, true, false, 1.0);
  tick(100.5, true, false, 0.016, true, false, 1.0);
  EXPECT_FALSE(rtc->isActivated(uuid));
}
}  // namespace autoware::behavior_path_planner
