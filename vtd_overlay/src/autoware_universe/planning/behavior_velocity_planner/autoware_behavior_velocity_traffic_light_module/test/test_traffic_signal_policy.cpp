// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../src/scene.hpp"

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <algorithm>
#include <memory>

namespace autoware::behavior_velocity_planner
{
class TrafficSignalPolicyTest : public ::testing::Test
{
protected:
  using Element = autoware_perception_msgs::msg::TrafficLightElement;
  using Group = autoware_perception_msgs::msg::TrafficLightGroup;

  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ASSERT_EQ(rcl_enable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
    setTime(100.0);
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {{"wheel_radius", 0.3},
       {"wheel_width", 0.2},
       {"wheel_base", 2.7},
       {"wheel_tread", 1.5},
       {"front_overhang", 1.0},
       {"rear_overhang", 1.0},
       {"left_overhang", 0.5},
       {"right_overhang", 0.5},
       {"vehicle_height", 1.5},
       {"max_steer_angle", 0.5},
       {"max_accel", 1.0},
       {"min_accel", -1.0},
       {"max_jerk", 1.0},
       {"min_jerk", -1.0},
       {"system_delay", 0.0},
       {"delay_response_time", 0.0},
       {"max_stop_acceleration_threshold", -2.0},
       {"max_stop_jerk_threshold", -1.0}});
    node_ = std::make_shared<rclcpp::Node>("traffic_signal_policy_test", options);
    planner_data_ = std::make_shared<PlannerData>(*node_);
    planner_data_->current_acceleration =
      std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
    factors_ = std::make_shared<planning_factor_interface::PlanningFactorInterface>(
      node_.get(), "traffic_policy_test");
    params_.stop_margin = 1.0;
    params_.tl_state_timeout = 1.0;
    params_.stop_time_hysteresis = 0.1;
    params_.yellow_lamp_period = 2.75;
    params_.yellow_light_stop_velocity = 1.0;
    params_.enable_pass_judge = true;
    params_.min_behind_dist_to_stop_for_restart_suppression = 0.5;
    params_.max_behind_dist_to_stop_for_restart_suppression = 1.0;
    stop_x_ = 20.0 - params_.stop_margin - planner_data_->vehicle_info_.max_longitudinal_offset_m;
    const auto line = [](int64_t id, double x0, double y0, double x1, double y1) {
      return lanelet::LineString3d(
        id, {lanelet::Point3d(lanelet::utils::getId(), x0, y0, 0),
             lanelet::Point3d(lanelet::utils::getId(), x1, y1, 0)});
    };
    stop_line_ = line(200, 20, -2, 20, 2);
    light_ = lanelet::TrafficLight::make(12345, {}, {line(300, 20, 2, 20, 3)}, stop_line_);
    lane_ = lanelet::Lanelet(100, line(101, 0, 2, 50, 2), line(102, 0, -2, 50, -2));
    resetModule();
  }

  void resetModule()
  {
    module_ = std::make_shared<TrafficLightModule>(
      100, *light_, lane_, stop_line_, false, false, params_,
      rclcpp::get_logger("traffic_policy_test"), clock_, nullptr, factors_);
    module_->setPlannerData(planner_data_);
    module_->setRTCEnabled(false);
  }

  void setTime(double now)
  {
    ASSERT_EQ(
      rcl_set_ros_time_override(clock_->get_clock_handle(), static_cast<int64_t>(now * 1e9)),
      RCL_RET_OK);
  }

  Group signal(uint8_t color, uint8_t status = Element::SOLID_ON)
  {
    Group group;
    group.traffic_light_group_id = 12345;
    Element element;
    element.color = color;
    element.shape = color == Element::UNKNOWN ? Element::UNKNOWN : Element::CIRCLE;
    element.status = color == Element::UNKNOWN ? Element::UNKNOWN : status;
    element.confidence = color == Element::UNKNOWN ? 0.0 : 1.0;
    group.elements.push_back(element);
    return group;
  }

  PathWithLaneId plan(double now, double speed = 0.0, double distance = 0.05)
  {
    setTime(now);
    auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose->pose.position.x = stop_x_ - distance;
    pose->pose.orientation.w = 1.0;
    planner_data_->current_odometry = pose;
    auto twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
    twist->header.stamp = clock_->now();
    twist->twist.linear.x = speed;
    planner_data_->current_velocity = twist;
    planner_data_->velocity_buffer = {*twist};
    PathWithLaneId path;
    for (int i = 0; i <= 50; ++i) {
      autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
      point.point.pose.position.x = i;
      point.point.pose.orientation.w = 1.0;
      point.point.longitudinal_velocity_mps = 10.0;
      point.lane_ids = {100};
      path.points.push_back(point);
      auto boundary = point.point.pose.position;
      boundary.y = 2.0;
      path.left_bound.push_back(boundary);
      boundary.y = -2.0;
      path.right_bound.push_back(boundary);
    }
    EXPECT_TRUE(module_->modifyPathVelocity(&path));
    return path;
  }

  PathWithLaneId step(const Group & group, double now, double speed = 0.0, double distance = 0.05)
  {
    setTime(now);
    TrafficSignalStamped stamped;
    stamped.stamp = clock_->now();
    stamped.signal = group;
    planner_data_->traffic_light_id_map_raw_[12345] = stamped;
    return plan(now, speed, distance);
  }

  bool stops(const PathWithLaneId & path)
  {
    return std::any_of(path.points.begin(), path.points.end(), [](const auto & point) {
      return point.point.longitudinal_velocity_mps == 0.0;
    });
  }

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<PlannerData> planner_data_;
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> factors_;
  std::shared_ptr<TrafficLightModule> module_;
  TrafficLightModule::PlannerParam params_{};
  lanelet::TrafficLight::Ptr light_;
  lanelet::Lanelet lane_;
  lanelet::LineString3d stop_line_;
  double stop_x_{};
};

TEST_F(TrafficSignalPolicyTest, FreshUnknownPassesAndReleasesPreviousRed)
{
  const auto red = signal(Element::RED);
  step(red, 100.0);
  EXPECT_TRUE(stops(step(red, 100.2)));
  EXPECT_FALSE(stops(step(signal(Element::UNKNOWN), 100.3)));
  EXPECT_TRUE(module_->isSafe());
}

TEST_F(TrafficSignalPolicyTest, MissingAndStaleUnknownRemainSeparateFromFreshUnknown)
{
  EXPECT_FALSE(stops(step(signal(Element::UNKNOWN), 100.0)));
  plan(101.1);
  EXPECT_TRUE(stops(plan(101.3)));
  EXPECT_FALSE(stops(step(signal(Element::UNKNOWN), 101.4)));
  planner_data_->traffic_light_id_map_raw_.clear();
  plan(101.5);
  EXPECT_TRUE(stops(plan(101.7)));
}

TEST_F(TrafficSignalPolicyTest, MixedRedAndUnknownStillStops)
{
  auto mixed = signal(Element::RED);
  mixed.elements.push_back(signal(Element::UNKNOWN).elements.front());
  step(mixed, 100.0);
  EXPECT_TRUE(stops(step(mixed, 100.2)));
}

TEST_F(TrafficSignalPolicyTest, FlashingStopsOnceThenContinuesEvenAfterUnknown)
{
  const auto flash = signal(Element::AMBER, Element::FLASHING);
  EXPECT_TRUE(stops(step(flash, 100.0)));
  EXPECT_TRUE(stops(step(flash, 100.5)));
  EXPECT_FALSE(stops(step(flash, 101.0)));
  EXPECT_FALSE(stops(step(flash, 101.2, 2.0)));
  EXPECT_FALSE(stops(step(signal(Element::UNKNOWN), 101.3, 2.0)));
  EXPECT_FALSE(stops(step(flash, 101.4, 0.0)));
}

TEST_F(TrafficSignalPolicyTest, FlashingDoesNotUseYellowRollingPass)
{
  EXPECT_TRUE(stops(step(signal(Element::AMBER, Element::FLASHING), 100.0, 8.0, 0.5)));
  EXPECT_FALSE(module_->isSafe());
}

TEST_F(TrafficSignalPolicyTest, QueueStopFarFromLineDoesNotCount)
{
  const auto flash = signal(Element::AMBER, Element::FLASHING);
  for (int i = 0; i <= 4; ++i) EXPECT_TRUE(stops(step(flash, 100.0 + i * 0.5, 0.0, 5.0)));
  EXPECT_TRUE(stops(step(flash, 102.5)));
  EXPECT_TRUE(stops(step(flash, 103.0)));
  EXPECT_FALSE(stops(step(flash, 103.5)));
}

TEST_F(TrafficSignalPolicyTest, CreepingAndInterruptedStopsDoNotComplete)
{
  const auto flash = signal(Element::AMBER, Element::FLASHING);
  for (int i = 0; i <= 4; ++i) EXPECT_TRUE(stops(step(flash, 100.0 + i * 0.5, 0.02)));
  EXPECT_TRUE(stops(step(flash, 102.5)));
  EXPECT_TRUE(stops(step(flash, 103.0, 0.02)));
  EXPECT_TRUE(stops(step(flash, 103.5)));
  EXPECT_TRUE(stops(step(flash, 104.0)));
  EXPECT_FALSE(stops(step(flash, 104.5)));
}

TEST_F(TrafficSignalPolicyTest, RestartSuppressionReleasesAfterFlashingStop)
{
  const auto flash = signal(Element::AMBER, Element::FLASHING);
  EXPECT_TRUE(stops(step(flash, 100.0, 0.0, 0.75)));
  EXPECT_TRUE(stops(step(flash, 100.5, 0.0, 0.75)));
  EXPECT_FALSE(stops(step(flash, 101.0, 0.0, 0.75)));
}

TEST_F(TrafficSignalPolicyTest, SolidRedAndSolidAmberStillStopAfterCompletedFlashing)
{
  const auto flash = signal(Element::AMBER, Element::FLASHING);
  step(flash, 100.0);
  step(flash, 100.5);
  EXPECT_FALSE(stops(step(flash, 101.0)));
  step(signal(Element::RED), 101.1);
  EXPECT_TRUE(stops(step(signal(Element::RED), 101.3)));
  EXPECT_TRUE(stops(step(signal(Element::AMBER), 101.4)));
}

TEST_F(TrafficSignalPolicyTest, MixedFlashingAndRedDoesNotRelease)
{
  auto mixed = signal(Element::AMBER, Element::FLASHING);
  mixed.elements.push_back(signal(Element::RED).elements.front());
  step(mixed, 100.0);
  for (int i = 1; i <= 4; ++i) EXPECT_TRUE(stops(step(mixed, 100.0 + i * 0.5)));
}

TEST_F(TrafficSignalPolicyTest, NewSignalAndRepeatedApproachRequireAnotherStop)
{
  const auto flash = signal(Element::AMBER, Element::FLASHING);
  step(flash, 100.0);
  step(flash, 100.5);
  EXPECT_FALSE(stops(step(flash, 101.0)));
  step(flash, 101.1, 2.0, -3.0);  // crossed the deadline
  step(flash, 101.2, 0.0, 5.0);   // re-approach the same light
  EXPECT_TRUE(stops(step(flash, 101.3)));
  resetModule();
  EXPECT_TRUE(stops(step(flash, 101.4)));
}

TEST_F(TrafficSignalPolicyTest, AdjacentStopLineUpdatePreservesCompletedStop)
{
  const auto flash = signal(Element::AMBER, Element::FLASHING);
  step(flash, 100.0);
  step(flash, 100.5);
  EXPECT_FALSE(stops(step(flash, 101.0)));
  auto adjacent = lanelet::LineString3d(201, {stop_line_.front(), stop_line_.back()});
  module_->updateStopLine(adjacent);
  EXPECT_FALSE(stops(step(flash, 101.2)));
}

TEST_F(TrafficSignalPolicyTest, ClockJumpAndObservationGapDoNotCompleteDwell)
{
  const auto flash = signal(Element::AMBER, Element::FLASHING);
  step(flash, 100.0);
  EXPECT_TRUE(stops(step(flash, 102.0)));
  EXPECT_TRUE(stops(step(flash, 102.5)));
  EXPECT_FALSE(stops(step(flash, 103.0)));
  EXPECT_TRUE(stops(step(flash, 90.0)));
}

TEST_F(TrafficSignalPolicyTest, ExistingZeroVelocityFromOtherModulesIsPreserved)
{
  step(signal(Element::UNKNOWN), 100.0);
  auto path = plan(100.1);
  path.points[10].point.longitudinal_velocity_mps = 0.0;
  EXPECT_TRUE(module_->modifyPathVelocity(&path));
  EXPECT_DOUBLE_EQ(path.points[10].point.longitudinal_velocity_mps, 0.0);
}
}  // namespace autoware::behavior_velocity_planner
