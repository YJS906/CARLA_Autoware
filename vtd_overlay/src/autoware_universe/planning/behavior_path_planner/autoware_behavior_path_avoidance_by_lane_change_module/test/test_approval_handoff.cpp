// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "interface.hpp"

#include <autoware/behavior_path_planner/planner_manager.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
namespace
{
using RTCMap = std::unordered_map<std::string, std::shared_ptr<RTCInterface>>;
using InterestMap =
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>;

BehaviorModuleOutput makeOutput(const double lateral)
{
  BehaviorModuleOutput output;
  for (const double x : {0.0, 10.0, 40.0}) {
    PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.position.y = lateral;
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 3.0;
    output.path.points.push_back(point);
  }
  output.reference_path = output.path;
  return output;
}

class ApprovalPath : public AvoidanceByLaneChange
{
public:
  ApprovalPath()
  : AvoidanceByLaneChange(
      std::make_shared<LaneChangeParameters>(),
      std::make_shared<AvoidanceByLCParameters>(AvoidanceParameters{}))
  {
    choose(Direction::LEFT);
  }
  void choose(const Direction direction) { direction_ = direction; }
  // This fixture stubs maneuver geometry, including an in-range triggering obstacle.
  bool isExecutionDistanceSatisfied() const override { return true; }
};

// Stub geometry and maneuver completion, NOT the production entry hook, RTC dispatch,
// SceneModuleInterface::run/updateCurrentState or SubPlannerManager arbitration.
class ApprovalInterface : public AvoidanceByLaneChangeInterface
{
public:
  ApprovalInterface(
    rclcpp::Node & node, const RTCMap & rtc, InterestMap & interest,
    const std::shared_ptr<PlanningFactorInterface> & factors)
  : AvoidanceByLaneChangeInterface(
      "avoidance_by_lane_change", node, std::make_shared<LaneChangeParameters>(),
      std::make_shared<AvoidanceByLCParameters>(AvoidanceParameters{}), rtc, interest, factors)
  {
    module_type_ = std::make_unique<ApprovalPath>();
  }

  bool isExecutionRequested() const override { return safe; }
  bool isExecutionReady() const override { return safe; }
  void updateData() override {}
  void postProcess() override {}
  void choose(const Direction direction)
  {
    static_cast<ApprovalPath &>(*module_type_).choose(direction);
  }
  UUID uuid(const std::string & side) const { return uuid_map_.at(side); }
  void publishRTC(const bool value, const uint8_t state)
  {
    // The same FOUR-argument call used by real LaneChangeInterface planning/abort code.
    LaneChangeInterface::updateRTCStatus(4.0, 34.0, value, state);
  }
  BehaviorModuleOutput planWaitingApproval() override
  {
    ++waiting_calls;
    publishRTC(safe, State::WAITING_FOR_EXECUTION);
    return makeOutput(0.0);
  }
  BehaviorModuleOutput plan() override
  {
    ++execution_calls;
    publishRTC(safe, State::RUNNING);
    return makeOutput(module_type_->getDirection() == Direction::LEFT ? 3.2 : -3.2);
  }
  BehaviorModuleOutput cycle()
  {
    const auto output = run();
    updateCurrentState();
    return output;
  }
  bool safe{true};
  int waiting_calls{0};
  int execution_calls{0};

protected:
  bool canTransitSuccessState() override { return false; }
  bool canTransitFailureState() override { return false; }
};

class ApprovalManager : public SceneModuleManagerInterface
{
public:
  ApprovalManager() : SceneModuleManagerInterface("avoidance_by_lane_change") {}
  void init(rclcpp::Node * node) override { initInterface(node, {"left", "right"}); }
  void updateModuleParams(const std::vector<rclcpp::Parameter> &) override {}
  ApprovalInterface * created{nullptr};

protected:
  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override
  {
    auto result = std::make_unique<ApprovalInterface>(
      *node_, rtc_interface_ptr_map_, objects_of_interest_marker_interface_ptr_map_,
      planning_factor_interface_);
    created = result.get();
    return result;
  }
};

class ApprovalHandoff : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("approval_handoff_test");
  }
  void TearDown() override
  {
    module.reset();
    rtc.clear();
    node.reset();
    rclcpp::shutdown();
  }
  void create(const bool manual = false)
  {
    for (const auto & side : {"left", "right"}) {
      rtc[side] = std::make_shared<RTCInterface>(
        node.get(), std::string("avoidance_by_lane_change_") + side, manual);
    }
    auto factors =
      std::make_shared<PlanningFactorInterface>(node.get(), "avoidance_by_lane_change");
    module = std::make_unique<ApprovalInterface>(*node, rtc, interest, factors);
    module->onEntry();
  }
  void checkManager(const bool manual)
  {
    node->declare_parameter("enable_all_modules_auto_mode", !manual);
    node->declare_parameter("avoidance_by_lane_change.enable_rtc", manual);
    node->declare_parameter(
      "avoidance_by_lane_change.enable_simultaneous_execution_as_approved_module", false);
    node->declare_parameter(
      "avoidance_by_lane_change.enable_simultaneous_execution_as_candidate_module", false);
    auto manager = std::make_shared<ApprovalManager>();
    manager->init(node.get());
    auto data = std::make_shared<PlannerData>();
    manager->setData(data);
    std::unordered_map<std::string, double> timing{{"avoidance_by_lane_change", 0.0}};
    ModuleUpdateInfo debug;
    SubPlannerManager slot(std::make_shared<std::optional<lanelet::ConstLanelet>>(), timing, debug);
    slot.addSceneModuleManager(manager);
    SlotOutput input;
    input.valid_output = makeOutput(0.0);
    const auto first = slot.propagateFull(data, input);
    EXPECT_DOUBLE_EQ(first.valid_output.path.points.front().point.pose.position.y, 0.0);
    const auto second = slot.propagateFull(data, input);
    EXPECT_DOUBLE_EQ(
      second.valid_output.path.points.front().point.pose.position.y, manual ? 0.0 : 3.2);
    EXPECT_EQ(slot.approved_modules().size(), manual ? 0u : 1u);
    EXPECT_EQ(slot.candidate_modules().size(), manual ? 1u : 0u);
    ASSERT_NE(manager->created, nullptr);
    EXPECT_EQ(manager->created->isWaitingApproval(), manual);
    EXPECT_EQ(manager->created->execution_calls, manual ? 0 : 1);
  }
  rclcpp::Node::SharedPtr node;
  RTCMap rtc;
  InterestMap interest;
  std::unique_ptr<ApprovalInterface> module;
};

TEST_F(ApprovalHandoff, AutoApprovalReachesLeftExecutionWithoutLatch)
{
  create();
  ASSERT_TRUE(module->isWaitingApproval());
  module->onEntry();  // registration may call entry again after idle evaluation
  EXPECT_DOUBLE_EQ(module->cycle().path.points.front().point.pose.position.y, 0.0);
  EXPECT_EQ(module->getCurrentStatus(), ModuleStatus::RUNNING);
  EXPECT_FALSE(module->isWaitingApproval());
  EXPECT_DOUBLE_EQ(module->cycle().path.points.front().point.pose.position.y, 3.2);
  EXPECT_EQ(module->execution_calls, 1);
}

TEST_F(ApprovalHandoff, AutoApprovalReachesRightExecution)
{
  create();
  module->choose(Direction::RIGHT);
  module->cycle();
  EXPECT_DOUBLE_EQ(module->cycle().path.points.front().point.pose.position.y, -3.2);
  EXPECT_FALSE(rtc.at("left")->isRegistered(module->uuid("left")));
}

TEST_F(ApprovalHandoff, ManualSafeCandidateStillWaits)
{
  create(true);
  for (int i = 0; i < 3; ++i) module->cycle();
  EXPECT_EQ(module->getCurrentStatus(), ModuleStatus::WAITING_APPROVAL);
  EXPECT_TRUE(module->isWaitingApproval());
  EXPECT_EQ(module->execution_calls, 0);
}

TEST_F(ApprovalHandoff, UnsafeCandidateWaitsThenCanBecomeSafe)
{
  create();
  module->safe = false;
  for (int i = 0; i < 3; ++i) module->cycle();
  EXPECT_TRUE(module->isWaitingApproval());
  EXPECT_EQ(module->execution_calls, 0);
  module->safe = true;
  module->cycle();
  EXPECT_FALSE(module->isWaitingApproval());
  EXPECT_DOUBLE_EQ(module->cycle().path.points.front().point.pose.position.y, 3.2);
}

TEST_F(ApprovalHandoff, FourArgumentRTCRegistersOnlySelectedSide)
{
  create();
  module->publishRTC(true, State::WAITING_FOR_EXECUTION);
  EXPECT_TRUE(rtc.at("left")->isRegistered(module->uuid("left")));
  EXPECT_TRUE(rtc.at("left")->isActivated(module->uuid("left")));
  EXPECT_FALSE(rtc.at("right")->isRegistered(module->uuid("right")));
}

TEST_F(ApprovalHandoff, WaitingDirectionChangeRemovesOldRTC)
{
  create();
  module->publishRTC(true, State::WAITING_FOR_EXECUTION);
  module->choose(Direction::RIGHT);
  module->publishRTC(true, State::WAITING_FOR_EXECUTION);
  EXPECT_FALSE(rtc.at("left")->isRegistered(module->uuid("left")));
  EXPECT_TRUE(rtc.at("right")->isRegistered(module->uuid("right")));
}

TEST_F(ApprovalHandoff, MissingDirectionDoesNotRegisterRightByDefault)
{
  create();
  module->publishRTC(true, State::WAITING_FOR_EXECUTION);
  module->choose(Direction::NONE);
  module->publishRTC(false, State::WAITING_FOR_EXECUTION);
  EXPECT_FALSE(rtc.at("left")->isRegistered(module->uuid("left")));
  EXPECT_FALSE(rtc.at("right")->isRegistered(module->uuid("right")));
}

TEST_F(ApprovalHandoff, ExplicitUnsafeAndAbortStateArePreserved)
{
  create();
  std::optional<tier4_rtc_msgs::msg::CooperateStatusArray> received;
  const auto subscription = node->create_subscription<tier4_rtc_msgs::msg::CooperateStatusArray>(
    "/planning/cooperate_status/avoidance_by_lane_change_left", 1,
    [&](const tier4_rtc_msgs::msg::CooperateStatusArray & msg) { received = msg; });
  // Follow legal RTC transitions: a new request cannot begin in ABORTING.
  module->publishRTC(true, State::WAITING_FOR_EXECUTION);
  module->publishRTC(true, State::RUNNING);
  module->publishRTC(false, State::ABORTING);
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received && std::chrono::steady_clock::now() < end) {
    rtc.at("left")->publishCooperateStatus(node->now());
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(received.has_value());
  ASSERT_EQ(received->statuses.size(), 1u);
  EXPECT_FALSE(received->statuses.front().safe);
  EXPECT_EQ(received->statuses.front().state.type, State::ABORTING);
  EXPECT_FALSE(rtc.at("right")->isRegistered(module->uuid("right")));
}

TEST_F(ApprovalHandoff, ManagerPromotesAutoCandidateAndOutputsLeftPath)
{
  checkManager(false);
}

TEST_F(ApprovalHandoff, ManagerRetainsManualCandidateWithoutExecuting)
{
  checkManager(true);
}
}  // namespace
}  // namespace autoware::behavior_path_planner
