// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "scene.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{
// Stub only lane/path generation: exercise the real bilateral selection, stationary-object
// safety check and winning-side restoration. No ROS publishers or simulator connection.
class TestAvoidanceByLaneChange : public AvoidanceByLaneChange
{
public:
  TestAvoidanceByLaneChange()
  : AvoidanceByLaneChange(
      std::make_shared<LaneChangeParameters>(),
      std::make_shared<AvoidanceByLCParameters>(AvoidanceParameters{}))
  {
    data = std::make_shared<PlannerData>();
    auto odometry = std::make_shared<Odometry>();
    odometry->pose.pose.orientation.w = 1.0;
    data->self_odometry = odometry;
    data->parameters.min_acc = -1.0;
    data->parameters.vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
      0.3, 0.2, 2.0, 1.6, 1.0, 1.0, 0.2, 0.2, 1.5, 0.6);
    data->route_handler = std::make_shared<RouteHandler>();
    data->route_handler->setMap(autoware::test_utils::makeMapBinMsg());
    data->route_handler->setRoute(autoware::test_utils::makeBehaviorNormalRoute());
    data->dynamic_object = std::make_shared<PredictedObjects>();
    setData(data);
  }

  void prepare(const std::vector<Direction> & directions)
  {
    target_lane_candidates_.clear();
    for (const auto direction : directions) {
      target_lane_candidates_.push_back({direction, direction == Direction::LEFT ? 1 : 2, 0, 1});
    }
  }

  void block(const Direction direction)
  {
    auto objects = std::make_shared<PredictedObjects>(*data->dynamic_object);
    PredictedObject object;
    object.kinematics.initial_pose_with_covariance.pose.position.x = 10.0;
    object.kinematics.initial_pose_with_covariance.pose.position.y =
      direction == Direction::LEFT ? 3.2 : -3.2;
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    object.shape.type = Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 4.0;
    object.shape.dimensions.y = 2.0;
    object.shape.dimensions.z = 1.5;
    objects->objects.push_back(object);
    data->dynamic_object = objects;
  }

  bool selectTargetLane(const TargetLaneCandidate & candidate) override
  {
    direction_ = candidate.direction;
    common_data_ptr_->direction = direction_;
    common_data_ptr_->requested_target_lane_id = candidate.lane_id;
    const lanelet::Lanelet lane(
      *common_data_ptr_->requested_target_lane_id, lanelet::LineString3d{},
      lanelet::LineString3d{});
    common_data_ptr_->lanes_ptr->target = {lane};
    return true;
  }
  bool specialRequiredCheck() const override { return true; }

  std::pair<bool, bool> getSafePath(LaneChangePath & path) const override
  {
    checked.push_back(direction_);
    if (invalid == direction_) return {false, false};
    for (const double x : {0.0, 10.0, 20.0, 100.0}) {
      PathPointWithLaneId point;
      point.point.pose.position.x = x;
      point.point.pose.position.y = direction_ == Direction::LEFT ? 3.2 : -3.2;
      point.point.pose.orientation.w = 1.0;
      path.path.points.push_back(point);
    }
    path.info.lane_changing_end = path.path.points.at(2).point.pose;
    return {true, true};
  }

  Direction selectedDirection() const { return common_data_ptr_->direction; }
  lanelet::Id selectedLane() const { return *common_data_ptr_->requested_target_lane_id; }
  void approve() { is_activated_ = true; }
  mutable std::vector<Direction> checked;
  Direction invalid{Direction::NONE};
  std::shared_ptr<PlannerData> data;
};

class BilateralSelection : public ::testing::Test
{
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(BilateralSelection, RightObstacleSelectsSafeLeft)
{
  TestAvoidanceByLaneChange module;
  module.prepare({Direction::RIGHT, Direction::LEFT});
  module.block(Direction::RIGHT);
  module.updateLaneChangeStatus();
  EXPECT_EQ(module.checked.size(), 2u);
  EXPECT_TRUE(module.isSafe());
  EXPECT_EQ(module.selectedDirection(), Direction::LEFT);
  EXPECT_EQ(module.selectedLane(), 1);
}

TEST_F(BilateralSelection, StopsSearchingAfterFirstSafeRouteCandidate)
{
  TestAvoidanceByLaneChange module;
  module.prepare({Direction::LEFT, Direction::RIGHT});
  module.updateLaneChangeStatus();
  EXPECT_EQ(module.checked.size(), 1u);
  EXPECT_TRUE(module.isSafe());
  EXPECT_EQ(module.selectedDirection(), Direction::LEFT);
  EXPECT_EQ(module.selectedLane(), 1);
}

TEST_F(BilateralSelection, NeitherSideSafeCannotBecomeReady)
{
  TestAvoidanceByLaneChange module;
  module.prepare({Direction::RIGHT, Direction::LEFT});
  module.block(Direction::RIGHT);
  module.block(Direction::LEFT);
  module.updateLaneChangeStatus();
  EXPECT_EQ(module.checked.size(), 2u);
  EXPECT_TRUE(module.getLaneChangeStatus().is_valid_path);
  EXPECT_FALSE(module.isSafe());
}

TEST_F(BilateralSelection, InvalidPreferredCandidateDoesNotHideOppositeSide)
{
  TestAvoidanceByLaneChange module;
  module.prepare({Direction::RIGHT, Direction::LEFT});
  module.invalid = Direction::RIGHT;
  module.updateLaneChangeStatus();
  EXPECT_EQ(module.checked.size(), 2u);
  EXPECT_TRUE(module.isSafe());
  EXPECT_EQ(module.selectedDirection(), Direction::LEFT);
}

TEST_F(BilateralSelection, ApprovedManeuverDoesNotSwitchSide)
{
  TestAvoidanceByLaneChange module;
  module.prepare({Direction::LEFT, Direction::RIGHT});
  module.updateLaneChangeStatus();
  module.approve();
  module.checked.clear();
  module.block(Direction::LEFT);
  module.updateLaneChangeStatus();
  EXPECT_TRUE(module.checked.empty());
  EXPECT_EQ(module.selectedDirection(), Direction::LEFT);
}

TEST_F(BilateralSelection, NoDirectionExpiresEvenIfDistanceIsEligible)
{
  TestAvoidanceByLaneChange module;
  ASSERT_TRUE(module.specialRequiredCheck());
  EXPECT_TRUE(module.specialExpiredCheck());
}
}  // namespace autoware::behavior_path_planner
