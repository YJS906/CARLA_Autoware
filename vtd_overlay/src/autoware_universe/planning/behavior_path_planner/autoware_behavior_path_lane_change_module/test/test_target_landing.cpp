// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0

#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/target_landing.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace
{
const auto vehicle = autoware::vehicle_info_utils::createVehicleInfo(
  .383, .235, 2.944, 1.64, 1.0, 1.1, .128, .128, 2.5, .7);

lanelet::ConstLanelet road(
  const std::vector<std::pair<double, double>> & widths, const lanelet::Id id = 1,
  const double yaw = 0.0)
{
  lanelet::Points3d left, right;
  lanelet::Id point_id = id * 100;
  const auto point = [&](double x, double y) {
    return lanelet::Point3d(
      ++point_id, x * std::cos(yaw) - y * std::sin(yaw), x * std::sin(yaw) + y * std::cos(yaw),
      0.0);
  };
  for (const auto & [x, width] : widths) {
    left.push_back(point(x, width / 2.0));
    right.push_back(point(x, -width / 2.0));
  }
  return lanelet::Lanelet(
    id, lanelet::LineString3d(id * 100 + 90, left), lanelet::LineString3d(id * 100 + 91, right));
}

Pose landing(const double x = 10.0, const double yaw = 0.0)
{
  Pose pose;
  pose.position.x = x * std::cos(yaw);
  pose.position.y = x * std::sin(yaw);
  pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  return pose;
}

using utils::lane_change::find_narrow_target_landing;

TEST(TargetLanding, RejectsNarrowLandingRegardlessOfRoadDirection)
{
  for (const double yaw : {0.0, 1.3, -2.4}) {
    const auto target = road({{-20, 3.0}, {5, 1.0}, {20, .01}}, 1, yaw);
    const auto result = find_narrow_target_landing({target}, landing(10, yaw), vehicle);
    ASSERT_TRUE(result);
    EXPECT_LT(result->width, vehicle.vehicle_width_m - .1);
    EXPECT_EQ(result->lane_id, target.id());
  }
}

TEST(TargetLanding, DoesNotRejectForNarrowingBeyondLandingBody)
{
  const auto target = road({{-20, 3.0}, {25, 3.0}, {40, .01}});
  EXPECT_FALSE(find_narrow_target_landing({target}, landing(), vehicle));
}

TEST(TargetLanding, AllowsShortWideLaneWithoutSuccessor)
{
  const auto target = road({{8, 3.0}, {12, 3.0}});
  EXPECT_FALSE(find_narrow_target_landing({target}, landing(), vehicle));
}

TEST(TargetLanding, AllowsFiniteBoundarySeamsAndSmallGaps)
{
  const auto first = road({{-20, 3.0}, {10.4, 3.0}}, 1);
  const auto next = road({{10.6, 3.0}, {40, 3.0}}, 2);
  EXPECT_FALSE(find_narrow_target_landing({first, next}, landing(), vehicle));
}

TEST(TargetLanding, DoesNotTurnSmallMapDeficitsIntoNewRejections)
{
  EXPECT_FALSE(find_narrow_target_landing({road({{-20, 1.82}, {40, 1.82}})}, landing(), vehicle));
  EXPECT_FALSE(find_narrow_target_landing({road({{-20, 2.5}, {40, 2.2}})}, landing(), vehicle));
}

TEST(TargetLanding, RequiresMoreThanOneNarrowVertex)
{
  const auto target = road({{-20, 3.0}, {10.3, 3.0}, {10.4, .5}, {10.5, 3.0}, {40, 3.0}});
  EXPECT_FALSE(find_narrow_target_landing({target}, landing(), vehicle));
}

TEST(TargetLanding, UsesWideContinuationAtAnOverlappingJunction)
{
  const auto ending = road({{-20, 3.0}, {8, 1.0}, {14, .1}}, 1);
  const auto continuation = road({{8, 3.0}, {40, 3.0}}, 2);
  EXPECT_FALSE(find_narrow_target_landing({ending, continuation}, landing(), vehicle));
}

TEST(TargetLanding, AllowsAdequateCurvedTarget)
{
  lanelet::Points3d left, right;
  for (int i = 0; i <= 40; ++i) {
    const double angle = -.4 + i * .02;
    left.emplace_back(100 + i, 48.5 * std::cos(angle), 48.5 * std::sin(angle), 0);
    right.emplace_back(200 + i, 51.5 * std::cos(angle), 51.5 * std::sin(angle), 0);
  }
  const lanelet::Lanelet target(
    1, lanelet::LineString3d(10, left), lanelet::LineString3d(11, right));
  Pose pose;
  pose.position.x = 50;
  pose.orientation = autoware_utils::create_quaternion_from_yaw(std::acos(-1.0) / 2.0);
  EXPECT_FALSE(find_narrow_target_landing({target}, pose, vehicle));
}

class LandingModule : public NormalLaneChange
{
public:
  explicit LandingModule(const LaneChangeModuleType type)
  : NormalLaneChange(std::make_shared<LaneChangeParameters>(), type, Direction::RIGHT)
  {
    auto data = std::make_shared<PlannerData>();
    data->parameters.vehicle_info = vehicle;
    planner_data_ = data;
    common_data_ptr_->lanes_ptr = std::make_shared<lane_change::Lanes>();
    common_data_ptr_->lanes_ptr->target = {road({{-20, .5}, {40, .5}})};
  }

  bool rejected(const bool approved, const double speed)
  {
    is_activated_ = approved;
    LaneChangePath path;
    path.info.lane_changing_end = landing();
    path.info.velocity.prepare = speed;
    return hasNarrowAvoidanceLanding(path);
  }
};

TEST(TargetLanding, AppliesOnlyToNewAvoidanceAndCannotBeBypassedBySlowing)
{
  rclcpp::init(0, nullptr);
  {
    LandingModule avoidance(LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE);
    EXPECT_TRUE(avoidance.rejected(false, 0.0));
    EXPECT_TRUE(avoidance.rejected(false, 1.27));
    EXPECT_TRUE(avoidance.rejected(false, 10.0));
    EXPECT_FALSE(avoidance.rejected(true, 1.27));
    LandingModule normal(LaneChangeModuleType::NORMAL);
    EXPECT_FALSE(normal.rejected(false, 1.27));
  }
  rclcpp::shutdown();
}
}  // namespace
}  // namespace autoware::behavior_path_planner
