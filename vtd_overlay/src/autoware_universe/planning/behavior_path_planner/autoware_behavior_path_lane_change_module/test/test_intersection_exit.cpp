// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#include "autoware/behavior_path_lane_change_module/utils/intersection_exit.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace autoware::behavior_path_planner::utils::lane_change
{
using autoware::route_handler::Direction;
namespace
{
lanelet::Lanelet road(lanelet::Id id, double begin, double end, double y)
{
  lanelet::Lanelet result(
    id,
    lanelet::LineString3d(
      id + 1,
      {lanelet::Point3d(id + 2, begin, y + 1.75, 0), lanelet::Point3d(id + 3, end, y + 1.75, 0)}),
    lanelet::LineString3d(
      id + 4,
      {lanelet::Point3d(id + 5, begin, y - 1.75, 0), lanelet::Point3d(id + 6, end, y - 1.75, 0)}));
  result.attributes()["subtype"] = "road";
  return result;
}
CommonDataPtr data(bool intersection = true, bool left = false)
{
  auto d = std::make_shared<behavior_path_planner::lane_change::CommonData>();
  auto connector = road(100, -20, 0, 0), outgoing = road(200, 0, 100, 0);
  if (intersection) connector.attributes()["turn_direction"] = "left";
  d->direction = left ? Direction::LEFT : Direction::RIGHT;
  d->lanes_ptr = std::make_shared<behavior_path_planner::lane_change::Lanes>();
  d->lanes_ptr->current = {connector, outgoing};
  d->lanes_ptr->target = {road(300, 0, 100, left ? 3.5 : -3.5)};
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->pose.pose.position.x = -4;
  odom->pose.pose.orientation.w = 1;
  d->self_odometry_ptr = odom;
  d->bpp_param_ptr = std::make_shared<BehaviorPathPlannerParameters>();
  d->bpp_param_ptr->vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
    0.3, 0.2, 2.0, 1.6, 1.0, 1.0, 0.2, 0.2, 1.5, 0.6);
  d->bpp_param_ptr->max_acc = 1.0;
  d->bpp_param_ptr->forward_path_length = 40;
  d->lc_param_ptr = std::make_shared<LaneChangeParameters>();
  d->lc_param_ptr->trajectory.target_lane_backward_overlap_length = 10;
  d->lc_param_ptr->trajectory.max_longitudinal_acc = 1;
  d->lc_param_ptr->trajectory.enable_lateral_acceleration_limit = false;
  d->lc_param_ptr->trajectory.enable_lateral_jerk_limit = false;
  d->lc_param_ptr->trajectory.lat_acc_map.add(0, 0.3, 0.7);
  d->lc_param_ptr->trajectory.lat_acc_map.add(10, 0.3, 0.7);
  d->current_lanes_path.header.frame_id = "map";
  d->target_lanes_path.header.frame_id = "map";
  for (double x = -20; x <= 100; x += .5) {
    PathPointWithLaneId p;
    p.point.pose.position.x = x;
    p.point.pose.orientation.w = 1;
    p.point.longitudinal_velocity_mps = 5;
    p.lane_ids = {x < 0 ? 100 : 200};
    d->current_lanes_path.points.push_back(p);
    if (x >= 0) {
      p.point.pose.position.y = left ? 3.5 : -3.5;
      p.lane_ids = {300};
      d->target_lanes_path.points.push_back(p);
    }
  }
  return d;
}
}  // namespace
TEST(IntersectionExit, AppliesOnlyToConnectedExitOnRequestedSide)
{
  for (bool left : {false, true}) {
    auto d = data(true, left);
    EXPECT_TRUE(is_intersection_exit_target(
      d->lanes_ptr->target.front(), d->lanes_ptr->current, d->direction));
    ASSERT_TRUE(intersection_exit_prepare_length(d));
    EXPECT_NEAR(*intersection_exit_prepare_length(d), 5.5, 1e-6);
    EXPECT_FALSE(is_intersection_exit_target(
      d->lanes_ptr->target.front(), d->lanes_ptr->current,
      left ? Direction::RIGHT : Direction::LEFT));
  }
  auto ordinary = data(false);
  EXPECT_FALSE(is_intersection_exit_target(
    ordinary->lanes_ptr->target.front(), ordinary->lanes_ptr->current, ordinary->direction));
  EXPECT_FALSE(intersection_exit_prepare_length(ordinary));
  auto disconnected = data();
  disconnected->lanes_ptr->target = {road(400, 2, 100, -3.5)};
  EXPECT_FALSE(intersection_exit_prepare_length(disconnected));
  auto oncoming = data();
  oncoming->lanes_ptr->target = {road(400, 0, -100, -3.5)};
  EXPECT_FALSE(intersection_exit_prepare_length(oncoming));
}
TEST(IntersectionExit, StopsApplyingAfterRearClearsAndDoesNotApplyToDistantIntersection)
{
  auto d = data();
  auto odom = std::make_shared<nav_msgs::msg::Odometry>(*d->self_odometry_ptr);
  d->self_odometry_ptr = odom;
  odom->pose.pose.position.x = 1.0;
  ASSERT_TRUE(intersection_exit_prepare_length(d));
  EXPECT_NEAR(*intersection_exit_prepare_length(d), .5, 1e-6);
  odom->pose.pose.position.x = 2.0;
  EXPECT_FALSE(intersection_exit_prepare_length(d));
  odom->pose.pose.position.x = -15.0;
  EXPECT_FALSE(intersection_exit_prepare_length(d));
  auto start = odom->pose.pose;
  start.position.x = 0;
  EXPECT_TRUE(starts_before_intersection_exit(d, start));
  start.position.x = 2;
  EXPECT_FALSE(starts_before_intersection_exit(d, start));
}
TEST(IntersectionExit, PreparationFollowsCurrentLaneBeforeLateralMotion)
{
  auto d = data();
  const auto candidate = generate_exit_lane_change_path(d, 6, 16, 1.5);
  ASSERT_TRUE(candidate);
  const auto & c = *candidate;
  EXPECT_EQ(c.path.points.front().point.pose, d->get_ego_pose());
  EXPECT_GT(c.info.shift_line.start_idx, 0u);
  EXPECT_GT(c.info.shift_line.end_idx, c.info.shift_line.start_idx);
  EXPECT_NEAR(c.info.length.prepare, 6, 1e-5);
  EXPECT_GT(c.info.duration.prepare, 4);
  EXPECT_NEAR(c.info.lane_changing_start.position.x, 2, 1e-5);
  EXPECT_FALSE(starts_before_intersection_exit(d, c.info.lane_changing_start));
  for (size_t i = 0; i <= c.info.shift_line.start_idx; ++i)
    EXPECT_NEAR(c.path.points[i].point.pose.position.y, 0, 1e-5);
  EXPECT_EQ(c.shifted_path.path.points.size(), c.info.shift_line.end_idx + 1);
  EXPECT_EQ(c.shifted_path.shift_length.size(), c.shifted_path.path.points.size());
  EXPECT_NEAR(c.info.lane_changing_end.position.y, -3.5, 1e-5);
}
TEST(IntersectionExit, CurvedApproachPreservesMeasuredPoseAndFollowsSourceCurve)
{
  auto d = data();
  for (auto & p : d->current_lanes_path.points) {
    double x = p.point.pose.position.x;
    p.point.pose.position.y = x * x / 120;
    p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(std::atan(x / 60));
  }
  for (auto & p : d->target_lanes_path.points) {
    double x = p.point.pose.position.x;
    p.point.pose.position.y = x * x / 120 - 3.5;
    p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(std::atan(x / 60));
  }
  auto odom = std::make_shared<nav_msgs::msg::Odometry>(*d->self_odometry_ptr);
  odom->pose.pose.position.y = 16.0 / 120 + .05;
  odom->pose.pose.orientation = autoware_utils::create_quaternion_from_yaw(std::atan(-4.0 / 60));
  d->self_odometry_ptr = odom;
  const auto c = generate_exit_lane_change_path(d, 8, 20, 1.0);
  ASSERT_TRUE(c);
  EXPECT_EQ(c->path.points.front().point.pose, d->get_ego_pose());
  for (size_t i = 0; i <= c->info.shift_line.start_idx; ++i) {
    const auto & p = c->path.points[i].point.pose.position;
    EXPECT_NEAR(p.y, p.x * p.x / 120, .06);
  }
}
TEST(IntersectionExit, RejectsInvalidOrUnavailablePreparationAndPreservesOldGenerator)
{
  auto d = data(false);
  EXPECT_FALSE(generate_exit_lane_change_path(d, 0, 16, 1.5));
  EXPECT_FALSE(generate_exit_lane_change_path(d, 200, 16, 1.5));
  EXPECT_FALSE(generate_exit_lane_change_path(d, 6, 16, 0));
  EXPECT_FALSE(
    generate_exit_lane_change_path(d, std::numeric_limits<double>::quiet_NaN(), 16, 1.5));
  const auto old = generate_low_speed_path(d, 30, 1.5);
  ASSERT_TRUE(old);
  EXPECT_DOUBLE_EQ(old->info.length.prepare, 0);
  EXPECT_EQ(old->info.lane_changing_start, d->get_ego_pose());
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
