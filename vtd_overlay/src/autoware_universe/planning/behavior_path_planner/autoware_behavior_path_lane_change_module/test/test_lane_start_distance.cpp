// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#include <autoware/behavior_path_lane_change_module/utils/calculation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <gtest/gtest.h>

namespace autoware::behavior_path_planner::utils::lane_change::calculation
{
namespace
{
lanelet::Lanelet lane(const std::vector<std::pair<double, double>> & points)
{
  lanelet::Points3d center, left, right;
  lanelet::Id id = 10;
  for (const auto & [x, y] : points) {
    center.emplace_back(id++, x, y, 0.0);
    left.emplace_back(id++, x, y + 1.75, 0.0);
    right.emplace_back(id++, x, y - 1.75, 0.0);
  }
  lanelet::Lanelet result(1, lanelet::LineString3d(2, left), lanelet::LineString3d(3, right));
  result.setCenterline(lanelet::LineString3d(4, center));
  return result;
}
Pose pose(double x, double y = 0.0)
{
  Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation.w = 1.0;
  return p;
}
}  // namespace

TEST(LaneStartDistance, PreservesAheadAndBehindOnBothSides)
{
  const auto current = lane({{-100, 0}, {100, 0}});
  for (double side : {-3.5, 3.5}) {
    EXPECT_NEAR(calc_distance_to_lane_start({current}, pose(10), pose(40, side).position), 30, 1e-6);
    EXPECT_NEAR(calc_distance_to_lane_start({current}, pose(10), pose(-80, side).position), -90, 1e-6);
    EXPECT_NEAR(calc_distance_to_lane_start({current}, pose(10), pose(10, side).position), 0, 1e-6);
  }
}

TEST(LaneStartDistance, LongLaneStartMustNotProjectOntoFutureRouteLeg)
{
  const auto current = lane({{0, 0}, {100, 0}, {100, 100}, {0, 100}, {0, 10}});
  const auto ego = pose(80);
  const auto target = pose(0, -3.5).position;
  PathWithLaneId clipped;
  for (const auto & p : {pose(70), pose(100), pose(100, 100), pose(0, 100), pose(0, 10)}) {
    PathPointWithLaneId point;
    point.point.pose = p;
    clipped.points.push_back(point);
  }
  // Reproduce the former positive start distance on the truncated route.
  EXPECT_GT(motion_utils::calcSignedArcLength(clipped.points, ego.position, target), 200.0);
  EXPECT_NEAR(calc_distance_to_lane_start({current}, ego, target), -80, 1e-6);
}

TEST(LaneStartDistance, NormalLaneChangeUsesCompleteLaneGeometry)
{
  auto d = std::make_shared<behavior_path_planner::lane_change::CommonData>();
  d->route_handler_ptr = std::make_shared<autoware::route_handler::RouteHandler>();
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->pose.pose = pose(80);
  d->self_odometry_ptr = odom;
  const auto current = lane({{-100, 0}, {100, 0}});
  for (const bool left : {true, false}) {
    d->direction = left ? autoware::route_handler::Direction::LEFT
                        : autoware::route_handler::Direction::RIGHT;
    const double y = left ? 3.5 : -3.5;
    const auto target = lane({{-100, y}, {100, y}});
    EXPECT_NEAR(calc_ego_dist_to_lanes_start(d, {current}, {target}), -180, 1e-6);
    const auto future = lane({{95, y}, {200, y}});
    EXPECT_NEAR(calc_ego_dist_to_lanes_start(d, {current}, {future}), 15, 1e-6);
  }
}

TEST(LaneStartDistance, MissingLanesRemainUnavailable)
{
  EXPECT_EQ(calc_distance_to_lane_start({}, pose(0), pose(10).position),
            std::numeric_limits<double>::max());
}
}  // namespace autoware::behavior_path_planner::utils::lane_change::calculation
