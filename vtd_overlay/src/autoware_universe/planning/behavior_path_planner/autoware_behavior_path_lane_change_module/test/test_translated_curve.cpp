// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0

#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"

#include <autoware/behavior_path_lane_change_module/utils/utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <cmath>
#include <limits>
#include <memory>

namespace autoware::behavior_path_planner
{
namespace
{
PathWithLaneId reference(double y)
{
  PathWithLaneId path;
  for (double x = -10.0; x <= 110.0; x += 0.5) {
    PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.position.y = y;
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 5.0;
    path.points.push_back(point);
  }
  return path;
}

class ReplanModule : public NormalLaneChange
{
public:
  ReplanModule()
  : NormalLaneChange(
      std::make_shared<LaneChangeParameters>(), LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE,
      Direction::LEFT)
  {
    data = std::make_shared<PlannerData>();
    odometry = std::make_shared<Odometry>();
    odometry->pose.pose.orientation.w = 1.0;
    data->self_odometry = odometry;
    data->self_acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
    data->dynamic_object = std::make_shared<PredictedObjects>();
    auto & bpp = data->parameters;
    bpp = {};
    bpp.vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
      .383, .235, 2.944, 1.64, 1.0, 1.1, .128, .128, 2.5, .7);
    bpp.vehicle_width = bpp.vehicle_info.vehicle_width_m;
    bpp.min_acc = -5.0;
    bpp.max_acc = .7;
    bpp.max_vel = 10.0;
    bpp.forward_path_length = 100.0;
    bpp.backward_path_length = 5.0;
    bpp.ego_nearest_dist_threshold = 3.0;
    bpp.ego_nearest_yaw_threshold = 1.57;

    lanelet::LineStrings3d bounds;
    auto map = std::make_shared<lanelet::LaneletMap>();
    for (int i = 0; i < 4; ++i) {
      const double y = -1.6 + 3.2 * i;
      lanelet::LineString3d bound(
        100 + i,
        {lanelet::Point3d(200 + 2 * i, -10, y, 0), lanelet::Point3d(201 + 2 * i, 110, y, 0)});
      bound.attributes()["type"] = "line_thin";
      bound.attributes()["subtype"] = "dashed";
      bound.attributes()["lane_change"] = "yes";
      bounds.push_back(bound);
    }
    for (int i = 0; i < 3; ++i) {
      lanelet::Lanelet lane(i + 1, bounds[i + 1], bounds[i]);
      lane.attributes()["subtype"] = "road";
      lane.attributes()["location"] = "urban";
      lane.attributes()["one_way"] = "yes";
      lane.attributes()["speed_limit"] = "30";
      map->add(lane);
    }
    data->route_handler->setMap(autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
    autoware_planning_msgs::msg::LaneletRoute route;
    route.start_pose = odometry->pose.pose;
    route.goal_pose = odometry->pose.pose;
    route.goal_pose.position.x = 100.0;
    route.goal_pose.position.y = 6.4;
    route.segments.resize(1);
    for (int i = 1; i <= 3; ++i) {
      autoware_planning_msgs::msg::LaneletPrimitive primitive;
      primitive.id = i;
      primitive.primitive_type = "lane";
      route.segments.front().primitives.push_back(primitive);
    }
    route.segments.front().preferred_primitive = route.segments.front().primitives.back();
    data->route_handler->setRoute(route);

    auto & p = *lane_change_parameters_;
    p.trajectory.lat_acc_map.add(0.0, .3, .5);
    p.trajectory.lat_acc_map.add(10.0, .3, .5);
    p.trajectory.enable_lateral_acceleration_limit = false;
    p.trajectory.enable_lateral_jerk_limit = false;
    p.trajectory.min_longitudinal_acc = -5.0;
    p.trajectory.max_longitudinal_acc = .7;
    p.trajectory.min_lane_changing_velocity = 1.5;
    p.trajectory_safety.lateral_margin = .1;
    p.trajectory_safety.optimization_margin = 0.0;
    p.trajectory_safety.stop_margin = 4.5;
    p.trajectory_safety.max_jerk = 1.4;
    p.trajectory_safety.min_jerk = -3.5;
    p.safety.collision_check.prediction_time_resolution = .2;
    p.safety.enable_target_lane_bound_check = false;
    p.delay.enable = false;
    p.th_stop_time = 3.0;
    p.th_stop_velocity = .5;
    p.stopped_replan_velocity = 1.5;
    p.time_limit = 2000.0;
    setData(data);
    const auto current = data->route_handler->getLaneletsFromId(1);
    const auto target = data->route_handler->getLaneletsFromId(3);
    auto & c = *common_data_ptr_;
    c.lanes_ptr->current = {current};
    c.lanes_ptr->target_neighbor = {current};
    c.lanes_ptr->target = {target};
    c.lanes_polygon_ptr->current = current.polygon2d().basicPolygon();
    c.lanes_polygon_ptr->target_neighbor = current.polygon2d().basicPolygon();
    c.lanes_polygon_ptr->target = target.polygon2d().basicPolygon();
    c.current_lanes_path = reference(0.0);
    c.target_lanes_path = reference(6.4);
    c.requested_target_lane_id = 3;
    c.transient_data.dist_to_terminal_end = 100.0;
    c.transient_data.dist_to_terminal_start = 80.0;
    c.transient_data.dist_to_target_end = 100.0;
    c.transient_data.current_path_velocity = 5.0;
    stop_time_ = 4.0;
  }

  void block(double x, double y, double width = 1.808, double length = 4.394)
  {
    auto objects = std::make_shared<PredictedObjects>(*data->dynamic_object);
    PredictedObject object;
    object.object_id.uuid.front() = objects->objects.size() + 1;
    object.kinematics.initial_pose_with_covariance.pose.position.x = x;
    object.kinematics.initial_pose_with_covariance.pose.position.y = y;
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = length;
    object.shape.dimensions.y = width;
    object.shape.dimensions.z = 1.5;
    objects->objects.push_back(object);
    data->dynamic_object = objects;
  }
  void approve(const LaneChangePath & path)
  {
    status_.lane_change_path = path;
    status_.is_valid_path = true;
    status_.is_safe = true;
    is_activated_ = true;
    setObstacleStopActive(true);
  }
  auto common() { return common_data_ptr_; }
  auto params() { return lane_change_parameters_; }
  auto cursor() const { return stopped_curve_cursor_; }
  void allow_next_replan() { last_replan_time_.reset(); }
  using NormalLaneChange::isStaticObstaclePathSafe;
  using NormalLaneChange::is_colliding;
  using NormalLaneChange::get_prepare_metrics;
  using NormalLaneChange::make_braking_prepare_metric;
  std::shared_ptr<PlannerData> data;
  std::shared_ptr<Odometry> odometry;
};

class TranslatedCurve : public ::testing::Test
{
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TranslatedCurve, CopiesEveryCurvePoseWithOneTranslation)
{
  ReplanModule module;
  const auto original = utils::lane_change::generate_low_speed_path(module.common(), 24, 1.5);
  ASSERT_TRUE(original);
  const auto shifted =
    utils::lane_change::translate_approved_curve(module.common(), *original, 8, 1.5);
  ASSERT_TRUE(shifted);
  const size_t start = shifted->info.shift_line.start_idx;
  const size_t finish = shifted->info.shift_line.end_idx;
  EXPECT_NEAR(shifted->info.length.lane_changing, original->info.length.lane_changing, 1e-8);
  ASSERT_EQ(finish - start, original->info.shift_line.end_idx);
  for (size_t i = 0; i <= finish - start; ++i) {
    const auto & a = original->path.points[i].point.pose;
    const auto & b = shifted->path.points[start + i].point.pose;
    EXPECT_NEAR(b.position.x - a.position.x, 8.0, 1e-10);
    EXPECT_DOUBLE_EQ(b.position.y, a.position.y);
    EXPECT_EQ(b.orientation, a.orientation);
  }
  EXPECT_EQ(shifted->path.points.front().point.pose, module.odometry->pose.pose);
  const auto prediction =
    utils::lane_change::convert_to_predicted_paths(module.common(), *shifted, 1);
  ASSERT_EQ(prediction.size(), 1u);
  EXPECT_NEAR(
    autoware_utils::calc_distance2d(
      prediction.front().back().pose, shifted->info.lane_changing_end),
    0.0, 0.01);
}

TEST_F(TranslatedCurve, TranslationFollowsSourceLaneInsteadOfAlreadyTurningSample)
{
  ReplanModule module;
  auto original = utils::lane_change::generate_low_speed_path(module.common(), 24, 1.5);
  ASSERT_TRUE(original);
  original->info.lane_changing_start.orientation = autoware_utils::create_quaternion_from_yaw(0.05);
  const auto shifted =
    utils::lane_change::translate_approved_curve(module.common(), *original, 8, 1.5);
  ASSERT_TRUE(shifted);
  const size_t start = shifted->info.shift_line.start_idx;
  for (size_t i = 0; i <= original->info.shift_line.end_idx; ++i) {
    const auto & a = original->path.points[i].point.pose;
    const auto & b = shifted->path.points[start + i].point.pose;
    EXPECT_NEAR(b.position.x - a.position.x, 8.0, 1e-10);
    EXPECT_NEAR(b.position.y - a.position.y, 0.0, 1e-10);
    EXPECT_EQ(b.orientation, a.orientation);
  }
}

TEST_F(TranslatedCurve, KeepsEveryFutureCurvePointAfterPartialEntry)
{
  ReplanModule module;
  const auto original = utils::lane_change::generate_low_speed_path(module.common(), 24, 1.5);
  ASSERT_TRUE(original);
  module.odometry->pose.pose.position.x = 4.0;
  module.odometry->pose.pose.position.y = 0.2;
  module.odometry->pose.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.08);
  const auto shifted =
    utils::lane_change::translate_approved_curve(module.common(), *original, 2, 1.5);
  ASSERT_TRUE(shifted);
  EXPECT_EQ(shifted->path.points.front().point.pose, module.odometry->pose.pose);
  auto source_start = shifted->info.lane_changing_start.position;
  source_start.x -= 2.0;
  const size_t begin = motion_utils::findNearestIndex(original->path.points, source_start);
  EXPECT_GT(begin, 0u);
  EXPECT_LT(begin, original->info.shift_line.end_idx);
  const size_t count = original->info.shift_line.end_idx - begin;
  EXPECT_EQ(shifted->info.shift_line.end_idx - shifted->info.shift_line.start_idx, count);
  for (size_t i = 0; i <= count; ++i) {
    const auto & a = original->path.points[begin + i].point.pose;
    const auto & b = shifted->path.points[shifted->info.shift_line.start_idx + i].point.pose;
    EXPECT_NEAR(b.position.x - a.position.x, 2.0, 1e-10);
    EXPECT_DOUBLE_EQ(b.position.y, a.position.y);
    EXPECT_EQ(b.orientation, a.orientation);
  }
  EXPECT_NEAR(
    shifted->info.length.lane_changing,
    motion_utils::calcSignedArcLength(
      original->path.points, begin, original->info.shift_line.end_idx),
    1e-8);
}

TEST_F(TranslatedCurve, ClearsSideCarWithoutShorteningApprovedCurve)
{
  ReplanModule module;
  const auto original = utils::lane_change::generate_low_speed_path(module.common(), 20, 1.5);
  ASSERT_TRUE(original);
  module.block(7, 3.2);
  ASSERT_FALSE(module.isStaticObstaclePathSafe(*original));
  module.approve(*original);
  ASSERT_TRUE(module.updateApprovedPath());
  const auto shifted = module.getLaneChangePath();
  EXPECT_NEAR(shifted.info.length.lane_changing, original->info.length.lane_changing, 1e-8);
  EXPECT_GT(shifted.info.lane_changing_start.position.x, 0.0);
  EXPECT_LE(shifted.info.lane_changing_start.position.x, 12.0);
  EXPECT_EQ(module.common()->requested_target_lane_id, 3);
  EXPECT_TRUE(module.isStaticObstaclePathSafe(shifted));
  const auto offset = shifted.info.lane_changing_start.position.x;
  const size_t start = shifted.info.shift_line.start_idx;
  for (size_t i = 0; i <= original->info.shift_line.end_idx; ++i) {
    EXPECT_NEAR(
      shifted.path.points[start + i].point.pose.position.x -
        original->path.points[i].point.pose.position.x,
      offset, 1e-10);
    EXPECT_DOUBLE_EQ(
      shifted.path.points[start + i].point.pose.position.y,
      original->path.points[i].point.pose.position.y);
  }
}

TEST_F(TranslatedCurve, RequiresActualObstacleStopAndStoppedEgo)
{
  ReplanModule module;
  auto original = utils::lane_change::generate_low_speed_path(module.common(), 20, 1.5);
  ASSERT_TRUE(original);
  module.approve(*original);
  module.block(7, 3.2);
  module.setObstacleStopActive(false);
  EXPECT_FALSE(module.updateApprovedPath());
  EXPECT_EQ(module.getLaneChangePath().path, original->path);
  module.setObstacleStopActive(true);
  module.odometry->twist.twist.linear.x = 1.0;
  EXPECT_FALSE(module.updateApprovedPath());
  EXPECT_EQ(module.getLaneChangePath().path, original->path);
}

TEST_F(TranslatedCurve, UpstreamZeroAndBlockedConnectorKeepApprovedCurve)
{
  ReplanModule module;
  const auto original = utils::lane_change::generate_low_speed_path(module.common(), 20, 1.5);
  ASSERT_TRUE(original);
  module.approve(*original);
  module.block(7, 3.2);
  module.common()->transient_data.current_path_velocity = 0.0;
  EXPECT_FALSE(module.updateApprovedPath());
  module.common()->transient_data.current_path_velocity = 5.0;
  module.block(3, 0, 30);
  module.allow_next_replan();
  EXPECT_FALSE(module.updateApprovedPath());
  EXPECT_EQ(module.getLaneChangePath().path, original->path);
  EXPECT_EQ(module.common()->requested_target_lane_id, 3);
}

TEST_F(TranslatedCurve, CannotCrossTerminalOrChangeTarget)
{
  ReplanModule module;
  auto original = utils::lane_change::generate_low_speed_path(module.common(), 20, 1.5);
  ASSERT_TRUE(original);
  module.approve(*original);
  module.common()->transient_data.dist_to_terminal_end = 20.0;
  EXPECT_FALSE(module.updateApprovedPath());
  EXPECT_EQ(module.getLaneChangePath().path, original->path);
  EXPECT_EQ(module.common()->requested_target_lane_id, 3);
}

TEST_F(TranslatedCurve, InvalidGeometryAndSteeringLimitsFailClosed)
{
  ReplanModule module;
  const auto original = utils::lane_change::generate_low_speed_path(module.common(), 20, 1.5);
  ASSERT_TRUE(original);
  for (double offset : {-1.0, 0.0, std::numeric_limits<double>::quiet_NaN()})
    EXPECT_FALSE(
      utils::lane_change::translate_approved_curve(module.common(), *original, offset, 1.5));
  module.odometry->pose.pose.position.x = 30;
  EXPECT_FALSE(utils::lane_change::translate_approved_curve(module.common(), *original, 8, 1.5));
  module.odometry->pose.pose.position.x = 0;
  module.common()->bpp_param_ptr->vehicle_info.max_steer_angle_rad = .001;
  EXPECT_FALSE(utils::lane_change::translate_approved_curve(module.common(), *original, 8, 1.5));
}

TEST_F(TranslatedCurve, BudgetResumesAndTranslationDoesNotAccumulate)
{
  ReplanModule module;
  const auto original = utils::lane_change::generate_low_speed_path(module.common(), 20, 1.5);
  ASSERT_TRUE(original);
  module.approve(*original);
  module.params()->time_limit = 1.0;
  module.block(3, 0, 30);
  EXPECT_FALSE(module.updateApprovedPath());
  const auto first = module.cursor();
  EXPECT_GT(first, 0u);
  module.allow_next_replan();
  EXPECT_FALSE(module.updateApprovedPath());
  EXPECT_NE(module.cursor(), first);
  module.data->dynamic_object = std::make_shared<PredictedObjects>();
  module.params()->time_limit = 2000;
  for (size_t i = 0; i < 14; ++i) {
    module.allow_next_replan();
    module.updateApprovedPath();
  }
  const auto last = module.getLaneChangePath();
  EXPECT_LE(last.info.lane_changing_start.position.x, 12.0);
  EXPECT_NEAR(last.info.length.lane_changing, original->info.length.lane_changing, 1e-8);
}

// Reuse the same legal three-lane map for the clearance/preparation regression cases.
using LaneChangeClearance = TranslatedCurve;
using MultiLanePreparation = TranslatedCurve;

TEST_F(MultiLanePreparation, DirectTargetUsesOneSecondDespiteSignalObstacleAndTerminal)
{
  ReplanModule module;
  namespace calc = utils::lane_change::calculation;
  auto common = module.common();
  module.params()->trajectory.multi_lane_prepare_duration = 1.0;
  module.odometry->twist.twist.linear.x = 2.0;
  for (const double signal_time : {0.0, 10.0}) {
    EXPECT_DOUBLE_EQ(calc::calc_actual_prepare_duration(common, 2.0, signal_time), 1.0);
  }
  for (const double obstacle : {std::numeric_limits<double>::infinity(), 10.0}) {
    for (const double terminal : {0.0, 80.0}) {
      common->transient_data.distance_to_static_obstacle = obstacle;
      common->transient_data.dist_to_terminal_start = terminal;
      common->transient_data.lane_change_prepare_duration = 4.0;
      const auto metrics = calc::calc_prepare_phase_metrics(common, 2.0, 5.0, 0.0, 10.0);
      ASSERT_FALSE(metrics.empty());
      for (const auto & metric : metrics) EXPECT_DOUBLE_EQ(metric.duration, 1.0);
    }
  }
}

TEST_F(MultiLanePreparation, FullPreparationSearchKeepsOneSecondForBrakingCandidates)
{
  ReplanModule module;
  module.params()->trajectory.multi_lane_prepare_duration = 1.0;
  module.params()->trajectory.min_prepare_duration = 0.25;
  module.odometry->twist.twist.linear.x = 2.0;
  const auto metrics = module.get_prepare_metrics();
  ASSERT_FALSE(metrics.empty());
  for (const auto & metric : metrics) EXPECT_NEAR(metric.duration, 1.0, 1e-8);
  EXPECT_FALSE(module.make_braking_prepare_metric(0.5));
}

TEST_F(MultiLanePreparation, AdjacentTargetKeepsOrdinaryTiming)
{
  ReplanModule module;
  auto common = module.common();
  module.params()->trajectory.multi_lane_prepare_duration = 1.0;
  module.params()->trajectory.max_prepare_duration = 3.0;
  common->lanes_ptr->target = {module.data->route_handler->getLaneletsFromId(2)};
  EXPECT_DOUBLE_EQ(
    utils::lane_change::calculation::calc_actual_prepare_duration(common, 2.0, 0.0), 3.0);
}

TEST_F(MultiLanePreparation, OneSecondDoesNotBypassAccelerationOrAvailableLength)
{
  ReplanModule module;
  auto common = module.common();
  module.params()->trajectory.multi_lane_prepare_duration = 1.0;
  common->transient_data.distance_to_static_obstacle = 10.0;
  namespace calc = utils::lane_change::calculation;
  EXPECT_TRUE(calc::calc_prepare_phase_metrics(common, 0.0, 5.0, 0.0, 10.0).empty());
  EXPECT_TRUE(calc::calc_prepare_phase_metrics(common, 2.0, 5.0, 0.0, 0.1).empty());
}

TEST_F(LaneChangeClearance, HalfScaleClearsMarginOverlapButStillRejectsBodyOverlap)
{
  ReplanModule module;
  namespace checker = utils::path_safety_checker;
  auto & safety = module.params()->safety;
  auto & rss = safety.rss_params_for_parked;
  rss.extended_polygon_policy = "rectangle";
  rss.lateral_distance_max_threshold = 1.0;
  rss.longitudinal_distance_min_threshold = 3.0;
  rss.front_vehicle_deceleration = -1.0;
  rss.rear_vehicle_deceleration = -2.0;
  rss.rear_vehicle_reaction_time = 1.0;
  rss.rear_vehicle_safety_time_margin = 0.8;
  safety.th_stopped_object_velocity = 0.3;
  LaneChangePath path;
  path.path = reference(0.0);
  path.info.duration.prepare = 0.0;
  geometry_msgs::msg::Pose ego;
  ego.orientation.w = 1.0;
  std::vector<checker::PoseWithVelocityStamped> prediction{{0.0, ego, 1.0}, {1.0, ego, 1.0}};
  auto collides = [&](const double x, const double y, const double scale,
                      checker::CollisionCheckDebugMap & debug) {
    checker::ExtendedPredictedObject object;
    object.initial_pose = ego;
    object.initial_pose.position.x = x;
    object.initial_pose.position.y = y;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 4.394;
    object.shape.dimensions.y = 1.808;
    object.initial_polygon = autoware_utils::to_polygon2d(object.initial_pose, object.shape);
    checker::PredictedPathWithPolygon object_path;
    object_path.path.emplace_back(0.0, object.initial_pose, 0.0, object.initial_polygon);
    object_path.path.emplace_back(1.0, object.initial_pose, 0.0, object.initial_polygon);
    object.predicted_paths.push_back(object_path);
    safety.polygon_expansion_scale = scale;
    return module.is_colliding(path, object, prediction, rss, debug, false);
  };
  checker::CollisionCheckDebugMap debug;
  EXPECT_TRUE(collides(7.2, 0.0, 1.0, debug));
  EXPECT_FALSE(collides(7.2, 0.0, 0.5, debug));  // Front extra clearance is halved.
  EXPECT_TRUE(collides(6.0, 2.5, 1.0, debug));
  EXPECT_FALSE(collides(6.0, 2.5, 0.5, debug));  // Side extra clearance is halved.
  prediction[0].velocity = prediction[1].velocity = 2.0;
  EXPECT_TRUE(collides(8.2, 0.0, 1.0, debug));
  EXPECT_FALSE(collides(8.2, 0.0, 0.5, debug));  // RSS distance also shrinks when above the floor.
  EXPECT_TRUE(collides(5.0, 0.0, 0.5, debug));  // Actual vehicle footprint still overlaps.
  ASSERT_FALSE(debug.empty());
  EXPECT_EQ(debug.begin()->second.unsafe_reason, "overlap_polygon");
}
}  // namespace
}  // namespace autoware::behavior_path_planner
