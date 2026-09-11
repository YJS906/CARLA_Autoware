// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#include "interface.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_path_planner
{
// Exercise real object geometry, request state and RTC cleanup; candidate
// generation is not needed to test whether a departing trigger may keep a
// previously prepared request alive.
class TestAvoidanceRequestLifecycle : public AvoidanceByLaneChange
{
public:
  TestAvoidanceRequestLifecycle()
  : AvoidanceByLaneChange(
      std::make_shared<LaneChangeParameters>(),
      std::make_shared<AvoidanceByLCParameters>(AvoidanceParameters{}))
  {
    data = std::make_shared<PlannerData>();
    odom = std::make_shared<Odometry>();
    odom->header.stamp.sec = 100;
    odom->pose.pose.orientation.w = 1.0;
    odom->twist.twist.linear.x = 1.4;
    data->self_odometry = odom;
    data->self_acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
    data->route_handler = std::make_shared<RouteHandler>();
    data->parameters.max_vel = 15.0;
    data->parameters.vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
      .383, .235, 2.944, 1.64, 1.0, 1.1, .128, .128, 2.5, .7);
    data->parameters.vehicle_width = data->parameters.vehicle_info.vehicle_width_m;
    setData(data);
    const lanelet::Lanelet lane(
      101,
      lanelet::LineString3d(
        102, {lanelet::Point3d(103, -10, 2, 0), lanelet::Point3d(104, 150, 2, 0)}),
      lanelet::LineString3d(
        105, {lanelet::Point3d(106, -10, -2, 0), lanelet::Point3d(107, 150, -2, 0)}));
    common_data_ptr_->lanes_ptr->current = {lane};
    common_data_ptr_->lanes_ptr->target_neighbor = {lane};
    common_data_ptr_->lanes_ptr->target = {lane};
    common_data_ptr_->transient_data.is_ego_near_current_terminal_start = true;
    common_data_ptr_->transient_data.dist_to_terminal_end = 100.0;
    planning.current_lanelets = {lane};
    planning.reference_pose = odom->pose.pose;
    for (const double x : {-10.0, 0.0, 50.0, 100.0, 150.0}) {
      PathPointWithLaneId point;
      point.point.pose.position.x = x;
      point.point.pose.orientation.w = 1.0;
      planning.reference_path.points.push_back(point);
    }
    planning.reference_path_rough = planning.reference_path;
    common_data_ptr_->current_lanes_path = planning.reference_path;
    auto & param = avoidance_parameters_->object_parameters[1];
    param.is_avoidance_target = true;
    param.moving_speed_threshold = 1.0;
    param.moving_time_threshold = 1.0;
    param.envelope_buffer_margin = .3;
    param.lateral_hard_margin = 0.;
    param.th_error_eclipse_long_radius = 2.;
    avoidance_parameters_->upper_distance_for_polygon_expansion = 1.;
    avoidance_parameters_->max_execution_distance = 35.;
    direction_ = Direction::LEFT;
  }

  bool target(const double x, const double speed, const double yaw = 0., const double age = 0.)
  {
    auto objects = std::make_shared<PredictedObjects>();
    objects->header.stamp = rclcpp::Time(static_cast<int64_t>((100. - age) * 1e9));
    PredictedObject object;
    object.object_id.uuid[0] = 1;
    object.classification.resize(1);
    object.classification.front().label = 1;
    object.classification.front().probability = 1.;
    auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    pose.position.x = x;
    pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    object.kinematics.initial_twist_with_covariance.twist.linear.x = speed;
    object.shape.type = Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 4.5;
    object.shape.dimensions.y = 1.9;
    object.shape.dimensions.z = 1.5;
    objects->objects.push_back(object);
    data->dynamic_object = objects;
    planning.target_objects.clear();
    planning.other_objects.clear();
    DebugData debug;
    fillAvoidanceTargetObjects(planning, debug);
    avoidance_data_ = planning;
    return getNearestAvoidanceTarget() != nullptr;
  }

  void rememberGeometry()
  {
    registered_objects_ = planning.target_objects;
    stopped_objects_.clear();  // Next sample represents an already confirmed moving vehicle.
  }
  void pending(const std::optional<uint8_t> id)
  {
    avoidance_data_.target_objects.clear();
    if (id) {
      ObjectData object;
      object.object.object_id.uuid[0] = *id;
      object.object.classification.resize(1);
      object.object.classification.front().label = 1;
      object.object.classification.front().probability = 1.;
      object.longitudinal = 60.;
      object.avoid_required = true;
      avoidance_data_.target_objects.push_back(object);
    }
    updatePendingTarget(getNearestAvoidanceTarget());
  }
  void prepare()
  {
    status_.is_valid_path = true;
    status_.is_safe = false;
    status_.lane_change_path.info.speed_preparation_target = 1.0;
    ASSERT_TRUE(hasSpeedPreparationRequest());
    speed_preparation_profile_.emplace();
    speed_preparation_start_.position.x = 5.;
  }
  bool preparationCleared() const
  {
    return !speed_preparation_target_ && !speed_preparation_profile_ &&
           speed_preparation_lane_id_ == lanelet::InvalId &&
           !status_.lane_change_path.info.speed_preparation_target &&
           speed_preparation_start_.position.x == 0.;
  }
  void running() { is_activated_ = true; }
  std::shared_ptr<PlannerData> data;
  std::shared_ptr<Odometry> odom;
  AvoidancePlanningData planning;
};

namespace
{
using RTCMap = std::unordered_map<std::string, std::shared_ptr<RTCInterface>>;
using InterestMap =
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>;
class LifecycleInterface : public AvoidanceByLaneChangeInterface
{
public:
  LifecycleInterface(rclcpp::Node & node, const RTCMap & rtc, InterestMap & interest)
  : AvoidanceByLaneChangeInterface(
      "avoidance_by_lane_change", node, std::make_shared<LaneChangeParameters>(),
      std::make_shared<AvoidanceByLCParameters>(AvoidanceParameters{}), rtc, interest,
      std::make_shared<PlanningFactorInterface>(&node, "avoidance_by_lane_change"))
  {
    module_type_ = std::make_unique<TestAvoidanceRequestLifecycle>();
  }
  TestAvoidanceRequestLifecycle & path()
  {
    return static_cast<TestAvoidanceRequestLifecycle &>(*module_type_);
  }
  void target(const std::optional<uint8_t> id)
  {
    const auto before = path().getPendingTargetId();
    path().pending(id);
    synchronizePendingRequest(before);
  }
  UUID uuid() const { return uuid_map_.at("left"); }
  void candidate() { path_candidate_ = std::make_shared<PathWithLaneId>(); }
};
class RequestLifecycle : public ::testing::Test
{
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(RequestLifecycle, DistantFasterLeadIsNotAnAvoidanceTrigger)
{
  TestAvoidanceRequestLifecycle module;
  EXPECT_FALSE(module.target(55.5, 2.22));
  EXPECT_FALSE(module.target(67.1, 2.22));
  ASSERT_EQ(module.planning.other_objects.size(), 1u);
  EXPECT_EQ(module.planning.other_objects.front().info, ObjectInfo::MOVING_OBJECT);
}

TEST_F(RequestLifecycle, StationaryAndClosingLeadRemainEligible)
{
  TestAvoidanceRequestLifecycle stationary;
  EXPECT_TRUE(stationary.target(60., 0.));
  TestAvoidanceRequestLifecycle closing;
  closing.odom->twist.twist.linear.x = 4.;
  EXPECT_TRUE(closing.target(60., 2.22));
}

TEST_F(RequestLifecycle, RecedingLeadFollowingRecordedCurveRemainsReleased)
{
  TestAvoidanceRequestLifecycle module;
  module.odom->twist.twist.linear.x = 1.4393;
  EXPECT_FALSE(module.target(67.1, 2.22, 0.58645));
}

TEST_F(RequestLifecycle, NearLeadAndEqualSpeedRemainEligible)
{
  TestAvoidanceRequestLifecycle module;
  EXPECT_TRUE(module.target(35., 2.22));
  EXPECT_TRUE(module.target(60., 1.5));
}

TEST_F(RequestLifecycle, CrossingAndOncomingObjectsAreNotReleasedAsReceding)
{
  TestAvoidanceRequestLifecycle module;
  for (const double yaw : {1.5707963267948966, 3.141592653589793}) {
    module.target(60., 2.22, yaw);
    // Existing lane/yaw filtering may already put an oncoming object in
    // other_objects. It must not be marked as a receding trigger by the new
    // release policy.
    EXPECT_TRUE(
      std::none_of(
        module.planning.other_objects.begin(), module.planning.other_objects.end(),
        [](const auto & object) { return object.info == ObjectInfo::MOVING_OBJECT; }));
  }
}

TEST_F(RequestLifecycle, StalePerceptionDoesNotProveDeparture)
{
  TestAvoidanceRequestLifecycle module;
  EXPECT_TRUE(module.target(60., 2.22, 0., 1.));
}

TEST_F(RequestLifecycle, OldEnvelopeCannotKeepDepartedTargetEligible)
{
  TestAvoidanceRequestLifecycle module;
  ASSERT_TRUE(module.target(25., 0.));
  module.rememberGeometry();
  EXPECT_FALSE(module.target(60., 2.22));
}

TEST_F(RequestLifecycle, RunningManeuverKeepsItsTarget)
{
  TestAvoidanceRequestLifecycle module;
  module.pending(1);
  module.prepare();
  module.running();
  EXPECT_TRUE(module.target(60., 2.22));
  module.pending(std::nullopt);
  EXPECT_TRUE(module.getPendingTargetId());
  EXPECT_TRUE(module.hasSpeedPreparationRequest());
}

TEST_F(RequestLifecycle, SameBlockerPreservesRequiredPreparation)
{
  TestAvoidanceRequestLifecycle module;
  module.pending(1);
  module.prepare();
  module.pending(1);
  EXPECT_TRUE(module.hasSpeedPreparationRequest());
}

TEST_F(RequestLifecycle, ReleasedTargetClearsCandidateRTCAndPreparationTogether)
{
  auto node = std::make_shared<rclcpp::Node>("request_cleanup_test");
  RTCMap rtc;
  InterestMap interest;
  rtc["left"] = std::make_shared<RTCInterface>(node.get(), "avoidance_by_lane_change_left", false);
  LifecycleInterface module(*node, rtc, interest);
  module.onEntry();
  module.target(1);
  module.path().prepare();
  module.candidate();
  const auto retired = module.uuid();
  const auto other = generate_uuid();
  rtc.at("left")->updateCooperateStatus(
    retired, true, State::WAITING_FOR_EXECUTION, 4., 34., node->now());
  rtc.at("left")->updateCooperateStatus(
    other, false, State::WAITING_FOR_EXECUTION, 4., 34., node->now());
  ASSERT_TRUE(rtc.at("left")->isActivated(retired));
  module.target(std::nullopt);
  EXPECT_FALSE(module.getPathCandidate());
  EXPECT_FALSE(rtc.at("left")->isRegistered(retired));
  EXPECT_TRUE(rtc.at("left")->isRegistered(other));
  EXPECT_NE(module.uuid(), retired);
  EXPECT_TRUE(module.path().preparationCleared());
  EXPECT_FALSE(module.isExecutionRequested());
  EXPECT_FALSE(module.isExecutionReady());
  EXPECT_TRUE(module.path().specialExpiredCheck());
}

TEST_F(RequestLifecycle, ReplacementObjectCannotInheritApprovalOrLowSpeedTarget)
{
  auto node = std::make_shared<rclcpp::Node>("replacement_cleanup_test");
  RTCMap rtc;
  InterestMap interest;
  rtc["left"] = std::make_shared<RTCInterface>(node.get(), "avoidance_by_lane_change_left", false);
  LifecycleInterface module(*node, rtc, interest);
  module.onEntry();
  module.target(1);
  module.path().prepare();
  const auto retired = module.uuid();
  rtc.at("left")->updateCooperateStatus(
    retired, true, State::WAITING_FOR_EXECUTION, 4., 34., node->now());
  module.target(2);
  EXPECT_NE(module.uuid(), retired);
  EXPECT_FALSE(rtc.at("left")->isRegistered(retired));
  EXPECT_FALSE(rtc.at("left")->isRegistered(module.uuid()));
  EXPECT_TRUE(module.path().preparationCleared());
}
}  // namespace
}  // namespace autoware::behavior_path_planner
