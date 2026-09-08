// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "scene.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/utils.hpp"

#include <autoware/behavior_path_lane_change_module/utils/calculation.hpp>
#include <autoware/behavior_path_lane_change_module/utils/utils.hpp>
#include <autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp>
#include <autoware/behavior_path_static_obstacle_avoidance_module/static_collision.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <rclcpp/logging.hpp>

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/strategies/cartesian/centroid_bashein_detmer.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
geometry_msgs::msg::Point32 create_point32(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Point32 p;
  p.x = static_cast<float>(pose.position.x);
  p.y = static_cast<float>(pose.position.y);
  p.z = static_cast<float>(pose.position.z);
  return p;
};

geometry_msgs::msg::Polygon create_execution_area(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & pose, double additional_lon_offset, double additional_lat_offset)
{
  const double & base_to_front = vehicle_info.max_longitudinal_offset_m;
  const double & width = vehicle_info.vehicle_width_m;
  const double & base_to_rear = vehicle_info.rear_overhang_m;

  // if stationary object, extend forward and backward by the half of lon length
  const double forward_lon_offset = base_to_front + additional_lon_offset;
  const double backward_lon_offset = -base_to_rear;
  const double lat_offset = width / 2.0 + additional_lat_offset;

  const auto p1 = autoware_utils::calc_offset_pose(pose, forward_lon_offset, lat_offset, 0.0);
  const auto p2 = autoware_utils::calc_offset_pose(pose, forward_lon_offset, -lat_offset, 0.0);
  const auto p3 = autoware_utils::calc_offset_pose(pose, backward_lon_offset, -lat_offset, 0.0);
  const auto p4 = autoware_utils::calc_offset_pose(pose, backward_lon_offset, lat_offset, 0.0);
  geometry_msgs::msg::Polygon polygon;

  polygon.points.push_back(create_point32(p1));
  polygon.points.push_back(create_point32(p2));
  polygon.points.push_back(create_point32(p3));
  polygon.points.push_back(create_point32(p4));

  return polygon;
}
}  // namespace

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::Direction;
using autoware::behavior_path_planner::LaneChangeModuleType;
using autoware::behavior_path_planner::ObjectInfo;
using autoware::behavior_path_planner::Point2d;

AvoidanceByLaneChange::AvoidanceByLaneChange(
  const std::shared_ptr<LaneChangeParameters> & parameters,
  std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters)
: NormalLaneChange(parameters, LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE, Direction::NONE),
  avoidance_parameters_(std::move(avoidance_parameters)),
  avoidance_helper_{std::make_shared<AvoidanceHelper>(avoidance_parameters_)}
{
  common_data_ptr_->max_lane_changing_length_scale =
    avoidance_parameters_->max_lane_changing_length_scale;
  common_data_ptr_->disable_lateral_acceleration_limit =
    avoidance_parameters_->disable_lateral_acceleration_limit;
}

const ObjectData * AvoidanceByLaneChange::getNearestAvoidanceTarget() const
{
  const auto & object_parameters = avoidance_parameters_->object_parameters;
  const auto is_target = [&](const auto & object) {
    const auto label = utils::getHighestProbLabel(object.object.classification);
    const auto parameter = object_parameters.find(label);
    return parameter != object_parameters.end() && parameter->second.is_avoidance_target &&
           object.avoid_required;
  };

  const auto target = std::find_if(
    avoidance_data_.target_objects.begin(), avoidance_data_.target_objects.end(), is_target);
  return target == avoidance_data_.target_objects.end() ? nullptr : &*target;
}

void AvoidanceByLaneChange::applyObstacleVelocityLimit()
{
  const double velocity_limit =
    std::max(0.0, getCommonParam().max_vel * avoidance_parameters_->obstacle_velocity_limit_ratio);
  const auto cap_velocity = [velocity_limit](auto & path) {
    for (auto & point : path.points) {
      point.point.longitudinal_velocity_mps =
        std::min(point.point.longitudinal_velocity_mps, static_cast<float>(velocity_limit));
    }
  };

  cap_velocity(prev_module_output_.path);
  cap_velocity(prev_module_output_.reference_path);
  cap_velocity(avoidance_data_.reference_path_rough);
  cap_velocity(avoidance_data_.reference_path);
  common_data_ptr_->transient_data.current_path_velocity =
    std::min(common_data_ptr_->transient_data.current_path_velocity, velocity_limit);

  RCLCPP_INFO_THROTTLE(
    logger_, clock_, 3000, "avoidance obstacle velocity cap applied: %.2f m/s (%.0f%% of max)",
    velocity_limit, avoidance_parameters_->obstacle_velocity_limit_ratio * 100.0);
}

bool AvoidanceByLaneChange::isExecutionDistanceSatisfied() const
{
  const auto * target = getNearestAvoidanceTarget();
  if (!target) return false;
  const auto distance = target->longitudinal;
  const auto limit = avoidance_parameters_->max_execution_distance;
  return std::isfinite(distance) && std::isfinite(limit) && limit > 0.0 && distance > 0.0 &&
         distance <= limit;
}

bool AvoidanceByLaneChange::specialRequiredCheck() const
{
  const auto * nearest_object = getNearestAvoidanceTarget();
  if (!nearest_object) {
    RCLCPP_DEBUG(logger_, "no avoidance target");
    return false;
  }

  const auto minimum_avoid_length = calcMinAvoidanceLength(*nearest_object);
  const auto minimum_lane_change_length = calc_minimum_dist_buffer();
  double request_distance = std::max(
    minimum_avoid_length, std::max(0.0, avoidance_parameters_->execute_object_longitudinal_margin));
  if (avoidance_parameters_->execute_only_when_lane_change_finish_before_object) {
    request_distance = std::max(request_distance, minimum_lane_change_length);
  }

  lane_change_debug_.execution_area = create_execution_area(
    getCommonParam().vehicle_info, getEgoPose(), request_distance, calcLateralOffset());

  RCLCPP_DEBUG(
    logger_,
    "early avoidance request object=%.2f request_distance=%.2f minimum_lane_change=%.2f "
    "minimum_avoidance=%.2f",
    nearest_object->longitudinal, request_distance, minimum_lane_change_length,
    minimum_avoid_length);
  return nearest_object->longitudinal > request_distance;
}

bool AvoidanceByLaneChange::specialExpiredCheck() const
{
  // The base interface applies this expiration only while WAITING_APPROVAL, before its RTC
  // transition to RUNNING. Withdraw a stale request even if a prior cycle was safe/approved;
  // an already RUNNING maneuver is never cancelled merely because its target distance changes.
  if (!isExecutionDistanceSatisfied()) {
    return true;
  }

  // No selected side means there is no RTC request to approve. Expire the candidate instead of
  // letting the common "no registered requests" transition treat it as executable.
  if (direction_ == Direction::NONE) {
    return true;
  }

  // Do not drop a safe candidate during the one-cycle auto-approval hand-off merely because the
  // ego crossed the early-request distance threshold in that cycle.
  if (status_.is_valid_path && status_.is_safe) {
    return false;
  }
  return !specialRequiredCheck();
}

void AvoidanceByLaneChange::updateSpecialData()
{
  updateStationaryObservations();
  const auto p = std::dynamic_pointer_cast<AvoidanceParameters>(avoidance_parameters_);

  avoidance_debug_data_ = DebugData();
  avoidance_data_ = calcAvoidancePlanningData(avoidance_debug_data_);

  // Stabilize the obstacle used for the early lane-change decision before selecting a side. A
  // one-frame perception dropout must not let static avoidance claim the exclusive slot first.
  auto current_target_objects = avoidance_data_.target_objects;
  utils::static_obstacle_avoidance::compensateLostTargetObjects(
    avoidance_data_, registered_objects_, planner_data_);
  utils::static_obstacle_avoidance::updateStoredObjects(
    registered_objects_, current_target_objects, clock_.now(), p);

  std::sort(
    avoidance_data_.target_objects.begin(), avoidance_data_.target_objects.end(),
    [](const auto & a, const auto & b) { return a.longitudinal < b.longitudinal; });

  const auto * nearest_avoidance_target = getNearestAvoidanceTarget();
  if (nearest_avoidance_target) {
    applyObstacleVelocityLimit();
  }

  // Once approved, keep the chosen side and terminal lane until the maneuver completes. Re-running
  // the farthest-clear-lane selection while RUNNING can otherwise replace the approved target as
  // object occupancy changes, even though update_lanes() intentionally freezes approved lanes.
  const bool preserve_approved_target =
    is_activated_ && common_data_ptr_->requested_target_lane_id.has_value();
  if (preserve_approved_target) {
    return;
  }

  const auto previous_target = common_data_ptr_->requested_target_lane_id;
  target_lane_candidates_.clear();
  current_lanes_to_preferred_ =
    avoidance_data_.current_lanelets.empty()
      ? 0
      : std::abs(common_data_ptr_->route_handler_ptr->getNumLaneToPreferredLane(
          avoidance_data_.current_lanelets.back()));
  common_data_ptr_->requested_target_lane_id.reset();
  if (nearest_avoidance_target) {
    const auto preferred = utils::static_obstacle_avoidance::isOnRight(*nearest_avoidance_target)
                             ? Direction::LEFT
                             : Direction::RIGHT;
    const auto opposite = preferred == Direction::LEFT ? Direction::RIGHT : Direction::LEFT;
    for (const auto direction : {preferred, opposite}) {
      const auto lanes = getTargetLaneCandidates(direction, *nearest_avoidance_target);
      for (std::size_t i = 0; i < lanes.size(); ++i) {
        target_lane_candidates_.push_back(
          {direction, lanes.at(i).id(),
           std::abs(common_data_ptr_->route_handler_ptr->getNumLaneToPreferredLane(lanes.at(i))),
           i + 1});
      }
    }
    std::stable_sort(
      target_lane_candidates_.begin(), target_lane_candidates_.end(),
      [&](const auto & a, const auto & b) {
        return std::make_tuple(
                 a.lanes_to_preferred, a.lateral_steps, previous_target != a.lane_id) <
               std::make_tuple(b.lanes_to_preferred, b.lateral_steps, previous_target != b.lane_id);
      });
  }

  // Initialize in this cycle, before the exclusive-slot arbitration can launch static avoidance.
  // Path validity, distance and safety are checked only for policy-eligible candidates below.
  if (!target_lane_candidates_.empty()) {
    selectTargetLane(target_lane_candidates_.front());
  } else {
    direction_ = Direction::NONE;
    common_data_ptr_->direction = Direction::NONE;
    common_data_ptr_->requested_target_lane_id.reset();
    update_lanes(false);
    status_.is_valid_path = false;
    status_.is_safe = false;
    lane_change_debug_.valid_paths.clear();
  }
}

std::vector<lanelet::ConstLanelet> AvoidanceByLaneChange::getTargetLaneCandidates(
  const Direction direction, const ObjectData & nearest_object) const
{
  std::vector<lanelet::ConstLanelet> candidates;
  if (
    (direction != Direction::LEFT && direction != Direction::RIGHT) ||
    avoidance_data_.current_lanelets.empty() || avoidance_data_.reference_path.points.empty()) {
    return candidates;
  }

  common_data_ptr_->direction = direction;
  common_data_ptr_->requested_target_lane_id.reset();
  const auto first_target = utils::lane_change::get_lane_change_target_lane(
    common_data_ptr_, avoidance_data_.current_lanelets);
  if (!first_target || !common_data_ptr_->route_handler_ptr->isRouteLanelet(*first_target)) {
    return candidates;
  }

  const auto & routing_graph = common_data_ptr_->route_handler_ptr->getRoutingGraphPtr();
  auto target_lane = *first_target;
  std::unordered_set<lanelet::Id> visited;
  while (visited.insert(target_lane.id()).second) {
    if (target_lane.centerline().empty()) {
      break;
    }

    const auto target_front =
      autoware::experimental::lanelet2_utils::to_ros(target_lane.centerline().front());
    const double target_start = autoware::motion_utils::calcSignedArcLength(
      avoidance_data_.reference_path.points, getEgoPosition(), target_front);
    const bool available_before_object = target_start <= nearest_object.longitudinal;
    const bool lane_clear = isLaneClear(target_lane);
    RCLCPP_DEBUG(
      logger_,
      "avoidance route-lane candidate direction=%s lane=%lld start=%.2f object=%.2f "
      "clear=%s usable=%s",
      direction == Direction::LEFT ? "LEFT" : "RIGHT", static_cast<long long>(target_lane.id()),
      target_start, nearest_object.longitudinal, lane_clear ? "true" : "false",
      available_before_object ? "true" : "false");

    if (!available_before_object) {
      break;
    }

    // Preserve the existing immediate-lane attempt even when it currently contains another
    // object; the full predicted-path safety checker may still find a valid time gap. Moving past
    // that lane in one continuous shift is allowed only when it is actually empty.
    if (candidates.empty() || lane_clear) {
      candidates.push_back(target_lane);
    }
    if (
      !avoidance_parameters_->enable_direct_multi_lane_change || !lane_clear ||
      candidates.back().id() != target_lane.id()) {
      break;
    }

    const auto next_lane = direction == Direction::LEFT ? routing_graph->left(target_lane)
                                                        : routing_graph->right(target_lane);
    if (!next_lane || !common_data_ptr_->route_handler_ptr->isRouteLanelet(*next_lane)) {
      break;
    }
    target_lane = *next_lane;
  }

  return candidates;
}

bool AvoidanceByLaneChange::isLaneClear(const lanelet::ConstLanelet & lane) const
{
  if (!planner_data_->dynamic_object || planner_data_->dynamic_object->objects.empty()) {
    return true;
  }

  auto lane_sequence = common_data_ptr_->route_handler_ptr->getLaneletSequence(
    lane, getEgoPose(), avoidance_parameters_->empty_lane_check_backward_distance,
    avoidance_parameters_->empty_lane_check_forward_distance);
  if (lane_sequence.empty()) {
    lane_sequence.push_back(lane);
  }

  for (const auto & object : planner_data_->dynamic_object->objects) {
    const auto & object_position = object.kinematics.initial_pose_with_covariance.pose.position;
    const double longitudinal = autoware::motion_utils::calcSignedArcLength(
      avoidance_data_.reference_path.points, getEgoPosition(), object_position);
    if (
      longitudinal < -avoidance_parameters_->empty_lane_check_backward_distance ||
      longitudinal > avoidance_parameters_->empty_lane_check_forward_distance) {
      continue;
    }

    const bool overlaps_lane =
      std::any_of(lane_sequence.begin(), lane_sequence.end(), [&](const auto & lanelet) {
        return utils::path_safety_checker::isPolygonOverlapLanelet(
          object, lanelet.polygon2d().basicPolygon());
      });
    if (overlaps_lane) {
      return false;
    }
  }
  return true;
}

void AvoidanceByLaneChange::updateLaneChangeStatus()
{
  if (is_activated_) {
    return;
  }

  struct Evaluation
  {
    TargetLaneCandidate target;
    LaneChangePath path;
    lane_change::Debug debug;
    std::optional<LaneChangePath> terminal_path;
    bool safe;
  };
  std::optional<Evaluation> selected;
  std::size_t eligible = 0;
  bool left_safe = false;
  bool right_safe = false;
  bool left_evaluated = false;
  bool right_evaluated = false;
  bool route_side_temporarily_blocked = false;
  selected_requires_return_ = false;
  status_.is_valid_path = false;
  status_.is_safe = false;
  for (const auto & target : target_lane_candidates_) {
    const bool departure =
      target.lanes_to_preferred != 0 && target.lanes_to_preferred >= current_lanes_to_preferred_;
    if (departure && (route_side_temporarily_blocked || !isRouteDepartureRequired())) {
      continue;
    }
    if (!selectTargetLane(target) || !specialRequiredCheck()) {
      // Missing geometry/regulatory clearance is not evidence of a permanent obstruction.
      route_side_temporarily_blocked |= !departure;
      continue;
    }
    ++eligible;
    left_evaluated |= target.direction == Direction::LEFT;
    right_evaluated |= target.direction == Direction::RIGHT;
    LaneChangePath candidate_path;
    const auto [found_valid_path, found_safe_path] = getSafePath(candidate_path);
    const bool static_safe = found_valid_path && isStaticObstaclePathSafe(candidate_path);
    bool safe = found_valid_path && found_safe_path && static_safe;
    if (!departure && !safe) {
      const auto collision = found_valid_path
                               ? check_static_path(candidate_path)
                               : utils::path_safety_checker::TrajectoryCollisionResult{};
      route_side_temporarily_blocked |=
        !collision.valid || !collision.object_id || !isPersistentBlocker(*collision.object_id);
    }
    if (departure && safe) safe = prepareRouteReturn(target, candidate_path);
    left_safe |= safe && target.direction == Direction::LEFT;
    right_safe |= safe && target.direction == Direction::RIGHT;
    candidate_path.path.header = getRouteHeader();
    RCLCPP_DEBUG(
      logger_,
      "avoidance bilateral candidate direction=%s target=%lld valid=%s predicted_safe=%s "
      "static_safe=%s lanes_to_mission=%d",
      target.direction == Direction::LEFT ? "LEFT" : "RIGHT",
      static_cast<long long>(target.lane_id), found_valid_path ? "true" : "false",
      found_safe_path ? "true" : "false", static_safe ? "true" : "false",
      target.lanes_to_preferred);
    if (found_valid_path && (!selected || (safe && !selected->safe))) {
      selected = Evaluation{
        target, std::move(candidate_path), lane_change_debug_, terminal_lane_change_path_, safe};
    }
    // No need to generate the remaining directions once the best-ranked usable path is found.
    if (safe) break;
  }

  if (!target_lane_candidates_.empty()) {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_, 3000,
      "avoidance route-first: candidates=%zu evaluated=%zu left=%s right=%s selected=%s "
      "ready=%s",
      target_lane_candidates_.size(), eligible,
      !left_evaluated ? "not_evaluated"
      : left_safe     ? "safe"
                      : "unsafe",
      !right_evaluated ? "not_evaluated"
      : right_safe     ? "safe"
                       : "unsafe",
      !selected                                       ? "NONE"
      : selected->target.direction == Direction::LEFT ? "LEFT"
                                                      : "RIGHT",
      selected && selected->safe ? "true" : "false");
  }
  if (!selected) {
    lane_change_debug_.valid_paths.clear();
    return;
  }

  // Restore ALL direction-dependent state, not just the path: the last evaluated side may differ
  // from the winner. Approval, turn signals and the next safety check must use the same target.
  if (!selectTargetLane(selected->target)) {
    return;
  }
  lane_change_debug_ = std::move(selected->debug);
  terminal_lane_change_path_ = std::move(selected->terminal_path);
  status_.lane_change_path = std::move(selected->path);
  status_.is_valid_path = true;
  status_.is_safe = selected->safe;
  selected_requires_return_ = selected->safe && selected->target.lanes_to_preferred != 0 &&
                              selected->target.lanes_to_preferred >= current_lanes_to_preferred_;
  if (selected_requires_return_) reserveRouteReturn(status_.lane_change_path);
}

bool AvoidanceByLaneChange::selectTargetLane(const TargetLaneCandidate & candidate)
{
  direction_ = candidate.direction;
  common_data_ptr_->direction = direction_;
  common_data_ptr_->requested_target_lane_id = candidate.lane_id;
  update_lanes(false);
  if (get_target_lanes().empty() || get_target_lanes().front().id() != candidate.lane_id) {
    return false;
  }
  update_filtered_objects();
  update_transient_data(false);
  terminal_lane_change_path_.reset();
  return !isLaneChangeRequired();
}

bool AvoidanceByLaneChange::isStaticObstaclePathSafe(const LaneChangePath & path) const
{
  return NormalLaneChange::isStaticObstaclePathSafe(path);
}

PathSafetyStatus AvoidanceByLaneChange::isApprovedPathSafe() const
{
  if (
    selected_requires_return_ && (!route_return_plan_ || !isRouteReturnClear(*route_return_plan_)))
    return {false, false};
  const auto predicted_safety = NormalLaneChange::isApprovedPathSafe();
  if (!predicted_safety.is_safe || isStaticObstaclePathSafe(status_.lane_change_path)) {
    return predicted_safety;
  }
  return {false, false};
}

AvoidancePlanningData AvoidanceByLaneChange::calcAvoidancePlanningData(
  AvoidanceDebugData & debug) const
{
  AvoidancePlanningData data;

  // reference pose
  data.reference_pose = getEgoPose();

  data.reference_path_rough = prev_module_output_.path;

  const auto resample_interval = avoidance_parameters_->resample_interval_for_planning;
  data.reference_path = utils::resamplePathWithSpline(data.reference_path_rough, resample_interval);

  data.current_lanelets = get_current_lanes();

  fillAvoidanceTargetObjects(data, debug);

  return data;
}

void AvoidanceByLaneChange::fillAvoidanceTargetObjects(
  AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug) const
{
  const auto p = std::dynamic_pointer_cast<AvoidanceParameters>(avoidance_parameters_);

  const auto [object_within_target_lane, object_outside_target_lane] =
    utils::path_safety_checker::separateObjectsByLanelets(
      *planner_data_->dynamic_object, data.current_lanelets,
      [](const auto & obj, const auto & lane, const auto yaw_threshold) {
        return utils::path_safety_checker::isPolygonOverlapLanelet(obj, lane, yaw_threshold);
      });

  // Assume that the maximum allocation for data.other object is the sum of
  // objects_within_target_lane and object_outside_target_lane. The maximum allocation for
  // data.target_objects is equal to object_within_target_lane
  {
    const auto other_objects_size =
      object_within_target_lane.objects.size() + object_outside_target_lane.objects.size();
    data.other_objects.reserve(other_objects_size);
    data.target_objects.reserve(object_within_target_lane.objects.size());
  }

  {
    const auto & objects = object_outside_target_lane.objects;
    std::transform(
      objects.cbegin(), objects.cend(), std::back_inserter(data.other_objects),
      [](const auto & object) {
        ObjectData other_object;
        other_object.object = object;
        other_object.info = ObjectInfo::OUT_OF_TARGET_AREA;
        return other_object;
      });
  }

  ObjectDataArray target_lane_objects;
  target_lane_objects.reserve(object_within_target_lane.objects.size());
  for (const auto & obj : object_within_target_lane.objects) {
    const auto target_lane_object = createObjectData(data, obj);
    if (!target_lane_object) {
      continue;
    }

    // A passed object must not sort ahead of every forward obstacle and suppress the request.
    // It remains visible to the lane-change safety checker through the other-object collection.
    if (target_lane_object->longitudinal <= 0.0) {
      ObjectData other_object = *target_lane_object;
      other_object.info = ObjectInfo::OUT_OF_TARGET_AREA;
      data.other_objects.push_back(std::move(other_object));
      continue;
    }

    target_lane_objects.push_back(*target_lane_object);
  }

  data.target_objects = target_lane_objects;
}

std::optional<ObjectData> AvoidanceByLaneChange::createObjectData(
  const AvoidancePlanningData & data, const PredictedObject & object) const
{
  using autoware::motion_utils::findNearestIndex;
  using autoware_utils::calc_distance2d;
  using autoware_utils::calc_lateral_deviation;
  using boost::geometry::return_centroid;

  const auto p = std::dynamic_pointer_cast<AvoidanceParameters>(avoidance_parameters_);

  const auto & path_points = data.reference_path.points;
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto object_closest_index = findNearestIndex(path_points, object_pose.position);
  const auto object_closest_pose = path_points.at(object_closest_index).point.pose;
  const auto t = utils::getHighestProbLabel(object.classification);
  const auto & object_parameter = avoidance_parameters_->object_parameters.at(t);

  ObjectData object_data{};
  // Calc lateral deviation from path to target object.
  object_data.to_centerline =
    autoware::experimental::lanelet2_utils::get_arc_coordinates(data.current_lanelets, object_pose)
      .distance;

  object_data.object = object;

  const auto lower = p->lower_distance_for_polygon_expansion;
  const auto upper = p->upper_distance_for_polygon_expansion;
  const auto clamp =
    std::clamp(calc_distance2d(getEgoPose(), object_pose) - lower, 0.0, upper) / upper;
  object_data.distance_factor = object_parameter.max_expand_ratio * clamp + 1.0;

  // Calc envelop polygon.
  utils::static_obstacle_avoidance::fillObjectEnvelopePolygon(
    object_data, registered_objects_, object_closest_pose, p);

  // calc object centroid.
  object_data.centroid = return_centroid<Point2d>(object_data.envelope_poly);

  // Calc moving time.
  utils::static_obstacle_avoidance::fillObjectMovingTime(object_data, stopped_objects_, p);

  object_data.direction = calc_lateral_deviation(object_closest_pose, object_pose.position) > 0.0
                            ? Direction::LEFT
                            : Direction::RIGHT;
  object_data.preferred_direction = object_data.direction;

  // Find the footprint point closest to the path, set to object_data.overhang_distance.
  object_data.overhang_points = utils::static_obstacle_avoidance::calcEnvelopeOverhangDistance(
    object_data, data.reference_path,
    planner_data_->parameters.vehicle_info.wheel_base_m +
      planner_data_->parameters.vehicle_info.front_overhang_m,
    planner_data_->parameters.vehicle_info.rear_overhang_m);

  // Check whether the the ego should avoid the object.
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;
  utils::static_obstacle_avoidance::fillAvoidanceNecessity(
    object_data, registered_objects_, vehicle_width, p);

  utils::static_obstacle_avoidance::fillLongitudinalAndLengthByClosestEnvelopeFootprint(
    data.reference_path_rough, getEgoPosition(), object_data);
  return object_data;
}

double AvoidanceByLaneChange::calcMinAvoidanceLength(const ObjectData & nearest_object) const
{
  const auto ego_width = getCommonParam().vehicle_width;
  const auto nearest_object_type = utils::getHighestProbLabel(nearest_object.object.classification);
  const auto nearest_object_parameter =
    avoidance_parameters_->object_parameters.at(nearest_object_type);
  const auto lateral_hard_margin = std::max(
    nearest_object_parameter.lateral_hard_margin,
    nearest_object_parameter.lateral_hard_margin_for_parked_vehicle);
  const auto avoid_margin = lateral_hard_margin * nearest_object.distance_factor +
                            nearest_object_parameter.lateral_soft_margin + 0.5 * ego_width;

  avoidance_helper_->setData(planner_data_);
  const auto shift_length = avoidance_helper_->getShiftLength(
    nearest_object, utils::static_obstacle_avoidance::isOnRight(nearest_object), avoid_margin);

  return avoidance_helper_->getMinAvoidanceDistance(shift_length);
}

double AvoidanceByLaneChange::calc_minimum_dist_buffer() const
{
  const auto [_, dist_buffer] = utils::lane_change::calculation::calc_lc_length_and_dist_buffer(
    common_data_ptr_, get_current_lanes());
  return dist_buffer.min;
}

double AvoidanceByLaneChange::calcLateralOffset() const
{
  auto additional_lat_offset{0.0};
  for (const auto & [type, p] : avoidance_parameters_->object_parameters) {
    const auto lateral_hard_margin =
      std::max(p.lateral_hard_margin, p.lateral_hard_margin_for_parked_vehicle);
    const auto offset =
      2.0 * p.envelope_buffer_margin + lateral_hard_margin + p.lateral_soft_margin;
    additional_lat_offset = std::max(additional_lat_offset, offset);
  }
  return additional_lat_offset;
}
}  // namespace autoware::behavior_path_planner
