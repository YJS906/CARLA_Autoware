// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "scene.hpp"

#include <autoware/behavior_path_lane_change_module/utils/utils.hpp>
#include <autoware/behavior_path_planner_common/utils/traffic_light_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

namespace autoware::behavior_path_planner
{
namespace
{
// A read-only, private planning context. Never change the live ego pose, route, RTC or objects.
// This proves a geometrically executable return with stationary-object clearance; moving traffic
// must be checked again by the ordinary lane-change module at the actual return time.
class RouteReturnProbe : public NormalLaneChange
{
public:
  using LaneChangeBase::get_target_lanes;

  explicit RouteReturnProbe(const std::shared_ptr<LaneChangeParameters> & parameters)
  : NormalLaneChange(parameters, LaneChangeModuleType::NORMAL, Direction::NONE)
  {
  }

  bool plan(
    const std::shared_ptr<PlannerData> & data, const PathWithLaneId & reference,
    LaneChangePath & result)
  {
    lanelet::ConstLanelet current;
    if (!data->route_handler->getClosestLaneletWithinRoute(
          data->self_odometry->pose.pose, &current)) {
      return false;
    }
    const auto distance = data->route_handler->getNumLaneToPreferredLane(current);
    if (distance == 0) return false;
    direction_ = distance > 0 ? Direction::LEFT : Direction::RIGHT;
    setData(data);
    BehaviorModuleOutput previous;
    previous.path = reference;
    previous.reference_path = reference;
    setPreviousModuleOutput(previous);
    update_lanes(false);
    if (!common_data_ptr_->is_lanes_available()) return false;
    update_filtered_objects();
    update_transient_data(false);
    if (isLaneChangeRequired()) return false;
    const auto [valid, safe] = getSafePath(result);
    return valid && safe && isValidPath(result.path);
  }
};

double arc_at(const PathWithLaneId & path, const Point & point)
{
  return motion_utils::calcSignedArcLength(
    path.points, path.points.front().point.pose.position, point);
}
}  // namespace

void AvoidanceByLaneChange::updateStationaryObservations()
{
  if (!planner_data_ || !planner_data_->dynamic_object) {
    stationary_observations_.clear();
    observation_stamp_.reset();
    return;
  }
  const auto & objects = *planner_data_->dynamic_object;
  const auto stamp = rclcpp::Time(objects.header.stamp).seconds();
  // Repeated/compensated perception samples must not accrue stationary evidence.
  if (observation_stamp_ && stamp == *observation_stamp_) return;
  if (
    !std::isfinite(stamp) || stamp <= 0.0 ||
    (observation_stamp_ && (stamp < *observation_stamp_ || stamp - *observation_stamp_ > 0.5))) {
    stationary_observations_.clear();
  }
  observation_stamp_ = stamp;
  for (const auto & object : objects.objects) {
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto & v = object.kinematics.initial_twist_with_covariance.twist.linear;
    if (
      !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(pose.position.x) ||
      !std::isfinite(pose.position.y))
      continue;
    const bool moving =
      std::hypot(v.x, v.y) > lane_change_parameters_->trajectory_safety.stationary_velocity;
    auto [it, inserted] = stationary_observations_.try_emplace(
      object.object_id.uuid, StationaryObservation{pose.position, stamp, stamp, moving});
    auto & observation = it->second;
    if (
      !inserted && (moving || autoware_utils::calc_distance2d(observation.anchor, pose.position) >
                                avoidance_parameters_->route_blockage_max_position_drift)) {
      observation.anchor = pose.position;
      observation.since = stamp;
      observation.observed_moving = true;
    }
    observation.last_seen = stamp;
  }
  for (auto it = stationary_observations_.begin(); it != stationary_observations_.end();) {
    if (it->second.last_seen != stamp)
      it = stationary_observations_.erase(it);
    else
      ++it;
  }
}

bool AvoidanceByLaneChange::isPersistentBlocker(const unique_identifier_msgs::msg::UUID & id) const
{
  const auto it = stationary_observations_.find(id.uuid);
  if (it == stationary_observations_.end() || !observation_stamp_) return false;
  const auto & observation = it->second;
  const auto data_age =
    rclcpp::Time(planner_data_->self_odometry->header.stamp).seconds() - *observation_stamp_;
  if (!std::isfinite(data_age) || data_age < -0.1 || data_age > 0.5) return false;
  return !observation.observed_moving && observation.last_seen == *observation_stamp_ &&
         observation.last_seen - observation.since >=
           avoidance_parameters_->route_blockage_min_duration;
}

bool AvoidanceByLaneChange::isRouteDepartureRequired() const
{
  const auto * obstacle = getNearestAvoidanceTarget();
  if (
    !obstacle || !common_data_ptr_->is_lanes_available() ||
    !isPersistentBlocker(obstacle->object.object_id) ||
    avoidance_data_.reference_path.points.size() < 2 || !planner_data_->dynamic_object)
    return false;

  // A signal/queue, an unsafe traffic gap or a geometry failure is not an unavoidable detour.
  if (utils::traffic_light::isTrafficSignalStop(avoidance_data_.current_lanelets, planner_data_)) {
    return false;
  }
  for (const auto & lane : avoidance_data_.current_lanelets) {
    if (
      utils::traffic_light::isTrafficSignalStop(
        getRouteHandler()->getNextLanelets(lane), planner_data_))
      return false;
  }
  const auto regulatory_distance =
    utils::lane_change::get_distance_to_next_regulatory_element(common_data_ptr_, false, false);
  if (
    std::isfinite(regulatory_distance) &&
    regulatory_distance <= obstacle->longitudinal + obstacle->length +
                             lane_change_parameters_->trajectory_safety.stop_margin)
    return false;

  const auto ego_arc = arc_at(avoidance_data_.reference_path, getEgoPosition());
  const auto collision = utils::path_safety_checker::checkStaticTrajectory(
    avoidance_data_.reference_path, *planner_data_->dynamic_object, getCommonParam().vehicle_info,
    getEgoPose(), ego_arc, motion_utils::calcArcLength(avoidance_data_.reference_path.points),
    lane_change_parameters_->trajectory_safety);
  if (
    !collision.valid || !collision.object_id ||
    *collision.object_id != obstacle->object.object_id) {
    return false;
  }
  // Do not overtake a stationary queue merely because its first vehicle has not moved recently.
  for (const auto & other : planner_data_->dynamic_object->objects) {
    if (other.object_id == obstacle->object.object_id) continue;
    const auto & pose = other.kinematics.initial_pose_with_covariance.pose;
    const auto distance = arc_at(avoidance_data_.reference_path, pose.position) - ego_arc;
    if (
      distance < obstacle->longitudinal ||
      distance > obstacle->longitudinal + obstacle->length + 20.0)
      continue;
    if (
      std::abs(
        motion_utils::calcLateralOffset(avoidance_data_.reference_path.points, pose.position)) <
      getCommonParam().vehicle_width)
      return false;
  }
  return true;
}

bool AvoidanceByLaneChange::prepareRouteReturn(
  const TargetLaneCandidate & target, LaneChangePath & path)
{
  const auto & header = getRouteHeader();
  const auto velocity = path.info.terminal_lane_changing_velocity;
  const bool matches =
    route_return_plan_ && route_return_plan_->outbound_lane_id == target.lane_id &&
    route_return_plan_->route_header == header &&
    route_return_plan_->outbound_end.orientation == path.info.lane_changing_end.orientation &&
    autoware_utils::calc_distance2d(route_return_plan_->outbound_end, path.info.lane_changing_end) <
      0.1 &&
    std::abs(route_return_plan_->velocity - velocity) < 0.1;
  if (matches && isRouteReturnClear(*route_return_plan_)) return true;
  route_return_plan_.reset();
  const auto now = clock_.now();
  if (
    last_return_search_ && (now - *last_return_search_).seconds() >= 0.0 &&
    (now - *last_return_search_).seconds() < avoidance_parameters_->route_return_search_interval) {
    return false;
  }
  last_return_search_ = now;
  try {
    route_return_plan_ = searchRouteReturn(target, path);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger_, "Route departure denied: return search failed: %s", e.what());
  }
  RCLCPP_INFO_THROTTLE(
    logger_, clock_, 3000, "route-first departure lane=%lld return_plan=%s",
    static_cast<long long>(target.lane_id), route_return_plan_ ? "available" : "unavailable");
  return route_return_plan_.has_value();
}

std::optional<AvoidanceByLaneChange::RouteReturnPlan> AvoidanceByLaneChange::searchRouteReturn(
  const TargetLaneCandidate & target, const LaneChangePath & outbound) const
{
  const auto start_time = std::chrono::steady_clock::now();
  const auto remaining_budget = [&]() {
    return avoidance_parameters_->route_return_time_budget_ms -
           std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
             .count();
  };
  const auto * obstacle = getNearestAvoidanceTarget();
  if (!obstacle || get_target_lanes().empty()) return std::nullopt;
  const auto & safety = lane_change_parameters_->trajectory_safety;
  const double velocity = outbound.info.terminal_lane_changing_velocity;
  if (!std::isfinite(velocity) || velocity <= 0.0) return std::nullopt;
  const auto stopping_distance = motion_utils::calcDecelDistWithJerkAndAccConstraints(
    velocity, 0.0, outbound.info.longitudinal_acceleration.lane_changing,
    std::max(getCommonParam().min_acc, lane_change_parameters_->trajectory.min_longitudinal_acc),
    safety.max_jerk, safety.min_jerk);
  if (!stopping_distance) return std::nullopt;

  auto reference = common_data_ptr_->target_lanes_path;
  if (reference.points.size() < 2) return std::nullopt;
  const double finish_margin = lane_change_parameters_->lane_change_finish_judge_buffer;
  double start_arc = arc_at(reference, outbound.info.lane_changing_end.position) +
                     *stopping_distance + finish_margin;
  // Start the return only after the rear of ego has cleared the entire obstacle envelope.
  for (const auto & point : obstacle->envelope_poly.outer()) {
    Point position;
    position.x = point.x();
    position.y = point.y();
    start_arc = std::max(
      start_arc, arc_at(reference, position) + getCommonParam().vehicle_info.rear_overhang_m +
                   safety.stop_margin);
  }
  start_arc += velocity * avoidance_parameters_->route_return_time_margin;
  if (start_arc >= motion_utils::calcArcLength(reference.points)) return std::nullopt;
  const auto origin = motion_utils::calcInterpolatedPose(reference.points, start_arc);

  const auto clearance = utils::path_safety_checker::checkStaticTrajectory(
    outbound.path, *planner_data_->dynamic_object, getCommonParam().vehicle_info, getEgoPose(),
    arc_at(outbound.path, getEgoPosition()),
    arc_at(outbound.path, origin.position) + safety.stop_margin, safety);
  if (
    !clearance.is_safe() || arc_at(outbound.path, origin.position) + safety.stop_margin >
                              motion_utils::calcArcLength(outbound.path.points))
    return std::nullopt;

  RouteReturnPlan plan{
    target.lane_id, outbound.info.lane_changing_end, origin, velocity, getRouteHeader(), {}};
  auto data = std::make_shared<PlannerData>(*planner_data_);
  auto odometry = std::make_shared<Odometry>(*planner_data_->self_odometry);
  odometry->pose.pose = origin;
  odometry->twist.twist = Twist{};
  odometry->twist.twist.linear.x = velocity;
  data->self_odometry = odometry;
  data->self_acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
  data->parameters.max_vel = std::min(data->parameters.max_vel, velocity);
  auto objects = std::make_shared<PredictedObjects>(*planner_data_->dynamic_object);
  objects->objects.erase(
    std::remove_if(
      objects->objects.begin(), objects->objects.end(),
      [&](const auto & o) {
        const auto & v = o.kinematics.initial_twist_with_covariance.twist.linear;
        return std::isfinite(v.x) && std::isfinite(v.y) &&
               std::hypot(v.x, v.y) > safety.stationary_velocity;
      }),
    objects->objects.end());
  data->dynamic_object = objects;

  int previous_distance = target.lanes_to_preferred;
  for (int step = 0; step < avoidance_parameters_->route_return_max_steps; ++step) {
    if (remaining_budget() <= 0.0) return std::nullopt;
    for (auto & point : reference.points) {
      point.point.longitudinal_velocity_mps =
        std::min(point.point.longitudinal_velocity_mps, static_cast<float>(velocity));
    }
    auto parameters = std::make_shared<LaneChangeParameters>(*lane_change_parameters_);
    parameters->time_limit = std::min(parameters->time_limit, remaining_budget());
    RouteReturnProbe probe(parameters);
    LaneChangePath stage;
    if (!probe.plan(data, reference, stage) || remaining_budget() <= 0.0) return std::nullopt;
    const auto & lanes = probe.get_target_lanes();
    if (lanes.empty()) return std::nullopt;
    const int distance = std::abs(getRouteHandler()->getNumLaneToPreferredLane(lanes.back()));
    if (distance >= previous_distance) return std::nullopt;
    plan.stages.push_back(stage);
    if (distance == 0) {
      const bool clear = isRouteReturnClear(plan);
      return clear && remaining_budget() > 0.0 ? std::make_optional(plan) : std::nullopt;
    }
    previous_distance = distance;
    reference =
      getRouteHandler()->getCenterLinePath(lanes, 0.0, std::numeric_limits<double>::max());
    const auto next_arc = arc_at(reference, stage.info.lane_changing_end.position) + finish_margin;
    if (next_arc >= motion_utils::calcArcLength(reference.points)) return std::nullopt;
    odometry->pose.pose = motion_utils::calcInterpolatedPose(reference.points, next_arc);
    odometry->twist.twist.linear.x = stage.info.terminal_lane_changing_velocity;
  }
  return std::nullopt;
}

bool AvoidanceByLaneChange::isRouteReturnClear(const RouteReturnPlan & plan) const
{
  if (!planner_data_->dynamic_object || plan.stages.empty()) return false;
  const auto & safety = lane_change_parameters_->trajectory_safety;
  for (const auto & stage : plan.stages) {
    if (stage.path.points.size() < 2) return false;
    const auto start = std::max(
      0.0, arc_at(stage.path, stage.info.lane_changing_start.position) - stage.info.length.prepare);
    const auto stopping = motion_utils::calcDecelDistWithJerkAndAccConstraints(
      stage.info.terminal_lane_changing_velocity, 0.0,
      stage.info.longitudinal_acceleration.lane_changing,
      std::max(getCommonParam().min_acc, lane_change_parameters_->trajectory.min_longitudinal_acc),
      safety.max_jerk, safety.min_jerk);
    if (!stopping) return false;
    const auto end =
      arc_at(stage.path, stage.info.lane_changing_end.position) + *stopping + safety.stop_margin;
    if (end > motion_utils::calcArcLength(stage.path.points)) return false;
    if (!utils::path_safety_checker::checkStaticTrajectory(
           stage.path, *planner_data_->dynamic_object, getCommonParam().vehicle_info,
           motion_utils::calcInterpolatedPose(stage.path.points, start), start, end, safety)
           .is_safe()) {
      return false;
    }
  }
  return true;
}

void AvoidanceByLaneChange::reserveRouteReturn(LaneChangePath & path) const
{
  if (!route_return_plan_ || path.path.points.size() < 2) return;
  const auto end = arc_at(path.path, path.info.lane_changing_end.position);
  double arc = 0.0;
  for (size_t i = 0; i < path.path.points.size(); ++i) {
    auto & point = path.path.points[i];
    if (i > 0)
      arc += autoware_utils::calc_distance2d(path.path.points[i - 1].point.pose, point.point.pose);
    if (arc + 1e-3 < end) continue;
    point.point.longitudinal_velocity_mps = std::min(
      point.point.longitudinal_velocity_mps, static_cast<float>(route_return_plan_->velocity));
  }
  utils::insertStopPoint(arc_at(path.path, route_return_plan_->start.position), path.path);
}

BehaviorModuleOutput AvoidanceByLaneChange::generateOutput()
{
  auto output = NormalLaneChange::generateOutput();
  if (selected_requires_return_ && route_return_plan_ && output.path.points.size() >= 2) {
    // Do not consume the reserved return window while finishing/handing off the outbound shift.
    set_stop_pose(
      arc_at(output.path, route_return_plan_->start.position), output.path,
      "reserved mission-lane return");
  }
  return output;
}

bool AvoidanceByLaneChange::updateApprovedPath()
{
  if (!selected_requires_return_) return NormalLaneChange::updateApprovedPath();
  return_handoff_ready_ = false;
  // Keep ownership of the capped departure/stop path until there is an executable return from
  // the real stopped pose. Otherwise finishing the departure discards its speed/stop reserve
  // before the ordinary lane-change module can acquire a safe return candidate.
  if (
    status_.is_valid_path && NormalLaneChange::hasFinishedLaneChange() &&
    std::abs(getEgoVelocity()) <= 0.1 && route_return_plan_) {
    const auto now = clock_.now();
    const auto age = last_return_handoff_check_ ? (now - *last_return_handoff_check_).seconds()
                                                : std::numeric_limits<double>::infinity();
    if (age < 0.0 || age >= avoidance_parameters_->route_return_search_interval) {
      last_return_handoff_check_ = now;
      try {
        auto parameters = std::make_shared<LaneChangeParameters>(*lane_change_parameters_);
        parameters->time_limit =
          std::min(parameters->time_limit, avoidance_parameters_->route_return_time_budget_ms);
        RouteReturnProbe probe(parameters);
        LaneChangePath candidate;
        // Unlike the preflight projection, this uses ALL current objects and the actual ego
        // speed/acceleration. No future-traffic assumption and no inherited RTC approval.
        auto data = std::make_shared<PlannerData>(*planner_data_);
        auto reference = getRouteHandler()->getCenterLinePath(
          get_target_lanes(), 0.0, std::numeric_limits<double>::max());
        return_handoff_ready_ =
          probe.plan(data, reference, candidate) && !probe.get_target_lanes().empty() &&
          std::abs(getRouteHandler()->getNumLaneToPreferredLane(probe.get_target_lanes().back())) <
            std::abs(getRouteHandler()->getNumLaneToPreferredLane(get_target_lanes().back())) &&
          isRouteReturnClear(*route_return_plan_);
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(logger_, "Waiting for live route return: %s", e.what());
      }
    }
    if (return_handoff_ready_) return false;
  }
  const auto previous_status = status_;
  const auto previous_return = route_return_plan_;
  if (!NormalLaneChange::updateApprovedPath()) return false;
  // A stopped-pose recovery may consume more road than the original outbound path. It must not
  // silently invalidate the return reservation that justified leaving the mission corridor.
  const auto & lanes = get_target_lanes();
  if (!lanes.empty()) {
    TargetLaneCandidate target{
      direction_, lanes.front().id(),
      std::abs(getRouteHandler()->getNumLaneToPreferredLane(lanes.front())), 1};
    if (prepareRouteReturn(target, status_.lane_change_path)) {
      reserveRouteReturn(status_.lane_change_path);
      return true;
    }
  }
  status_ = previous_status;
  route_return_plan_ = previous_return;
  approved_path_blocked_ = true;
  toStopState();
  return false;
}

void AvoidanceByLaneChange::resetParameters()
{
  NormalLaneChange::resetParameters();
  selected_requires_return_ = false;
  route_return_plan_.reset();
  return_handoff_ready_ = false;
  last_return_handoff_check_.reset();
  // Keep perception evidence and the search rate limit across waiting-candidate resets.
}

bool AvoidanceByLaneChange::hasFinishedLaneChange() const
{
  return NormalLaneChange::hasFinishedLaneChange() &&
         (!selected_requires_return_ || return_handoff_ready_);
}
}  // namespace autoware::behavior_path_planner
