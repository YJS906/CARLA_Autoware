// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "local_velocity_profile.hpp"

#include "autoware/behavior_path_static_obstacle_avoidance_module/scene.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/static_collision.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace utils::static_obstacle_avoidance
{
std::vector<utils::path_safety_checker::PoseWithVelocityStamped> predictManeuverPath(
  const PathWithLaneId & path, const Pose & ego_pose, double velocity, const double horizon,
  const double resolution, const double max_acceleration, const double min_acceleration)
{
  using autoware::motion_utils::calcInterpolatedPose;
  std::vector<utils::path_safety_checker::PoseWithVelocityStamped> result;
  if (
    path.points.size() < 2 || resolution <= 0.0 || horizon <= 0.0 || max_acceleration <= 0.0 ||
    min_acceleration >= 0.0)
    return result;
  std::vector<double> arcs(path.points.size(), 0.0);
  for (size_t i = 1; i < arcs.size(); ++i) {
    arcs[i] = arcs[i - 1] + autoware_utils::calc_distance2d(path.points[i - 1], path.points[i]);
  }
  double arc = std::max(
    0.0, autoware::motion_utils::calcSignedArcLength(
           path.points, path.points.front().point.pose.position, ego_pose.position));
  double stop = arcs.back();
  for (size_t i = 0; i < arcs.size(); ++i) {
    if (arcs[i] >= arc && path.points[i].point.longitudinal_velocity_mps <= 1e-3) {
      stop = arcs[i];
      break;
    }
  }
  double time = 0.0;
  for (double sample_time = 0.0; sample_time <= horizon; sample_time += resolution) {
    while (time + 1e-6 < sample_time) {
      const double dt = std::min(0.05, sample_time - time);
      const auto it = std::upper_bound(arcs.begin(), arcs.end(), arc);
      const auto next = std::min<size_t>(std::distance(arcs.begin(), it), arcs.size() - 1);
      const auto prev = next > 0 ? next - 1 : 0;
      const double ratio =
        next == prev ? 0.0 : std::clamp((arc - arcs[prev]) / (arcs[next] - arcs[prev]), 0.0, 1.0);
      const double target = (1.0 - ratio) * path.points[prev].point.longitudinal_velocity_mps +
                            ratio * path.points[next].point.longitudinal_velocity_mps;
      const double next_velocity = std::max(
        0.0,
        velocity + std::clamp(target - velocity, min_acceleration * dt, max_acceleration * dt));
      arc += 0.5 * (velocity + next_velocity) * dt;
      velocity = next_velocity;
      if (arc >= stop) {
        arc = stop;
        velocity = 0.0;
      }
      time += dt;
    }
    result.emplace_back(sample_time, calcInterpolatedPose(path.points, arc), velocity);
  }
  return result;
}
}  // namespace utils::static_obstacle_avoidance

bool StaticObstacleAvoidanceModule::prepareManeuverPath(
  PathWithLaneId & path, const ShiftLineArray & lines) const
{
  if (path.points.size() < 3 || !planner_data_->dynamic_object) return false;
  const auto & vehicle = planner_data_->parameters.vehicle_info;
  const auto & safety = parameters_->trajectory_collision;
  const auto ego_pose = getEgoPose();
  const auto ego_motion = selfcar::trajectory_safety::measuredEgoMotion(
    getEgoSpeed(), planner_data_->parameters.max_acc,
    planner_data_->self_acceleration->accel.accel.linear.x);
  const auto & origin = path.points.front().point.pose.position;
  const auto arc_of = [&](const Point & point) {
    return autoware::motion_utils::calcSignedArcLength(path.points, origin, point);
  };
  const double ego_arc = arc_of(getEgoPosition());
  const auto path_pose =
    autoware::motion_utils::calcInterpolatedPose(path.points, std::max(0.0, ego_arc));
  // Use the maximum normalized yaw difference (180 degrees); retain all checks below.
  if (std::abs(autoware_utils::calc_yaw_deviation(path_pose, ego_pose)) > M_PI) return false;
  std::vector<double> arcs(path.points.size(), 0.0);
  for (size_t i = 1; i < path.points.size(); ++i) {
    arcs[i] = arcs[i - 1] + autoware_utils::calc_distance2d(path.points[i - 1], path.points[i]);
    if (!std::isfinite(arcs[i]) || arcs[i] <= arcs[i - 1]) return false;
  }

  // Only the first remaining shift and its local target must finish before the safe stop.
  // A later return/second avoidance is not part of this independently stoppable prefix.
  double first_end = std::numeric_limits<double>::infinity();
  std::vector<std::pair<double, double>> shift_intervals;
  for (const auto & line : lines) {
    if (std::abs(line.end_shift_length - line.start_shift_length) < 1e-3) continue;
    // ShiftLine endpoints live on the unshifted reference, not on the execution path.
    const double start = arc_of(autoware_utils::calc_offset_pose(
      line.start, 0.0, line.start_shift_length, 0.0).position);
    const double end = arc_of(autoware_utils::calc_offset_pose(
      line.end, 0.0, line.end_shift_length, 0.0).position);
    if (!std::isfinite(start) || !std::isfinite(end)) return false;
    if (end <= ego_arc) continue;
    if (end <= start) return false;
    shift_intervals.emplace_back(std::max(ego_arc, start), end);
    first_end = std::min(first_end, end);
  }
  double required_end = std::isfinite(first_end) ? first_end : ego_arc;
  double nearest_target_begin = std::numeric_limits<double>::infinity();
  double nearest_target_clear = ego_arc;
  for (const auto & object : avoid_data_.target_objects) {
    if (!object.avoid_required) continue;
    if (object.envelope_poly.outer().size() < 4) return false;

    // longitudinal is the NEAR edge of the buffered envelope, not its center or far edge.
    // It can be negative while ego is still alongside the object. Project the full envelope
    // onto this execution path, keeping target extent and shift/stop arcs in one coordinate.
    double target_begin = std::numeric_limits<double>::infinity();
    double target_end = -std::numeric_limits<double>::infinity();
    for (const auto & vertex : object.envelope_poly.outer()) {
      if (!std::isfinite(vertex.x()) || !std::isfinite(vertex.y())) return false;
      const double arc = arc_of(autoware_utils::create_point(vertex.x(), vertex.y(), 0.0));
      if (!std::isfinite(arc)) return false;
      target_begin = std::min(target_begin, arc);
      target_end = std::max(target_end, arc);
    }
    const double target_clear = target_end + vehicle.rear_overhang_m;
    if (target_clear <= ego_arc) continue;  // Entire buffered object has passed ego's rear.

    // Associate targets with the first remaining shift, not just the next object in the
    // perception list. An outbound shift may finish before the object by the generator's
    // existing front-clearance distance. Once that object is passed, a return shift must
    // not inherit the completion requirement of an unrelated distant avoidance.
    const double front_clearance = helper_->getFrontConstantDistance(object);
    if (!std::isfinite(front_clearance) || front_clearance < 0.0) return false;
    if (std::isfinite(first_end) && target_begin > first_end + front_clearance) continue;

    if (target_begin < nearest_target_begin) {
      nearest_target_begin = target_begin;
      nearest_target_clear = target_clear;
    }
  }
  required_end = std::max(required_end, nearest_target_clear);

  // Preserve the incoming speed profile. The nominal avoidance speed is a local
  // shift ceiling, never a ceiling on the straight approach or the whole path.
  const double nominal_velocity =
    std::min(parameters_->nominal_avoidance_speed, helper_->getAvoidanceEgoSpeed());
  const double lateral_acc = helper_->getLateralMaxAccelLimit();
  const double lateral_jerk = helper_->getLateralMaxJerkLimit();
  const double max_curvature = vehicle.calcMaxCurvature();
  const double max_acceleration =
    std::min(parameters_->max_acceleration, planner_data_->parameters.max_acc);
  const double min_acceleration =
    std::max(parameters_->max_deceleration, planner_data_->parameters.min_acc);
  if (
    !std::isfinite(nominal_velocity) || nominal_velocity <= 0.0 ||
    !std::isfinite(lateral_acc) || lateral_acc <= 0.0 ||
    !std::isfinite(lateral_jerk) || lateral_jerk <= 0.0 ||
    !std::isfinite(max_curvature) || max_curvature <= 0.0 ||
    !std::isfinite(max_acceleration) || max_acceleration <= 0.0 ||
    !std::isfinite(min_acceleration) || min_acceleration >= 0.0 ||
    !std::isfinite(safety.max_jerk) || safety.max_jerk <= 0.0 ||
    !std::isfinite(safety.min_jerk) || safety.min_jerk >= 0.0)
    return false;
  std::vector<double> original;
  original.reserve(path.points.size());
  for (const auto & point : path.points) {
    const double velocity = point.point.longitudinal_velocity_mps;
    if (!std::isfinite(velocity) || velocity < 0.0) return false;
    original.push_back(velocity);
  }

  const auto collision = utils::static_obstacle_avoidance::checkStaticObstacleCollision(
    path, *planner_data_->dynamic_object, vehicle, *parameters_, ego_arc, arcs.back(), &ego_pose,
    ego_motion);
  if (!collision.valid) return false;
  double stop_arc = collision.object_id ? collision.collision_arc - safety.stop_margin
                                        : std::numeric_limits<double>::infinity();
  // Preserve a red light, goal stop, or an earlier planner's stop. Never raise a zero speed.
  for (size_t i = 0; i < path.points.size(); ++i) {
    if (arcs[i] >= ego_arc && path.points[i].point.longitudinal_velocity_mps <= 1e-3) {
      stop_arc = std::min(stop_arc, arcs[i]);
      break;
    }
  }
  // Use the preceding sample: this keeps ShiftedPath's index/shift arrays aligned and only
  // increases clearance. Candidate and resampled execution paths both run this calculation.
  if (std::isfinite(stop_arc)) {
    const auto it = std::upper_bound(arcs.begin(), arcs.end(), stop_arc);
    if (it == arcs.begin()) return false;
    stop_arc = *std::prev(it);
  }
  const double checked_end = std::min(stop_arc, arcs.back());
  if (checked_end <= ego_arc || required_end > checked_end) {
    RCLCPP_DEBUG(
      getLogger(), "static avoidance prefix rejected: shift_end=%.3f target_clear=%.3f stop=%.3f",
      std::isfinite(first_end) ? first_end - ego_arc : 0.0, nearest_target_clear - ego_arc,
      checked_end - ego_arc);
    return false;
  }

  const size_t begin = std::min<size_t>(
    std::distance(arcs.begin(), std::lower_bound(arcs.begin(), arcs.end(), ego_arc)),
    arcs.size() - 1);
  // Geometry caps are independent of already-modified point speeds. Otherwise a
  // second validation would spread an earlier low speed/stop to adjacent samples.
  const double reference_max = *std::max_element(original.begin(), original.end());
  std::vector<double> local(original.size(), std::numeric_limits<double>::infinity());
  for (const auto & [start, end] : shift_intervals) {
    const auto first = std::upper_bound(arcs.begin(), arcs.end(), start);
    const size_t from = std::max(
      begin, first == arcs.begin() ? size_t{0} :
      static_cast<size_t>(std::distance(arcs.begin(), first) - 1));
    const size_t to = std::min<size_t>(
      std::distance(arcs.begin(), std::lower_bound(arcs.begin(), arcs.end(), end)),
      arcs.size() - 1);
    for (size_t i = from; i <= to; ++i) local[i] = std::min(local[i], nominal_velocity);
  }
  auto limits = local;
  double previous_curvature = 0.0;
  bool have_previous = false;
  for (size_t i = 1; i + 1 < path.points.size(); ++i) {
    if (arcs[i + 1] < ego_arc) continue;
    if (arcs[i - 1] > checked_end) break;
    const double curvature = autoware_utils::calc_curvature(
      path.points[i - 1].point.pose.position, path.points[i].point.pose.position,
      path.points[i + 1].point.pose.position);
    // Hardware steering is NEVER multiplied by the 110% planning allowance.
    if (!std::isfinite(curvature) || std::abs(curvature) > max_curvature) return false;
    // Reset for every geometric sample; a tighter preceding curve must not impose
    // its speed on a following straight or an unrelated future shift.
    double velocity = local[i];
    if (std::abs(curvature) > 1e-6) {
      velocity = std::min(velocity, std::sqrt(lateral_acc / std::abs(curvature)));
    }
    if (have_previous) {
      const double derivative =
        std::abs(curvature - previous_curvature) / (arcs[i] - arcs[i - 1]);
      const double longitudinal_acc =
        std::max(parameters_->max_acceleration, std::abs(planner_data_->parameters.min_acc));
      const auto jerk_at = [&](const double speed) {
        return speed * speed * speed * derivative +
               2.0 * speed * longitudinal_acc * std::abs(curvature);
      };
      if (derivative > 1e-12) {
        velocity = std::min(velocity, std::cbrt(lateral_jerk / derivative));
      }
      if (std::abs(curvature) > 1e-12) {
        velocity = std::min(
          velocity, lateral_jerk / (2.0 * longitudinal_acc * std::abs(curvature)));
      }
      if (std::isfinite(velocity) && jerk_at(velocity) > lateral_jerk) {
        double low = 0.0;
        double high = velocity;
        for (size_t iteration = 0; iteration < 20; ++iteration) {
          const double mid = 0.5 * (low + high);
          if (jerk_at(mid) <= lateral_jerk)
            low = mid;
          else
            high = mid;
        }
        velocity = low;
      }
    }
    // Cover the segments supporting the three-point curvature and its derivative.
    for (size_t j = std::max(begin, i - 1); j <= i + 1; ++j) {
      limits[j] = std::min(limits[j], velocity);
    }
    previous_curvature = curvature;
    have_previous = true;
  }
  for (size_t i = begin; i < arcs.size(); ++i) {
    if (arcs[i] >= stop_arc) limits[i] = 0.0;
  }

  if (std::isfinite(stop_arc)) {
    const auto ego_stop = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
      getEgoSpeed(), 0.0, planner_data_->self_acceleration->accel.accel.linear.x,
      min_acceleration, safety.max_jerk, safety.min_jerk);
    if (!ego_stop || !std::isfinite(*ego_stop) || *ego_stop > stop_arc - ego_arc)
      return false;
  }

  // Check reachable local constraints against measured speed/acceleration, rather
  // than requiring the slowest future speed before the first shift starts.
  for (size_t i = begin; i < arcs.size() && arcs[i] <= checked_end; ++i) {
    if (getEgoSpeed() <= limits[i] + 1e-2) continue;
    const auto distance = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
      getEgoSpeed(), limits[i], planner_data_->self_acceleration->accel.accel.linear.x,
      min_acceleration, safety.max_jerk, safety.min_jerk);
    if (!distance || !std::isfinite(*distance) || *distance > arcs[i] - ego_arc)
      return false;
  }
  const auto braking_distance = [&](const double high, const double low) {
    if (high <= low) return 0.0;
    const auto distance = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
      high, low, max_acceleration, min_acceleration, safety.max_jerk, safety.min_jerk);
    return distance && std::isfinite(*distance) ? *distance
                                                : std::numeric_limits<double>::infinity();
  };
  const auto recovery_distance = [&](const double high, const double low) {
    if (high <= low) return 0.0;
    // Time reversal of zero-acceleration-endpoint braking gives acceleration with
    // the same jerk bounds. This is a speed envelope, not a replacement smoother.
    const auto distance = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
      high, low, 0.0, -max_acceleration, safety.max_jerk, safety.min_jerk);
    return distance && std::isfinite(*distance) ? *distance
                                                : std::numeric_limits<double>::infinity();
  };
  auto braking_limits = limits;
  auto recovery_limits = limits;
  if (
    !detail::extendLocalVelocityLimits(
      arcs, braking_limits, reference_max, braking_distance, true, begin) ||
    !detail::extendLocalVelocityLimits(
      arcs, recovery_limits, reference_max, recovery_distance, false, begin))
    return false;
  for (size_t i = 0; i < limits.size(); ++i) {
    limits[i] = std::min({original[i], braking_limits[i], recovery_limits[i]});
  }

  if (std::isfinite(stop_arc)) {
    const auto next = std::lower_bound(arcs.begin(), arcs.end(), required_end);
    const size_t hi = std::min<size_t>(std::distance(arcs.begin(), next), arcs.size() - 1);
    const size_t lo = hi > 0 ? hi - 1 : 0;
    const double ratio = hi == lo ? 0.0 :
      std::clamp((required_end - arcs[lo]) / (arcs[hi] - arcs[lo]), 0.0, 1.0);
    const double completion_velocity = (1.0 - ratio) * limits[lo] + ratio * limits[hi];
    if (required_end + braking_distance(completion_velocity, 0.0) > stop_arc + 1e-3)
      return false;
  }
  for (size_t i = begin; i < path.points.size(); ++i) {
    path.points[i].point.longitudinal_velocity_mps = std::min(original[i], limits[i]);
  }
  // Recheck the complete reachable prefix, with its final speed/current-pose swept polygons.
  return utils::static_obstacle_avoidance::checkStaticObstacleCollision(
           path, *planner_data_->dynamic_object, vehicle, *parameters_, ego_arc, checked_end,
           &ego_pose, ego_motion)
    .is_safe();
}
}  // namespace autoware::behavior_path_planner
