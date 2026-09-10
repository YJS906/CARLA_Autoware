// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace autoware::behavior_path_planner
{
namespace calculation = utils::lane_change::calculation;
std::optional<LaneChangePhaseMetrics> NormalLaneChange::make_braking_prepare_metric(
  const double target, const double deceleration_scale) const
{
  const auto & c = *common_data_ptr_;
  const auto & p = *lane_change_parameters_;
  if (!c.current_acceleration) return std::nullopt;
  const auto profile = lane_change::make_braking_profile(
    c.get_ego_speed(), c.get_current_accel(), target,
    -std::max(c.bpp_param_ptr->min_acc, p.trajectory.min_longitudinal_acc) * deceleration_scale,
    p.trajectory_safety.max_jerk, p.trajectory_safety.min_jerk, p.trajectory.min_prepare_duration);
  if (!profile) return std::nullopt;
  const double available =
    std::min(c.transient_data.dist_to_terminal_start, c.transient_data.distance_to_static_obstacle);
  // A finite one-metre settling segment follows braking before lateral motion starts.
  const double length = std::max(
    profile->distance() + 1.0, calculation::calc_ego_dist_to_lanes_start(
                                 common_data_ptr_, get_current_lanes(), get_target_lanes()));
  if (length > available) return std::nullopt;
  const double duration = profile->time_at_distance(length);
  const double acc = (target - c.get_ego_speed()) / duration;
  LaneChangePhaseMetrics metric(duration, length, target, acc, acc, 0.0);
  metric.braking_profile = profile;
  return metric;
}

std::optional<double> NormalLaneChange::curvature_speed_limit(const LaneChangePath & path) const
{
  const auto & points = path.path.points;
  if (points.size() < 3) return std::nullopt;
  const auto & vehicle = planner_data_->parameters.vehicle_info;
  const auto ego = motion_utils::findNearestSegmentIndex(points, getEgoPosition());
  const auto start =
    motion_utils::findNearestSegmentIndex(points, path.info.lane_changing_start.position);
  const auto end =
    motion_utils::findNearestSegmentIndex(points, path.info.lane_changing_end.position);
  double cap =
    std::min(path.info.velocity.lane_changing, path.info.terminal_lane_changing_velocity);
  double previous = 0.0;
  bool have_previous = false;
  const auto & trajectory = lane_change_parameters_->trajectory;
  for (size_t i = std::max<size_t>(1, ego); i + 1 < points.size(); ++i) {
    const double k = autoware_utils::calc_curvature(
      points[i - 1].point.pose.position, points[i].point.pose.position,
      points[i + 1].point.pose.position);
    if (!std::isfinite(k) || std::abs(k) > vehicle.calcMaxCurvature()) return std::nullopt;
    if (i >= start && i <= end &&
        (trajectory.enable_lateral_acceleration_limit || trajectory.enable_lateral_jerk_limit)) {
      const double ds = autoware_utils::calc_distance2d(points[i - 1], points[i]);
      if (ds < 1e-6) return std::nullopt;
      const double dk = have_previous ? (k - previous) / ds : 0.0;
      const double acceleration = calculation::calc_path_longitudinal_acceleration(path, i);
      if (!std::isfinite(acceleration)) return std::nullopt;
      double low = 0.0;
      double high = cap;
      for (int iteration = 0; iteration < 24; ++iteration) {
        const double v = 0.5 * (low + high);
        const double lateral_acc = lane_change_parameters_->trajectory.lat_acc_map.find(v).second;
        if (
          (!trajectory.enable_lateral_acceleration_limit || v * v * std::abs(k) <= lateral_acc) &&
          (!trajectory.enable_lateral_jerk_limit ||
           // A conservative monotonic envelope is required by binary search. Phase-specific
           // acceleration prevents preparation acceleration leaking into a constant-speed shift.
           v * v * v * std::abs(dk) + 2.0 * v * std::abs(acceleration * k) <=
             trajectory.lateral_jerk))
          low = v;
        else
          high = v;
      }
      cap = low;
    }
    previous = k;
    have_previous = true;
  }
  return std::isfinite(cap) && cap > 0.0 ? std::optional<double>{cap} : std::nullopt;
}

bool NormalLaneChange::adapt_candidate_speed(
  LaneChangePath & path, const lane_change::TargetObjects & objects) const
try {
  const auto cap = curvature_speed_limit(path);
  if (!cap || *cap < 0.1 || !planner_data_->dynamic_object) return false;
  const auto & c = *common_data_ptr_;
  const auto & p = *lane_change_parameters_;
  const auto & points = path.path.points;
  const auto arc_of = [&](const auto & position) {
    return motion_utils::calcSignedArcLength(points, points.front().point.pose.position, position);
  };
  const double origin = arc_of(getEgoPosition());
  const double shift_start = std::max(0.0, arc_of(path.info.lane_changing_start.position) - origin);
  const double shift_end = arc_of(path.info.lane_changing_end.position) - origin;
  if (shift_end <= 0.0) return false;

  // Establish geometric clearance independently of the speed-dependent post-merge stopping
  // reserve. A speed hint does NOT certify a lateral path, or bypass an occupied footprint.
  const auto clearance = utils::path_safety_checker::checkStaticTrajectory(
    path.path, *planner_data_->dynamic_object, planner_data_->parameters.vehicle_info, getEgoPose(),
    origin, origin + shift_end, p.trajectory_safety,
    selfcar::trajectory_safety::measuredEgoMotion(
      c.get_ego_speed(), c.bpp_param_ptr->max_acc, c.get_current_accel()));
  if (!clearance.is_safe()) return false;
  if (*cap + 0.05 < c.get_ego_speed()) {
    path.info.speed_preparation_target = std::max(0.1, 0.95 * *cap);
  }

  // Retiming keeps the candidate's checked geometry instead of moving its shift start
  // downstream every time a lower speed is sampled. Recheck REAL measured-speed prediction,
  // acceleration/jerk, footprint, target bounds and RSS for every retimed proposal.
  autoware_utils::StopWatch<std::chrono::milliseconds> timer;
  double previous_target = -1.0;
  for (const double scale : {0.95, 0.65, 0.4}) {
    if (timer.toc() > std::min(8.0, p.time_limit * 0.25)) break;
    const double target = std::max(0.1, std::min(*cap, c.get_ego_speed()) * scale);
    if (target >= c.get_ego_speed() - 0.05 || std::abs(target - previous_target) < 0.05) continue;
    previous_target = target;
    const auto profile = lane_change::make_braking_profile(
      c.get_ego_speed(), c.get_current_accel(), target,
      -std::max(c.bpp_param_ptr->min_acc, p.trajectory.min_longitudinal_acc),
      p.trajectory_safety.max_jerk, p.trajectory_safety.min_jerk,
      p.trajectory.min_prepare_duration);
    if (!profile || profile->distance() > shift_end) continue;
    auto candidate = path;
    auto & info = candidate.info;
    info.braking_profile = profile;
    info.braking_start_pose = getEgoPose();
    info.duration = {
      profile->time_at_distance(shift_start),
      profile->time_at_distance(shift_end) - profile->time_at_distance(shift_start)};
    info.velocity = {profile->at_distance(shift_start).velocity, target};
    info.terminal_lane_changing_velocity = target;
    info.longitudinal_acceleration = {
      (info.velocity.prepare - c.get_ego_speed()) / std::max(0.01, info.duration.prepare), 0.0};
    bool respects_existing_limits = true;
    const auto apply = [&](auto & path_points) {
      for (auto & point : path_points) {
        const double distance = arc_of(point.point.pose.position) - origin;
        if (distance < 0.0) continue;
        if (
          distance <= shift_end &&
          point.point.longitudinal_velocity_mps + 0.05 < profile->at_distance(distance).velocity) {
          respects_existing_limits = false;
        }
        point.point.longitudinal_velocity_mps = std::min<double>(
          point.point.longitudinal_velocity_mps, profile->at_distance(distance).velocity);
      }
    };
    apply(candidate.path.points);
    apply(candidate.shifted_path.path.points);
    if (!respects_existing_limits) continue;
    try {
      if (!check_candidate_path_safety(candidate, objects)) continue;
    } catch (const std::exception &) {
      continue;
    }
    candidate.info.speed_preparation_target = target;
    path = std::move(candidate);
    RCLCPP_INFO_THROTTLE(
      logger_, clock_, 1000, "speed adaptation: same geometry safe after braking %.2f -> %.2f m/s",
      c.get_ego_speed(), target);
    return true;
  }
  return false;
} catch (const std::exception &) {
  return false;
}

bool NormalLaneChange::hasSpeedPreparationRequest() const
{
  if (!status_.is_valid_path || get_target_lanes().empty() || isAbortState()) {
    speed_preparation_target_.reset();
    speed_preparation_profile_.reset();
    return false;
  }
  const auto lane_id = get_target_lanes().front().id();
  if (speed_preparation_lane_id_ != lane_id) {
    speed_preparation_target_.reset();
    speed_preparation_profile_.reset();
  }
  speed_preparation_lane_id_ = lane_id;
  const auto & info = status_.lane_change_path.info;
  auto target = info.speed_preparation_target;
  if (status_.is_safe && info.braking_profile) target = info.braking_profile->target_velocity;
  if (target && std::isfinite(*target) && *target > 0.0) {
    speed_preparation_target_ =
      speed_preparation_target_ ? std::min(*target, *speed_preparation_target_) : *target;
  }
  return speed_preparation_target_.has_value();
}

BehaviorModuleOutput NormalLaneChange::braking_wait_output() const
{
  // Execute ONLY longitudinal preparation on the current-lane output. Unsafe lateral
  // geometry stays a candidate and RTC stays not-ready until the full maneuver is safe.
  auto output = prev_module_output_;
  if (!hasSpeedPreparationRequest() || output.path.points.size() < 2) return output;
  const auto & c = *common_data_ptr_;
  const auto & p = *lane_change_parameters_;
  const double target = *speed_preparation_target_;
  const auto stop = [&](const char * reason) {
    for (auto & point : output.path.points) point.point.longitudinal_velocity_mps = 0.0;
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000, "speed preparation: stop requested (%s)", reason);
    return output;
  };
  const double origin = motion_utils::calcSignedArcLength(
    output.path.points, output.path.points.front().point.pose.position, getEgoPosition());
  // Anchor the profile once. Recreating its 0.2 s latency at ego on every frame would
  // move the braking start forwards forever while the vehicle keeps driving.
  if (!speed_preparation_profile_ || speed_preparation_profile_->target_velocity > target + 0.05) {
    speed_preparation_profile_ = lane_change::make_braking_profile(
      c.get_ego_speed(), c.get_current_accel(), target,
      -std::max(c.bpp_param_ptr->min_acc, p.trajectory.min_longitudinal_acc),
      p.trajectory_safety.max_jerk, p.trajectory_safety.min_jerk,
      p.trajectory.min_prepare_duration);
    speed_preparation_start_ = getEgoPose();
  }
  const auto & profile = speed_preparation_profile_;
  const double profile_origin = motion_utils::calcSignedArcLength(
    output.path.points, output.path.points.front().point.pose.position,
    speed_preparation_start_.position);
  if (!profile && c.get_ego_speed() > target + 0.01) {
    return stop("no physically reachable longitudinal profile");
  }
  if (profile && profile_origin + profile->distance() > origin + 0.1) {
    double remaining = std::min(
      {motion_utils::calcArcLength(output.path.points) - origin,
       c.transient_data.dist_to_terminal_start, c.transient_data.distance_to_static_obstacle});
    for (const auto & point : output.path.points) {
      if (point.point.longitudinal_velocity_mps > 0.0) continue;
      const double arc = motion_utils::calcSignedArcLength(
        output.path.points, output.path.points.front().point.pose.position,
        point.point.pose.position);
      if (arc >= origin) remaining = std::min(remaining, arc - origin);
    }
    if (
      profile_origin + profile->distance() - origin + p.trajectory_safety.stop_margin > remaining) {
      return stop("insufficient braking room; do not continue at the original speed");
    }
    LaneChangePath preparation;
    preparation.path = output.path;
    preparation.info.braking_profile = profile;
    preparation.info.braking_start_pose = speed_preparation_start_;
    preparation.info.lane_changing_start = getEgoPose();
    preparation.info.lane_changing_end =
      motion_utils::calcInterpolatedPose(output.path.points, profile_origin + profile->distance());
    preparation.info.duration = {profile->duration(), 0.0};
    const auto predictions =
      utils::lane_change::convert_to_predicted_paths(common_data_ptr_, preparation, 1);
    CollisionCheckDebugMap debug;
    if (!isLaneChangePathSafe(
           preparation, predictions, get_target_objects(filtered_objects_, get_current_lanes()),
           p.safety.rss_params, debug)
           .is_safe) {
      return stop("current-lane braking prediction is not safe");
    }
  }
  double arc = 0.0;
  for (size_t i = 0; i < output.path.points.size(); ++i) {
    if (i > 0)
      arc += autoware_utils::calc_distance2d(output.path.points[i - 1], output.path.points[i]);
    if (arc < origin) continue;
    const double cap = profile ? profile->at_distance(arc - profile_origin).velocity : target;
    auto & velocity = output.path.points[i].point.longitudinal_velocity_mps;
    velocity = std::min<double>(velocity, cap);
  }
  RCLCPP_INFO_THROTTLE(
    logger_, clock_, 1000,
    "speed preparation: current-lane speed %.2f -> %.2f m/s, lateral_ready=%s", c.get_ego_speed(),
    target, status_.is_safe ? "true" : "false");
  return output;
}
}  // namespace autoware::behavior_path_planner
