// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
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
    p.trajectory_safety.max_jerk, p.trajectory_safety.min_jerk,
    p.trajectory.min_prepare_duration);
  if (!profile) return std::nullopt;
  const double available = std::min(
    c.transient_data.dist_to_terminal_start, c.transient_data.distance_to_static_obstacle);
  // A finite one-metre settling segment follows braking before lateral motion starts.
  const double length = profile->distance() + 1.0;
  if (length > available || length < calculation::calc_ego_dist_to_lanes_start(
        common_data_ptr_, get_current_lanes(), get_target_lanes()))
    return std::nullopt;
  const double acc = (target - c.get_ego_speed()) / profile->duration();
  LaneChangePhaseMetrics metric(profile->duration(), length, target, acc, acc, 0.0);
  metric.braking_profile = profile;
  return metric;
}

std::optional<double> NormalLaneChange::curvature_speed_limit(const LaneChangePath & path) const
{
  const auto & points = path.path.points;
  if (points.size() < 3) return std::nullopt;
  const auto & vehicle = planner_data_->parameters.vehicle_info;
  const auto ego = motion_utils::findNearestSegmentIndex(points, getEgoPosition());
  const auto start = motion_utils::findNearestSegmentIndex(points, path.info.lane_changing_start.position);
  const auto end = motion_utils::findNearestSegmentIndex(points, path.info.lane_changing_end.position);
  double cap = std::min(path.info.velocity.lane_changing, path.info.terminal_lane_changing_velocity);
  double previous = 0.0;
  bool have_previous = false;
  for (size_t i = std::max<size_t>(1, ego); i + 1 < points.size(); ++i) {
    const double k = autoware_utils::calc_curvature(
      points[i - 1].point.pose.position, points[i].point.pose.position,
      points[i + 1].point.pose.position);
    if (!std::isfinite(k) || std::abs(k) > vehicle.calcMaxCurvature()) return std::nullopt;
    if (i >= start && i <= end) {
      const double ds = autoware_utils::calc_distance2d(points[i - 1], points[i]);
      if (ds < 1e-6) return std::nullopt;
      const double dk = have_previous ? std::abs(k - previous) / ds : 0.0;
      double low = 0.0;
      double high = cap;
      for (int iteration = 0; iteration < 24; ++iteration) {
        const double v = 0.5 * (low + high);
        const double lateral_acc = lane_change_parameters_->trajectory.lat_acc_map.find(v).second;
        if (v * v * std::abs(k) <= lateral_acc &&
            v * v * v * dk <= lane_change_parameters_->trajectory.lateral_jerk)
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

BehaviorModuleOutput NormalLaneChange::braking_wait_output() const
{
  // Execute only longitudinal preparation while RTC is pending. Never publish the unapproved
  // lateral geometry. The candidate must already pass collision and reachable-speed checks.
  auto output = prev_module_output_;
  const auto & info = status_.lane_change_path.info;
  if (!status_.is_safe || !status_.is_valid_path || !info.braking_profile ||
      output.path.points.size() < 2)
    return output;
  const auto & points = output.path.points;
  const double origin = motion_utils::calcSignedArcLength(
    points, points.front().point.pose.position, info.braking_start_pose.position);
  double arc = 0.0;
  for (size_t i = 0; i < output.path.points.size(); ++i) {
    if (i > 0) arc += autoware_utils::calc_distance2d(points[i - 1], points[i]);
    if (arc < origin) continue;
    const double cap = info.braking_profile->at_distance(arc - origin).velocity;
    auto & velocity = output.path.points[i].point.longitudinal_velocity_mps;
    velocity = std::min<double>(velocity, cap);
  }
  return output;
}
}  // namespace autoware::behavior_path_planner
