// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__BRAKING_PROFILE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__BRAKING_PROFILE_HPP_

#include <algorithm>
#include <cmath>
#include <optional>

namespace autoware::behavior_path_planner::lane_change
{
struct BrakingState
{
  double distance;
  double velocity;
  double acceleration;
};

// Finite, jerk-bounded preparation, including controller/planning latency. No instantaneous
// velocity clamp. Braking uses a jerk-down / constant-deceleration / jerk-up profile.
struct BrakingProfile
{
  double initial_velocity{0.0};
  double initial_acceleration{0.0};
  double target_velocity{0.0};
  double delay{0.2};
  double ramp_time{0.0};
  double ramp_jerk{0.0};
  double brake_time{0.0};
  double brake_down_time{0.0};
  double brake_hold_time{0.0};
  double brake_up_time{0.0};
  double brake_down_jerk{0.0};
  double brake_up_jerk{0.0};
  double hold_time{0.0};

  double duration() const { return delay + ramp_time + brake_time + hold_time; }

  BrakingState at(double time) const
  {
    time = std::max(0.0, time);
    const double td = std::min(time, delay);
    double s = initial_velocity * td + 0.5 * initial_acceleration * td * td;
    double v = initial_velocity + initial_acceleration * td;
    if (time <= delay) return {s, v, initial_acceleration};
    time -= delay;
    const double tr = std::min(time, ramp_time);
    s += v * tr + 0.5 * initial_acceleration * tr * tr + ramp_jerk * tr * tr * tr / 6.0;
    v += initial_acceleration * tr + 0.5 * ramp_jerk * tr * tr;
    if (time < ramp_time) return {s, v, initial_acceleration + ramp_jerk * tr};
    time -= ramp_time;
    double a = 0.0;
    const auto advance = [&](const double phase_time, const double jerk) {
      const double t = std::min(time, phase_time);
      s += v * t + 0.5 * a * t * t + jerk * t * t * t / 6.0;
      v += a * t + 0.5 * jerk * t * t;
      a += jerk * t;
      time -= t;
    };
    advance(brake_down_time, brake_down_jerk);
    if (time <= 0.0) return {s, v, a};
    advance(brake_hold_time, 0.0);
    if (time <= 0.0) return {s, v, a};
    advance(brake_up_time, brake_up_jerk);
    if (time <= 0.0) return {s, v, a};
    s += target_velocity * time;
    return {s, target_velocity, 0.0};
  }

  double distance() const { return at(duration()).distance; }

  double time_at_distance(const double distance) const
  {
    if (distance <= 0.0) return 0.0;
    if (distance >= this->distance()) {
      return duration() + (distance - this->distance()) / target_velocity;
    }
    double low = 0.0;
    double high = duration();
    for (int i = 0; i < 40; ++i) {
      const double mid = 0.5 * (low + high);
      if (at(mid).distance < distance) low = mid; else high = mid;
    }
    return 0.5 * (low + high);
  }

  BrakingState at_distance(const double distance) const { return at(time_at_distance(distance)); }
};

inline std::optional<BrakingProfile> make_braking_profile(
  const double velocity, const double acceleration, const double target, const double deceleration,
  const double positive_jerk, const double negative_jerk, const double minimum_duration)
{
  if (
    !std::isfinite(velocity) || !std::isfinite(acceleration) || !std::isfinite(target) ||
    !std::isfinite(deceleration) || !std::isfinite(positive_jerk) ||
    !std::isfinite(negative_jerk) || !std::isfinite(minimum_duration) ||
    target <= 0.0 || velocity <= target + 0.01 || deceleration <= 0.0 ||
    positive_jerk <= 0.0 || negative_jerk >= 0.0 || minimum_duration < 0.0 ||
    std::abs(acceleration) > deceleration)
    return std::nullopt;
  BrakingProfile p;
  p.initial_velocity = velocity;
  p.initial_acceleration = acceleration;
  p.target_velocity = target;
  p.ramp_jerk = acceleration > 0.0 ? negative_jerk : positive_jerk;
  p.ramp_time = std::abs(acceleration / p.ramp_jerk);
  const double before_brake = velocity + acceleration * p.delay +
                             0.5 * acceleration * p.ramp_time;
  if (before_brake <= target) return std::nullopt;
  const double dv = before_brake - target;
  const double peak_decel = std::min(
    deceleration, std::sqrt(2.0 * dv / (1.0 / positive_jerk - 1.0 / negative_jerk)));
  p.brake_down_jerk = negative_jerk;
  p.brake_up_jerk = positive_jerk;
  p.brake_down_time = peak_decel / -negative_jerk;
  p.brake_up_time = peak_decel / positive_jerk;
  p.brake_hold_time = std::max(
    0.0, (dv - 0.5 * peak_decel * (p.brake_down_time + p.brake_up_time)) / peak_decel);
  p.brake_time = p.brake_down_time + p.brake_hold_time + p.brake_up_time;
  p.hold_time = std::max(0.0, minimum_duration - p.duration());
  if (!std::isfinite(p.duration()) || p.duration() > 20.0 || !std::isfinite(p.distance()))
    return std::nullopt;
  return p;
}
}  // namespace autoware::behavior_path_planner::lane_change
#endif
