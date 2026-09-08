// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#ifndef STATIC_OBSTACLE_AVOIDANCE_LOCAL_VELOCITY_PROFILE_HPP_
#define STATIC_OBSTACLE_AVOIDANCE_LOCAL_VELOCITY_PROFILE_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace autoware::behavior_path_planner::detail
{
// Input limits are intrinsic geometry/stop constraints; infinity means unconstrained.
// Compute braking and recovery in separate sweeps, then intersect them with the
// incoming path. Never use an already-propagated/input speed as a fresh anchor:
// repeated validation would otherwise restart the jerk ramp and spread low speeds.
// distance(high, low) returns the required jerk/acceleration-bounded distance.
template <class Distance>
bool extendLocalVelocityLimits(
  const std::vector<double> & arcs, std::vector<double> & limits,
  const double upper_velocity, const Distance & distance, const bool reverse,
  const size_t begin)
{
  if (
    arcs.size() != limits.size() || begin >= arcs.size() ||
    !std::isfinite(upper_velocity) || upper_velocity < 0.0)
    return false;
  for (size_t i = 0; i < arcs.size(); ++i) {
    if (
      !std::isfinite(arcs[i]) || std::isnan(limits[i]) || limits[i] < 0.0 ||
      (i > 0 && arcs[i] <= arcs[i - 1]))
      return false;
  }
  std::optional<size_t> anchor;
  for (size_t step = 0; step < arcs.size() - begin; ++step) {
    const size_t i = reverse ? arcs.size() - 1 - step : begin + step;
    const double local = limits[i];
    limits[i] = std::min(local, upper_velocity);
    bool propagated = false;
    if (anchor && limits[i] > limits[*anchor]) {
      const double available = std::abs(arcs[i] - arcs[*anchor]);
      const double required = distance(limits[i], limits[*anchor]);
      if (!std::isfinite(required) || required < 0.0) return false;
      if (required > available) {
        double low = limits[*anchor];
        double high = limits[i];
        for (size_t iteration = 0; iteration < 24; ++iteration) {
          const double mid = 0.5 * (low + high);
          const double needed = distance(mid, limits[*anchor]);
          if (!std::isfinite(needed) || needed < 0.0) return false;
          if (needed <= available)
            low = mid;
          else
            high = mid;
        }
        limits[i] = low;
        propagated = true;
      }
    }
    if (!propagated && local < upper_velocity) anchor = i;
  }
  return true;
}
}  // namespace autoware::behavior_path_planner::detail
#endif
