// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_INTERSECTION_MODULE__POST_PASS_JUDGE_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_INTERSECTION_MODULE__POST_PASS_JUDGE_HPP_

#include "autoware/behavior_velocity_intersection_module/decision_result.hpp"

#include <cstddef>
#include <string>

namespace autoware::behavior_velocity_planner
{
// The caller has already re-evaluated objects and updated the collision state machine.
// Passing the judge line is historical information, not permission to ignore a new collision.
inline DecisionResult makePostPassJudgeDecision(
  const bool has_collision_with_margin, const bool can_stop_at_collision_line,
  const size_t closest_idx, const size_t collision_stopline_idx, const size_t occlusion_stopline_idx,
  const std::string & occlusion_diag)
{
  if (!has_collision_with_margin) {
    return OverPassJudge{
      "no collision is detected", "ego can safely pass the intersection at this rate"};
  }

  // Keep the generated/previous stop line when it is still reachable. If it is behind ego or
  // no longer reachable under the normal deceleration limits, request braking immediately.
  // Do not turn an infeasible stop into GO; this request cannot guarantee collision avoidance.
  const auto stop_idx = can_stop_at_collision_line && collision_stopline_idx > closest_idx
                          ? collision_stopline_idx
                          : closest_idx;
  return NonOccludedCollisionStop{
    closest_idx, stop_idx, occlusion_stopline_idx,
    "post-pass-judge collision recheck: " + occlusion_diag};
}
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_INTERSECTION_MODULE__POST_PASS_JUDGE_HPP_
