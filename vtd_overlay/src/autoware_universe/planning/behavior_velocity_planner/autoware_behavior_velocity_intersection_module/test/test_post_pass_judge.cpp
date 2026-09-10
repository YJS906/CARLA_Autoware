// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#include "autoware/behavior_velocity_intersection_module/post_pass_judge.hpp"

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner
{
TEST(PostPassJudge, ClearCorridorKeepsGo)
{
  const auto result = makePostPassJudgeDecision(false, false, 10, 20, 22, "NotOccluded");
  EXPECT_TRUE(std::holds_alternative<OverPassJudge>(result));
}

TEST(PostPassJudge, NewCollisionUsesReachableStopLine)
{
  const auto result = makePostPassJudgeDecision(true, true, 10, 20, 22, "NotOccluded");
  ASSERT_TRUE(std::holds_alternative<NonOccludedCollisionStop>(result));
  const auto & stop = std::get<NonOccludedCollisionStop>(result);
  EXPECT_EQ(stop.closest_idx, 10U);
  EXPECT_EQ(stop.collision_stopline_idx, 20U);
  EXPECT_EQ(stop.occlusion_stopline_idx, 22U);
  EXPECT_NE(stop.occlusion_report.find("post-pass-judge collision recheck"), std::string::npos);
}

TEST(PostPassJudge, InfeasibleStopRequestsImmediateBrakingInsteadOfGo)
{
  const auto result = makePostPassJudgeDecision(true, false, 10, 20, 22, "NotOccluded");
  ASSERT_TRUE(std::holds_alternative<NonOccludedCollisionStop>(result));
  EXPECT_EQ(std::get<NonOccludedCollisionStop>(result).collision_stopline_idx, 10U);
}

TEST(PostPassJudge, PassedStopLineNeverPlacesStopBehindEgo)
{
  for (const auto line : {5U, 10U}) {
    const auto result = makePostPassJudgeDecision(true, true, 10, line, 22, "NotOccluded");
    ASSERT_TRUE(std::holds_alternative<NonOccludedCollisionStop>(result));
    EXPECT_EQ(std::get<NonOccludedCollisionStop>(result).collision_stopline_idx, 10U);
  }
}

TEST(PostPassJudge, StopPersistsThroughHoldAndRechecksAfterRelease)
{
  // These are the collision state machine outputs: clear, danger, clear-time hold,
  // clear-time expired, then another newly approaching object. GO is never latched over STOP.
  for (const bool collision_or_hold : {false, true, true, false, true}) {
    const auto result =
      makePostPassJudgeDecision(collision_or_hold, true, 10, 20, 22, "NotOccluded");
    EXPECT_EQ(std::holds_alternative<NonOccludedCollisionStop>(result), collision_or_hold);
  }
}
}  // namespace autoware::behavior_velocity_planner
