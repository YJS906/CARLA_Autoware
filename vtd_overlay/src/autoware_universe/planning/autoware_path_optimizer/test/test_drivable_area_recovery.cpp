// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#include "autoware/path_optimizer/utils/drivable_area_recovery.hpp"

#include <gtest/gtest.h>

using autoware::path_optimizer::DrivableAreaRecoveryState;

TEST(BoundaryColdRetry, OncePerEpisodeEvenAfterTenSeconds)
{
  DrivableAreaRecoveryState state;
  EXPECT_TRUE(state.update(100.0, true, true));
  for (int i = 1; i <= 200; ++i) EXPECT_FALSE(state.update(100.0 + i * 0.1, true, true));
}
TEST(BoundaryColdRetry, MovingDoesNotConsumeTheStoppedRetry)
{
  DrivableAreaRecoveryState state;
  EXPECT_FALSE(state.update(100.0, true, false));
  EXPECT_TRUE(state.update(100.1, true, true));
  EXPECT_FALSE(state.update(100.2, false, true));
  EXPECT_FALSE(state.update(100.3, true, true));  // A single clear frame is not a new episode.
  for (int i = 0; i <= 12; ++i) EXPECT_FALSE(state.update(100.4 + i * 0.1, false, true));
  EXPECT_TRUE(state.update(101.7, true, true));
}
TEST(BoundaryColdRetry, ClockResetAndRestartDoNotRetainOldEpisode)
{
  DrivableAreaRecoveryState state;
  EXPECT_TRUE(state.update(100.0, true, true));
  EXPECT_TRUE(state.update(10.0, true, true));
  EXPECT_FALSE(state.update(10.1, true, true));
  EXPECT_TRUE(state.update(12.0, true, true));
}
