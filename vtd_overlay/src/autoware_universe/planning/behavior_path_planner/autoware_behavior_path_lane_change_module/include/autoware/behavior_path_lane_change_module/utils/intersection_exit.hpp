// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__INTERSECTION_EXIT_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__INTERSECTION_EXIT_HPP_

#include "autoware/behavior_path_lane_change_module/structs/data.hpp"

#include <optional>

namespace autoware::behavior_path_planner::utils::lane_change
{
// A target opening next to the same outgoing road at the end of a mapped intersection connector.
// An ordinary road split or a merely nearby intersection does not satisfy this test. Pass the
// real map target, not a lanelet carrying the legacy backward-overlap geometry.
bool is_intersection_exit_target(
  const lanelet::ConstLanelet & real_target, const lanelet::ConstLanelets & current_lanes,
  const route_handler::Direction direction);

// Distance along the current path until the rear clears that exit by 0.5 m. Applied only while
// the exit is at most max(backward-overlap length, vehicle length) ahead of ego and until the
// clearance has elapsed. Other locations return nullopt. Resolves a legacy synthetic target to
// its actual map geometry by ID before checking the topology.
std::optional<double> intersection_exit_prepare_length(
  const behavior_path_planner::lane_change::CommonDataPtr & data);
// Candidate geometry gate; unlike collision safety it must also reject force-approval candidates.
bool starts_before_intersection_exit(
  const behavior_path_planner::lane_change::CommonDataPtr & data,
  const geometry_msgs::msg::Pose & lane_change_start);
}  // namespace autoware::behavior_path_planner::utils::lane_change

#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__INTERSECTION_EXIT_HPP_
