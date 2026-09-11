// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__TARGET_LANDING_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__TARGET_LANDING_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>

namespace autoware::behavior_path_planner::utils::lane_change
{
struct NarrowTargetLanding
{
  lanelet::Id lane_id;
  double width;
  double longitudinal_offset;
};

// Evidence of insufficient target width under the vehicle at lane-change completion only.
// An empty result is NOT a safety certificate: missing/ambiguous finite boundaries defer to
// the existing checks. Do not extrapolate lane ends or reserve a hypothetical future return.
std::optional<NarrowTargetLanding> find_narrow_target_landing(
  const lanelet::ConstLanelets & target_lanes, const geometry_msgs::msg::Pose & landing,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle);
}  // namespace autoware::behavior_path_planner::utils::lane_change

#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__TARGET_LANDING_HPP_
