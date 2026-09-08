// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef DATA_STRUCTS_HPP_
#define DATA_STRUCTS_HPP_

#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::AvoidanceParameters;

struct AvoidanceByLCParameters : public AvoidanceParameters
{
  // execute only when the target object longitudinal distance is larger than this param.
  double execute_object_longitudinal_margin{0.0};

  // Maximum path-relative distance to the nearest target envelope for a NEW approval.
  // Object lookup and candidate generation keep their existing longer horizons.
  double max_execution_distance{20.0};

  // execute only when lane change end point is before the object.
  bool execute_only_when_lane_change_finish_before_object{false};

  // Scale the dynamically available lane-changing distance for obstacle avoidance only.
  double max_lane_changing_length_scale{1.0};

  // Cap the approach/candidate velocity while an avoidance target overlaps the ego path.
  double obstacle_velocity_limit_ratio{1.0};

  // VTD-only temporary override. The ordinary lane-change modules keep their configured limit.
  bool disable_lateral_acceleration_limit{false};

  // Allow one continuous shift across consecutive, route-approved same-direction lanes.
  bool enable_direct_multi_lane_change{false};

  // Longitudinal window used to decide whether another outward lane is empty.
  double empty_lane_check_forward_distance{80.0};
  double empty_lane_check_backward_distance{20.0};

  // Leaving the mission-lane corridor is an exception, never a response to one unsafe sample.
  double route_blockage_min_duration{3.0};
  double route_blockage_max_position_drift{0.5};
  double route_return_search_interval{1.0};
  double route_return_time_budget_ms{20.0};
  double route_return_time_margin{1.0};
  int route_return_max_steps{3};

  explicit AvoidanceByLCParameters(const AvoidanceParameters & param) : AvoidanceParameters(param)
  {
  }
};
}  // namespace autoware::behavior_path_planner

#endif  // DATA_STRUCTS_HPP_
