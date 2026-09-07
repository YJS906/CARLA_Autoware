// Copyright 2023 TIER IV, Inc.
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

#include "interface.hpp"

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::State;
using autoware::route_handler::Direction;

AvoidanceByLaneChangeInterface::AvoidanceByLaneChangeInterface(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<LaneChangeParameters> & parameters,
  const std::shared_ptr<AvoidanceByLCParameters> & avoidance_by_lane_change_parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface)
: LaneChangeInterface{
    name,
    node,
    parameters,
    rtc_interface_ptr_map,
    objects_of_interest_marker_interface_ptr_map,
    planning_factor_interface,
    std::make_unique<AvoidanceByLaneChange>(parameters, avoidance_by_lane_change_parameters)}
{
}

bool AvoidanceByLaneChangeInterface::isExecutionRequested() const
{
  // isLaneChangeRequired() returns an error string when a lane-change path cannot be used.
  // An avoidance request therefore requires the absence of that error, just like the normal
  // lane-change interface.
  return !module_type_->isLaneChangeRequired() && module_type_->specialRequiredCheck() &&
         module_type_->isValidPath() && module_type_->isSafe();
}

void AvoidanceByLaneChangeInterface::updateRTCStatus(
  const double start_distance, const double finish_distance)
{
  update_rtc_status(start_distance, finish_distance);
}

void AvoidanceByLaneChangeInterface::update_rtc_status(
  const double start_distance, const double finish_distance, const std::optional<bool> safe,
  const std::optional<uint8_t> state)
{
  const auto dir = module_type_->getDirection();
  const std::string direction = dir == Direction::LEFT    ? "left"
                                : dir == Direction::RIGHT ? "right"
                                                          : "";

  // A waiting candidate may change or lose its direction. Remove only its own old requests;
  // never register the opposite side with the selected path's safety result.
  if (isWaitingApproval()) {
    for (const auto & [side, rtc] : rtc_interface_ptr_map_) {
      if (side != direction && rtc && rtc->isRegistered(uuid_map_.at(side))) {
        rtc->removeCooperateStatus(uuid_map_.at(side));
      }
    }
  }
  if (direction.empty()) return;

  const auto & rtc = rtc_interface_ptr_map_.at(direction);
  const auto & uuid = uuid_map_.at(direction);
  const auto default_state =
    !rtc->isRegistered(uuid) || isWaitingApproval() ? State::WAITING_FOR_EXECUTION : State::RUNNING;
  rtc->updateCooperateStatus(
    uuid, safe.value_or(isExecutionReady()), state.value_or(default_state), start_distance,
    finish_distance, clock_->now());
}
}  // namespace autoware::behavior_path_planner
