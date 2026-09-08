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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/helper.hpp"
#include "data_structs.hpp"

#include <array>
#include <map>
#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::DebugData;
using AvoidanceDebugData = DebugData;
using autoware::behavior_path_planner::AvoidancePlanningData;
using autoware::behavior_path_planner::LaneChangeParameters;
using autoware::behavior_path_planner::NormalLaneChange;
using autoware::behavior_path_planner::ObjectData;
using autoware::behavior_path_planner::ObjectDataArray;
using autoware::behavior_path_planner::PredictedObject;
using autoware::behavior_path_planner::helper::static_obstacle_avoidance::AvoidanceHelper;

class AvoidanceByLaneChange : public NormalLaneChange
{
public:
  AvoidanceByLaneChange(
    const std::shared_ptr<LaneChangeParameters> & parameters,
    std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters);

  bool specialRequiredCheck() const override;

  // A separate approval gate; do not use it to truncate perception or candidate search.
  virtual bool isExecutionDistanceSatisfied() const;

  bool specialExpiredCheck() const override;

  void updateSpecialData() override;

  void updateLaneChangeStatus() override;

  PathSafetyStatus isApprovedPathSafe() const override;

  BehaviorModuleOutput generateOutput() override;
  bool updateApprovedPath() override;
  void resetParameters() override;
  bool hasFinishedLaneChange() const override;

private:
  friend class TestAvoidanceByLaneChange;

  std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters_;

  AvoidancePlanningData calcAvoidancePlanningData(AvoidanceDebugData & debug) const;
  AvoidancePlanningData avoidance_data_;
  mutable AvoidanceDebugData avoidance_debug_data_;

  ObjectDataArray registered_objects_;
  mutable ObjectDataArray stopped_objects_;
  std::shared_ptr<AvoidanceHelper> avoidance_helper_;

  std::optional<ObjectData> createObjectData(
    const AvoidancePlanningData & data, const PredictedObject & object) const;

  void fillAvoidanceTargetObjects(AvoidancePlanningData & data, AvoidanceDebugData & debug) const;

  double calcMinAvoidanceLength(const ObjectData & nearest_object) const;
  double calc_minimum_dist_buffer() const;
  double calcLateralOffset() const;
  const ObjectData * getNearestAvoidanceTarget() const;
  void applyObstacleVelocityLimit();

  std::vector<lanelet::ConstLanelet> getTargetLaneCandidates(
    Direction direction, const ObjectData & nearest_object) const;
  bool isLaneClear(const lanelet::ConstLanelet & lane) const;

  struct TargetLaneCandidate
  {
    Direction direction;
    lanelet::Id lane_id;
    int lanes_to_preferred;
    std::size_t lateral_steps;
  };

  struct StationaryObservation
  {
    Point anchor;
    double since{0.0};
    double last_seen{0.0};
    bool observed_moving{false};
  };

  struct RouteReturnPlan
  {
    lanelet::Id outbound_lane_id{0};
    Pose outbound_end;
    Pose start;
    double velocity{0.0};
    std_msgs::msg::Header route_header;
    std::vector<LaneChangePath> stages;
  };

  void updateStationaryObservations();
  bool isPersistentBlocker(const unique_identifier_msgs::msg::UUID & id) const;
  bool isRouteDepartureRequired() const;
  bool prepareRouteReturn(const TargetLaneCandidate & target, LaneChangePath & path);
  std::optional<RouteReturnPlan> searchRouteReturn(
    const TargetLaneCandidate & target, const LaneChangePath & path) const;
  bool isRouteReturnClear(const RouteReturnPlan & plan) const;
  void reserveRouteReturn(LaneChangePath & path) const;

  std::map<std::array<uint8_t, 16>, StationaryObservation> stationary_observations_;
  std::optional<double> observation_stamp_;
  int current_lanes_to_preferred_{0};
  std::optional<rclcpp::Time> last_return_search_;
  std::optional<RouteReturnPlan> route_return_plan_;
  bool selected_requires_return_{false};
  bool return_handoff_ready_{false};
  std::optional<rclcpp::Time> last_return_handoff_check_;

  virtual bool selectTargetLane(const TargetLaneCandidate & candidate);
  bool isStaticObstaclePathSafe(const LaneChangePath & path) const;

  // Evaluate route-progress candidates first. Departures additionally require a persistent
  // physical blockage and a complete, bounded return-plan search; traffic alone is not a reason.
  std::vector<TargetLaneCandidate> target_lane_candidates_;
};
}  // namespace autoware::behavior_path_planner

#endif  // SCENE_HPP_
