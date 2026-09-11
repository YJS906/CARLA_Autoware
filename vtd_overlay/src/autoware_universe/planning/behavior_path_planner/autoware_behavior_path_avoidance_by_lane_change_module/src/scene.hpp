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
    std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters,
    std::shared_ptr<AvoidanceMotionHistory> motion_history = nullptr);

  bool specialRequiredCheck() const override;

  // A separate approval gate; do not use it to truncate perception or candidate search.
  virtual bool isExecutionDistanceSatisfied() const;

  bool specialExpiredCheck() const override;

  void updateSpecialData() override;

  void updateLaneChangeStatus() override;

  PathSafetyStatus isApprovedPathSafe() const override;

  std::optional<UUID> getPendingTargetId() const { return pending_target_id_; }

private:
  friend class TestAvoidanceByLaneChange;
  friend class TestAvoidanceRequestLifecycle;

  bool isRecedingTarget(const AvoidancePlanningData & data, const ObjectData & object) const;
  void updatePendingTarget(const ObjectData * target);
  std::optional<UUID> pending_target_id_;

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

  void updateStationaryObservations();
  bool isPersistentBlocker(const unique_identifier_msgs::msg::UUID & id) const;
  bool isRouteDepartureRequired() const;

  std::shared_ptr<AvoidanceMotionHistory> motion_history_;
  int current_lanes_to_preferred_{0};

  virtual bool selectTargetLane(const TargetLaneCandidate & candidate);
  bool isStaticObstaclePathSafe(const LaneChangePath & path) const;

  // Evaluate route-progress candidates first. Departures additionally require a persistent
  // physical blockage; traffic alone is not a reason. Future returns are planned independently
  // by the ordinary lane-change modules, not reserved by the avoidance maneuver.
  std::vector<TargetLaneCandidate> target_lane_candidates_;
};
}  // namespace autoware::behavior_path_planner

#endif  // SCENE_HPP_
