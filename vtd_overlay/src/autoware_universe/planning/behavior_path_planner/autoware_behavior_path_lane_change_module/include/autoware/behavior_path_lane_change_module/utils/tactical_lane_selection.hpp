// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__TACTICAL_LANE_SELECTION_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__TACTICAL_LANE_SELECTION_HPP_

#include "autoware/behavior_path_lane_change_module/utils/corridor_choice.hpp"

#include <rclcpp/node.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
struct PlannerData;
namespace lane_change
{
struct Parameters;
class TacticalLaneSelection
{
public:
  static std::shared_ptr<TacticalLaneSelection> shared(rclcpp::Node & node);
  ~TacticalLaneSelection();
  void remember(
    const PlannerData & data, const std::vector<unique_identifier_msgs::msg::UUID> & reasons);
  tactical::Score evaluate(
    const PlannerData & data, const lanelet::ConstLanelets & current, lanelet::Id target,
    const Parameters & parameters);
  std::optional<std::string> deferReturn(
    const PlannerData & data, const lanelet::ConstLanelets & current,
    const lanelet::ConstLanelets & target, const Parameters & parameters);

private:
  TacticalLaneSelection();
  struct State;
  std::unique_ptr<State> state_;
};
}  // namespace lane_change
}  // namespace autoware::behavior_path_planner
#endif
