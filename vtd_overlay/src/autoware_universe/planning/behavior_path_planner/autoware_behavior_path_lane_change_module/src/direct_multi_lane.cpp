// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>

namespace autoware::behavior_path_planner
{
namespace
{
struct RestoreOnExit
{
  std::function<void()> restore;
  ~RestoreOnExit() { restore(); }
};
}  // namespace

std::pair<bool, bool> NormalLaneChange::getSafePathWithDirectFallback(LaneChangePath & safe_path)
{
  if (
    is_activated_ || getModuleType() != LaneChangeModuleType::NORMAL ||
    !common_data_ptr_->is_lanes_available() ||
    !lane_change_parameters_->enable_direct_multi_lane_change) {
    return getSafePath(safe_path);
  }

  // Reconsider the ordinary target while waiting. Never change an approved maneuver here.
  if (common_data_ptr_->requested_target_lane_id) {
    common_data_ptr_->requested_target_lane_id.reset();
    update_lanes(false);
    update_filtered_objects();
    update_transient_data(false);
    terminal_lane_change_path_.reset();
  }
  const auto direct = utils::lane_change::get_direct_mission_target(common_data_ptr_);
  if (!direct || get_target_lanes().empty() || direct->id() == get_target_lanes().front().id()) {
    return getSafePath(safe_path);
  }

  const auto started = std::chrono::steady_clock::now();
  const auto original_parameters = lane_change_parameters_;
  const double total_budget = original_parameters->time_limit;
  if (!std::isfinite(total_budget) || total_budget <= 0.0) return {false, false};
  // Only private copies get a reduced time budget; never mutate manager-owned parameters.
  const RestoreOnExit restore_parameters{[&]() {
    lane_change_parameters_ = original_parameters;
    common_data_ptr_->lc_param_ptr = original_parameters;
  }};
  auto bounded_parameters = std::make_shared<LaneChangeParameters>(*original_parameters);
  bounded_parameters->time_limit = 0.4 * total_budget;
  lane_change_parameters_ = bounded_parameters;
  common_data_ptr_->lc_param_ptr = bounded_parameters;
  const auto primary_result = getSafePath(safe_path);
  if (primary_result.first && primary_result.second) return primary_result;

  const auto remaining = [&]() {
    return 0.9 * total_budget -
           std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
             .count();
  };
  // Leave a little of the caller's original budget for final return-clearance checks.
  if (remaining() <= 0.0) return primary_result;

  const auto primary_common = common_data_ptr_;
  const auto primary_objects = filtered_objects_;
  const auto primary_debug = lane_change_debug_;
  const auto primary_terminal = terminal_lane_change_path_;
  const auto primary_intersection_path = path_after_intersection_;
  const auto primary_speed_target = speed_preparation_target_;
  const auto primary_speed_lane = speed_preparation_lane_id_;
  const auto primary_speed_profile = speed_preparation_profile_;
  const auto primary_speed_start = speed_preparation_start_;
  const auto primary_stop_time = stop_time_;
  bool keep_direct = false;
  const RestoreOnExit restore_primary{[&]() {
    if (keep_direct) return;
    common_data_ptr_ = primary_common;
    filtered_objects_ = primary_objects;
    lane_change_debug_ = primary_debug;
    terminal_lane_change_path_ = primary_terminal;
    path_after_intersection_ = primary_intersection_path;
    speed_preparation_target_ = primary_speed_target;
    speed_preparation_lane_id_ = primary_speed_lane;
    speed_preparation_profile_ = primary_speed_profile;
    speed_preparation_start_ = primary_speed_start;
    stop_time_ = primary_stop_time;
  }};

  // Keep target lanes, polygons, filters and debug state paired with the winning path.
  common_data_ptr_ = std::make_shared<lane_change::CommonData>(*primary_common);
  common_data_ptr_->lanes_ptr = std::make_shared<lane_change::Lanes>(*primary_common->lanes_ptr);
  common_data_ptr_->lanes_polygon_ptr =
    std::make_shared<lane_change::LanesPolygon>(*primary_common->lanes_polygon_ptr);
  common_data_ptr_->requested_target_lane_id = direct->id();
  try {
    update_lanes(false);
    update_filtered_objects();
    update_transient_data(false);
    terminal_lane_change_path_.reset();
    if (
      get_target_lanes().empty() || get_target_lanes().front().id() != direct->id() ||
      isLaneChangeRequired() || remaining() <= 0.0) {
      return primary_result;
    }
    bounded_parameters->time_limit = remaining();
    LaneChangePath alternative;
    const auto result = getSafePath(alternative);
    const bool valid = result.first && isValidPath(alternative.path);
    const bool safe = valid && result.second && remaining() > 0.0;
    RCLCPP_INFO_THROTTLE(
      logger_, clock_, 1000, "mission return direct fallback target=%lld valid=%s safe=%s",
      static_cast<long long>(direct->id()), valid ? "true" : "false", safe ? "true" : "false");
    if (safe) {
      safe_path = std::move(alternative);
      keep_direct = true;
      return {true, true};
    }
  } catch (const std::exception & error) {
    RCLCPP_DEBUG(logger_, "Direct mission return rejected: %s", error.what());
  }
  return primary_result;
}
}  // namespace autoware::behavior_path_planner
