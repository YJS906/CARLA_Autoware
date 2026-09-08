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

#include "manager.hpp"

#include "autoware/behavior_path_static_obstacle_avoidance_module/parameter_helper.hpp"
#include "autoware_utils/ros/parameter.hpp"
#include "autoware_utils/ros/update_param.hpp"
#include "data_structs.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::getParameter;
using autoware::behavior_path_planner::ObjectParameter;

void AvoidanceByLaneChangeModuleManager::init(rclcpp::Node * node)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_utils::get_or_declare_parameter;

  // init manager interface
  initInterface(node, {"left", "right"});

  // init lane change manager
  LaneChangeModuleManager::initParams(node);

  const auto avoidance_params = getParameter(node);
  AvoidanceByLCParameters p(avoidance_params);

  // unique parameters
  {
    const std::string ns = "avoidance_by_lane_change.";
    p.execute_object_longitudinal_margin =
      get_or_declare_parameter<double>(*node, ns + "execute_object_longitudinal_margin");
    const auto execution_distance_key = ns + "max_execution_distance";
    p.max_execution_distance = node->has_parameter(execution_distance_key)
                                 ? node->get_parameter(execution_distance_key).as_double()
                                 : node->declare_parameter<double>(execution_distance_key, 20.0);
    if (!std::isfinite(p.max_execution_distance) || p.max_execution_distance <= 0.0) {
      throw std::invalid_argument(
        "avoidance_by_lane_change.max_execution_distance must be positive and finite");
    }
    p.execute_only_when_lane_change_finish_before_object = get_or_declare_parameter<bool>(
      *node, ns + "execute_only_when_lane_change_finish_before_object");
    p.max_lane_changing_length_scale = std::clamp(
      get_or_declare_parameter<double>(*node, ns + "max_lane_changing_length_scale"), 0.0, 1.0);
    p.obstacle_velocity_limit_ratio = std::clamp(
      get_or_declare_parameter<double>(*node, ns + "obstacle_velocity_limit_ratio"), 0.0, 1.0);
    p.disable_lateral_acceleration_limit =
      get_or_declare_parameter<bool>(*node, ns + "disable_lateral_acceleration_limit");
    p.enable_direct_multi_lane_change =
      get_or_declare_parameter<bool>(*node, ns + "enable_direct_multi_lane_change");
    p.empty_lane_check_forward_distance = std::max(
      0.0, get_or_declare_parameter<double>(*node, ns + "empty_lane_check_forward_distance"));
    p.empty_lane_check_backward_distance = std::max(
      0.0, get_or_declare_parameter<double>(*node, ns + "empty_lane_check_backward_distance"));

    const auto positive = [&](const std::string & name, const double fallback) {
      const auto key = ns + "route_priority." + name;
      const auto value = node->has_parameter(key) ? node->get_parameter(key).as_double()
                                                  : node->declare_parameter<double>(key, fallback);
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Invalid route-priority parameter: " + name);
      }
      return value;
    };
    p.route_blockage_min_duration = positive("blockage_min_duration", 3.0);
    p.route_blockage_max_position_drift = positive("blockage_max_position_drift", 0.5);
    p.route_return_search_interval = positive("return_search_interval", 1.0);
    p.route_return_time_budget_ms = positive("return_time_budget_ms", 20.0);
    p.route_return_time_margin = positive("return_time_margin", 1.0);
    const auto steps_key = ns + "route_priority.return_max_steps";
    const auto steps = node->has_parameter(steps_key) ? node->get_parameter(steps_key).as_int()
                                                      : node->declare_parameter<int>(steps_key, 3);
    if (steps < 1 || steps > 4) {
      throw std::invalid_argument("route_priority.return_max_steps must be in [1, 4]");
    }
    p.route_return_max_steps = static_cast<int>(steps);
  }

  // general params
  {
    const std::string ns = "avoidance.";
    p.resample_interval_for_planning =
      get_or_declare_parameter<double>(*node, ns + "resample_interval_for_planning");
    p.resample_interval_for_output =
      get_or_declare_parameter<double>(*node, ns + "resample_interval_for_output");
  }

  // target object
  {
    const auto set_object_param = [&](const uint8_t object_type, const std::string & ns) {
      // The inherited map already contains every class. Update its fields instead of emplace,
      // preserving longitudinal/error/safety fields not overridden by the LC configuration.
      auto & param = p.object_parameters.at(object_type);
      param.moving_speed_threshold =
        get_or_declare_parameter<double>(*node, ns + "th_moving_speed");
      param.moving_time_threshold = get_or_declare_parameter<double>(*node, ns + "th_moving_time");
      param.max_expand_ratio = get_or_declare_parameter<double>(*node, ns + "max_expand_ratio");
      param.envelope_buffer_margin =
        get_or_declare_parameter<double>(*node, ns + "envelope_buffer_margin");
      param.lateral_soft_margin =
        get_or_declare_parameter<double>(*node, ns + "lateral_margin.soft_margin");
      param.lateral_hard_margin =
        get_or_declare_parameter<double>(*node, ns + "lateral_margin.hard_margin");
      param.lateral_hard_margin_for_parked_vehicle = get_or_declare_parameter<double>(
        *node, ns + "lateral_margin.hard_margin_for_parked_vehicle");
    };

    const std::string ns = "avoidance_by_lane_change.target_object.";
    set_object_param(ObjectClassification::MOTORCYCLE, ns + "motorcycle.");
    set_object_param(ObjectClassification::CAR, ns + "car.");
    set_object_param(ObjectClassification::TRUCK, ns + "truck.");
    set_object_param(ObjectClassification::TRAILER, ns + "trailer.");
    set_object_param(ObjectClassification::BUS, ns + "bus.");
    set_object_param(ObjectClassification::PEDESTRIAN, ns + "pedestrian.");
    set_object_param(ObjectClassification::BICYCLE, ns + "bicycle.");
    set_object_param(ObjectClassification::UNKNOWN, ns + "unknown.");

    p.lower_distance_for_polygon_expansion =
      get_or_declare_parameter<double>(*node, ns + "lower_distance_for_polygon_expansion");
    p.upper_distance_for_polygon_expansion =
      get_or_declare_parameter<double>(*node, ns + "upper_distance_for_polygon_expansion");
  }

  // target filtering
  {
    const auto set_target_flag = [&](const uint8_t & object_type, const std::string & ns) {
      if (p.object_parameters.count(object_type) == 0) {
        return;
      }
      p.object_parameters.at(object_type).is_avoidance_target =
        get_or_declare_parameter<bool>(*node, ns);
    };

    const std::string ns = "avoidance.target_filtering.";
    set_target_flag(ObjectClassification::CAR, ns + "target_type.car");
    set_target_flag(ObjectClassification::TRUCK, ns + "target_type.truck");
    set_target_flag(ObjectClassification::TRAILER, ns + "target_type.trailer");
    set_target_flag(ObjectClassification::BUS, ns + "target_type.bus");
    set_target_flag(ObjectClassification::PEDESTRIAN, ns + "target_type.pedestrian");
    set_target_flag(ObjectClassification::BICYCLE, ns + "target_type.bicycle");
    set_target_flag(ObjectClassification::MOTORCYCLE, ns + "target_type.motorcycle");
    set_target_flag(ObjectClassification::UNKNOWN, ns + "target_type.unknown");

    p.object_check_goal_distance =
      get_or_declare_parameter<double>(*node, ns + "object_check_goal_distance");
    p.object_last_seen_threshold =
      get_or_declare_parameter<double>(*node, ns + "max_compensation_time");
  }

  {
    const std::string ns = "avoidance.target_filtering.parked_vehicle.";
    p.threshold_distance_object_is_on_center =
      get_or_declare_parameter<double>(*node, ns + "th_offset_from_centerline");
    p.object_check_shiftable_ratio =
      get_or_declare_parameter<double>(*node, ns + "th_shiftable_ratio");
    p.object_check_min_road_shoulder_width =
      get_or_declare_parameter<double>(*node, ns + "min_road_shoulder_width");
  }

  {
    const std::string ns = "avoidance.target_filtering.avoidance_for_ambiguous_vehicle.";
    p.policy_ambiguous_vehicle = get_or_declare_parameter<std::string>(*node, ns + "policy");
    p.wait_and_see_target_behaviors = get_or_declare_parameter<std::vector<std::string>>(
      *node, ns + "wait_and_see.target_behaviors");
    p.wait_and_see_th_closest_distance =
      get_or_declare_parameter<double>(*node, ns + "wait_and_see.th_closest_distance");
    p.time_threshold_for_ambiguous_vehicle =
      get_or_declare_parameter<double>(*node, ns + "condition.th_stopped_time");
    p.distance_threshold_for_ambiguous_vehicle =
      get_or_declare_parameter<double>(*node, ns + "condition.th_moving_distance");
    p.object_ignore_section_traffic_light_in_front_distance =
      get_or_declare_parameter<double>(*node, ns + "ignore_area.traffic_light.front_distance");
    p.object_ignore_section_crosswalk_in_front_distance =
      get_or_declare_parameter<double>(*node, ns + "ignore_area.crosswalk.front_distance");
    p.object_ignore_section_crosswalk_behind_distance =
      get_or_declare_parameter<double>(*node, ns + "ignore_area.crosswalk.behind_distance");
  }

  // avoidance maneuver (longitudinal)
  {
    const std::string ns = "avoidance.avoidance.longitudinal.";
    p.min_prepare_time = get_or_declare_parameter<double>(*node, ns + "min_prepare_time");
    p.max_prepare_time = get_or_declare_parameter<double>(*node, ns + "max_prepare_time");
    p.min_prepare_distance = get_or_declare_parameter<double>(*node, ns + "min_prepare_distance");
    p.min_slow_down_speed = get_or_declare_parameter<double>(*node, ns + "min_slow_down_speed");
    p.buf_slow_down_speed = get_or_declare_parameter<double>(*node, ns + "buf_slow_down_speed");
    p.nominal_avoidance_speed =
      get_or_declare_parameter<double>(*node, ns + "nominal_avoidance_speed");
  }

  {
    const std::string ns = "avoidance.target_filtering.detection_area.";
    p.use_static_detection_area = get_or_declare_parameter<bool>(*node, ns + "static");
    p.object_check_min_forward_distance =
      get_or_declare_parameter<double>(*node, ns + "min_forward_distance");
    p.object_check_max_forward_distance =
      get_or_declare_parameter<double>(*node, ns + "max_forward_distance");
    p.object_check_backward_distance =
      get_or_declare_parameter<double>(*node, ns + "backward_distance");
  }

  // safety check
  {
    const std::string ns = "avoidance.safety_check.";
    p.hysteresis_factor_expand_rate =
      get_or_declare_parameter<double>(*node, ns + "hysteresis_factor_expand_rate");
  }

  utils::static_obstacle_avoidance::enforceTrajectorySafetyMargins(p);
  avoidance_parameters_ = std::make_shared<AvoidanceByLCParameters>(p);
}

SMIPtr AvoidanceByLaneChangeModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<AvoidanceByLaneChangeInterface>(
    name_, *node_, parameters_, avoidance_parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_, planning_factor_interface_);
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::AvoidanceByLaneChangeModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
