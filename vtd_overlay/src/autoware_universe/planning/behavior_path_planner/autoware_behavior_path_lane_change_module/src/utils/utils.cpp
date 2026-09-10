// Copyright 2021 Tier IV, Inc.
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

#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"

#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_lane_change_module/structs/path.hpp"
#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"
#include "autoware/behavior_path_planner_common/parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/behavior_path_planner_common/utils/traffic_light_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware_utils/math/unit_conversion.hpp"

// for the geometry types
#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
// for the svg mapper
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_frenet_planner/frenet_planner.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <range/v3/action/remove_if.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/numeric.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/detail/disjoint/interface.hpp>
#include <boost/geometry/io/svg/svg_mapper.hpp>
#include <boost/geometry/io/svg/write.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>
namespace
{
template <typename T>
double calc_arc_length(
  const T & reference_path, const geometry_msgs::msg::Pose & pose,
  const BehaviorPathPlannerParameters & bpp_params)
{
  if (reference_path.empty()) {
    return 0.0;
  }
  const auto nearest_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      reference_path, pose, bpp_params.ego_nearest_dist_threshold,
      bpp_params.ego_nearest_yaw_threshold);

  auto frenet_point = autoware::behavior_path_planner::utils::convertToFrenetPoint(
    reference_path, pose.position, nearest_seg_idx);
  return frenet_point.length;
}
}  // namespace

namespace autoware::behavior_path_planner::utils::lane_change
{
using autoware::route_handler::RouteHandler;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_utils::LineString2d;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;
using behavior_path_planner::lane_change::PathType;
using geometry_msgs::msg::Pose;

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;

rclcpp::Logger get_logger()
{
  constexpr const char * name{"lane_change.utils"};
  static rclcpp::Logger logger = rclcpp::get_logger(name);
  return logger;
}

bool is_mandatory_lane_change(const ModuleType lc_type)
{
  // Obstacle avoidance must select a legal adjacent lane even when ego already occupies the
  // route-preferred lane.  Only route-driven normal lane changes are mandatory/preferred-lane
  // changes; avoidance-by-lane-change uses the non-mandatory adjacent-lane selection path.
  return lc_type == LaneChangeModuleType::NORMAL;
}

void set_prepare_velocity(
  PathWithLaneId & prepare_segment, const double current_velocity, const double prepare_velocity)
{
  // Do not leave road-speed points ahead of ego while requesting a speed-holding or
  // decelerating prepare phase. The downstream smoother still enforces braking/jerk limits.
  const auto ceiling = static_cast<float>(std::max(current_velocity, prepare_velocity));
  for (auto & point : prepare_segment.points) {
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, ceiling);
  }
  if (current_velocity >= prepare_velocity) {
    // deceleration
    prepare_segment.points.back().point.longitudinal_velocity_mps = std::min(
      prepare_segment.points.back().point.longitudinal_velocity_mps,
      static_cast<float>(prepare_velocity));
    return;
  }
  // acceleration
  for (auto & point : prepare_segment.points) {
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, static_cast<float>(prepare_velocity));
  }
}

lanelet::ConstLanelets get_target_neighbor_lanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const LaneChangeModuleType & type)
{
  // Obstacle avoidance may start from either a preferred or a non-preferred route lane. Its
  // available longitudinal distance must always be measured on the current/source lane chain;
  // applying the ordinary mandatory/non-mandatory preferred-lane filter can otherwise make the
  // source set empty and reject every avoidance candidate.
  if (type == LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE) {
    return current_lanes;
  }

  lanelet::ConstLanelets neighbor_lanes;

  for (const auto & current_lane : current_lanes) {
    const auto mandatory_lane_change = is_mandatory_lane_change(type);
    if (route_handler.getNumLaneToPreferredLane(current_lane) != 0) {
      if (mandatory_lane_change) {
        neighbor_lanes.push_back(current_lane);
      }
    } else {
      if (!mandatory_lane_change) {
        neighbor_lanes.push_back(current_lane);
      }
    }
  }
  return neighbor_lanes;
}

bool path_footprint_exceeds_target_lane_bound(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & path, const VehicleInfo & ego_info,
  const double margin)
{
  if (common_data_ptr->direction == Direction::NONE || path.points.empty()) {
    return false;
  }

  const auto & target_lanes = common_data_ptr->lanes_ptr->target;
  const bool is_left = common_data_ptr->direction == Direction::LEFT;

  const auto combined_target_lane_opt =
    autoware::experimental::lanelet2_utils::combine_lanelets_shape(target_lanes);
  if (!combined_target_lane_opt) {
    // empty target_lanes -> no boundary
    return false;
  }
  const auto & combined_target_lane = combined_target_lane_opt.value();

  for (const auto & path_point : path.points) {
    const auto & pose = path_point.point.pose;
    const auto front_vertex = getEgoFrontVertex(pose, ego_info, is_left);

    const auto sign = is_left ? -1.0 : 1.0;
    const auto dist_to_boundary =
      sign * utils::getSignedDistanceFromLaneBoundary(combined_target_lane, front_vertex, is_left);

    if (dist_to_boundary < margin) {
      RCLCPP_DEBUG(get_logger(), "Path footprint exceeds target lane boundary");
      return true;
    }
  }

  return false;
}

std::vector<DrivableLanes> generateDrivableLanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & lane_change_lanes)
{
  size_t current_lc_idx = 0;
  struct LateralTarget
  {
    lanelet::ConstLanelet lane;
    size_t index;
    lanelet::ConstLanelets middle;
  };
  std::vector<DrivableLanes> drivable_lanes(current_lanes.size());
  for (size_t i = 0; i < current_lanes.size(); ++i) {
    const auto & current_lane = current_lanes.at(i);
    drivable_lanes.at(i).left_lane = current_lane;
    drivable_lanes.at(i).right_lane = current_lane;

    // Search through every lateral neighbor so a direct multi-lane shift includes the complete
    // corridor, while preserving the one-lane behavior when the target is adjacent.
    const auto find_target_on_side = [&](const bool search_left) -> std::optional<LateralTarget> {
      auto lateral_lane = current_lane;
      lanelet::ConstLanelets middle;
      std::unordered_set<lanelet::Id> visited{current_lane.id()};
      while (true) {
        const auto next_lane = search_left
                                 ? route_handler.getLeftLanelet(lateral_lane, true, false)
                                 : route_handler.getRightLanelet(lateral_lane, true, false);
        if (!next_lane || !visited.insert(next_lane->id()).second) {
          break;
        }
        lateral_lane = *next_lane;
        for (size_t lc_idx = current_lc_idx; lc_idx < lane_change_lanes.size(); ++lc_idx) {
          if (lane_change_lanes.at(lc_idx).id() == lateral_lane.id()) {
            if (search_left) std::reverse(middle.begin(), middle.end());
            return LateralTarget{lateral_lane, lc_idx, middle};
          }
        }
        middle.push_back(lateral_lane);
      }
      return std::nullopt;
    };

    if (const auto left_target = find_target_on_side(true)) {
      drivable_lanes.at(i).left_lane = left_target->lane;
      drivable_lanes.at(i).middle_lanes = left_target->middle;
      current_lc_idx = left_target->index;
    } else if (const auto right_target = find_target_on_side(false)) {
      drivable_lanes.at(i).right_lane = right_target->lane;
      drivable_lanes.at(i).middle_lanes = right_target->middle;
      current_lc_idx = right_target->index;
    }
  }

  for (size_t i = current_lc_idx + 1; i < lane_change_lanes.size(); ++i) {
    const auto & lc_lane = lane_change_lanes.at(i);
    DrivableLanes drivable_lane;
    drivable_lane.left_lane = lc_lane;
    drivable_lane.right_lane = lc_lane;
    drivable_lanes.push_back(drivable_lane);
  }

  return drivable_lanes;
}

void expandDrivableLaneCorridors(
  const RouteHandler & route_handler, std::vector<DrivableLanes> & drivable_lanes)
{
  for (auto & lanes : drivable_lanes) {
    // Preserve the existing maneuver corridor, including intermediate lanes. Its outer
    // edges seed the same legal, directional expansion used by the reference-path builder.
    lanelet::ConstLanelets corridor;
    std::unordered_set<lanelet::Id> visited;
    const auto append = [&](const lanelet::ConstLanelet & lane) {
      if (visited.insert(lane.id()).second) corridor.push_back(lane);
    };
    append(lanes.left_lane);
    for (const auto & lane : lanes.middle_lanes) append(lane);
    append(lanes.right_lane);

    const auto expanded = utils::expandLaneletCorridor(corridor, route_handler);
    lanes.left_lane = expanded.front();
    lanes.right_lane = expanded.back();
    lanes.middle_lanes.clear();
    if (expanded.size() > 2) {
      lanes.middle_lanes.assign(std::next(expanded.begin()), std::prev(expanded.end()));
    }
  }
}

double getLateralShift(const LaneChangePath & path)
{
  if (path.shifted_path.shift_length.empty()) {
    return 0.0;
  }

  const auto start_idx =
    std::min(path.info.shift_line.start_idx, path.shifted_path.shift_length.size() - 1);
  const auto end_idx =
    std::min(path.info.shift_line.end_idx, path.shifted_path.shift_length.size() - 1);

  return path.shifted_path.shift_length.at(end_idx) - path.shifted_path.shift_length.at(start_idx);
}

std::vector<std::vector<int64_t>> get_sorted_lane_ids(const CommonDataPtr & common_data_ptr)
{
  const auto & current_lanes = common_data_ptr->lanes_ptr->current;
  const auto & target_lanes = common_data_ptr->lanes_ptr->target;
  const auto & route_handler = *common_data_ptr->route_handler_ptr;
  const auto & current_pose = common_data_ptr->get_ego_pose();

  const auto rough_shift_length =
    autoware::experimental::lanelet2_utils::get_arc_coordinates(target_lanes, current_pose)
      .distance;

  std::vector<std::vector<int64_t>> sorted_lane_ids{};
  sorted_lane_ids.reserve(target_lanes.size());
  const auto get_sorted_lane_ids = [&](const lanelet::ConstLanelet & target_lane) {
    const auto routing_graph_ptr = route_handler.getRoutingGraphPtr();
    std::vector<int64_t> lane_ids{target_lane.id()};
    auto lane = target_lane;
    std::unordered_set<lanelet::Id> visited{target_lane.id()};
    while (rough_shift_length != 0.0) {
      const auto next_lane =
        rough_shift_length < 0.0 ? routing_graph_ptr->right(lane) : routing_graph_ptr->left(lane);
      if (!next_lane || !visited.insert(next_lane->id()).second) {
        break;
      }
      lane = *next_lane;
      lane_ids.push_back(lane.id());
      const bool reached_current_lane = std::any_of(
        current_lanes.begin(), current_lanes.end(),
        [&](const auto & current_lane) { return current_lane.id() == lane.id(); });
      if (reached_current_lane) {
        std::sort(lane_ids.begin(), lane_ids.end());
        return lane_ids;
      }
    }
    return std::vector{target_lane.id()};
  };

  std::transform(
    target_lanes.cbegin(), target_lanes.cend(), std::back_inserter(sorted_lane_ids),
    get_sorted_lane_ids);

  return sorted_lane_ids;
}

std::vector<int64_t> replace_with_sorted_ids(
  const std::vector<int64_t> & current_lane_ids,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids, std::vector<int64_t> & prev_lane_ids,
  std::vector<int64_t> & prev_sorted_lane_ids)
{
  if (current_lane_ids == prev_lane_ids) {
    return prev_sorted_lane_ids;
  }

  for (const auto original_id : current_lane_ids) {
    for (const auto & sorted_id : sorted_lane_ids) {
      if (std::find(sorted_id.cbegin(), sorted_id.cend(), original_id) != sorted_id.cend()) {
        prev_lane_ids = current_lane_ids;
        prev_sorted_lane_ids = sorted_id;
        return prev_sorted_lane_ids;
      }
    }
  }

  return current_lane_ids;
}

CandidateOutput assignToCandidate(
  const LaneChangePath & lane_change_path, const Point & ego_position)
{
  CandidateOutput candidate_output;
  candidate_output.path_candidate = lane_change_path.path;
  candidate_output.lateral_shift = utils::lane_change::getLateralShift(lane_change_path);
  candidate_output.start_distance_to_path_change = autoware::motion_utils::calcSignedArcLength(
    lane_change_path.path.points, ego_position, lane_change_path.info.shift_line.start.position);
  candidate_output.finish_distance_to_path_change = autoware::motion_utils::calcSignedArcLength(
    lane_change_path.path.points, ego_position, lane_change_path.info.shift_line.end.position);

  return candidate_output;
}

std::optional<lanelet::ConstLanelet> get_lane_change_target_lane(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & current_lanes)
{
  const bool is_mandatory_lc = is_mandatory_lane_change(common_data_ptr->lc_type);
  return get_target_lane(common_data_ptr, current_lanes, is_mandatory_lc);
}

bool should_use_direct_multi_lane_change(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelet & ref_lane)
{
  if (!common_data_ptr->lc_param_ptr->enable_direct_multi_lane_change) {
    return false;
  }

  const auto & route_handler = *common_data_ptr->route_handler_ptr;
  const auto shift_intervals =
    route_handler.getLateralIntervalsToPreferredLane(ref_lane, common_data_ptr->direction);
  if (shift_intervals.size() <= 1) {
    return false;
  }

  const auto sequential_lengths =
    calculation::calc_min_lane_change_lengths(common_data_ptr, shift_intervals);
  const double sequential_distance =
    calculation::calc_distance_buffer(common_data_ptr->lc_param_ptr, sequential_lengths);

  const std::vector<double> direct_shift{
    std::accumulate(shift_intervals.begin(), shift_intervals.end(), 0.0)};
  const auto direct_lengths =
    calculation::calc_min_lane_change_lengths(common_data_ptr, direct_shift);
  const double direct_distance =
    calculation::calc_distance_buffer(common_data_ptr->lc_param_ptr, direct_lengths);
  const double remaining_distance = autoware::behavior_path_planner::utils::getDistanceToEndOfLane(
    common_data_ptr->get_ego_pose(), current_lanes);

  return remaining_distance >= direct_distance && remaining_distance < sequential_distance;
}

std::optional<lanelet::ConstLanelet> get_target_lane_for_mandatory_lane_change(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelet & ref_lane,
  const bool use_direct_multi_lane_change)
{
  const auto direction = common_data_ptr->direction;
  const auto route_handler_ptr = common_data_ptr->route_handler_ptr;
  const auto routing_graph_ptr = route_handler_ptr->getRoutingGraphPtr();

  const bool is_intersection_ll = std::invoke([&]() -> bool {
    const std::string id = ref_lane.attributeOr("intersection_area", "else");
    return id != "else" && std::atoi(id.c_str());
  });

  const int num = route_handler_ptr->getNumLaneToPreferredLane(ref_lane, direction);
  if (num == 0) return std::nullopt;

  if (use_direct_multi_lane_change && !is_intersection_ll && std::abs(num) > 1) {
    auto target_lane = ref_lane;
    for (int i = 0; i < std::abs(num); ++i) {
      const auto next_lane =
        num > 0 ? routing_graph_ptr->left(target_lane) : routing_graph_ptr->right(target_lane);
      if (!next_lane) {
        break;
      }
      target_lane = *next_lane;
    }
    if (route_handler_ptr->getNumLaneToPreferredLane(target_lane, direction) == 0) {
      return target_lane;
    }
  }

  if (direction == Direction::NONE || direction == Direction::RIGHT) {
    if (num < 0) {
      const auto right_lanes = is_intersection_ll ? routing_graph_ptr->adjacentRight(ref_lane)
                                                  : routing_graph_ptr->right(ref_lane);
      if (right_lanes) return *right_lanes;
    }
  }

  if (direction == Direction::NONE || direction == Direction::LEFT) {
    if (num > 0) {
      const auto left_lanes = is_intersection_ll ? routing_graph_ptr->adjacentLeft(ref_lane)
                                                 : routing_graph_ptr->left(ref_lane);
      if (left_lanes) return *left_lanes;
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> get_target_lane_for_non_mandatory_lane_change(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelet & ref_lane)
{
  const auto direction = common_data_ptr->direction;
  const auto route_handler_ptr = common_data_ptr->route_handler_ptr;
  const auto routing_graph_ptr = route_handler_ptr->getRoutingGraphPtr();

  // Avoidance is allowed to move away from the preferred lane when that is the safe side of the
  // obstacle. Select its immediate neighbor without the preferred-lane restriction used by
  // ordinary non-mandatory lane changes. If the avoidance module requested a farther lane after
  // checking every intermediate lane, follow the same route-approved lateral chain to that exact
  // target. Lane-changeable left/right relations preserve travel direction and exclude shoulders
  // and oncoming lanes.
  if (
    common_data_ptr->lc_type == LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE &&
    (direction == Direction::LEFT || direction == Direction::RIGHT)) {
    auto lateral_lane = ref_lane;
    std::unordered_set<lanelet::Id> visited{ref_lane.id()};
    while (true) {
      const auto next_lane = direction == Direction::LEFT ? routing_graph_ptr->left(lateral_lane)
                                                          : routing_graph_ptr->right(lateral_lane);
      if (
        !next_lane || !visited.insert(next_lane->id()).second ||
        !route_handler_ptr->isRouteLanelet(*next_lane)) {
        break;
      }
      if (
        !common_data_ptr->requested_target_lane_id ||
        next_lane->id() == common_data_ptr->requested_target_lane_id.value()) {
        return *next_lane;
      }
      lateral_lane = *next_lane;
    }
    return std::nullopt;
  }

  if (direction == Direction::RIGHT) {
    // Get right lanelet if preferred lane is on the left
    if (route_handler_ptr->getNumLaneToPreferredLane(ref_lane, direction) < 0) {
      return std::nullopt;
    }

    const auto right_lanes = routing_graph_ptr->right(ref_lane);
    if (right_lanes && route_handler_ptr->isRouteLanelet(*right_lanes)) {
      return *right_lanes;
    }
  }

  if (direction == Direction::LEFT) {
    // Get left lanelet if preferred lane is on the right
    if (route_handler_ptr->getNumLaneToPreferredLane(ref_lane, direction) > 0) {
      return std::nullopt;
    }
    const auto left_lanes = routing_graph_ptr->left(ref_lane);
    if (left_lanes && route_handler_ptr->isRouteLanelet(*left_lanes)) {
      return *left_lanes;
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> get_direct_mission_target(
  const CommonDataPtr & common_data_ptr)
{
  if (
    !common_data_ptr || !common_data_ptr->is_data_available() ||
    common_data_ptr->lc_type != LaneChangeModuleType::NORMAL ||
    !common_data_ptr->lc_param_ptr->enable_direct_multi_lane_change) {
    return std::nullopt;
  }
  const auto & route = *common_data_ptr->route_handler_ptr;
  const auto & graph = route.getRoutingGraphPtr();
  lanelet::ConstLanelet lane;
  if (!route.getClosestLaneletWithinRoute(common_data_ptr->get_ego_pose(), &lane)) {
    return std::nullopt;
  }
  const int distance = route.getNumLaneToPreferredLane(lane);
  const auto direction = common_data_ptr->direction;
  if (
    std::abs(distance) < 2 || (distance > 0 && direction != Direction::LEFT) ||
    (distance < 0 && direction != Direction::RIGHT)) {
    return std::nullopt;
  }
  std::unordered_set<lanelet::Id> visited{lane.id()};
  for (int i = 0; i < std::abs(distance); ++i) {
    // Do not use adjacentLeft/Right, shoulders, opposing lanes or intersection shortcuts.
    if (
      lane.attributeOr("intersection_area", std::string{"0"}) != "0" &&
      lane.attributeOr("intersection_area", std::string{"0"}) != "else") {
      return std::nullopt;
    }
    const auto next = distance > 0 ? graph->left(lane) : graph->right(lane);
    if (!next || !visited.insert(next->id()).second || !route.isRouteLanelet(*next)) {
      return std::nullopt;
    }
    lane = *next;
  }
  if (
    lane.attributeOr("intersection_area", std::string{"0"}) != "0" &&
    lane.attributeOr("intersection_area", std::string{"0"}) != "else") {
    return std::nullopt;
  }
  return route.getNumLaneToPreferredLane(lane) == 0 ? std::make_optional(lane) : std::nullopt;
}

std::optional<lanelet::ConstLanelet> get_target_lane(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & current_lanes,
  const bool is_mandatory_lc)
{
  if (
    is_mandatory_lc && common_data_ptr->lc_type == LaneChangeModuleType::NORMAL &&
    common_data_ptr->requested_target_lane_id) {
    const auto direct = get_direct_mission_target(common_data_ptr);
    return direct && direct->id() == *common_data_ptr->requested_target_lane_id ? direct
                                                                                : std::nullopt;
  }
  const auto & ego_pose = common_data_ptr->get_ego_pose();
  const auto & route_handler_ptr = common_data_ptr->route_handler_ptr;
  const auto current_lanes_path =
    route_handler_ptr->getCenterLinePath(current_lanes, 0.0, std::numeric_limits<double>::max());

  auto is_lanelet_behind_ego = [&](const lanelet::ConstLanelet & lanelet) {
    const auto lanelet_end = lanelet.centerline2d().back().basicPoint2d();
    const auto lanelet_end_position =
      autoware_utils::create_point(lanelet_end.x(), lanelet_end.y(), 0.0);
    const auto dist_from_ego = autoware::motion_utils::calcSignedArcLength(
      current_lanes_path.points, ego_pose.position, lanelet_end_position);
    return dist_from_ego < 0.0;
  };

  for (const auto & lanelet : current_lanes) {
    if (is_lanelet_behind_ego(lanelet)) continue;

    const bool use_direct_multi_lane_change =
      is_mandatory_lc &&
      should_use_direct_multi_lane_change(common_data_ptr, current_lanes, lanelet);
    const auto target_lane =
      is_mandatory_lc ? get_target_lane_for_mandatory_lane_change(
                          common_data_ptr, lanelet, use_direct_multi_lane_change)
                      : get_target_lane_for_non_mandatory_lane_change(common_data_ptr, lanelet);

    if (target_lane) return target_lane;
  }

  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> extend_target_lane_backward(
  const lanelet::ConstLanelet & target_lane, const lanelet::ConstLanelets & current_lanes,
  const Direction direction, const double overlap_length)
{
  if (
    current_lanes.empty() || overlap_length <= 0.0 ||
    (direction != Direction::LEFT && direction != Direction::RIGHT)) {
    return std::nullopt;
  }

  const auto current_shape_opt =
    autoware::experimental::lanelet2_utils::combine_lanelets_shape(current_lanes);
  if (!current_shape_opt) {
    return std::nullopt;
  }

  const bool is_left = direction == Direction::LEFT;
  const auto & current_inner_bound =
    is_left ? current_shape_opt->leftBound3d() : current_shape_opt->rightBound3d();
  const auto & target_inner_bound =
    is_left ? target_lane.rightBound3d() : target_lane.leftBound3d();
  const auto & target_outer_bound =
    is_left ? target_lane.leftBound3d() : target_lane.rightBound3d();

  if (
    current_inner_bound.size() < 2 || target_inner_bound.size() < 2 ||
    target_outer_bound.size() < 2) {
    return std::nullopt;
  }

  const auto & target_inner_front = target_inner_bound.front();
  const auto & target_outer_front = target_outer_bound.front();
  const double target_width = std::hypot(
    target_outer_front.x() - target_inner_front.x(),
    target_outer_front.y() - target_inner_front.y());
  if (target_width < 0.5) {
    return std::nullopt;
  }

  struct Projection
  {
    double distance{std::numeric_limits<double>::max()};
    double arc_length{0.0};
  } best;

  std::vector<double> accumulated_length(current_inner_bound.size(), 0.0);
  for (size_t i = 1; i < current_inner_bound.size(); ++i) {
    const auto & p0 = current_inner_bound[i - 1];
    const auto & p1 = current_inner_bound[i];
    const double dx = p1.x() - p0.x();
    const double dy = p1.y() - p0.y();
    const double segment_length = std::hypot(dx, dy);
    accumulated_length.at(i) = accumulated_length.at(i - 1) + segment_length;
    if (segment_length < 1e-6) {
      continue;
    }

    const double ratio = std::clamp(
      ((target_inner_front.x() - p0.x()) * dx + (target_inner_front.y() - p0.y()) * dy) /
        (segment_length * segment_length),
      0.0, 1.0);
    const double x = p0.x() + ratio * dx;
    const double y = p0.y() + ratio * dy;
    const double distance = std::hypot(target_inner_front.x() - x, target_inner_front.y() - y);
    if (distance < best.distance) {
      best.distance = distance;
      best.arc_length = accumulated_length.at(i - 1) + ratio * segment_length;
    }
  }

  // Only extend a real split/opening. A loose threshold is derived from the target lane width so
  // a crossing or unrelated predecessor can never be turned into an overlap lane.
  const double connection_threshold = std::max(0.5, 0.25 * target_width);
  if (best.distance > connection_threshold || best.arc_length < 0.5) {
    return std::nullopt;
  }

  const double overlap_start = std::max(0.0, best.arc_length - overlap_length);
  const double actual_overlap = best.arc_length - overlap_start;
  if (actual_overlap < 0.5) {
    return std::nullopt;
  }

  const auto interpolate_bound_point = [&](const double arc_length) {
    for (size_t i = 1; i < current_inner_bound.size(); ++i) {
      if (accumulated_length.at(i) + 1e-6 < arc_length) {
        continue;
      }
      const auto & p0 = current_inner_bound[i - 1];
      const auto & p1 = current_inner_bound[i];
      const double segment_length = accumulated_length.at(i) - accumulated_length.at(i - 1);
      const double ratio =
        segment_length > 1e-6
          ? std::clamp((arc_length - accumulated_length.at(i - 1)) / segment_length, 0.0, 1.0)
          : 0.0;
      return lanelet::Point3d(
        lanelet::InvalId, p0.x() + ratio * (p1.x() - p0.x()), p0.y() + ratio * (p1.y() - p0.y()),
        p0.z() + ratio * (p1.z() - p0.z()));
    }
    const auto & p = current_inner_bound.back();
    return lanelet::Point3d(lanelet::InvalId, p.x(), p.y(), p.z());
  };

  lanelet::Points3d inner_prefix;
  std::vector<double> inner_prefix_arcs;
  inner_prefix.push_back(interpolate_bound_point(overlap_start));
  inner_prefix_arcs.push_back(overlap_start);
  for (size_t i = 1; i + 1 < current_inner_bound.size(); ++i) {
    if (
      accumulated_length.at(i) > overlap_start + 1e-4 &&
      accumulated_length.at(i) < best.arc_length - 1e-4) {
      const auto & p = current_inner_bound[i];
      inner_prefix.emplace_back(lanelet::InvalId, p.x(), p.y(), p.z());
      inner_prefix_arcs.push_back(accumulated_length.at(i));
    }
  }
  inner_prefix.emplace_back(
    lanelet::InvalId, target_inner_front.x(), target_inner_front.y(), target_inner_front.z());
  inner_prefix_arcs.push_back(best.arc_length);

  lanelet::Points3d outer_prefix;
  outer_prefix.reserve(inner_prefix.size());
  const double offset_x = target_outer_front.x() - target_inner_front.x();
  const double offset_y = target_outer_front.y() - target_inner_front.y();
  const double offset_z = target_outer_front.z() - target_inner_front.z();
  for (size_t i = 0; i < inner_prefix.size(); ++i) {
    const auto & p = inner_prefix.at(i);
    const double ratio =
      std::clamp((inner_prefix_arcs.at(i) - overlap_start) / actual_overlap, 0.0, 1.0);
    const double smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio);
    outer_prefix.emplace_back(
      lanelet::InvalId, p.x() + smooth_ratio * offset_x, p.y() + smooth_ratio * offset_y,
      p.z() + smooth_ratio * offset_z);
  }

  const auto append_original_bound = [](lanelet::Points3d & points, const auto & bound) {
    for (size_t i = 1; i < bound.size(); ++i) {
      points.emplace_back(lanelet::Point3d(bound[i]));
    }
  };
  append_original_bound(inner_prefix, target_inner_bound);
  append_original_bound(outer_prefix, target_outer_bound);

  const auto inner_line =
    lanelet::LineString3d(lanelet::InvalId, inner_prefix, target_inner_bound.attributes());
  const auto outer_line =
    lanelet::LineString3d(lanelet::InvalId, outer_prefix, target_outer_bound.attributes());

  auto target_attributes = target_lane.attributes();
  target_attributes["lane_change_backward_overlap"] = "yes";

  return lanelet::ConstLanelet(
    target_lane.id(), is_left ? outer_line : inner_line, is_left ? inner_line : outer_line,
    target_attributes);
}

bool isParkedObject(
  const PathWithLaneId & path, const RouteHandler & route_handler,
  const ExtendedPredictedObject & object, const double object_check_min_road_shoulder_width,
  const double object_shiftable_ratio_threshold, const double static_object_velocity_threshold)
{
  // ============================================ <- most_left_lanelet.leftBound()
  // y              road shoulder
  // ^ ------------------------------------------
  // |   x                                +
  // +---> --- object closest lanelet --- o ----- <- object_closest_lanelet.centerline()
  //
  // --------------------------------------------
  // +: object position
  // o: nearest point on centerline

  const double object_vel_norm =
    std::hypot(object.initial_twist.linear.x, object.initial_twist.linear.y);
  if (object_vel_norm > static_object_velocity_threshold) {
    return false;
  }

  const auto & object_pose = object.initial_pose;
  const auto object_closest_index =
    autoware::motion_utils::findNearestIndex(path.points, object_pose.position);
  const auto object_closest_pose = path.points.at(object_closest_index).point.pose;

  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler.getClosestLaneletWithinRoute(object_closest_pose, &closest_lanelet)) {
    return false;
  }

  const double lat_dist =
    autoware::motion_utils::calcLateralOffset(path.points, object_pose.position);
  const auto most_side_lanelet =
    lat_dist > 0.0 ? route_handler.getMostLeftLanelet(closest_lanelet, false, true)
                   : route_handler.getMostRightLanelet(closest_lanelet, false, true);
  const auto bound = lat_dist > 0.0 ? most_side_lanelet.leftBound2d().basicLineString()
                                    : most_side_lanelet.rightBound2d().basicLineString();
  const lanelet::Attribute lanelet_sub_type =
    most_side_lanelet.attribute(lanelet::AttributeName::Subtype);
  const auto center_to_bound_buffer =
    lanelet_sub_type.value() == "road_shoulder" ? 0.0 : object_check_min_road_shoulder_width;

  return isParkedObject(
    closest_lanelet, bound, object, center_to_bound_buffer, object_shiftable_ratio_threshold);
}

bool isParkedObject(
  const lanelet::ConstLanelet & closest_lanelet, const lanelet::BasicLineString2d & boundary,
  const ExtendedPredictedObject & object, const double buffer_to_bound,
  const double ratio_threshold)
{
  using lanelet::geometry::distance2d;

  const auto & obj_pose = object.initial_pose;
  const auto & obj_shape = object.shape;
  const auto obj_poly = autoware_utils::to_polygon2d(obj_pose, obj_shape);
  const auto obj_point = obj_pose.position;

  double max_dist_to_bound = std::numeric_limits<double>::lowest();
  double min_dist_to_bound = std::numeric_limits<double>::max();
  for (const auto & edge : obj_poly.outer()) {
    const auto ll_edge = lanelet::Point2d(lanelet::InvalId, edge.x(), edge.y());
    const auto dist = distance2d(boundary, ll_edge);
    max_dist_to_bound = std::max(dist, max_dist_to_bound);
    min_dist_to_bound = std::min(dist, min_dist_to_bound);
  }
  const double obj_width = std::max(max_dist_to_bound - min_dist_to_bound, 0.0);

  // distance from centerline to the boundary line with object width
  const auto centerline_pose = autoware::experimental::lanelet2_utils::get_closest_center_pose(
    closest_lanelet, autoware::experimental::lanelet2_utils::from_ros(obj_point));
  const lanelet::BasicPoint3d centerline_point(
    centerline_pose.position.x, centerline_pose.position.y, centerline_pose.position.z);
  const double dist_bound_to_centerline =
    std::abs(distance2d(boundary, centerline_point)) - 0.5 * obj_width + buffer_to_bound;

  // distance from object point to centerline
  const auto centerline = closest_lanelet.centerline();
  const auto ll_obj_point = lanelet::Point2d(lanelet::InvalId, obj_point.x, obj_point.y);
  const double dist_obj_to_centerline = std::abs(distance2d(centerline, ll_obj_point));

  const double ratio = dist_obj_to_centerline / std::max(dist_bound_to_centerline, 1e-6);
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  return clamped_ratio > ratio_threshold;
}

bool is_delay_lane_change(
  const CommonDataPtr & common_data_ptr, const LaneChangePath & lane_change_path,
  const ExtendedPredictedObjects & target_objects, CollisionCheckDebugMap & object_debug)
{
  const auto & current_lane_path = common_data_ptr->current_lanes_path;
  const auto & delay_lc_param = common_data_ptr->lc_param_ptr->delay;

  if (
    !delay_lc_param.enable || target_objects.empty() || lane_change_path.path.points.empty() ||
    current_lane_path.points.empty()) {
    return false;
  }

  const auto dist_to_end = common_data_ptr->transient_data.dist_to_terminal_end;
  const auto dist_buffer = common_data_ptr->transient_data.current_dist_buffer.min;
  auto is_near_end = [&dist_to_end, &dist_buffer](const ExtendedPredictedObject & obj) {
    const auto dist_obj_to_end = dist_to_end - obj.dist_from_ego;
    return dist_obj_to_end <= dist_buffer;
  };

  const auto ego_vel = common_data_ptr->get_ego_speed();
  const auto min_lon_acc = common_data_ptr->lc_param_ptr->trajectory.min_longitudinal_acc;
  const auto gap_threshold = std::abs((ego_vel * ego_vel) / (2 * min_lon_acc));
  auto is_sufficient_gap = [&gap_threshold](const auto & current_obj, const auto & next_obj) {
    const auto curr_obj_half_length = current_obj.shape.dimensions.x;
    const auto next_obj_half_length = next_obj.shape.dimensions.x;
    const auto dist_current_to_next = next_obj.dist_from_ego - current_obj.dist_from_ego;
    const auto gap_length = dist_current_to_next - curr_obj_half_length - next_obj_half_length;
    return gap_length > gap_threshold;
  };

  for (auto it = target_objects.begin(); it < target_objects.end(); ++it) {
    if (is_near_end(*it)) break;

    if (it->dist_from_ego < lane_change_path.info.length.lane_changing) continue;

    if (
      delay_lc_param.check_only_parked_vehicle &&
      !isParkedObject(
        lane_change_path.path, *common_data_ptr->route_handler_ptr, *it,
        delay_lc_param.min_road_shoulder_width, delay_lc_param.th_parked_vehicle_shift_ratio)) {
      continue;
    }

    auto next_it = std::next(it);
    if (next_it == target_objects.end() || is_sufficient_gap(*it, *next_it)) {
      auto debug = utils::path_safety_checker::createObjectDebug(*it);
      debug.second.unsafe_reason = "delay lane change";
      utils::path_safety_checker::updateCollisionCheckDebugMap(object_debug, debug, false);
      return true;
    }
  }

  return false;
}

lanelet::BasicPolygon2d create_polygon(
  const lanelet::ConstLanelets & lanes, const double start_dist, const double end_dist)
{
  if (lanes.empty()) {
    return {};
  }

  const auto polygon_3d_opt = autoware::experimental::lanelet2_utils::get_polygon_from_arc_length(
    lanes, start_dist, end_dist);

  if (!polygon_3d_opt.has_value()) {
    return {};
  }

  const auto & polygon_3d = polygon_3d_opt.value();
  return lanelet::utils::to2D(polygon_3d).basicPolygon();
}

std::optional<PredictedPathWithPolygon> transform_predicted_path(
  const autoware_perception_msgs::msg::PredictedPath & path,
  const autoware_perception_msgs::msg::Shape & obj_shape, const double obj_normal_velocity,
  const double time_resolution)
{
  if (path.path.empty()) {
    return std::nullopt;
  }
  PredictedPathWithPolygon pred_path_with_poly;
  pred_path_with_poly.confidence = path.confidence;

  const auto end_time =
    rclcpp::Duration(path.time_step).seconds() * static_cast<double>(path.path.size() - 1);
  constexpr auto eps = std::numeric_limits<double>::epsilon();
  const auto num_iterations = static_cast<size_t>(std::ceil(end_time / time_resolution)) + 1;
  pred_path_with_poly.path.reserve(num_iterations);

  for (double t = 0.0; t < end_time + eps; t += time_resolution) {
    if (
      const auto obj_pose_opt = autoware::object_recognition_utils::calcInterpolatedPose(path, t)) {
      const auto obj_polygon = autoware_utils::to_polygon2d(*obj_pose_opt, obj_shape);
      pred_path_with_poly.path.emplace_back(t, *obj_pose_opt, obj_normal_velocity, obj_polygon);
    }
  }

  if (pred_path_with_poly.path.empty()) {
    return std::nullopt;
  }

  return pred_path_with_poly;
}

ExtendedPredictedObject transform(
  const PredictedObject & object, const LaneChangeParameters & lane_change_parameters)
{
  ExtendedPredictedObject extended_object(object);

  const auto & time_resolution =
    lane_change_parameters.safety.collision_check.prediction_time_resolution;
  const double obj_vel_norm =
    std::hypot(extended_object.initial_twist.linear.x, extended_object.initial_twist.linear.y);

  const auto object_predicted_paths = path_safety_checker::get_object_predicted_paths(
    object.kinematics.predicted_paths,
    lane_change_parameters.safety.collision_check.use_all_predicted_paths);

  extended_object.predicted_paths.reserve(object.kinematics.predicted_paths.size());

  for (const auto & pred_path : object_predicted_paths) {
    if (
      const auto ext_path_opt =
        transform_predicted_path(pred_path, object.shape, obj_vel_norm, time_resolution)) {
      extended_object.predicted_paths.push_back(*ext_path_opt);
    }
  }

  return extended_object;
}

bool is_collided_polygons_in_lanelet(
  const std::vector<Polygon2d> & collided_polygons, const lanelet::BasicPolygon2d & lanes_polygon)
{
  const auto is_in_lanes = [&](const auto & collided_polygon) {
    return !lanes_polygon.empty() && !boost::geometry::disjoint(collided_polygon, lanes_polygon);
  };

  return std::any_of(collided_polygons.begin(), collided_polygons.end(), is_in_lanes);
}

lanelet::ConstLanelets generateExpandedLanelets(
  const lanelet::ConstLanelets & lanes, const Direction direction, const double left_offset,
  const double right_offset)
{
  const auto left_extend_offset = (direction == Direction::LEFT) ? left_offset : 0.0;
  const auto right_extend_offset = (direction == Direction::RIGHT) ? -right_offset : 0.0;

  const auto expand_lanelets_opt =
    autoware::experimental::lanelet2_utils::get_dirty_expanded_lanelets(
      lanes, left_extend_offset, right_extend_offset);
  if (expand_lanelets_opt) {
    return *expand_lanelets_opt;
  }

  return lanes;
}

rclcpp::Logger getLogger(const std::string & type)
{
  return rclcpp::get_logger("lane_change").get_child(type);
}

Polygon2d get_ego_footprint(const Pose & ego_pose, const VehicleInfo & ego_info)
{
  const auto base_to_front = ego_info.max_longitudinal_offset_m;
  const auto base_to_rear = ego_info.rear_overhang_m;
  const auto width = ego_info.vehicle_width_m;

  return autoware_utils::to_footprint(ego_pose, base_to_front, base_to_rear, width);
}

Point getEgoFrontVertex(
  const Pose & ego_pose, const autoware::vehicle_info_utils::VehicleInfo & ego_info, bool left)
{
  const double lon_offset = ego_info.wheel_base_m + ego_info.front_overhang_m;
  const double lat_offset = 0.5 * (left ? ego_info.vehicle_width_m : -ego_info.vehicle_width_m);
  return autoware_utils::calc_offset_pose(ego_pose, lon_offset, lat_offset, 0.0).position;
}

bool is_within_intersection(
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelet & lanelet,
  const Polygon2d & polygon)
{
  const std::string id = lanelet.attributeOr("intersection_area", "else");
  if (id == "else" || !std::atoi(id.c_str())) {
    return false;
  }

  if (!route_handler || !route_handler->getLaneletMapPtr()) {
    return false;
  }

  const auto & polygon_layer = route_handler->getLaneletMapPtr()->polygonLayer;
  const auto lanelet_polygon_opt = polygon_layer.find(std::atoi(id.c_str()));
  if (lanelet_polygon_opt == polygon_layer.end()) {
    return false;
  }
  const auto & lanelet_polygon = *lanelet_polygon_opt;

  return boost::geometry::within(
    polygon, utils::toPolygon2d(lanelet::utils::to2D(lanelet_polygon.basicPolygon())));
}

bool is_within_turn_direction_lanes(
  const lanelet::ConstLanelet & lanelet, const Polygon2d & polygon)
{
  const std::string turn_direction = lanelet.attributeOr("turn_direction", "else");
  if (turn_direction == "else" || turn_direction == "straight") {
    return false;
  }

  return !boost::geometry::disjoint(
    polygon, utils::toPolygon2d(lanelet::utils::to2D(lanelet.polygon2d().basicPolygon())));
}

LanesPolygon create_lanes_polygon(const CommonDataPtr & common_data_ptr)
{
  const auto & lanes = common_data_ptr->lanes_ptr;
  LanesPolygon lanes_polygon;

  lanes_polygon.current =
    utils::lane_change::create_polygon(lanes->current, 0.0, std::numeric_limits<double>::max());

  lanes_polygon.target =
    utils::lane_change::create_polygon(lanes->target, 0.0, std::numeric_limits<double>::max());

  const auto & params = common_data_ptr->lc_param_ptr->safety;
  const auto expanded_target_lanes = utils::lane_change::generateExpandedLanelets(
    lanes->target, common_data_ptr->direction, params.lane_expansion_left_offset,
    params.lane_expansion_right_offset);
  lanes_polygon.expanded_target = utils::lane_change::create_polygon(
    expanded_target_lanes, 0.0, std::numeric_limits<double>::max());

  lanes_polygon.target_neighbor = utils::lane_change::create_polygon(
    lanes->target_neighbor, 0.0, std::numeric_limits<double>::max());

  lanes_polygon.preceding_target.reserve(lanes->preceding_target.size());
  for (const auto & preceding_lane : lanes->preceding_target) {
    auto lane_polygon =
      utils::lane_change::create_polygon(preceding_lane, 0.0, std::numeric_limits<double>::max());

    if (!lane_polygon.empty()) {
      lanes_polygon.preceding_target.push_back(lane_polygon);
    }
  }
  return lanes_polygon;
}

bool is_same_lane_with_prev_iteration(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes)
{
  if (current_lanes.empty() || target_lanes.empty()) {
    return false;
  }
  const auto & prev_current_lanes = common_data_ptr->lanes_ptr->current;
  const auto & prev_target_lanes = common_data_ptr->lanes_ptr->target;
  if (prev_current_lanes.empty() || prev_target_lanes.empty()) {
    return false;
  }

  if (
    (prev_current_lanes.front().id() != current_lanes.front().id()) ||
    (prev_current_lanes.back().id() != current_lanes.back().id())) {
    return false;
  }
  return (prev_target_lanes.front().id() == target_lanes.front().id()) &&
         (prev_target_lanes.back().id() == target_lanes.back().id());
}

MinMaxValue calc_polygon_dist_range_from_terminal_end(
  const PathWithLaneId & path, const autoware_utils_geometry::Polygon2d & polygon)
{
  MinMaxValue dist_from_terminal_end;

  const auto & vertices = polygon.outer();
  if (path.points.empty() || vertices.empty()) {
    return {};
  }

  dist_from_terminal_end.max = -std::numeric_limits<double>::infinity();
  dist_from_terminal_end.min = std::numeric_limits<double>::infinity();

  for (const auto & vertex : vertices) {
    const auto vertex_pt = autoware_utils::create_point(vertex.x(), vertex.y(), 0.0);
    const auto dist_to_end = autoware::motion_utils::calcSignedArcLength(
      path.points, vertex_pt, path.points.back().point.pose.position);
    dist_from_terminal_end.min = std::min(dist_to_end, dist_from_terminal_end.min);
    dist_from_terminal_end.max = std::max(dist_to_end, dist_from_terminal_end.max);
  }

  return dist_from_terminal_end;
}

EgoObjectProximity calc_ego_object_proximity(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & path,
  const ExtendedPredictedObject & object)
{
  const auto & ego_info = common_data_ptr->bpp_param_ptr->vehicle_info;
  const auto lon_dev = std::max(
    ego_info.max_longitudinal_offset_m + ego_info.rear_overhang_m, object.shape.dimensions.x);

  // we don't always have to check the distance accurately.
  EgoObjectProximity ego_obj_proximity;
  if (std::abs(object.dist_from_ego) > lon_dev) {
    ego_obj_proximity.is_ahead_of_ego = object.dist_from_ego >= 0.0;
    return ego_obj_proximity;
  }

  ego_obj_proximity.ego_dist_to_terminal_end =
    common_data_ptr->transient_data.ego_to_terminal_end_proximity;
  ego_obj_proximity.object_dist_to_terminal_end =
    calc_polygon_dist_range_from_terminal_end(path, object.initial_polygon);

  if (ego_obj_proximity.ego_dist_to_terminal_end && ego_obj_proximity.object_dist_to_terminal_end) {
    ego_obj_proximity.is_ahead_of_ego =
      (ego_obj_proximity.ego_dist_to_terminal_end->min >=
       ego_obj_proximity.object_dist_to_terminal_end->max);
  }

  return ego_obj_proximity;
}

bool is_before_terminal(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & path,
  const ExtendedPredictedObject & object)
{
  const auto & route_handler_ptr = common_data_ptr->route_handler_ptr;
  const auto & lanes_ptr = common_data_ptr->lanes_ptr;
  const auto terminal_position = (lanes_ptr->current_lane_in_goal_section)
                                   ? route_handler_ptr->getGoalPose().position
                                   : path.points.back().point.pose.position;
  double current_max_dist = std::numeric_limits<double>::lowest();

  const auto & obj_position = object.initial_pose.position;
  const auto dist_to_base_link =
    autoware::motion_utils::calcSignedArcLength(path.points, obj_position, terminal_position);
  // we don't always have to check the distance accurately.
  if (std::abs(dist_to_base_link) > object.shape.dimensions.x) {
    return dist_to_base_link >= 0.0;
  }

  for (const auto & polygon_p : object.initial_polygon.outer()) {
    const auto obj_p = autoware_utils::create_point(polygon_p.x(), polygon_p.y(), 0.0);
    const auto dist_obj_to_terminal =
      autoware::motion_utils::calcSignedArcLength(path.points, obj_p, terminal_position);
    current_max_dist = std::max(dist_obj_to_terminal, current_max_dist);
  }
  return current_max_dist >= 0.0;
}

double calc_angle_to_lanelet_segment(const lanelet::ConstLanelets & lanelets, const Pose & pose)
{
  const auto closest_lanelet_opt =
    autoware::experimental::lanelet2_utils::get_closest_lanelet(lanelets, pose);

  if (!closest_lanelet_opt) {
    return autoware_utils::deg2rad(180);
  }
  const auto & closest_lanelet = closest_lanelet_opt.value();
  const auto closest_pose = autoware::experimental::lanelet2_utils::get_closest_center_pose(
    closest_lanelet, autoware::experimental::lanelet2_utils::from_ros(pose));
  return std::abs(autoware_utils::calc_yaw_deviation(closest_pose, pose));
}

double get_distance_to_next_regulatory_element(
  const CommonDataPtr & common_data_ptr, const bool ignore_crosswalk,
  const bool ignore_intersection)
{
  double distance = std::numeric_limits<double>::max();

  const auto current_pose = common_data_ptr->get_ego_pose();
  const auto & current_lanes = common_data_ptr->lanes_ptr->current;
  const auto & route_handler = *common_data_ptr->route_handler_ptr;
  const auto overall_graphs_ptr = route_handler.getOverallGraphPtr();

  if (!ignore_intersection && common_data_ptr->lc_param_ptr->regulate_on_intersection) {
    distance =
      std::min(distance, utils::getDistanceToNextIntersection(current_pose, current_lanes));
  }
  if (!ignore_crosswalk && common_data_ptr->lc_param_ptr->regulate_on_crosswalk) {
    distance = std::min(
      distance, utils::getDistanceToCrosswalk(current_pose, current_lanes, *overall_graphs_ptr));
  }
  if (common_data_ptr->lc_param_ptr->regulate_on_traffic_light) {
    distance = std::min(
      distance, utils::traffic_light::getDistanceToNextTrafficLight(current_pose, current_lanes));
  }

  return distance;
}

double get_min_dist_to_current_lanes_obj(
  const CommonDataPtr & common_data_ptr, const FilteredLanesObjects & filtered_objects,
  const double dist_to_target_lane_start, const PathWithLaneId & path)
{
  const auto & path_points = path.points;
  auto min_dist_to_obj = std::numeric_limits<double>::max();
  for (const auto & object : filtered_objects.current_lane) {
    // check if stationary
    const auto obj_v = std::abs(object.initial_twist.linear.x);
    if (obj_v > common_data_ptr->lc_param_ptr->th_stop_velocity) {
      continue;
    }

    // provide "estimation" based on size of object
    const auto dist_to_obj =
      motion_utils::calcSignedArcLength(
        path_points, path_points.front().point.pose.position, object.initial_pose.position) -
      (object.shape.dimensions.x / 2);

    if (dist_to_obj < dist_to_target_lane_start) {
      continue;
    }

    // check if object is on ego path
    const auto obj_half_width = object.shape.dimensions.y / 2;
    const auto obj_lat_dist_to_path =
      std::abs(motion_utils::calcLateralOffset(path_points, object.initial_pose.position)) -
      obj_half_width;
    if (obj_lat_dist_to_path > (common_data_ptr->bpp_param_ptr->vehicle_width / 2)) {
      continue;
    }

    min_dist_to_obj = std::min(min_dist_to_obj, dist_to_obj);
    break;
  }
  return min_dist_to_obj;
}

bool has_blocking_target_object(
  const TargetLaneLeadingObjects & target_leading_objects, const double stop_arc_length,
  const PathWithLaneId & path)
{
  return ranges::any_of(target_leading_objects.stopped, [&](const auto & object) {
    const auto arc_length_to_target_lane_obj = motion_utils::calcSignedArcLength(
      path.points, path.points.front().point.pose.position, object.initial_pose.position);
    const auto width_margin = object.shape.dimensions.x / 2;
    return (arc_length_to_target_lane_obj - width_margin) >= stop_arc_length;
  });
}

bool has_passed_intersection_turn_direction(const CommonDataPtr & common_data_ptr)
{
  const auto & transient_data = common_data_ptr->transient_data;
  if (transient_data.in_intersection && transient_data.in_turn_direction_lane) {
    return false;
  }

  return transient_data.dist_from_prev_intersection >
         common_data_ptr->lc_param_ptr->backward_length_from_intersection;
}

std::vector<LineString2d> get_line_string_paths(const ExtendedPredictedObject & object)
{
  const auto to_linestring_2d = [](const auto & predicted_path) -> LineString2d {
    LineString2d line_string;
    const auto & path = predicted_path.path;
    line_string.reserve(path.size());
    for (const auto & path_point : path) {
      const auto point = autoware_utils::from_msg(path_point.pose.position).to_2d();
      line_string.push_back(point);
    }

    return line_string;
  };

  return object.predicted_paths | ranges::views::transform(to_linestring_2d) |
         ranges::to<std::vector>();
}

bool has_overtaking_turn_lane_object(
  const CommonDataPtr & common_data_ptr, const ExtendedPredictedObjects & trailing_objects)
{
  // Note: This situation is only applicable if the ego is in a turn lane.
  if (has_passed_intersection_turn_direction(common_data_ptr)) {
    return false;
  }

  const auto is_object_overlap_with_target = [&](const auto & object) {
    // to compensate for perception issue, or if object is from behind ego, and tries to overtake,
    // but stop all of sudden
    if (!boost::geometry::disjoint(
          object.initial_polygon, common_data_ptr->lanes_polygon_ptr->current)) {
      return true;
    }

    return object_path_overlaps_lanes(object, common_data_ptr->lanes_polygon_ptr->target);
  };

  return std::any_of(
    trailing_objects.begin(), trailing_objects.end(), is_object_overlap_with_target);
}

bool filter_target_lane_objects(
  const CommonDataPtr & common_data_ptr, const ExtendedPredictedObject & object,
  const double dist_ego_to_current_lanes_center, const EgoObjectProximity & ego_object_proximity,
  const bool before_terminal, TargetLaneLeadingObjects & leading_objects,
  ExtendedPredictedObjects & trailing_objects)
{
  using behavior_path_planner::utils::path_safety_checker::filter::is_vehicle;
  using behavior_path_planner::utils::path_safety_checker::filter::velocity_filter;
  const auto & current_lanes = common_data_ptr->lanes_ptr->current;
  const auto & vehicle_width = common_data_ptr->bpp_param_ptr->vehicle_info.vehicle_width_m;
  const auto & lanes_polygon = *common_data_ptr->lanes_polygon_ptr;
  const auto stopped_obj_vel_th = common_data_ptr->lc_param_ptr->safety.th_stopped_object_velocity;

  const auto is_lateral_far = std::invoke([&]() -> bool {
    const auto dist_object_to_current_lanes_center =
      autoware::experimental::lanelet2_utils::get_lateral_distance_to_centerline(
        current_lanes, object.initial_pose);
    const auto lateral = dist_object_to_current_lanes_center - dist_ego_to_current_lanes_center;
    return std::abs(lateral) > (vehicle_width / 2);
  });

  const auto ahead_of_ego = ego_object_proximity.is_ahead_of_ego;

  const auto is_stopped = velocity_filter(
    object.initial_twist, -std::numeric_limits<double>::epsilon(), stopped_obj_vel_th);
  if (is_lateral_far && before_terminal) {
    const auto overlapping_with_target_lanes =
      !boost::geometry::disjoint(object.initial_polygon, lanes_polygon.target) ||
      (!is_stopped && is_vehicle(object.classification) &&
       object_path_overlaps_lanes(object, lanes_polygon.target));

    if (overlapping_with_target_lanes) {
      if ((!ahead_of_ego && !is_stopped) || ego_object_proximity.is_overlapping()) {
        trailing_objects.push_back(object);
        return true;
      }

      if (ahead_of_ego) {
        if (is_stopped) {
          leading_objects.stopped.push_back(object);
        } else {
          leading_objects.moving.push_back(object);
        }
        return true;
      }
    }

    // Check if the object is positioned outside the lane boundary but still close to its edge.
    const auto in_expanded_target_lanes =
      !boost::geometry::disjoint(object.initial_polygon, lanes_polygon.expanded_target);

    if (in_expanded_target_lanes && is_stopped && ahead_of_ego) {
      leading_objects.stopped_at_bound.push_back(object);
      return true;
    }
  }

  const auto is_overlap_target_backward =
    ranges::any_of(lanes_polygon.preceding_target, [&](const auto & target_backward_polygon) {
      return !boost::geometry::disjoint(object.initial_polygon, target_backward_polygon);
    });

  // check if the object intersects with target backward lanes
  if (is_overlap_target_backward && !is_stopped) {
    trailing_objects.push_back(object);
    return true;
  }

  return false;
}

std::vector<lanelet::ConstLanelets> get_preceding_lanes(const CommonDataPtr & common_data_ptr)
{
  const auto & route_handler_ptr = common_data_ptr->route_handler_ptr;
  const auto & target_lanes = common_data_ptr->lanes_ptr->target;
  const auto & ego_pose = common_data_ptr->get_ego_pose();
  const auto backward_lane_length = common_data_ptr->lc_param_ptr->backward_lane_length;

  const auto preceding_lanes_list =
    utils::getPrecedingLanelets(*route_handler_ptr, target_lanes, ego_pose, backward_lane_length);

  const auto & current_lanes = common_data_ptr->lanes_ptr->current;
  std::unordered_set<lanelet::Id> current_lanes_id;
  for (const auto & lane : current_lanes) {
    current_lanes_id.insert(lane.id());
  }
  const auto is_overlapping = [&](const lanelet::ConstLanelet & lane) {
    return current_lanes_id.find(lane.id()) != current_lanes_id.end();
  };

  std::vector<lanelet::ConstLanelets> non_overlapping_lanes_vec;
  for (const auto & lanes : preceding_lanes_list) {
    auto lanes_reversed = lanes | ranges::views::reverse;
    auto overlapped_itr = ranges::find_if(lanes_reversed, is_overlapping);

    if (overlapped_itr == lanes_reversed.begin()) {
      continue;
    }

    // Lanes are not reversed by default. Avoid returning reversed lanes to prevent undefined
    // behavior.
    lanelet::ConstLanelets non_overlapping_lanes(overlapped_itr.base(), lanes.end());
    non_overlapping_lanes_vec.push_back(non_overlapping_lanes);
  }

  return non_overlapping_lanes_vec;
}

bool object_path_overlaps_lanes(
  const ExtendedPredictedObject & object, const lanelet::BasicPolygon2d & lanes_polygon)
{
  return ranges::any_of(get_line_string_paths(object), [&](const auto & path) {
    return !boost::geometry::disjoint(path, lanes_polygon);
  });
}

std::vector<PoseWithVelocityStamped> convert_to_predicted_path(
  const CommonDataPtr & common_data_ptr, const LaneChangePath & lane_change_path,
  const double lane_changing_acceleration)
{
  if (lane_change_path.path.points.empty()) {
    return {};
  }

  const auto & path = lane_change_path.path;
  const auto & vehicle_pose = common_data_ptr->get_ego_pose();
  const auto & bpp_param_ptr = common_data_ptr->bpp_param_ptr;
  const auto ego_arc_length = calc_arc_length(path.points, vehicle_pose, *bpp_param_ptr);

  const auto initial_velocity = common_data_ptr->get_ego_speed();
  const auto prepare_acc = lane_change_path.info.longitudinal_acceleration.prepare;
  const auto duration = lane_change_path.info.duration.sum();
  const auto prepare_time = lane_change_path.info.duration.prepare;
  const auto & lc_param_ptr = common_data_ptr->lc_param_ptr;
  const auto resolution = lc_param_ptr->safety.collision_check.prediction_time_resolution;
  std::vector<PoseWithVelocityStamped> predicted_path;
  predicted_path.reserve(static_cast<size_t>(std::ceil(duration / resolution)));

  // prepare segment
  for (double t = 0.0; t < prepare_time; t += resolution) {
    const auto velocity =
      std::clamp(initial_velocity + prepare_acc * t, 0.0, lane_change_path.info.velocity.prepare);
    const auto length = initial_velocity * t + 0.5 * prepare_acc * t * t;
    const auto pose =
      autoware::motion_utils::calcInterpolatedPose(path.points, ego_arc_length + length);
    predicted_path.emplace_back(t, pose, velocity);
  }

  // lane changing segment
  const auto lane_changing_velocity = std::clamp(
    initial_velocity + prepare_acc * prepare_time, 0.0, lane_change_path.info.velocity.prepare);
  const auto offset =
    initial_velocity * prepare_time + 0.5 * prepare_acc * prepare_time * prepare_time;

  for (double t = prepare_time; t < duration; t += resolution) {
    const auto delta_t = t - prepare_time;
    const auto velocity = std::clamp(
      lane_changing_velocity + lane_changing_acceleration * delta_t, 0.0,
      lane_change_path.info.velocity.lane_changing);
    const auto length = lane_changing_velocity * delta_t +
                        0.5 * lane_changing_acceleration * delta_t * delta_t + offset;
    const auto pose =
      autoware::motion_utils::calcInterpolatedPose(path.points, ego_arc_length + length);

    predicted_path.emplace_back(t, pose, velocity);
  }

  return predicted_path;
}

std::vector<PoseWithVelocityStamped> convert_to_predicted_path(
  const CommonDataPtr & common_data_ptr, const LaneChangePath & lane_change_path)
{
  if (lane_change_path.path.points.empty()) {
    return {};
  }

  const auto & path = lane_change_path.path;
  const auto & vehicle_pose = common_data_ptr->get_ego_pose();
  const auto & bpp_param_ptr = common_data_ptr->bpp_param_ptr;
  const auto ego_arc_length = calc_arc_length(path.points, vehicle_pose, *bpp_param_ptr);

  const auto & lc_start = lane_change_path.info.lane_changing_start;
  const auto lc_start_arc_length = calc_arc_length(path.points, lc_start, *bpp_param_ptr);

  const auto & lc_end = lane_change_path.info.lane_changing_end;
  const auto lc_end_arc_length = calc_arc_length(path.points, lc_end, *bpp_param_ptr);

  const auto initial_velocity = common_data_ptr->get_ego_speed();
  const auto duration = lane_change_path.info.duration.sum();
  const auto & lc_param_ptr = common_data_ptr->lc_param_ptr;
  const auto resolution = lc_param_ptr->safety.collision_check.prediction_time_resolution;
  std::vector<PoseWithVelocityStamped> predicted_path;
  predicted_path.reserve(static_cast<size_t>(std::ceil(duration / resolution)));
  const auto curr_acc = std::max(common_data_ptr->get_current_accel(), 0.0);

  auto prev_vel = initial_velocity;
  const auto calc_velocity = [&](const auto dt) {
    auto vel = prev_vel + curr_acc * dt;
    if (ego_arc_length < lc_start_arc_length) {
      vel = std::clamp(vel, 0.0, lane_change_path.info.velocity.prepare);
    } else {
      vel = std::clamp(
        vel,
        std::min(
          lc_param_ptr->trajectory.min_lane_changing_velocity,
          lane_change_path.info.velocity.lane_changing),
        lane_change_path.info.velocity.lane_changing);
    }
    return vel;
  };

  for (double t = 0.0; t < duration; t += resolution) {
    const auto velocity = calc_velocity(t);
    const auto length = velocity * t + 0.5 * curr_acc * t * t;
    const auto offset = std::min(ego_arc_length + length, lc_end_arc_length);
    const auto pose = autoware::motion_utils::calcInterpolatedPose(path.points, offset);

    predicted_path.emplace_back(t, pose, velocity);
    if (offset + std::numeric_limits<double>::epsilon() > lc_end_arc_length) {
      break;
    }
    prev_vel = velocity;
  }

  return predicted_path;
}

std::vector<std::vector<PoseWithVelocityStamped>> convert_to_predicted_paths(
  const CommonDataPtr & common_data_ptr, const LaneChangePath & lane_change_path,
  const size_t deceleration_sampling_num, const bool is_approved)
{
  if (lane_change_path.info.braking_profile) {
    const auto & points = lane_change_path.path.points;
    const auto & info = lane_change_path.info;
    const auto & profile = *info.braking_profile;
    if (points.size() < 3) return {};
    const auto & c = *common_data_ptr;
    const auto & p = *c.lc_param_ptr;
    const double resolution = p.safety.collision_check.prediction_time_resolution;
    const double min_acc = std::max(c.bpp_param_ptr->min_acc, p.trajectory.min_longitudinal_acc);
    const double max_acc = std::min(c.bpp_param_ptr->max_acc, p.trajectory.max_longitudinal_acc);
    if (resolution <= 0.0 || min_acc >= 0.0 || max_acc <= 0.0) return {};
    std::vector<double> arcs(points.size(), 0.0), curvature(points.size(), 0.0);
    for (size_t i = 1; i < points.size(); ++i) {
      arcs[i] = arcs[i - 1] + autoware_utils::calc_distance2d(points[i - 1], points[i]);
      if (arcs[i] <= arcs[i - 1]) return {};
      if (i + 1 < points.size())
        curvature[i] = autoware_utils::calc_curvature(
          points[i - 1].point.pose.position, points[i].point.pose.position,
          points[i + 1].point.pose.position);
    }
    const auto arc_of = [&](const auto & position) {
      return motion_utils::calcSignedArcLength(
        points, points.front().point.pose.position, position);
    };
    const double origin = arc_of(info.braking_start_pose.position);
    const double end = arc_of(info.lane_changing_end.position);
    double arc = arc_of(c.get_ego_pose().position);
    double velocity = c.get_ego_speed();
    double acceleration = c.get_current_accel();
    if (
      !std::isfinite(velocity) || !std::isfinite(acceleration) || velocity < 0.0 ||
      acceleration < min_acc - 1e-3 || acceleration > max_acc + 1e-3)
      return {};
    std::vector<PoseWithVelocityStamped> prediction;
    double next_sample = 0.0;
    const double dt = std::min(0.02, resolution);
    // Start from MEASURED velocity/acceleration, including after approval. Follow the same
    // spatial braking profile with bounded acceleration and jerk; never clamp ego to cruise.
    const double horizon = std::min(60.0, info.duration.sum() + 10.0);
    for (double time = 0.0; time <= horizon; time += dt) {
      const auto it = std::upper_bound(arcs.begin(), arcs.end(), arc);
      const size_t i = std::clamp<size_t>(std::distance(arcs.begin(), it), 1, points.size() - 2);
      const double ratio = std::clamp((arc - arcs[i - 1]) / (arcs[i] - arcs[i - 1]), 0.0, 1.0);
      const double k = curvature[i - 1] + ratio * (curvature[i] - curvature[i - 1]);
      const double dk = (curvature[i] - curvature[i - 1]) / (arcs[i] - arcs[i - 1]);
      if (
        !std::isfinite(k) || std::abs(k) > c.bpp_param_ptr->vehicle_info.calcMaxCurvature() ||
        (p.trajectory.enable_lateral_acceleration_limit &&
         velocity * velocity * std::abs(k) >
           p.trajectory.lat_acc_map.find(velocity).second + 1e-3) ||
        (p.trajectory.enable_lateral_jerk_limit &&
         std::abs(calculation::calc_lateral_jerk(velocity, acceleration, k, dk)) >
           p.trajectory.lateral_jerk + 1e-3))
        return {};
      if (time + 1e-6 >= next_sample || arc >= end) {
        const auto pose =
          time == 0.0 ? c.get_ego_pose() : motion_utils::calcInterpolatedPose(points, arc, false);
        prediction.emplace_back(time, pose, velocity);
        next_sample += resolution;
      }
      if (arc >= end) return {prediction};
      if (arc < arcs.front() || arc > arcs.back()) return {};
      const auto reference = profile.at_distance(std::max(0.0, arc - origin));
      const double target_acc =
        std::clamp(reference.acceleration + (reference.velocity - velocity), min_acc, max_acc);
      const double next_acc =
        acceleration + std::clamp(
                         target_acc - acceleration, p.trajectory_safety.min_jerk * dt,
                         p.trajectory_safety.max_jerk * dt);
      const double next_velocity = velocity + 0.5 * (acceleration + next_acc) * dt;
      if (next_velocity < 0.0 || !std::isfinite(next_velocity)) return {};
      arc = std::min(end, arc + 0.5 * (velocity + next_velocity) * dt);
      velocity = next_velocity;
      acceleration = next_acc;
    }
    return {};  // An incomplete prediction is not a clear maneuver.
  }
  if (lane_change_path.type == PathType::LowSpeed) {
    const auto & path = lane_change_path.path;
    if (path.points.size() < 2) return {};
    const auto start = motion_utils::calcSignedArcLength(
      path.points, path.points.front().point.pose.position,
      common_data_ptr->get_ego_pose().position);
    const auto end = motion_utils::calcSignedArcLength(
      path.points, path.points.front().point.pose.position,
      lane_change_path.info.lane_changing_end.position);
    const auto max_velocity = lane_change_path.info.velocity.lane_changing;
    const auto acceleration = std::min(
      common_data_ptr->bpp_param_ptr->max_acc,
      common_data_ptr->lc_param_ptr->trajectory.max_longitudinal_acc);
    const auto dt =
      common_data_ptr->lc_param_ptr->safety.collision_check.prediction_time_resolution;
    if (dt <= 0.0 || acceleration <= 0.0 || max_velocity <= 0.0) return {};
    std::vector<PoseWithVelocityStamped> prediction;
    auto arc = start;
    auto velocity = std::clamp(common_data_ptr->get_ego_speed(), 0.0, max_velocity);
    const auto jerk = common_data_ptr->lc_param_ptr->trajectory_safety.max_jerk;
    if (!std::isfinite(jerk) || jerk <= 0.0) return {};
    auto current_acceleration = 0.0;
    const auto duration = lane_change_path.info.duration.sum() + acceleration / jerk;
    for (double time = 0.0; time <= duration + dt; time += dt) {
      const auto pose = time == 0.0 ? common_data_ptr->get_ego_pose()
                                    : motion_utils::calcInterpolatedPose(path.points, arc, false);
      prediction.emplace_back(time, pose, velocity);
      if (arc >= end) break;
      const auto next_acceleration = std::min(acceleration, current_acceleration + jerk * dt);
      const auto next_velocity =
        std::min(max_velocity, velocity + (current_acceleration + next_acceleration) * dt * 0.5);
      arc = std::min(end, arc + (velocity + next_velocity) * dt * 0.5);
      velocity = next_velocity;
      current_acceleration = next_acceleration;
    }
    return {prediction};
  }
  static constexpr double floating_err_th{1e-3};
  const auto bpp_param = *common_data_ptr->bpp_param_ptr;
  const auto global_min_acc = bpp_param.min_acc;
  const auto lane_changing_acc = lane_change_path.info.longitudinal_acceleration.lane_changing;

  const auto min_acc = std::min(lane_changing_acc, global_min_acc);
  const auto sampling_num =
    std::abs(min_acc - lane_changing_acc) > floating_err_th ? deceleration_sampling_num : 1;
  const auto acc_resolution = (min_acc - lane_changing_acc) / static_cast<double>(sampling_num);

  const auto ego_predicted_path = [&](size_t n) {
    if (lane_change_path.type == PathType::FrenetPlanner) {
      return convert_to_predicted_path(
        common_data_ptr, lane_change_path.frenet_path, deceleration_sampling_num);
    }
    auto acc = lane_changing_acc + static_cast<double>(n) * acc_resolution;
    if (is_approved) {
      return utils::lane_change::convert_to_predicted_path(common_data_ptr, lane_change_path);
    }
    return utils::lane_change::convert_to_predicted_path(common_data_ptr, lane_change_path, acc);
  };

  return ranges::views::iota(0UL, sampling_num) | ranges::views::transform(ego_predicted_path) |
         ranges::to<std::vector>();
}

std::vector<PoseWithVelocityStamped> convert_to_predicted_path(
  const CommonDataPtr & common_data_ptr, const lane_change::TrajectoryGroup & frenet_candidate,
  [[maybe_unused]] const size_t deceleration_sampling_num)
{
  const auto initial_velocity = common_data_ptr->get_ego_speed();
  const auto prepare_time = frenet_candidate.prepare_metric.duration;
  const auto resolution =
    common_data_ptr->lc_param_ptr->safety.collision_check.prediction_time_resolution;
  const auto prepare_acc = frenet_candidate.prepare_metric.sampled_lon_accel;
  std::vector<PoseWithVelocityStamped> predicted_path;
  const auto & path = frenet_candidate.prepare.points;
  const auto & vehicle_pose = common_data_ptr->get_ego_pose();
  const auto & bpp_param_ptr = common_data_ptr->bpp_param_ptr;

  const auto ego_arc_length = calc_arc_length(path, vehicle_pose, *bpp_param_ptr);

  for (double t = 0.0; t < prepare_time; t += resolution) {
    const auto velocity =
      std::clamp(initial_velocity + prepare_acc * t, 0.0, frenet_candidate.prepare_metric.velocity);
    const auto length = initial_velocity * t + 0.5 * prepare_acc * t * t;
    const auto pose = autoware::motion_utils::calcInterpolatedPose(path, ego_arc_length + length);
    predicted_path.emplace_back(t, pose, velocity);
  }

  const auto & poses = frenet_candidate.lane_changing.poses;
  const auto & velocities = frenet_candidate.lane_changing.longitudinal_velocities;
  const auto & times = frenet_candidate.lane_changing.times;

  for (const auto [t, pose, velocity] :
       ranges::views::zip(times, poses, velocities) | ranges::views::drop(1)) {
    predicted_path.emplace_back(prepare_time + t, pose, velocity);
  }

  return predicted_path;
}

bool is_valid_start_point(const lane_change::CommonDataPtr & common_data_ptr, const Pose & pose)
{
  const lanelet::BasicPoint2d lc_start_point(pose.position.x, pose.position.y);

  const auto & target_neighbor_poly = common_data_ptr->lanes_polygon_ptr->target_neighbor;
  const auto & target_lane_poly = common_data_ptr->lanes_polygon_ptr->target;

  // Check the target lane because the previous approved path might be shifted by avoidance module
  return boost::geometry::covered_by(lc_start_point, target_neighbor_poly) ||
         boost::geometry::covered_by(lc_start_point, target_lane_poly);
}

bool is_moving_object(const CommonDataPtr & common_data_ptr, const ExtendedPredictedObject & object)
{
  return object.initial_twist.linear.x >
         common_data_ptr->lc_param_ptr->safety.th_stopped_object_velocity;
}

bool is_lanelet_in_lanelet_collections(
  const lanelet::ConstLanelets & lanelet_collections, const lanelet::ConstLanelet & lanelet)
{
  return std::any_of(
    lanelet_collections.begin(), lanelet_collections.end(),
    [&](const auto & lane) { return lane.id() == lanelet.id(); });
}

void trim_preferred_after_alternative(
  lanelet::ConstLanelets & base_lanes, const lanelet::ConstLanelets & preferred_lanes)
{
  // Build lookup set
  std::unordered_set<lanelet::Id> preferred_ids;
  for (const auto & l : preferred_lanes) {
    preferred_ids.insert(l.id());
  }

  auto is_preferred = [&](const lanelet::ConstLanelet & ll) {
    return preferred_ids.count(ll.id()) == 1;
  };

  auto is_alternative = [&](const lanelet::ConstLanelet & ll) {
    return preferred_ids.count(ll.id()) == 0;
  };

  auto first_alt_it = ranges::find_if(base_lanes, is_alternative);
  if (first_alt_it == base_lanes.end()) {
    return;
  }

  auto first_pref_after_alt_it =
    ranges::find_if(ranges::make_subrange(std::next(first_alt_it), base_lanes.end()), is_preferred);

  if (first_pref_after_alt_it == base_lanes.end()) {
    return;
  }

  base_lanes.erase(first_pref_after_alt_it, base_lanes.end());
}

std::vector<lanelet::ConstLineString3d> get_no_lane_change_lines(
  const lanelet::ConstLanelets & target_lanes, const Direction direction)
{
  std::vector<lanelet::ConstLineString3d> no_lane_change_lines;
  no_lane_change_lines.reserve(target_lanes.size());

  for (const auto & ll : target_lanes) {
    const auto & ls = (direction == Direction::LEFT) ? ll.leftBound() : ll.rightBound();

    // 1. Check if the physical line is solid
    const bool is_solid =
      (ls.attributeOr(lanelet::AttributeName::Subtype, "") == lanelet::AttributeValueString::Solid);

    // 2. Check for explicit lane_change permission tags
    const std::string lane_change_val = ls.attributeOr("lane_change", "");

    const bool explicit_no = (lane_change_val == "no");
    const bool explicit_yes = (lane_change_val == "yes");

    if ((is_solid && !explicit_yes) || explicit_no) {
      no_lane_change_lines.push_back(ls);
    }
  }

  return no_lane_change_lines;
}

std::vector<std::pair<double, double>> get_interval_dist_no_lane_change_lines(
  const std::vector<lanelet::ConstLineString3d> & no_lane_change_lines,
  const PathWithLaneId & centerline_path, const Pose & ego_pose)
{
  std::vector<std::pair<double, double>> interval;

  for (const auto & line : no_lane_change_lines) {
    const auto & start = line.front();

    const auto dist_front = autoware::motion_utils::calcSignedArcLength(
      centerline_path.points, ego_pose.position,
      autoware_utils::create_point(start.x(), start.y(), start.z()));

    const auto & back = line.back();
    const auto dist_back = autoware::motion_utils::calcSignedArcLength(
      centerline_path.points, ego_pose.position,
      autoware_utils::create_point(back.x(), back.y(), back.z()));

    if (dist_front <= dist_back) {
      interval.emplace_back(dist_front, dist_back);
    } else {
      interval.emplace_back(dist_back, dist_front);
    }
  }

  return interval;
}

bool is_intersecting_no_lane_change_lines(
  const CommonDataPtr & common_data_ptr, const PhaseInfo lc_length,
  const std::vector<PathPointWithLaneId> & lane_changing_path)
{
  const auto & intervals = common_data_ptr->transient_data.interval_dist_no_lane_change_lines;
  const auto & lines = common_data_ptr->no_lane_change_lines;
  const auto buffer = common_data_ptr->lc_param_ptr->lane_change_finish_judge_buffer;

  // intervals are sorted in ascending order
  if (intervals.empty() || lc_length.prepare >= intervals.back().second) {
    return false;
  }

  const auto prepare_length = lc_length.prepare;
  const auto total_length = lc_length.sum();
  for (const auto & zip : ranges::views::zip(intervals, lines)) {
    const auto & interval = std::get<0>(zip);
    const auto [interval_start, interval_end] = interval;

    const auto interval_upper_bound = interval_end + buffer;
    if (prepare_length >= interval_upper_bound) {
      continue;
    }

    const auto interval_lower_bound = interval_start - buffer;
    // intervals are sorted in ascending order
    if (total_length <= interval_lower_bound) {
      return false;
    }

    const auto & line = std::get<1>(zip);

    bool is_intersecting =
      ranges::any_of(ranges::views::sliding(lane_changing_path, 2), [&](const auto & path_segment) {
        const auto & path_p1 = path_segment[0].point.pose.position;
        const auto & path_p2 = path_segment[1].point.pose.position;

        for (size_t i = 0; i + 1 < line.size(); ++i) {
          const auto line_p1 = experimental::lanelet2_utils::to_ros(line[i]);
          const auto line_p2 = experimental::lanelet2_utils::to_ros(line[i + 1]);

          if (autoware_utils_geometry::intersect(path_p1, path_p2, line_p1, line_p2)) {
            return true;
          }
        }

        return false;
      });

    if (is_intersecting) {
      return true;
    }
  }

  return false;
}

}  // namespace autoware::behavior_path_planner::utils::lane_change
