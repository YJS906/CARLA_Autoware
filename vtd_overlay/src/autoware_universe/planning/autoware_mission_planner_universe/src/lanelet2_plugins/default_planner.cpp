// Copyright 2019-2024 Autoware Foundation
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

#include "default_planner.hpp"

#include "utility_functions.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <tf2/utils.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/is_empty.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::mission_planner_universe::lanelet2
{

namespace
{
lanelet::ConstLanelets get_lanelets_to(
  const lanelet::ConstLanelet & start_lanelet, const double distance, const bool backward,
  const route_handler::RouteHandler & route_handler)
{
  lanelet::ConstLanelets lanelets;
  if (!std::isfinite(distance) || distance <= 0.0) {
    return lanelets;
  }

  auto current_lanelet = start_lanelet;
  auto remaining_distance = distance;
  std::unordered_set<lanelet::Id> visited{start_lanelet.id()};
  while (remaining_distance > 0.0) {
    const auto next_lanelets = backward ? route_handler.getPreviousLanelets(current_lanelet)
                                        : route_handler.getNextLanelets(current_lanelet);
    if (next_lanelets.empty()) {
      break;
    }
    const auto & next_lanelet = next_lanelets.front();
    const auto length = lanelet::geometry::length2d(next_lanelet);
    // Invalid centerlines must not turn the remaining distance into NaN, which previously
    // recursed indefinitely on a connected loop. Visit each lanelet at most once in either
    // direction.
    if (!std::isfinite(length) || length <= 0.0 || !visited.insert(next_lanelet.id()).second) {
      break;
    }
    lanelets.insert(backward ? lanelets.begin() : lanelets.end(), next_lanelet);
    remaining_distance -= length;
    current_lanelet = next_lanelet;
  }

  return lanelets;
}

/**
 * @brief Check if a lanelet has the direction_change tag
 * @param lanelet The lanelet to check
 * @return true if the lanelet has the direction_change attribute set to "yes"
 */
bool hasDirectionChangeTag(const lanelet::ConstLanelet & lanelet)
{
  const std::string direction_change_tag = lanelet.attributeOr("direction_change", "none");
  return direction_change_tag == "yes";
}

bool contains_lane_primitive(
  const autoware_planning_msgs::msg::LaneletSegment & section, const lanelet::Id lane_id)
{
  return std::any_of(section.primitives.begin(), section.primitives.end(), [&](const auto & p) {
    return p.primitive_type == "lane" && p.id == lane_id;
  });
}

autoware_planning_msgs::msg::LaneletPrimitive make_lane_primitive(const lanelet::Id lane_id)
{
  autoware_planning_msgs::msg::LaneletPrimitive primitive;
  primitive.id = lane_id;
  primitive.primitive_type = "lane";
  return primitive;
}

std::optional<lanelet::ConstLanelets> get_lane_change_path(
  const lanelet::ConstLanelet & from, const lanelet::ConstLanelet & to,
  const route_handler::RouteHandler & route_handler)
{
  std::deque<lanelet::ConstLanelet> queue{from};
  std::unordered_map<lanelet::Id, lanelet::Id> parent{{from.id(), lanelet::InvalId}};
  while (!queue.empty()) {
    const auto lane = queue.front();
    queue.pop_front();
    if (lane.id() == to.id()) {
      lanelet::ConstLanelets path;
      for (auto id = to.id(); id != lanelet::InvalId; id = parent.at(id)) {
        path.push_back(route_handler.getLaneletsFromId(id));
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    for (const auto & neighbor : route_handler.getLaneChangeableNeighbors(lane)) {
      if (!route_handler.isRoadLanelet(neighbor) || parent.count(neighbor.id()) != 0) {
        continue;
      }
      parent.emplace(neighbor.id(), lane.id());
      queue.push_back(neighbor);
    }
  }
  return std::nullopt;
}

void order_lane_primitives(
  autoware_planning_msgs::msg::LaneletSegment & section,
  const route_handler::RouteHandler & route_handler)
{
  std::unordered_set<lanelet::Id> remaining;
  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> non_lane_primitives;
  for (const auto & primitive : section.primitives) {
    if (primitive.primitive_type == "lane") {
      remaining.insert(primitive.id);
    } else {
      non_lane_primitives.push_back(primitive);
    }
  }
  if (remaining.empty() || section.preferred_primitive.primitive_type != "lane") {
    return;
  }

  auto leftmost = route_handler.getLaneletsFromId(section.preferred_primitive.id);
  while (const auto left = route_handler.getLeftLanelet(leftmost, true, false)) {
    if (remaining.count(left->id()) == 0) {
      break;
    }
    leftmost = *left;
  }

  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> ordered;
  auto lane = leftmost;
  while (remaining.erase(lane.id()) != 0) {
    ordered.push_back(make_lane_primitive(lane.id()));
    const auto right = route_handler.getRightLanelet(lane, true, false);
    if (!right || remaining.count(right->id()) == 0) {
      break;
    }
    lane = *right;
  }
  for (const auto & primitive : section.primitives) {
    if (primitive.primitive_type == "lane" && remaining.erase(primitive.id) != 0) {
      ordered.push_back(primitive);
    }
  }
  ordered.insert(ordered.end(), non_lane_primitives.begin(), non_lane_primitives.end());
  section.primitives = std::move(ordered);
}

struct CorridorPredecessor
{
  lanelet::ConstLanelet lane;
  lanelet::ConstLanelets lane_change_path;
};

std::optional<CorridorPredecessor> find_corridor_predecessor(
  const autoware_planning_msgs::msg::LaneletSegment & section,
  const lanelet::ConstLanelet & next_preferred_lane,
  const route_handler::RouteHandler & route_handler)
{
  if (section.preferred_primitive.primitive_type != "lane") {
    return std::nullopt;
  }
  const auto current_preferred_lane =
    route_handler.getLaneletsFromId(section.preferred_primitive.id);
  std::optional<CorridorPredecessor> selected;
  for (const auto & predecessor : route_handler.getPreviousLanelets(next_preferred_lane)) {
    if (!route_handler.isRoadLanelet(predecessor)) {
      continue;
    }
    const auto path = get_lane_change_path(current_preferred_lane, predecessor, route_handler);
    if (!path) {
      continue;
    }
    if (
      !selected || path->size() < selected->lane_change_path.size() ||
      (path->size() == selected->lane_change_path.size() &&
       predecessor.id() == current_preferred_lane.id())) {
      selected = CorridorPredecessor{predecessor, *path};
    }
  }
  return selected;
}

void prefer_downstream_reachable_corridors(
  std::vector<autoware_planning_msgs::msg::LaneletSegment> & route_sections,
  const route_handler::RouteHandler & route_handler)
{
  // A shortest path may postpone its lateral edge until the last short lanelet. Propagate the
  // required downstream corridor backward through lane-changeable predecessors so behavior
  // planning can start the same lane change on an earlier parallel section.
  for (size_t next_index = route_sections.size(); next_index > 1; --next_index) {
    auto & section = route_sections.at(next_index - 2);
    const auto & next_section = route_sections.at(next_index - 1);
    if (next_section.preferred_primitive.primitive_type != "lane") {
      continue;
    }
    const auto next_preferred_lane =
      route_handler.getLaneletsFromId(next_section.preferred_primitive.id);
    const auto predecessor = find_corridor_predecessor(section, next_preferred_lane, route_handler);
    if (!predecessor) {
      continue;
    }
    for (const auto & lane : predecessor->lane_change_path) {
      if (!contains_lane_primitive(section, lane.id())) {
        section.primitives.push_back(make_lane_primitive(lane.id()));
      }
    }
    section.preferred_primitive = make_lane_primitive(predecessor->lane.id());
    order_lane_primitives(section, route_handler);
  }
}

bool is_preferred_route_reachable(
  const std::vector<autoware_planning_msgs::msg::LaneletSegment> & route_sections,
  const route_handler::RouteHandler & route_handler)
{
  for (size_t i = 1; i < route_sections.size(); ++i) {
    const auto & previous_section = route_sections.at(i - 1);
    const auto & section = route_sections.at(i);
    if (
      previous_section.preferred_primitive.primitive_type != "lane" ||
      section.preferred_primitive.primitive_type != "lane") {
      continue;
    }
    const auto previous_preferred =
      route_handler.getLaneletsFromId(previous_section.preferred_primitive.id);
    const auto preferred = route_handler.getLaneletsFromId(section.preferred_primitive.id);
    const auto successors = route_handler.getNextLanelets(previous_preferred);
    const bool reachable =
      std::any_of(successors.begin(), successors.end(), [&](const auto & successor) {
        return contains_lane_primitive(section, successor.id()) &&
               get_lane_change_path(successor, preferred, route_handler).has_value();
      });
    if (!reachable) {
      return false;
    }
  }
  return true;
}

struct ReachableGoalFallback
{
  geometry_msgs::msg::Pose goal_pose;
  lanelet::ConstLaneletOrAreas path;
  lanelet::Id goal_lane_id;
  double lanelet_distance;
};

bool has_valid_path_geometry(const lanelet::ConstLaneletOrAreas & path)
{
  if (path.empty()) {
    return false;
  }
  for (const auto & element : path) {
    if (!element.isLanelet()) {
      continue;
    }
    const auto centerline = static_cast<const lanelet::ConstLanelet &>(element).centerline();
    if (
      centerline.size() < 2 ||
      std::any_of(centerline.begin(), centerline.end(), [](const auto & p) {
        return !std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z());
      })) {
      return false;
    }
  }
  return true;
}

geometry_msgs::msg::Pose project_to_lanelet_centerline(
  const lanelet::ConstLanelet & target_lanelet, const geometry_msgs::msg::Pose & pose)
{
  const auto centerline =
    autoware::experimental::lanelet2_utils::get_fine_centerline(target_lanelet, 1.0);
  const auto point = autoware::experimental::lanelet2_utils::from_ros(pose.position);
  const auto projected = lanelet::geometry::project(centerline, point.basicPoint());
  const auto yaw =
    autoware::experimental::lanelet2_utils::get_lanelet_angle(target_lanelet, projected);
  return convertBasicPoint3dToPose(projected, yaw);
}

std::optional<ReachableGoalFallback> find_reachable_goal_fallback(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & requested_goal,
  const route_handler::RouteHandler & route_handler, const bool consider_no_drivable_lanes,
  const double max_goal_angle_difference)
{
  constexpr double max_lanelet_distance = 5.0;
  constexpr double max_height_difference = 2.0;
  constexpr size_t max_candidate_count = 50;

  const auto map = route_handler.getLaneletMapPtr();
  const auto graph = route_handler.getRoutingGraphPtr();
  if (!map || !graph) {
    return std::nullopt;
  }

  const auto start_lanelets = route_handler.getRoadLaneletsAtPose(start_pose);
  const auto candidates = autoware::experimental::lanelet2_utils::find_nearest(
    map->laneletLayer, requested_goal, max_candidate_count, max_lanelet_distance,
    max_height_difference);

  for (const auto & [distance, candidate] : candidates) {
    if (!route_handler.isRoadLanelet(candidate)) {
      continue;
    }

    const auto candidate_goal = project_to_lanelet_centerline(candidate, requested_goal);
    const auto candidate_yaw = tf2::getYaw(candidate_goal.orientation);
    const auto requested_yaw = tf2::getYaw(requested_goal.orientation);
    const auto forward_angle_difference =
      std::abs(autoware_utils::normalize_radian(candidate_yaw - requested_yaw));
    const auto reverse_angle_difference =
      std::abs(autoware_utils::normalize_radian(candidate_yaw + M_PI - requested_yaw));
    const auto angle_difference = hasDirectionChangeTag(candidate)
                                    ? std::min(forward_angle_difference, reverse_angle_difference)
                                    : forward_angle_difference;
    if (angle_difference > max_goal_angle_difference) {
      continue;
    }

    // Avoid retrying candidates that are topologically unreachable. When the start pose is not
    // directly inside a road lanelet, let RouteHandler perform its normal nearest-lane fallback.
    if (
      !start_lanelets.empty() &&
      std::none_of(start_lanelets.begin(), start_lanelets.end(), [&](const auto & start_lanelet) {
        return static_cast<bool>(graph->getRoute(start_lanelet, candidate, 0));
      })) {
      continue;
    }

    lanelet::ConstLaneletOrAreas candidate_path;
    if (
      route_handler.planPathLaneletsBetweenCheckpoints(
        start_pose, candidate_goal, &candidate_path, consider_no_drivable_lanes) &&
      has_valid_path_geometry(candidate_path)) {
      return ReachableGoalFallback{candidate_goal, candidate_path, candidate.id(), distance};
    }
  }
  return std::nullopt;
}
}  // namespace

void DefaultPlanner::initialize_common(rclcpp::Node * node)
{
  is_graph_ready_ = false;
  node_ = node;

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  pub_goal_footprint_marker_ =
    node_->create_publisher<MarkerArray>("~/debug/goal_footprint", durable_qos);

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo();
  param_.goal_angle_threshold_deg = node_->declare_parameter<double>("goal_angle_threshold_deg");
  param_.enable_correct_goal_pose = node_->declare_parameter<bool>("enable_correct_goal_pose");
  param_.consider_no_drivable_lanes = node_->declare_parameter<bool>("consider_no_drivable_lanes");
  param_.check_footprint_inside_lanes =
    node_->declare_parameter<bool>("check_footprint_inside_lanes");
  param_.allow_area = node_->declare_parameter<bool>("allow_area", false);
  route_handler_.setAllowArea(param_.allow_area);
}

void DefaultPlanner::initialize(rclcpp::Node * node)
{
  initialize_common(node);
  map_subscriber_ = node_->create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{10}.transient_local(),
    std::bind(&DefaultPlanner::map_callback, this, std::placeholders::_1));
}

void DefaultPlanner::initialize(rclcpp::Node * node, const LaneletMapBin::ConstSharedPtr msg)
{
  initialize_common(node);
  map_callback(msg);
}

bool DefaultPlanner::ready() const
{
  return is_graph_ready_;
}

void DefaultPlanner::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  route_handler_.setMap(*msg);
  is_graph_ready_ = true;
}

PlannerPlugin::MarkerArray DefaultPlanner::visualize(
  const LaneletRoute & route, float goal_lanelet_transparency) const
{
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets end_lanelets;
  lanelet::ConstLanelets goal_lanelets;

  visualization_msgs::msg::MarkerArray area_markers;
  int area_id = 0;

  const std_msgs::msg::ColorRGBA cl_end = autoware_utils::create_marker_color(0.2, 0.2, 0.4, 0.05);
  const std_msgs::msg::ColorRGBA cl_goal =
    autoware_utils::create_marker_color(0.2, 0.4, 0.4, goal_lanelet_transparency);

  for (const auto & route_section : route.segments) {
    for (const auto & prim : route_section.primitives) {
      if (prim.primitive_type == "area") {
        const auto area = route_handler_.getAreaFromId(prim.id);
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = "route_areas";
        m.id = area_id++;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.scale.x = 0.08;
        const bool is_preferred = route_section.preferred_primitive.id == prim.id;
        m.color = is_preferred ? cl_goal : cl_end;
        for (const auto & ls : area.outerBound()) {
          for (const auto & pt : ls) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            m.points.push_back(p);
          }
        }
        if (!m.points.empty()) {
          m.points.push_back(m.points.front());
        }
        area_markers.markers.push_back(m);
        continue;
      }

      auto lanelet = route_handler_.getLaneletsFromId(prim.id);
      route_lanelets.push_back(lanelet);
      if (route_section.preferred_primitive.id == prim.id) {
        goal_lanelets.push_back(lanelet);
      } else {
        end_lanelets.push_back(lanelet);
      }
    }
  }

  const std_msgs::msg::ColorRGBA cl_route =
    autoware_utils::create_marker_color(0.8, 0.99, 0.8, 0.15);
  const std_msgs::msg::ColorRGBA cl_ll_borders =
    autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.999);

  visualization_msgs::msg::MarkerArray route_marker_array;
  insert_marker_array(&route_marker_array, area_markers);
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(route_lanelets, cl_ll_borders, false));
  insert_marker_array(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "route_lanelets", route_lanelets, cl_route));
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("end_lanelets", end_lanelets, cl_end));
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("goal_lanelets", goal_lanelets, cl_goal));

  return route_marker_array;
}

visualization_msgs::msg::MarkerArray DefaultPlanner::visualize_debug_footprint(
  autoware_utils::LinearRing2d goal_footprint)
{
  visualization_msgs::msg::MarkerArray msg;
  auto marker = autoware_utils::create_default_marker(
    "map", rclcpp::Clock().now(), "goal_footprint", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.05, 0.0, 0.0),
    autoware_utils::create_marker_color(0.99, 0.99, 0.2, 1.0));
  marker.lifetime = rclcpp::Duration::from_seconds(2.5);

  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[0][0], goal_footprint[0][1], 0.0));
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[1][0], goal_footprint[1][1], 0.0));
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[2][0], goal_footprint[2][1], 0.0));
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[3][0], goal_footprint[3][1], 0.0));
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[4][0], goal_footprint[4][1], 0.0));
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[5][0], goal_footprint[5][1], 0.0));
  marker.points.push_back(marker.points.front());

  msg.markers.push_back(marker);

  return msg;
}

bool DefaultPlanner::check_goal_footprint_inside_lanes(
  const lanelet::ConstLanelets & lanelets_near_goal,
  const autoware_utils::Polygon2d & goal_footprint) const
{
  lanelet::Points3d left_bound_points;
  lanelet::Points3d right_bound_points;

  for (const auto & lanelet : lanelets_near_goal) {
    if (const auto left_shoulder = route_handler_.getLeftShoulderLanelet(lanelet)) {
      for (const auto & point : left_shoulder->leftBound()) {
        left_bound_points.push_back(lanelet::Point3d(point));
      }
    } else {
      for (const auto & point : lanelet.leftBound()) {
        left_bound_points.push_back(lanelet::Point3d(point));
      }
    }

    if (const auto right_shoulder = route_handler_.getRightShoulderLanelet(lanelet)) {
      for (const auto & point : right_shoulder->rightBound()) {
        right_bound_points.push_back(lanelet::Point3d(point));
      }
    } else {
      for (const auto & point : lanelet.rightBound()) {
        right_bound_points.push_back(lanelet::Point3d(point));
      }
    }
  }

  auto lane_polygon =
    lanelet::Lanelet(
      lanelet::InvalId, lanelet::LineString3d(lanelet::InvalId, left_bound_points),
      lanelet::LineString3d(lanelet::InvalId, right_bound_points))
      .polygon2d()
      .basicPolygon();
  boost::geometry::correct(lane_polygon);

  return boost::geometry::covered_by(goal_footprint, lane_polygon);
}

bool DefaultPlanner::is_goal_valid(const geometry_msgs::msg::Pose & goal)
{
  const auto logger = node_->get_logger();

  const auto goal_lanelet_pt = experimental::lanelet2_utils::from_ros(goal.position);

  // check if goal is in shoulder lanelet
  const auto shoulder_lanelets = route_handler_.getShoulderLaneletsAtPose(goal);
  if (const auto closest_shoulder_lanelet_opt =
        experimental::lanelet2_utils::get_closest_lanelet_within_constraint(
          shoulder_lanelets, goal);
      closest_shoulder_lanelet_opt) {
    const auto & closest_shoulder_lanelet = closest_shoulder_lanelet_opt.value();
    const auto lane_yaw = autoware::experimental::lanelet2_utils::get_lanelet_angle(
      closest_shoulder_lanelet,
      autoware::experimental::lanelet2_utils::from_ros(goal.position).basicPoint());
    const auto goal_yaw = tf2::getYaw(goal.orientation);
    const auto angle_diff = autoware_utils::normalize_radian(lane_yaw - goal_yaw);
    const double th_angle = autoware_utils::deg2rad(param_.goal_angle_threshold_deg);
    const bool has_direction_change_tag = hasDirectionChangeTag(closest_shoulder_lanelet);
    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
    if (has_direction_change_tag) {
      const double reversed_angle_diff =
        std::abs(autoware_utils::normalize_radian(angle_diff - M_PI));
      if (reversed_angle_diff < th_angle) {
        return true;
      }
    }
  }
  const auto road_lanelets_at_goal = route_handler_.getRoadLaneletsAtPose(goal);
  auto closest_lanelet_to_goal_opt =
    experimental::lanelet2_utils::get_closest_lanelet(road_lanelets_at_goal, goal);
  if (!closest_lanelet_to_goal_opt) {
    // if no road lanelets directly at the goal, find the closest one
    const lanelet::BasicPoint2d goal_point{goal.position.x, goal.position.y};
    auto closest_dist = std::numeric_limits<double>::max();
    const auto closest_road_lanelet_found =
      route_handler_.getLaneletMapPtr()->laneletLayer.nearestUntil(
        goal_point, [&](const auto & bbox, const auto & ll) {
          // this search is done by increasing distance between the bounding box and the goal
          // we stop the search when the bounding box is further than the closest dist found
          if (lanelet::geometry::distance2d(bbox, goal_point) > closest_dist)
            return true;  // stop the search
          const auto dist = lanelet::geometry::distance2d(goal_point, ll.polygon2d());
          if (route_handler_.isRoadLanelet(ll) && dist < closest_dist) {
            closest_dist = dist;
            closest_lanelet_to_goal_opt = ll;
          }
          return false;  // continue the search
        });
    if (!closest_road_lanelet_found) return false;
  }

  // If the goal is at the very beginning or the end of closest_lanelet_to_goal, base link to rear
  // part of ego footprint will be outside of it. To tolerate it, add previous and next lanelets
  const auto & closest_lanelet_to_goal = closest_lanelet_to_goal_opt.value();
  lanelet::ConstLanelets lanelets_near_goal{closest_lanelet_to_goal};
  const auto previous_lanelets = get_lanelets_to(
    closest_lanelet_to_goal, vehicle_info_.max_longitudinal_offset_m, true, route_handler_);
  lanelets_near_goal.insert(
    lanelets_near_goal.begin(), previous_lanelets.begin(), previous_lanelets.end());
  const auto next_lanelets = get_lanelets_to(
    closest_lanelet_to_goal, vehicle_info_.max_longitudinal_offset_m, false, route_handler_);
  lanelets_near_goal.insert(lanelets_near_goal.end(), next_lanelets.begin(), next_lanelets.end());

  const autoware_utils::LinearRing2d goal_footprint = vehicle_info_.createFootprint(0.0, goal);
  pub_goal_footprint_marker_->publish(visualize_debug_footprint(goal_footprint));
  const auto polygon_footprint = convert_linear_ring_to_polygon(goal_footprint);

  // check if goal footprint exceeds lane when the goal isn't in parking_lot
  if (
    param_.check_footprint_inside_lanes &&
    !check_goal_footprint_inside_lanes(lanelets_near_goal, polygon_footprint) &&
    !is_in_parking_lot(
      lanelet::utils::query::getAllParkingLots(route_handler_.getLaneletMapPtr()),
      experimental::lanelet2_utils::from_ros(goal.position))) {
    RCLCPP_WARN(logger, "Goal's footprint exceeds lane!");
    return false;
  }

  if (is_in_lane(closest_lanelet_to_goal, goal_lanelet_pt)) {
    const auto lane_yaw = autoware::experimental::lanelet2_utils::get_lanelet_angle(
      closest_lanelet_to_goal,
      autoware::experimental::lanelet2_utils::from_ros(goal.position).basicPoint());
    const auto goal_yaw = tf2::getYaw(goal.orientation);
    const auto angle_diff = autoware_utils::normalize_radian(lane_yaw - goal_yaw);

    const double th_angle = autoware_utils::deg2rad(param_.goal_angle_threshold_deg);
    const bool has_direction_change_tag = hasDirectionChangeTag(closest_lanelet_to_goal);
    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
    if (has_direction_change_tag) {
      const double reversed_angle_diff =
        std::abs(autoware_utils::normalize_radian(angle_diff - M_PI));
      if (reversed_angle_diff < th_angle) {
        return true;
      }
    }
  }

  // check if goal is in parking space
  const auto parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(route_handler_.getLaneletMapPtr());
  if (is_in_parking_space(parking_spaces, goal_lanelet_pt)) {
    return true;
  }

  // check if goal is in parking lot
  const auto parking_lots =
    lanelet::utils::query::getAllParkingLots(route_handler_.getLaneletMapPtr());
  return is_in_parking_lot(parking_lots, goal_lanelet_pt);
}

PlannerPlugin::LaneletRoute DefaultPlanner::plan(const RoutePoints & points)
{
  const auto logger = node_->get_logger();

  std::stringstream log_ss;
  for (const auto & point : points) {
    log_ss << "x: " << point.position.x << " "
           << "y: " << point.position.y << std::endl;
  }
  RCLCPP_DEBUG_STREAM(
    logger, "start planning route with check points: " << std::endl
                                                       << log_ss.str());

  LaneletRoute route_msg;
  if (points.size() < 2) {
    RCLCPP_WARN(logger, "Route planning requires at least a start and a goal checkpoint.");
    return route_msg;
  }
  RouteSections route_sections;
  auto planning_points = points;

  lanelet::ConstLaneletOrAreas all_route_lanelets_or_areas;
  for (std::size_t i = 1; i < planning_points.size(); i++) {
    const auto start_check_point = planning_points.at(i - 1);
    const auto goal_check_point = planning_points.at(i);

    lanelet::ConstLaneletOrAreas path_lanelets_or_areas;
    const bool path_found = route_handler_.planPathLaneletsBetweenCheckpoints(
      start_check_point, goal_check_point, &path_lanelets_or_areas,
      param_.consider_no_drivable_lanes);
    if (path_found && !has_valid_path_geometry(path_lanelets_or_areas)) {
      // RouteHandler may report success without selecting a path when a malformed map has
      // non-finite centerline lengths (and therefore non-finite route costs). Never pass that
      // empty result to route segmentation, which requires a last lanelet. Also reject a selected
      // path containing non-finite geometry, e.g. when RouteHandler prefers an existing route.
      // Malformed geometry is not an unreachable goal: do not relocate the goal through fallback.
      RCLCPP_WARN(logger, "Route planning produced an empty path or non-finite lanelet geometry.");
      return route_msg;
    }
    if (!path_found) {
      const bool is_final_goal = i + 1 == planning_points.size();
      const auto fallback = is_final_goal
                              ? find_reachable_goal_fallback(
                                  start_check_point, goal_check_point, route_handler_,
                                  param_.consider_no_drivable_lanes,
                                  autoware_utils::deg2rad(param_.goal_angle_threshold_deg))
                              : std::nullopt;
      if (!fallback) {
        RCLCPP_WARN(logger, "Failed to plan route.");
        return route_msg;
      }

      planning_points.at(i) = fallback->goal_pose;
      path_lanelets_or_areas = fallback->path;
      RCLCPP_WARN(
        logger,
        "Requested goal lanelet is unreachable. Using reachable lanelet %ld at %.2f m from the "
        "requested goal.",
        fallback->goal_lane_id, fallback->lanelet_distance);
    }

    for (const auto & elem : path_lanelets_or_areas) {
      if (
        !all_route_lanelets_or_areas.empty() &&
        elem.id() == all_route_lanelets_or_areas.back().id())
        continue;
      all_route_lanelets_or_areas.push_back(elem);
    }
  }

  for (const auto & elem : all_route_lanelets_or_areas) {
    if (elem.isLanelet()) {
      RCLCPP_INFO(logger, "Planned lanelet id: %ld", elem.id());
    } else if (elem.isArea()) {
      RCLCPP_INFO(logger, "Planned area id: %ld", elem.id());
    }
  }

  // Extract only lanelets for setRouteLanelets (it requires ConstLanelets)
  lanelet::ConstLanelets all_route_lanelets;
  for (const auto & elem : all_route_lanelets_or_areas) {
    if (elem.isLanelet()) {
      all_route_lanelets.push_back(static_cast<const lanelet::ConstLanelet &>(elem));
    }
  }
  route_handler_.setRouteLanelets(all_route_lanelets);
  route_sections =
    route_handler_.createMapSegmentsFromLaneletOrAreaPath(all_route_lanelets_or_areas);
  if (route_sections.empty()) {
    RCLCPP_WARN(logger, "Route planning produced no route segments.");
    return route_msg;
  }
  // Route segmentation can merge the final lateral edge into one section. Keep the actual path
  // endpoint as the anchor before propagating its reachable corridor backward.
  if (!route_sections.empty() && !all_route_lanelets_or_areas.empty()) {
    const auto & goal_element = all_route_lanelets_or_areas.back();
    if (goal_element.isLanelet()) {
      const auto goal_lane_id = goal_element.id();
      auto & goal_section = route_sections.back();
      if (contains_lane_primitive(goal_section, goal_lane_id)) {
        goal_section.preferred_primitive = make_lane_primitive(goal_lane_id);
      }
    }
  }
  prefer_downstream_reachable_corridors(route_sections, route_handler_);
  if (!is_preferred_route_reachable(route_sections, route_handler_)) {
    RCLCPP_WARN(logger, "Preferred route corridor is disconnected.");
    return route_msg;
  }

  {
    size_t n_lane_seg = 0;
    size_t n_area_seg = 0;
    for (const auto & seg : route_sections) {
      if (seg.preferred_primitive.primitive_type == "area") {
        ++n_area_seg;
      } else {
        ++n_lane_seg;
      }
    }
    RCLCPP_INFO(
      logger,
      "[DefaultPlanner] Route segments for message: total=%zu (lane_segments=%zu, "
      "area_segments=%zu)",
      route_sections.size(), n_lane_seg, n_area_seg);
    for (size_t si = 0; si < route_sections.size(); ++si) {
      const auto & seg = route_sections[si];
      RCLCPP_DEBUG(
        logger, "[DefaultPlanner]   segment[%zu] preferred id=%ld type=%s primitives=%zu", si,
        seg.preferred_primitive.id, seg.preferred_primitive.primitive_type.c_str(),
        seg.primitives.size());
    }
  }

  auto goal_pose = planning_points.back();
  if (param_.enable_correct_goal_pose) {
    goal_pose = get_closest_centerline_pose(
      lanelet::utils::query::laneletLayer(route_handler_.getLaneletMapPtr()), goal_pose,
      vehicle_info_);
  }

  if (!is_goal_valid(goal_pose)) {
    RCLCPP_WARN(logger, "Goal is not valid! Please check position and angle of goal_pose");
    return route_msg;
  }

  if (route_handler::RouteHandler::isRouteLooped(route_sections)) {
    RCLCPP_WARN(logger, "Loop detected within route!");
    return route_msg;
  }

  const auto refined_goal = refine_goal_height(goal_pose, route_sections);
  RCLCPP_DEBUG(logger, "Goal Pose Z : %lf", refined_goal.position.z);

  // The header is assigned by mission planner.
  route_msg.start_pose = planning_points.front();
  route_msg.goal_pose = refined_goal;
  route_msg.segments = route_sections;
  return route_msg;
}

geometry_msgs::msg::Pose DefaultPlanner::refine_goal_height(
  const Pose & goal, const RouteSections & route_sections)
{
  const auto & pref = route_sections.back().preferred_primitive;
  const auto goal_pt = experimental::lanelet2_utils::from_ros(goal.position);
  double goal_height;

  if (pref.primitive_type == "area") {
    const auto area = route_handler_.getAreaFromId(pref.id);
    goal_height = project_goal_to_area(area, goal_pt);
    RCLCPP_DEBUG(
      node_->get_logger(), "[DefaultPlanner] refine_goal_height: goal on area id=%ld z=%.3f",
      pref.id, goal_height);
  } else {
    const auto goal_lanelet = route_handler_.getLaneletsFromId(pref.id);
    goal_height = project_goal_to_map(goal_lanelet, goal_pt);
    RCLCPP_DEBUG(
      node_->get_logger(), "[DefaultPlanner] refine_goal_height: goal on lane id=%ld z=%.3f",
      pref.id, goal_height);
  }

  Pose refined_goal = goal;
  refined_goal.position.z = goal_height;
  return refined_goal;
}

void DefaultPlanner::updateRoute(const PlannerPlugin::LaneletRoute & route)
{
  route_handler_.setRoute(route);
}

void DefaultPlanner::clearRoute()
{
  route_handler_.clearRoute();
}

}  // namespace autoware::mission_planner_universe::lanelet2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::mission_planner_universe::lanelet2::DefaultPlanner,
  autoware::mission_planner_universe::PlannerPlugin)
