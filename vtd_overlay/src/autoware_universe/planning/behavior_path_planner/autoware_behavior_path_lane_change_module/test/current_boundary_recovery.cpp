// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0
// Offline probe: captured map/route/path/odometry only, no publisher or vehicle command.
#include "autoware/behavior_path_lane_change_module/manager.hpp"
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/intersection_exit.hpp"
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/serialization.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/path.hpp>

#include <cstring>
#include <fstream>
#include <iostream>

template <class T>
T read_message(const std::string & filename)
{
  std::ifstream file(filename, std::ios::binary);
  if (!file) throw std::runtime_error("Cannot read " + filename);
  const std::vector<char> data{std::istreambuf_iterator<char>(file), {}};
  rclcpp::SerializedMessage serialized(data.size());
  auto & storage = serialized.get_rcl_serialized_message();
  std::memcpy(storage.buffer, data.data(), data.size());
  storage.buffer_length = data.size();
  T message;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &message);
  return message;
}
template <class T>
void write_message(const std::string & filename, const T & message)
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<T>().serialize_message(&message, &serialized);
  const auto & storage = serialized.get_rcl_serialized_message();
  std::ofstream file(filename, std::ios::binary);
  file.write(reinterpret_cast<const char *>(storage.buffer), storage.buffer_length);
}

namespace bpp = autoware::behavior_path_planner;
class CapturedLaneChange : public bpp::NormalLaneChange
{
public:
  CapturedLaneChange(
    const bpp::lane_change::LCParamPtr & params, const std::shared_ptr<bpp::PlannerData> & data,
    const bpp::PathWithLaneId & path, lanelet::Id current_id, lanelet::Id target_id)
  : NormalLaneChange(
      params, bpp::LaneChangeModuleType::NORMAL, autoware::route_handler::Direction::RIGHT)
  {
    setData(data);
    const auto route = data->route_handler;
    const auto map = route->getLaneletMapPtr();
    auto & lanes = *common_data_ptr_->lanes_ptr;
    lanes.current = route->getLaneletSequence(map->laneletLayer.get(current_id), 30.0, 300.0);
    lanes.target = route->getLaneletSequence(map->laneletLayer.get(target_id), 30.0, 300.0);
    lanes.ego_lane = map->laneletLayer.get(target_id);
    lanes.target_neighbor = lanes.current;
    common_data_ptr_->current_lanes_path = route->getCenterLinePath(lanes.current, 0.0, 1e6);
    common_data_ptr_->target_lanes_path = route->getCenterLinePath(lanes.target, 0.0, 1e6);
    *common_data_ptr_->lanes_polygon_ptr =
      bpp::utils::lane_change::create_lanes_polygon(common_data_ptr_);
    common_data_ptr_->no_lane_change_lines =
      bpp::utils::lane_change::get_no_lane_change_lines(lanes.current, direction_);
    prev_module_output_.path = path;
    prev_module_output_.reference_path = common_data_ptr_->current_lanes_path;
    status_.lane_change_path.path = path;
    status_.lane_change_path.info.duration.prepare = 1.0;
    status_.lane_change_path.info.lane_changing_end = path.points.back().point.pose;
    status_.is_valid_path = true;
    status_.is_safe = true;
    is_activated_ = true;
    update_filtered_objects();
    update_transient_data(true);
    common_data_ptr_->transient_data.is_ego_stuck = true;
    std::cout << "current lanes:";
    for (const auto & lane : lanes.current) std::cout << ' ' << lane.id();
    std::cout << "\ntarget lanes:";
    for (const auto & lane : lanes.target) std::cout << ' ' << lane.id();
    std::cout << '\n';
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    if (argc < 4)
      throw std::runtime_error(
        "usage: probe INPUT_DIR CURRENT_LANE_ID TARGET_LANE_ID --ros-args --params-file PARAMS");
    const std::string dir = argv[1];
    auto node = std::make_shared<rclcpp::Node>("behavior_path_planner");
    auto data = std::make_shared<bpp::PlannerData>();
    data->init_parameters(*node);
    auto params = bpp::LaneChangeModuleManager::set_params(node.get(), node->get_name());
    data->route_handler = std::make_shared<autoware::route_handler::RouteHandler>(
      read_message<autoware_map_msgs::msg::LaneletMapBin>(dir + "/map.cdr"));
    data->route_handler->setRoute(
      read_message<autoware_planning_msgs::msg::LaneletRoute>(dir + "/route.cdr"));
    data->self_odometry = std::make_shared<nav_msgs::msg::Odometry>(
      read_message<nav_msgs::msg::Odometry>(dir + "/odom.cdr"));
    data->self_acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>(
      read_message<geometry_msgs::msg::AccelWithCovarianceStamped>(dir + "/acceleration.cdr"));
    data->dynamic_object = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>(
      read_message<autoware_perception_msgs::msg::PredictedObjects>(dir + "/filtered_objects.cdr"));
    const auto path = read_message<bpp::PathWithLaneId>(dir + "/behavior_path.cdr");
    CapturedLaneChange module(params, data, path, std::stoll(argv[2]), std::stoll(argv[3]));
    bool found = false;
    for (int i = 0; i < 10 && !found; ++i) found = module.replanAfterDrivableAreaStop();
    if (!found) throw std::runtime_error("No valid replacement for captured corridor");
    const auto & info = module.getLaneChangePath().info;
    std::cout << "prepare_length=" << info.length.prepare << " change_length=" << info.length.lane_changing
              << " start=(" << info.lane_changing_start.position.x << ',' << info.lane_changing_start.position.y << ")\n";
    auto output = module.generateOutput();
    // Match the real BPP stage, including its 2m re-sampling before publishing bounds.
    output.path = bpp::utils::resamplePathWithSpline(output.path, data->parameters.output_path_interval, false);
    const auto & di = output.drivable_area_info;
    const auto shorten = bpp::utils::cutOverlappedLanes(output.path, di.drivable_lanes);
    const auto & dp = data->drivable_area_expansion_parameters;
    const auto expanded = bpp::utils::expandLanelets(
      shorten, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
      dp.drivable_area_types_to_skip);
    bpp::utils::generateDrivableArea(
      output.path, expanded, di.enable_expanding_hatched_road_markings,
      di.enable_expanding_intersection_areas, di.enable_expanding_freespace_areas, data, true);
    bpp::utils::extractObstaclesFromDrivableArea(output.path, di.obstacles);
    write_message(dir + "/replanned_behavior_path.cdr", output.path);
    autoware_planning_msgs::msg::Path result;
    result.header = output.path.header;
    result.left_bound = output.path.left_bound;
    result.right_bound = output.path.right_bound;
    for (const auto & point : output.path.points) result.points.push_back(point.point);
    write_message(dir + "/replanned_path.cdr", result);
    std::cout << "replacement points=" << result.points.size()
              << " left=" << result.left_bound.size() << " right=" << result.right_bound.size()
              << '\n';
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
