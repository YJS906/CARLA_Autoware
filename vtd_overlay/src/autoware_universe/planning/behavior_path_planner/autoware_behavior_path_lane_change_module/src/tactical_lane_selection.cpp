// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#include "autoware/behavior_path_lane_change_module/utils/tactical_lane_selection.hpp"

#include "autoware/behavior_path_lane_change_module/structs/parameters.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"

#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include <lanelet2_routing/RoutingGraph.h>

#include <array>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <utility>

namespace autoware::behavior_path_planner::lane_change
{
namespace
{
constexpr double horizon = 60.0;
using Key = std::array<uint8_t, 16>;
struct XY
{
  double x;
  double y;
};
struct Projection
{
  double s{};
  double d{};
  double distance{1e100};
};
Projection project(const std::vector<XY> & line, const XY & point)
{
  Projection best;
  double arc = 0.0;
  for (size_t i = 1; i < line.size(); ++i) {
    const double dx = line[i].x - line[i - 1].x, dy = line[i].y - line[i - 1].y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-6) continue;
    const double px = point.x - line[i - 1].x, py = point.y - line[i - 1].y;
    const double raw_t = (px * dx + py * dy) / (length * length);
    const double t = std::clamp(raw_t, 0.0, 1.0);
    const double along =
      (i == 1 && raw_t < 0.0) || (i + 1 == line.size() && raw_t > 1.0) ? raw_t : t;
    const double distance = std::hypot(px - t * dx, py - t * dy);
    if (distance < best.distance)
      best = {arc + along * length, (dx * py - dy * px) / length, distance};
    arc += length;
  }
  return best;
}
double stamp(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}
lanelet::ConstLanelet closest(const lanelet::ConstLanelets & lanes, const XY & ego)
{
  return *std::min_element(lanes.begin(), lanes.end(), [&](const auto & a, const auto & b) {
    const auto distance = [&](const auto & lane) {
      std::vector<XY> line;
      for (const auto & p : lane.centerline()) line.push_back({p.x(), p.y()});
      return project(line, ego).distance;
    };
    return distance(a) < distance(b);
  });
}
}  // namespace

struct TacticalLaneSelection::State
{
  std::mutex mutex;
  Key route{};
  const void * map{};
  double last_stamp{-1.0};
  std::set<Key> reasons;
  double cached_stamp{-1.0};
  const void * cached_objects{};
  lanelet::Id cached_current{};
  std::vector<tactical::Corridor> lanes;
  std::vector<std::set<lanelet::Id>> ids;
  size_t current{};

  bool synchronize(const PlannerData & data)
  {
    if (!data.self_odometry || !data.dynamic_object || !data.route_handler) return false;
    const double now = stamp(data.self_odometry->header.stamp);
    const auto uuid = data.route_handler->getRouteUuid().uuid;
    const auto map_ptr = data.route_handler->getLaneletMapPtr().get();
    if (uuid != route || map_ptr != map || now < last_stamp) {
      reasons.clear();
      cached_stamp = -1.0;
    }
    route = uuid;
    map = map_ptr;
    last_stamp = now;
    const double age = now - stamp(data.dynamic_object->header.stamp);
    return age >= -0.1 && age <= 0.5;
  }

  bool refresh(
    const PlannerData & data, const lanelet::ConstLanelets & current_lanes,
    const Parameters & parameters)
  {
    if (!synchronize(data) || reasons.empty() || current_lanes.empty()) return false;
    const auto & pose = data.self_odometry->pose.pose;
    const XY ego{pose.position.x, pose.position.y};
    const auto origin = closest(current_lanes, ego);
    if (
      cached_stamp == last_stamp && cached_objects == data.dynamic_object.get() &&
      cached_current == origin.id())
      return !lanes.empty();
    lanes.clear();
    ids.clear();
    cached_stamp = last_stamp;
    cached_objects = data.dynamic_object.get();
    cached_current = origin.id();
    std::set<Key> present;
    for (const auto & obj : data.dynamic_object->objects) present.insert(obj.object_id.uuid);
    for (auto it = reasons.begin(); it != reasons.end();) {
      if (!present.count(*it))
        it = reasons.erase(it);
      else
        ++it;
    }
    if (reasons.empty()) return false;

    // Only legal, same-direction, in-route neighbors; no adjacent/opposite-lane fallback.
    const auto & route_handler = data.route_handler;
    const auto & graph = route_handler->getRoutingGraphPtr();
    if (!graph) return false;
    std::vector<lanelet::ConstLanelet> anchors{origin};
    std::set<lanelet::Id> visited{origin.id()};
    for (const bool left : {true, false}) {
      auto lane = origin;
      for (size_t step = 0; step < 8; ++step) {
        const auto next = left ? graph->left(lane) : graph->right(lane);
        if (!next || !route_handler->isRouteLanelet(*next) || !visited.insert(next->id()).second)
          break;
        if (left)
          anchors.insert(anchors.begin(), *next);
        else
          anchors.push_back(*next);
        lane = *next;
      }
    }
    const auto & vehicle = data.parameters.vehicle_info;
    std::set<Key> still_ahead;
    std::vector<lanelet::ConstLanelets> sequences;
    std::vector<std::vector<XY>> lines;
    std::vector<double> ego_arcs;
    for (const auto & anchor : anchors) {
      auto sequence = route_handler->getLaneletSequence(anchor, pose, 10.0, horizon + 10.0);
      if (sequence.empty()) sequence.push_back(anchor);
      const auto path =
        route_handler->getCenterLinePath(sequence, 0.0, std::numeric_limits<double>::max());
      std::vector<XY> line;
      std::set<lanelet::Id> lane_ids;
      for (const auto & lane : sequence) lane_ids.insert(lane.id());
      for (const auto & p : path.points)
        line.push_back({p.point.pose.position.x, p.point.pose.position.y});
      if (line.size() < 2) {
        lanes.clear();
        return false;
      }
      const auto ego_projection = project(line, ego);
      double end = 0.0;
      for (size_t i = 1; i < line.size(); ++i)
        end += std::hypot(line[i].x - line[i - 1].x, line[i].y - line[i - 1].y);
      end = std::min(horizon, end - ego_projection.s - vehicle.max_longitudinal_offset_m);
      if (!std::isfinite(end) || end <= 0.0) {
        lanes.clear();
        return false;
      }
      tactical::Corridor corridor;
      corridor.lateral = -ego_projection.d;
      std::vector<tactical::Interval> blocked;
      for (const auto & object : data.dynamic_object->objects) {
        const auto & velocity = object.kinematics.initial_twist_with_covariance.twist.linear;
        const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
        if (
          !std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
          !std::isfinite(object_pose.position.x) || !std::isfinite(object_pose.position.y)) {
          lanes.clear();
          return false;
        }
        // Motion classification and predicted-path/RSS safety are untouched. Moving traffic
        // provides no persistent corridor evidence; it remains checked by the real maneuver.
        if (
          std::hypot(velocity.x, velocity.y) > parameters.trajectory_safety.stationary_velocity ||
          std::hypot(object_pose.position.x - ego.x, object_pose.position.y - ego.y) >
            horizon + 20.0)
          continue;
        const auto polygon = autoware_utils::to_polygon2d(object_pose, object.shape);
        if (polygon.outer().size() < 4) {
          lanes.clear();
          return false;
        }
        double min_s = 1e100, max_s = -1e100, min_d = 1e100, max_d = -1e100;
        for (const auto & vertex : polygon.outer()) {
          if (!std::isfinite(vertex.x()) || !std::isfinite(vertex.y())) {
            lanes.clear();
            return false;
          }
          const auto point = project(line, {vertex.x(), vertex.y()});
          min_s = std::min(min_s, point.s - ego_projection.s);
          max_s = std::max(max_s, point.s - ego_projection.s);
          min_d = std::min(min_d, point.d);
          max_d = std::max(max_d, point.d);
        }
        double margin = parameters.trajectory_safety.lateral_margin;
        if (!object.classification.empty()) {
          const auto label =
            std::max_element(
              object.classification.begin(), object.classification.end(),
              [](const auto & a, const auto & b) { return a.probability < b.probability; })
              ->label;
          const auto it = parameters.trajectory_safety.class_lateral_margins.find(label);
          if (it != parameters.trajectory_safety.class_lateral_margins.end()) margin = it->second;
        }
        margin += parameters.trajectory_safety.optimization_margin;
        const double half_width = vehicle.vehicle_width_m * 0.5 + margin;
        if (min_d > half_width || max_d < -half_width) continue;
        const double clear_after = max_s + vehicle.rear_overhang_m + margin;
        if (
          clear_after <= 0.0 || min_s > horizon ||
          min_s - vehicle.max_longitudinal_offset_m - parameters.trajectory_safety.stop_margin >=
            end)
          continue;
        if (reasons.count(object.object_id.uuid)) {
          corridor.reason_ahead = true;
          still_ahead.insert(object.object_id.uuid);
        }
        blocked.push_back(
          {min_s - vehicle.max_longitudinal_offset_m - parameters.trajectory_safety.stop_margin,
           clear_after});
      }
      corridor.free = tactical::freeIntervals(std::move(blocked), end);
      if (anchor.id() == origin.id()) current = lanes.size();
      lanes.push_back(std::move(corridor));
      ids.push_back(std::move(lane_ids));
      sequences.push_back(std::move(sequence));
      lines.push_back(std::move(line));
      ego_arcs.push_back(ego_projection.s);
    }
    // Permission is local in s: a dotted boundary here does not authorize a hypothetical
    // second change after that boundary becomes solid or the neighboring lane ends.
    for (size_t i = 0; i < lanes.size(); ++i) {
      for (const int direction : {-1, 1}) {
        const int next = static_cast<int>(i) + direction;
        if (next < 0 || next >= static_cast<int>(lanes.size())) continue;
        auto & legal = direction < 0 ? lanes[i].to_left : lanes[i].to_right;
        for (const auto & lane : sequences[i]) {
          const auto neighbor = direction < 0 ? graph->left(lane) : graph->right(lane);
          if (!neighbor || !ids[next].count(neighbor->id()) || lane.centerline().size() < 2)
            continue;
          const auto a = lane.centerline().front(), b = lane.centerline().back();
          const double begin = project(lines[i], {a.x(), a.y()}).s - ego_arcs[i];
          const double end = project(lines[i], {b.x(), b.y()}).s - ego_arcs[i];
          if (end > begin) {
            if (!legal.empty() && begin <= legal.back().end + 0.1)
              legal.back().end = end;
            else
              legal.push_back({begin, end});
          }
        }
      }
    }
    reasons = std::move(still_ahead);
    if (reasons.empty()) {
      lanes.clear();
      return false;
    }
    return true;
  }
};

TacticalLaneSelection::TacticalLaneSelection() : state_(std::make_unique<State>())
{
}
TacticalLaneSelection::~TacticalLaneSelection() = default;
std::shared_ptr<TacticalLaneSelection> TacticalLaneSelection::shared(rclcpp::Node & node)
{
  static std::mutex mutex;
  static std::map<rclcpp::Node *, std::weak_ptr<TacticalLaneSelection>> registry;
  std::lock_guard<std::mutex> lock(mutex);
  for (auto it = registry.begin(); it != registry.end();) {
    if (it->second.expired())
      it = registry.erase(it);
    else
      ++it;
  }
  auto shared = registry[&node].lock();
  if (!shared) {
    shared = std::shared_ptr<TacticalLaneSelection>(new TacticalLaneSelection());
    registry[&node] = shared;
  }
  return shared;
}
void TacticalLaneSelection::remember(
  const PlannerData & data, const std::vector<unique_identifier_msgs::msg::UUID> & reasons)
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->synchronize(data)) return;
  for (const auto & reason : reasons) {
    if (state_->reasons.insert(reason.uuid).second) state_->cached_stamp = -1.0;
  }
}
tactical::Score TacticalLaneSelection::evaluate(
  const PlannerData & data, const lanelet::ConstLanelets & current, const lanelet::Id target,
  const Parameters & parameters)
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  try {
    if (!state_->refresh(data, current, parameters)) return {};
    size_t index = state_->current;
    if (target != lanelet::InvalId) {
      const auto it = std::find_if(state_->ids.begin(), state_->ids.end(), [&](const auto & ids) {
        return ids.count(target) != 0;
      });
      if (it == state_->ids.end()) return {};
      if (std::count_if(state_->ids.begin(), state_->ids.end(), [&](const auto & ids) {
            return ids.count(target) != 0;
          }) != 1)
        return {};  // A merge is not a distinct parallel corridor.
      index = std::distance(state_->ids.begin(), it);
    }
    return tactical::score(
      state_->lanes, state_->current, index, data.parameters.vehicle_info.calcMaxCurvature());
  } catch (const std::exception &) {
    // Failure of a preference hint is not proof of insufficient physical space.
    state_->lanes.clear();
    state_->cached_stamp = -1.0;
    return {};
  }
}
std::optional<std::string> TacticalLaneSelection::deferReturn(
  const PlannerData & data, const lanelet::ConstLanelets & current,
  const lanelet::ConstLanelets & target, const Parameters & parameters)
{
  if (target.empty()) return std::nullopt;
  const auto stay = evaluate(data, current, lanelet::InvalId, parameters);
  if (!stay.known) return std::nullopt;
  if (!data.self_odometry) return std::nullopt;
  const auto & position = data.self_odometry->pose.pose.position;
  const auto target_lane = closest(target, {position.x, position.y});
  const auto move = evaluate(data, current, target_lane.id(), parameters);
  const bool intermediate_stage =
    data.route_handler && data.route_handler->getNumLaneToPreferredLane(target_lane) != 0;
  if (!tactical::dominatedReturn(
        stay, move, data.parameters.vehicle_info.vehicle_length_m, intermediate_stage))
    return std::nullopt;
  std::ostringstream reason;
  reason << "Tactical selection: defer return into observed avoidance blockage; stay progress="
         << stay.progress << "m, return progress=" << move.progress
         << "m; no final-return reservation";
  return reason.str();
}
}  // namespace autoware::behavior_path_planner::lane_change
