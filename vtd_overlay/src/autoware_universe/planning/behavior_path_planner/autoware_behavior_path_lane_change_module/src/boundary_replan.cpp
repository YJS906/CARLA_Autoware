// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#include "autoware/behavior_path_lane_change_module/utils/intersection_exit.hpp"
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"
#include "autoware/behavior_path_lane_change_module/utils/target_landing.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/difference.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <vector>

namespace autoware::behavior_path_planner
{
bool NormalLaneChange::replanAfterDrivableAreaStop()
{
  // The interface owns the fresh-feedback, AUTONOMOUS, RUNNING and continuous-stop gates.
  // Keep the approved target, RTC and the old path until a complete replacement passes.
  if (
    !is_activated_ || !status_.is_valid_path || isAbortState() ||
    !std::isfinite(getEgoVelocity()) || std::abs(getEgoVelocity()) > 0.1 ||
    !common_data_ptr_->is_lanes_available() || !planner_data_->dynamic_object ||
    !planner_data_->self_acceleration)
    return false;

  const double max_velocity = std::min(
    lane_change_parameters_->stopped_replan_velocity,
    common_data_ptr_->transient_data.current_path_velocity);
  const double budget_ms = lane_change_parameters_->time_limit;
  if (
    !std::isfinite(max_velocity) || max_velocity <= 0.0 || !std::isfinite(budget_ms) ||
    budget_ms <= 0.0)
    return false;
  const auto target_objects = get_target_objects(filtered_objects_, get_current_lanes());
  const auto exit_prepare = utils::lane_change::intersection_exit_prepare_length(common_data_ptr_);
  auto planning_data = common_data_ptr_;
  auto target_lanes = get_target_lanes();
  auto corridor = get_lane_change_corridor();
  if (exit_prepare) {
    // An approved legacy path may still carry the synthetic target. Recovery uses only real
    // map geometry at an intersection exit, without mutating the old path on failure.
    const auto map = getRouteHandler()->getLaneletMapPtr();
    if (!map) return false;
    for (auto & lane : target_lanes) {
      if (!map->laneletLayer.exists(lane.id())) return false;
      lane = map->laneletLayer.get(lane.id());
    }
    for (auto & lane : corridor) {
      if (!map->laneletLayer.exists(lane.id())) return false;
      lane = map->laneletLayer.get(lane.id());
    }
    planning_data = std::make_shared<lane_change::CommonData>(*common_data_ptr_);
    planning_data->lanes_ptr = std::make_shared<lane_change::Lanes>(*common_data_ptr_->lanes_ptr);
    planning_data->lanes_ptr->target = target_lanes;
    planning_data->target_lanes_path = getRouteHandler()->getCenterLinePath(target_lanes, 0.0, 1e6);
  }
  if (corridor.empty()) return false;
  std::vector<std::pair<lanelet::Id, lanelet::BasicPolygon2d>> polygons;
  for (const auto & lane : corridor)
    polygons.emplace_back(lane.id(), lane.polygon2d().basicPolygon());

  // Validate only a recovery candidate, never all normal lane changes. Crop a distant invalid
  // tail only after the merge; the existing stopping-envelope check must still fit in the tail.
  const auto fit_corridor = [&](LaneChangePath & candidate) {
    const auto end_index = candidate.info.shift_line.end_idx;
    for (size_t i = 0; i < candidate.path.points.size(); ++i) {
      auto & point = candidate.path.points[i];
      const auto footprint = utils::lane_change::get_ego_footprint(
        point.point.pose, planner_data_->parameters.vehicle_info);
      std::vector<autoware_utils::Polygon2d> remaining{footprint};
      point.lane_ids.clear();
      const autoware_utils::Point2d center(
        point.point.pose.position.x, point.point.pose.position.y);
      for (const auto & [id, polygon] : polygons) {
        if (boost::geometry::covered_by(center, polygon)) point.lane_ids.push_back(id);
        std::vector<autoware_utils::Polygon2d> next;
        for (const auto & part : remaining) boost::geometry::difference(part, polygon, next);
        remaining = std::move(next);
      }
      const bool outside = point.lane_ids.empty() ||
                           std::any_of(remaining.begin(), remaining.end(), [](const auto & p) {
                             return std::abs(boost::geometry::area(p)) > 1e-4;
                           });
      if (outside) {
        if (i <= end_index) return false;
        candidate.path.points.resize(i);
        break;
      }
    }
    if (candidate.path.points.size() <= end_index + 1) return false;
    candidate.path.points.back().point.longitudinal_velocity_mps = 0.0;
    for (size_t i = 0; i < candidate.shifted_path.path.points.size(); ++i) {
      candidate.shifted_path.path.points[i].lane_ids = candidate.path.points[i].lane_ids;
    }
    return true;
  };

  constexpr std::array<double, 10> distances{6.0,  8.0,  10.0, 12.0, 16.0,
                                             20.0, 25.0, 30.0, 35.0, 40.0};
  constexpr std::array<double, 3> speed_scales{1.0, 0.75, 0.5};
  constexpr std::array<double, 4> preparation_offsets{0.0, 2.0, 5.0, 10.0};
  const size_t geometry_samples = distances.size() * speed_scales.size();
  const size_t samples = geometry_samples * (exit_prepare ? preparation_offsets.size() : 1);
  boundary_replan_cursor_ %= samples;
  const auto started = std::chrono::steady_clock::now();
  const auto timed_out = [&]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
             .count() >= budget_ms;
  };
  for (size_t checked = 0; checked < samples && !timed_out(); ++checked) {
    const auto sample = boundary_replan_cursor_;
    boundary_replan_cursor_ = (boundary_replan_cursor_ + 1) % samples;
    const size_t geometry_sample = sample % geometry_samples;
    const double distance = distances[geometry_sample / speed_scales.size()];
    const double velocity = max_velocity * speed_scales[geometry_sample % speed_scales.size()];
    const double prepare = exit_prepare
      ? *exit_prepare + preparation_offsets[sample / geometry_samples] : 0.0;
    auto candidate = exit_prepare
      ? utils::lane_change::generate_exit_lane_change_path(planning_data, prepare, distance, velocity)
      : utils::lane_change::generate_low_speed_path(common_data_ptr_, distance, velocity);
    if (!candidate) continue;
    if (
      utils::lane_change::find_narrow_target_landing(
        target_lanes, candidate->info.lane_changing_end,
        planner_data_->parameters.vehicle_info))
      continue;
    if (!fit_corridor(*candidate)) continue;
    if (exit_prepare && utils::lane_change::starts_before_intersection_exit(
          planning_data, candidate->info.lane_changing_start)) continue;
    if (
      utils::lane_change::is_intersecting_no_lane_change_lines(
        common_data_ptr_, candidate->info.length, candidate->shifted_path.path.points))
      continue;
    try {
      // Includes measured-motion stopping space, static swept collision, steering geometry,
      // enabled comfort limits, RSS prediction and moving objects. No force-safe shortcut.
      if (!check_candidate_path_safety(*candidate, target_objects)) continue;
    } catch (const std::logic_error &) {
      continue;
    }
    // A completed valid check may cross the budget. Keep that result instead of starving
    // recovery forever on a corridor whose single geometry check costs more than one slice.
    if (exit_prepare) {
      RCLCPP_WARN(logger_, "Intersection-exit recovery: follow current lane %.2fm before change %.1fm",
        candidate->info.length.prepare, distance);
    }
    status_.lane_change_path = std::move(*candidate);
    status_.is_valid_path = true;
    status_.is_safe = true;
    unsafe_hysteresis_count_ = 0;
    approved_path_blocked_ = false;
    stopped_curve_template_.reset();
    stopped_curve_cursor_ = 0;
    stopped_curve_offset_ = 0.0;
    speed_preparation_target_.reset();
    speed_preparation_profile_.reset();
    speed_preparation_lane_id_ = lanelet::InvalId;
    toNormalState();
    RCLCPP_WARN(
      logger_,
      "Boundary-stop recovery: new path from ego, landing=%.1fm speed<=%.2fm/s; "
      "approved target retained, downstream boundary validation required",
      distance, velocity);
    return true;
  }
  RCLCPP_WARN_THROTTLE(
    logger_, clock_, 3000, "Boundary-stop recovery: no valid replacement; keep existing stop");
  return false;
}
}  // namespace autoware::behavior_path_planner
