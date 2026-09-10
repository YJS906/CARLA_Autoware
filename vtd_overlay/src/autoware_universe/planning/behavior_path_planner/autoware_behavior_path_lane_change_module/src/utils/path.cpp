// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_lane_change_module/utils/path.hpp"

#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/behavior_path_planner_common/utils/path_shifter/shift_constraints.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_frenet_planner/frenet_planner.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <range/v3/action/insert.hpp>
#include <range/v3/action/remove_if.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

namespace
{
using autoware::behavior_path_planner::LaneChangeInfo;
using autoware::behavior_path_planner::PathPointWithLaneId;
using autoware::behavior_path_planner::PathShifter;
using autoware::behavior_path_planner::PathWithLaneId;
using autoware::behavior_path_planner::ShiftedPath;
using autoware::behavior_path_planner::lane_change::CommonDataPtr;
using autoware::behavior_path_planner::lane_change::LCParamPtr;

using autoware::behavior_path_planner::LaneChangePhaseMetrics;
using autoware::behavior_path_planner::ShiftLine;
using autoware::behavior_path_planner::lane_change::TrajectoryGroup;
using autoware::frenet_planner::FrenetState;
using autoware::frenet_planner::SamplingParameters;
using autoware::route_handler::Direction;
using autoware::sampler_common::FrenetPoint;
using autoware::sampler_common::transform::Spline2D;
using autoware_utils::LineString2d;
using autoware_utils::Point2d;
using geometry_msgs::msg::Pose;

template <typename T>
T sign(const Direction direction)
{
  return static_cast<T>(direction == Direction::LEFT ? 1.0 : -1.0);
}

double calc_resample_interval(
  const double lane_changing_length, const double lane_changing_velocity)
{
  constexpr auto min_resampling_points{30.0};
  constexpr auto resampling_dt{0.2};
  return std::max(
    lane_changing_length / min_resampling_points, lane_changing_velocity * resampling_dt);
}

PathWithLaneId get_reference_path_from_target_lane(
  const CommonDataPtr & common_data_ptr, const Pose & lane_changing_start_pose,
  const double lane_changing_length, const double resample_interval)
{
  const auto & route_handler = *common_data_ptr->route_handler_ptr;
  const auto & target_lanes = common_data_ptr->lanes_ptr->target;
  const auto target_lane_length = common_data_ptr->transient_data.target_lane_length;
  const auto is_goal_in_route = common_data_ptr->lanes_ptr->target_lane_in_goal_section;
  const auto next_lc_buffer = common_data_ptr->transient_data.next_dist_buffer.min;
  const auto forward_path_length = common_data_ptr->bpp_param_ptr->forward_path_length;

  const auto lane_change_start_arc_position =
    autoware::experimental::lanelet2_utils::get_arc_coordinates(
      target_lanes, lane_changing_start_pose);

  const double s_start = lane_change_start_arc_position.length;
  const double s_end = std::invoke([&]() {
    const auto dist_from_lc_start = s_start + lane_changing_length + forward_path_length;
    if (is_goal_in_route) {
      const double s_goal = autoware::experimental::lanelet2_utils::get_arc_coordinates(
                              target_lanes, route_handler.getGoalPose())
                              .length -
                            next_lc_buffer;
      return std::min(dist_from_lc_start, s_goal);
    }
    return std::min(dist_from_lc_start, target_lane_length - next_lc_buffer);
  });

  constexpr double epsilon = 1e-4;
  if (s_end - s_start + epsilon < lane_changing_length) {
    return PathWithLaneId();
  }

  const auto lane_changing_reference_path =
    route_handler.getCenterLinePath(target_lanes, s_start, s_end);

  return autoware::behavior_path_planner::utils::resamplePathWithSpline(
    lane_changing_reference_path, resample_interval, true, {0.0, lane_changing_length});
}

ShiftLine get_lane_changing_shift_line(
  const Pose & lane_changing_start_pose, const Pose & lane_changing_end_pose,
  const PathWithLaneId & reference_path, const double shift_length)
{
  ShiftLine shift_line;
  shift_line.end_shift_length = shift_length;
  shift_line.start = lane_changing_start_pose;
  shift_line.end = lane_changing_end_pose;
  shift_line.start_idx = autoware::motion_utils::findNearestIndex(
    reference_path.points, lane_changing_start_pose.position);
  shift_line.end_idx = autoware::motion_utils::findNearestIndex(
    reference_path.points, lane_changing_end_pose.position);

  return shift_line;
}

ShiftedPath get_shifted_path(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & target_lane_reference_path,
  const LaneChangeInfo & lane_change_info)
{
  const auto longitudinal_acceleration = lane_change_info.longitudinal_acceleration;

  PathShifter path_shifter;
  path_shifter.setPath(target_lane_reference_path);
  path_shifter.addShiftLine(lane_change_info.shift_line);
  path_shifter.setLongitudinalAcceleration(longitudinal_acceleration.lane_changing);
  const auto initial_lane_changing_velocity = lane_change_info.velocity.lane_changing;
  path_shifter.setVelocity(initial_lane_changing_velocity);
  const auto lateral_acceleration_limit =
    !common_data_ptr->lc_param_ptr->trajectory.enable_lateral_acceleration_limit ||
      (common_data_ptr->lc_type ==
          autoware::behavior_path_planner::LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE &&
        common_data_ptr->disable_lateral_acceleration_limit)
      ? std::numeric_limits<double>::max()
      : std::abs(lane_change_info.lateral_acceleration);
  path_shifter.setLateralAccelerationLimit(lateral_acceleration_limit);

  constexpr auto offset_back = false;
  ShiftedPath shifted_path;
  [[maybe_unused]] const auto success = path_shifter.generate(&shifted_path, offset_back);
  return shifted_path;
}

std::optional<double> exceed_yaw_threshold(
  const PathWithLaneId & prepare_segment, const PathWithLaneId & lane_changing_segment,
  const double yaw_th_rad)
{
  const auto & prepare = prepare_segment.points;
  const auto & lane_changing = lane_changing_segment.points;

  if (prepare.size() <= 1 || lane_changing.size() <= 1) {
    return std::nullopt;
  }

  const auto & p1 = std::prev(prepare.end() - 1)->point.pose;
  const auto & p2 = std::next(lane_changing.begin())->point.pose;
  const auto yaw_diff_rad = std::abs(
    autoware_utils::normalize_radian(tf2::getYaw(p1.orientation) - tf2::getYaw(p2.orientation)));
  if (yaw_diff_rad > yaw_th_rad) {
    return yaw_diff_rad;
  }
  return std::nullopt;
}

Spline2D init_reference_spline(const std::vector<PathPointWithLaneId> & target_lanes_ref_path)
{
  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(target_lanes_ref_path.size());
  ys.reserve(target_lanes_ref_path.size());
  for (const auto & p : target_lanes_ref_path) {
    xs.push_back(p.point.pose.position.x);
    ys.push_back(p.point.pose.position.y);
  }

  return {xs, ys};
}

FrenetState init_frenet_state(
  const FrenetPoint & start_position, const LaneChangePhaseMetrics & prepare_metrics)
{
  FrenetState initial_state;
  initial_state.position = start_position;
  initial_state.longitudinal_velocity = prepare_metrics.velocity;
  initial_state.lateral_velocity =
    0.0;  // TODO(Maxime): this can be sampled if we want but it would impact the LC duration
  initial_state.longitudinal_acceleration = prepare_metrics.sampled_lon_accel;
  initial_state.lateral_acceleration = prepare_metrics.lat_accel;
  return initial_state;
}

std::optional<SamplingParameters> init_sampling_parameters(
  const CommonDataPtr & common_data_ptr, const LaneChangePhaseMetrics & prepare_metrics,
  const FrenetState & initial_state, const Spline2D & ref_spline)
{
  const auto & trajectory = common_data_ptr->lc_param_ptr->trajectory;
  const auto min_lc_vel = trajectory.min_lane_changing_velocity;
  const auto max_lateral_acc =
    trajectory.lat_acc_map.find(std::max(min_lc_vel, prepare_metrics.velocity)).second;
  const auto initial_velocity = prepare_metrics.velocity;
  const auto lon_accel = initial_state.longitudinal_acceleration;

  const auto use_remaining_distance =
    common_data_ptr->lc_param_ptr->frenet.use_entire_remaining_distance;

  const auto [lc_length, duration, final_velocity] = std::invoke([&]() {
    auto duration = autoware::behavior_path_planner::utils::lane_change::calculation::calc_lateral_shift_time(
      std::abs(initial_state.position.d), trajectory, max_lateral_acc);
    if (initial_velocity <= 0.0) return std::make_tuple(0.0, 0.0, 0.0);
    duration = std::max(
      duration, (trajectory.enable_lateral_acceleration_limit &&
                 trajectory.enable_lateral_jerk_limit ? 1.0 : 2.0) *
                  autoware::behavior_path_planner::utils::minimumGeometricShiftLength(
                  initial_state.position.d, common_data_ptr->bpp_param_ptr->vehicle_info) /
                  initial_velocity);
    auto final_velocity = std::max(min_lc_vel, initial_velocity + lon_accel * duration);
    auto length = duration * (initial_velocity + final_velocity) * 0.5;
    if (!use_remaining_distance) return std::make_tuple(length, duration, final_velocity);
    const auto dist_lc_start_to_terminal_end =
      common_data_ptr->transient_data.dist_to_terminal_end - prepare_metrics.length -
      common_data_ptr->lc_param_ptr->lane_change_finish_judge_buffer;
    length = std::max(length, dist_lc_start_to_terminal_end);
    final_velocity = std::max(
      min_lc_vel, std::sqrt((initial_velocity * initial_velocity) + (2.0 * lon_accel * length)));
    duration = 2.0 * length / (initial_velocity + final_velocity);
    return std::make_tuple(length, duration, final_velocity);
  });

  if (!std::isfinite(duration) || duration <= 0.0) return std::nullopt;
  const auto target_s = std::min(ref_spline.lastS(), initial_state.position.s + lc_length);
  // for smooth lateral motion we want a constant lateral acceleration profile
  // this means starting from a 0 lateral velocity and setting a positive target lateral velocity
  const auto max_lat_vel = duration * max_lateral_acc;
  const auto target_lat_vel = trajectory.enable_lateral_acceleration_limit
    ? std::min(max_lat_vel, initial_state.position.d / duration)
    : initial_state.position.d / duration;

  SamplingParameters sampling_parameters;
  const auto & safety = common_data_ptr->lc_param_ptr->safety;
  sampling_parameters.resolution = safety.collision_check.prediction_time_resolution;
  sampling_parameters.parameters.emplace_back();
  sampling_parameters.parameters.back().target_duration = duration;
  sampling_parameters.parameters.back().target_state.position = {target_s, 0.0};
  sampling_parameters.parameters.back().target_state.lateral_velocity =
    std::copysign(target_lat_vel, sign<double>(common_data_ptr->direction));
  sampling_parameters.parameters.back().target_state.lateral_acceleration = 0.0;
  sampling_parameters.parameters.back().target_state.longitudinal_velocity = final_velocity;
  sampling_parameters.parameters.back().target_state.longitudinal_acceleration = lon_accel;
  return sampling_parameters;
}

std::vector<double> calc_curvatures(
  const std::vector<PathPointWithLaneId> & points, const Pose & current_pose)
{
  const auto nearest_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(points, current_pose.position);

  // Ignore all path points behind ego vehicle.
  if (points.size() <= nearest_segment_idx + 2) {
    return {};
  }

  std::vector<double> curvatures;
  curvatures.reserve(points.size() - nearest_segment_idx + 2);
  for (const auto & [p1, p2, p3] : ranges::views::zip(
         points | ranges::views::drop(nearest_segment_idx),
         points | ranges::views::drop(nearest_segment_idx + 1),
         points | ranges::views::drop(nearest_segment_idx + 2))) {
    const auto point1 = autoware_utils::get_point(p1);
    const auto point2 = autoware_utils::get_point(p2);
    const auto point3 = autoware_utils::get_point(p3);

    curvatures.push_back(autoware_utils::calc_curvature(point1, point2, point3));
  }

  return curvatures;
}

double calc_average_curvature(const std::vector<double> & curvatures)
{
  const auto filter_zeros = [](const auto & k) { return k != 0.0; };
  auto filtered_k = curvatures | ranges::views::filter(filter_zeros);
  if (filtered_k.empty()) {
    return 0.0;
  }
  const auto sums_of_curvatures = [](float sum, const double k) { return sum + std::abs(k); };
  const auto sum_of_k = ranges::accumulate(filtered_k, 0.0, sums_of_curvatures);
  const auto count_k = static_cast<double>(ranges::distance(filtered_k));
  return sum_of_k / count_k;
}

LineString2d get_linestring_bound(const lanelet::ConstLanelets & lanes, const Direction direction)
{
  LineString2d line_string;
  const auto & get_bound = [&](const lanelet::ConstLanelet & lane) {
    return (direction == Direction::LEFT) ? lane.leftBound2d() : lane.rightBound2d();
  };

  const auto reserve_size = ranges::accumulate(
    lanes, 0UL,
    [&](auto sum, const lanelet::ConstLanelet & lane) { return sum + get_bound(lane).size(); });
  line_string.reserve(reserve_size);
  for (const auto & lane : lanes) {
    ranges::for_each(get_bound(lane), [&line_string](const auto & point) {
      boost::geometry::append(line_string, Point2d(point.x(), point.y()));
    });
  }
  return line_string;
}

Point2d shift_point(const Point2d & p1, const Point2d & p2, const double offset)
{
  // Calculate the perpendicular vector
  double dx = p2.x() - p1.x();
  double dy = p2.y() - p1.y();
  double length = std::sqrt(dx * dx + dy * dy);

  // Normalize and find the perpendicular direction
  double nx = -dy / length;
  double ny = dx / length;

  return {p1.x() + nx * offset, p1.y() + ny * offset};
}

bool check_out_of_bound_paths(
  const CommonDataPtr & common_data_ptr, const std::vector<Pose> & lane_changing_poses,
  const LineString2d & lane_boundary, const Direction direction)
{
  const auto distance = (0.5 * common_data_ptr->bpp_param_ptr->vehicle_width + 0.1);
  const auto offset = sign<double>(direction) * distance;  // invert direction
  if (lane_changing_poses.size() <= 2) {
    return true;  // Remove candidates with insufficient poses
  }

  LineString2d path_ls;
  path_ls.reserve(lane_changing_poses.size());

  const auto segments = lane_changing_poses | ranges::views::sliding(2);
  ranges::for_each(segments | ranges::views::drop(1), [&](const auto & segment) {
    const auto & p1 = segment[0].position;
    const auto & p2 = segment[1].position;
    boost::geometry::append(path_ls, shift_point({p2.x, p2.y}, {p1.x, p1.y}, offset));
  });

  return boost::geometry::disjoint(path_ls, lane_boundary);  // Remove if disjoint
}

double calc_limit(const CommonDataPtr & common_data_ptr, const Pose & lc_end_pose)
{
  const auto dist_to_target_end = std::invoke([&]() {
    if (common_data_ptr->lanes_ptr->target_lane_in_goal_section) {
      return autoware::motion_utils::calcSignedArcLength(
        common_data_ptr->target_lanes_path.points, lc_end_pose.position,
        common_data_ptr->route_handler_ptr->getGoalPose().position);
    }
    return autoware::motion_utils::calcSignedArcLength(
      common_data_ptr->target_lanes_path.points, lc_end_pose.position,
      common_data_ptr->target_lanes_path.points.back().point.pose.position);
  });

  // v2 = u2 + 2ad
  // u = sqrt(2ad)
  return std::clamp(
    std::sqrt(
      std::abs(2.0 * common_data_ptr->bpp_param_ptr->min_acc * std::max(dist_to_target_end, 0.0))),
    common_data_ptr->lc_param_ptr->trajectory.min_lane_changing_velocity,
    common_data_ptr->bpp_param_ptr->max_vel);
}

};  // namespace

namespace autoware::behavior_path_planner::utils::lane_change
{
using behavior_path_planner::lane_change::CommonDataPtr;
using behavior_path_planner::lane_change::PathType;

bool get_prepare_segment(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & prev_module_path,
  const double prep_length, PathWithLaneId & prepare_segment)
{
  return get_prepare_segment(common_data_ptr, prev_module_path, prep_length, prepare_segment, false);
}

bool get_prepare_segment(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & prev_module_path,
  const double prep_length, PathWithLaneId & prepare_segment, const bool has_braking_profile)
{
  const auto & current_lanes = common_data_ptr->lanes_ptr->current;
  const auto & target_lanes = common_data_ptr->lanes_ptr->target;
  const auto backward_path_length = common_data_ptr->bpp_param_ptr->backward_path_length;

  if (current_lanes.empty() || target_lanes.empty()) {
    throw std::logic_error("Empty lanes!");
  }

  prepare_segment = prev_module_path;
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      prepare_segment.points, common_data_ptr->get_ego_pose(), 3.0, 1.0);
  if (has_braking_profile) {
    const auto & original = prev_module_path.points;
    if (original.size() < 2) return false;
    std::vector<double> arcs(original.size(), 0.0);
    for (size_t i = 1; i < original.size(); ++i) {
      arcs[i] = arcs[i - 1] + autoware_utils::calc_distance2d(original[i - 1], original[i]);
    }
    const double ego_arc = motion_utils::calcSignedArcLength(
      original, original.front().point.pose.position, common_data_ptr->get_ego_pose().position);
    const double end_arc = ego_arc + prep_length;
    if (!std::isfinite(end_arc) || end_arc < 0.0 || end_arc > arcs.back()) return false;
    const auto it = std::lower_bound(arcs.begin(), arcs.end(), end_arc);
    const size_t end_idx = std::min<size_t>(std::distance(arcs.begin(), it), original.size() - 1);
    auto endpoint = original[end_idx];
    endpoint.point.pose = motion_utils::calcInterpolatedPose(original, end_arc);
    if (end_idx > 0) endpoint.point.longitudinal_velocity_mps = std::min(
      endpoint.point.longitudinal_velocity_mps, original[end_idx - 1].point.longitudinal_velocity_mps);
    prepare_segment.points.clear();
    for (size_t i = 0; i < original.size(); ++i) {
      if (arcs[i] < ego_arc - backward_path_length || arcs[i] >= end_arc - 1e-6) continue;
      prepare_segment.points.push_back(original[i]);
    }
    prepare_segment.points.push_back(endpoint);
  } else {
    utils::clipPathLength(prepare_segment, current_seg_idx, prep_length, backward_path_length);
  }

  if (prepare_segment.points.empty()) return false;

  const auto & lc_start_pose = prepare_segment.points.back().point.pose;

  // TODO(Quda, Azu): Is it possible to remove these checks if we ensure prepare segment length is
  // larger than distance to target lane start
  if (!is_valid_start_point(common_data_ptr, lc_start_pose)) return false;

  // lane changing start is at the end of prepare segment
  const auto target_length_from_lane_change_start_pose =
    utils::getArcLengthToTargetLanelet(current_lanes, target_lanes.front(), lc_start_pose);

  // Check if the lane changing start point is not on the lanes next to target lanes,
  if (target_length_from_lane_change_start_pose > std::numeric_limits<double>::epsilon()) {
    throw std::logic_error("lane change start is behind target lanelet!");
  }

  const auto curvatures = calc_curvatures(prepare_segment.points, common_data_ptr->get_ego_pose());

  // curvatures may be empty if ego is near the terminal start of the path.
  if (curvatures.empty()) {
    return true;
  }

  const auto average_curvature = calc_average_curvature(curvatures);

  RCLCPP_DEBUG(get_logger(), "average curvature: %.3f", average_curvature);
  if (has_braking_profile) {
    const double maximum = common_data_ptr->bpp_param_ptr->vehicle_info.calcMaxCurvature();
    return std::all_of(curvatures.begin(), curvatures.end(), [&](const double k) {
      return std::isfinite(k) && std::abs(k) <= maximum;
    });
  }
  return average_curvature <= common_data_ptr->lc_param_ptr->trajectory.th_prepare_curvature;
}

LaneChangePath get_candidate_path(
  const CommonDataPtr & common_data_ptr, const LaneChangePhaseMetrics & prep_metric,
  const LaneChangePhaseMetrics & lc_metric, const PathWithLaneId & prep_segment,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids, const double shift_length)
{
  const auto & route_handler = *common_data_ptr->route_handler_ptr;
  const auto & target_lanes = common_data_ptr->lanes_ptr->target;

  const auto resample_interval = calc_resample_interval(lc_metric.length, prep_metric.velocity);

  if (prep_segment.points.empty()) {
    throw std::logic_error("Empty prepare segment!");
  }

  const auto & lc_start_pose = prep_segment.points.back().point.pose;
  const auto target_lane_reference_path = get_reference_path_from_target_lane(
    common_data_ptr, lc_start_pose, lc_metric.length, resample_interval);

  if (target_lane_reference_path.points.empty()) {
    throw std::logic_error("Empty target reference!");
  }

  const auto lc_end_pose = std::invoke([&]() {
    const auto dist_to_lc_start =
      autoware::experimental::lanelet2_utils::get_arc_coordinates(target_lanes, lc_start_pose)
        .length;
    const auto dist_to_lc_end = dist_to_lc_start + lc_metric.length;
    return route_handler.get_pose_from_2d_arc_length(target_lanes, dist_to_lc_end);
  });

  const auto shift_line = get_lane_changing_shift_line(
    lc_start_pose, lc_end_pose, target_lane_reference_path, shift_length);

  LaneChangeInfo lane_change_info{prep_metric, lc_metric, lc_start_pose, lc_end_pose, shift_line};

  if (
    lane_change_info.length.sum() + common_data_ptr->transient_data.next_dist_buffer.min >
    common_data_ptr->transient_data.dist_to_terminal_end) {
    throw std::logic_error("invalid candidate path length!");
  }

  return utils::lane_change::construct_candidate_path(
    common_data_ptr, lane_change_info, prep_segment, target_lane_reference_path, sorted_lane_ids);
}

LaneChangePath construct_candidate_path(
  const CommonDataPtr & common_data_ptr, const LaneChangeInfo & lane_change_info,
  const PathWithLaneId & prepare_segment, const PathWithLaneId & target_lane_reference_path,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids)
{
  const auto & shift_line = lane_change_info.shift_line;
  const auto terminal_lane_changing_velocity = lane_change_info.terminal_lane_changing_velocity;

  auto shifted_path =
    get_shifted_path(common_data_ptr, target_lane_reference_path, lane_change_info);

  if (shifted_path.path.points.empty()) {
    throw std::logic_error("Failed to generate shifted path.");
  }

  if (shifted_path.path.points.size() < shift_line.end_idx + 1) {
    throw std::logic_error("Path points are removed by PathShifter.");
  }

  const auto lc_end_idx_opt = autoware::motion_utils::findNearestIndex(
    shifted_path.path.points, lane_change_info.lane_changing_end);

  if (!lc_end_idx_opt) {
    throw std::logic_error("Lane change end idx not found on target path.");
  }

  std::vector<int64_t> prev_ids;
  std::vector<int64_t> prev_sorted_ids;
  if (shifted_path.path.points.size() != target_lane_reference_path.points.size()) {
    throw std::logic_error("Shifted/reference path size mismatch.");
  }
  double reference_arc = 0.0;
  for (size_t i = 0; i < shifted_path.path.points.size(); ++i) {
    auto & point = shifted_path.path.points.at(i);
    if (i > 0) {
      reference_arc += autoware_utils::calc_distance2d(
        target_lane_reference_path.points[i - 1], target_lane_reference_path.points[i]);
    }
    // Include the shift endpoint: road speed is available only after lateral completion.
    // Use the same longitudinal motion as PathShifter, not the terminal speed at every point.
    if (i <= *lc_end_idx_opt) {
      const auto initial_velocity = lane_change_info.velocity.lane_changing;
      const auto acceleration = lane_change_info.longitudinal_acceleration.lane_changing;
      const auto velocity = std::sqrt(std::max(
        0.0,
        initial_velocity * initial_velocity +
          2.0 * acceleration * std::min(reference_arc, lane_change_info.length.lane_changing)));
      point.point.longitudinal_velocity_mps = std::min(
        point.point.longitudinal_velocity_mps,
        static_cast<float>(std::min(velocity, terminal_lane_changing_velocity)));
    }
    if (i < *lc_end_idx_opt) {
      const auto & current_ids = point.lane_ids;
      point.lane_ids =
        replace_with_sorted_ids(current_ids, sorted_lane_ids, prev_ids, prev_sorted_ids);
      continue;
    }
    const auto nearest_idx =
      autoware::motion_utils::findNearestIndex(target_lane_reference_path.points, point.point.pose);
    point.lane_ids = target_lane_reference_path.points.at(*nearest_idx).lane_ids;
  }

  if (utils::lane_change::is_intersecting_no_lane_change_lines(
        common_data_ptr, lane_change_info.length, shifted_path.path.points)) {
    throw std::logic_error("Intersect no lane change lines.");
  }

  constexpr auto yaw_diff_th = autoware_utils::deg2rad(5.0);
  if (
    const auto yaw_diff_opt =
      exceed_yaw_threshold(prepare_segment, shifted_path.path, yaw_diff_th)) {
    std::stringstream err_msg;
    err_msg << "Excessive yaw difference " << yaw_diff_opt.value() << " which exceeds the "
            << yaw_diff_th << " radian threshold.";
    throw std::logic_error(err_msg.str());
  }

  LaneChangePath candidate_path;
  candidate_path.path = utils::combinePath(prepare_segment, shifted_path.path);
  candidate_path.shifted_path = shifted_path;
  candidate_path.info = lane_change_info;
  candidate_path.type = PathType::ConstantJerk;

  if (candidate_path.info.braking_profile) {
    auto & info = candidate_path.info;
    auto & profile = *info.braking_profile;
    info.braking_start_pose = common_data_ptr->get_ego_pose();
    auto & points = candidate_path.path.points;
    const auto & origin = points.front().point.pose.position;
    const double ego_arc = motion_utils::calcSignedArcLength(
      points, origin, info.braking_start_pose.position);
    const double start_arc = motion_utils::calcSignedArcLength(
      points, origin, info.lane_changing_start.position);
    const double actual_length = start_arc - ego_arc;
    if (actual_length + 1e-3 < profile.distance()) {
      throw std::logic_error("Preparation clips the required braking distance");
    }
    profile.hold_time += std::max(0.0, actual_length - profile.distance()) / profile.target_velocity;
    info.duration.prepare = profile.duration();
    info.length.prepare = actual_length;
    if (info.length.sum() + common_data_ptr->transient_data.next_dist_buffer.min >
        common_data_ptr->transient_data.dist_to_terminal_end + 1e-3) {
      throw std::logic_error("Reachable braking exceeds the lane-change terminal");
    }
    info.longitudinal_acceleration.prepare =
      (profile.target_velocity - profile.initial_velocity) / profile.duration();
    double arc = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
      if (i > 0) arc += autoware_utils::calc_distance2d(points[i - 1], points[i]);
      if (arc < ego_arc) continue;
      const double speed = profile.at_distance(arc - ego_arc).velocity;
      if (arc <= start_arc && points[i].point.longitudinal_velocity_mps + 1e-3 < speed) {
        throw std::logic_error("Braking profile cannot reach an existing earlier speed limit");
      }
      points[i].point.longitudinal_velocity_mps =
        std::min<double>(points[i].point.longitudinal_velocity_mps, speed);
    }
  }

  return candidate_path;
}

std::vector<lane_change::TrajectoryGroup> generate_frenet_candidates(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & prev_module_path,
  const std::vector<LaneChangePhaseMetrics> & prep_metrics)
{
  std::vector<lane_change::TrajectoryGroup> trajectory_groups;
  autoware_utils::StopWatch<std::chrono::microseconds> sw;

  const auto & transient_data = common_data_ptr->transient_data;
  const auto & current_lanes = common_data_ptr->lanes_ptr->current;
  const auto & target_lanes = common_data_ptr->lanes_ptr->target;
  const auto direction = common_data_ptr->direction;
  const auto current_lane_boundary = get_linestring_bound(current_lanes, direction);

  for (const auto & metric : prep_metrics) {
    PathWithLaneId prepare_segment;
    try {
      if (!utils::lane_change::get_prepare_segment(
            common_data_ptr, prev_module_path, metric.length, prepare_segment)) {
        RCLCPP_DEBUG(get_logger(), "Reject: failed to get valid prepare segment!");
        continue;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
      break;
    }
    const auto lc_start_pose = prepare_segment.points.back().point.pose;

    const auto dist_to_end_from_lc_start =
      calculation::calc_dist_from_pose_to_terminal_end(
        common_data_ptr, target_lanes, lc_start_pose) -
      common_data_ptr->lc_param_ptr->lane_change_finish_judge_buffer;
    const auto raw_max_lc_len = transient_data.lane_changing_length.max;
    const auto max_lc_len =
      common_data_ptr->lc_type == LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE
        ? raw_max_lc_len * std::clamp(common_data_ptr->max_lane_changing_length_scale, 0.0, 1.0)
        : raw_max_lc_len;
    const auto max_lane_changing_length = std::min(dist_to_end_from_lc_start, max_lc_len);

    constexpr auto resample_interval = 0.5;
    const auto target_lane_reference_path = get_reference_path_from_target_lane(
      common_data_ptr, lc_start_pose, max_lane_changing_length, resample_interval);
    if (target_lane_reference_path.points.empty()) {
      continue;
    }

    std::vector<double> target_ref_path_sums{0.0};
    target_ref_path_sums.reserve(target_lane_reference_path.points.size() - 1);
    double ref_s = 0.0;
    for (auto it = target_lane_reference_path.points.begin();
         std::next(it) != target_lane_reference_path.points.end(); ++it) {
      ref_s += autoware_utils::calc_distance2d(*it, *std::next(it));
      target_ref_path_sums.push_back(ref_s);
    }

    const auto reference_spline = init_reference_spline(target_lane_reference_path.points);

    auto initial_state = init_frenet_state(
      reference_spline.frenet({lc_start_pose.position.x, lc_start_pose.position.y}), metric);
    const auto lateral_acc =
      common_data_ptr->lc_param_ptr->trajectory.lat_acc_map.find(metric.velocity).second;
    const auto shift_time = std::max(
      calculation::calc_lateral_shift_time(
        std::abs(initial_state.position.d), common_data_ptr->lc_param_ptr->trajectory, lateral_acc),
      utils::minimumGeometricShiftLength(
        initial_state.position.d, common_data_ptr->bpp_param_ptr->vehicle_info) /
        std::max(metric.velocity, calculation::eps));
    initial_state.longitudinal_acceleration = calculation::calc_lane_changing_acceleration(
      common_data_ptr, metric.velocity, transient_data.current_path_velocity, shift_time,
      std::max(0.0, metric.sampled_lon_accel));

    RCLCPP_DEBUG(
      get_logger(), "Initial state [s=%2.2f, d=%2.2f, s'=%2.2f, d'=%2.2f, s''=%2.2f, d''=%2.2f]",
      initial_state.position.s, initial_state.position.d, initial_state.longitudinal_velocity,
      initial_state.lateral_velocity, initial_state.longitudinal_acceleration,
      initial_state.lateral_acceleration);

    const auto sampling_parameters_opt =
      init_sampling_parameters(common_data_ptr, metric, initial_state, reference_spline);

    if (!sampling_parameters_opt) {
      continue;
    }

    set_prepare_velocity(prepare_segment, common_data_ptr->get_ego_speed(), metric.velocity);
    auto frenet_trajectories = frenet_planner::generateTrajectories(
      reference_spline, initial_state, *sampling_parameters_opt);

    trajectory_groups.reserve(trajectory_groups.size() + frenet_trajectories.size());
    for (const auto & traj : frenet_trajectories) {
      if (!trajectory_groups.empty()) {
        const auto diff = std::abs(
          traj.frenet_points.back().s -
          trajectory_groups.back().lane_changing.frenet_points.back().s);
        if (diff < common_data_ptr->lc_param_ptr->trajectory.th_lane_changing_length_diff) {
          continue;
        }
      }

      const auto out_of_bound =
        check_out_of_bound_paths(common_data_ptr, traj.poses, current_lane_boundary, direction);

      if (out_of_bound) {
        continue;
      }

      const auto lane_changing_average_curvature = calc_average_curvature(traj.curvatures);
      trajectory_groups.emplace_back(
        prepare_segment, target_lane_reference_path, target_ref_path_sums, metric, traj,
        initial_state, lane_changing_average_curvature, max_lane_changing_length);
    }
  }

  const auto limit_vel = [&](TrajectoryGroup & group) {
    const auto max_vel = calc_limit(common_data_ptr, group.lane_changing.poses.back());
    for (auto & vel : group.lane_changing.longitudinal_velocities) {
      vel = std::clamp(vel, 0.0, max_vel);
    }
  };

  ranges::for_each(trajectory_groups, limit_vel);

  ranges::sort(trajectory_groups, [&](const auto & p1, const auto & p2) {
    return p1.lc_average_curvature < p2.lc_average_curvature;
  });

  if (trajectory_groups.size() > 1) {
    auto subrange = ranges::subrange(trajectory_groups.begin() + 1, trajectory_groups.end());

    auto remove_start = ranges::remove_if(subrange, [&](const auto & traj) {
      return traj.lc_average_curvature > common_data_ptr->lc_param_ptr->frenet.th_average_curvature;
    });

    trajectory_groups.erase(remove_start, trajectory_groups.end());
  }

  // Curvature filtering above remains unchanged. Among surviving candidates prefer
  // completion time, rather than rewarding a longer, gentler lateral maneuver.
  std::stable_sort(
    trajectory_groups.begin(), trajectory_groups.end(), [](const auto & a, const auto & b) {
      return std::make_tuple(
               a.lane_changing.sampling_parameter.target_duration, a.lc_average_curvature) <
             std::make_tuple(
               b.lane_changing.sampling_parameter.target_duration, b.lc_average_curvature);
    });
  return trajectory_groups;
}

std::optional<LaneChangePath> get_candidate_path(
  const TrajectoryGroup & trajectory_group, const CommonDataPtr & common_data_ptr,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids)
{
  if (trajectory_group.lane_changing.frenet_points.empty()) {
    return std::nullopt;
  }

  ShiftedPath shifted_path;
  std::vector<int64_t> prev_ids;
  std::vector<int64_t> prev_sorted_ids;
  const auto & lane_changing_candidate = trajectory_group.lane_changing;
  const auto & target_lane_ref_path = trajectory_group.target_lane_ref_path;
  const auto & prepare_segment = trajectory_group.prepare;
  const auto & prepare_metric = trajectory_group.prepare_metric;
  const auto & initial_state = trajectory_group.initial_state;
  const auto & target_ref_sums = trajectory_group.target_lane_ref_path_dist;
  auto zipped_candidates = ranges::views::zip(
    lane_changing_candidate.poses, lane_changing_candidate.frenet_points,
    lane_changing_candidate.longitudinal_velocities, lane_changing_candidate.lateral_velocities,
    lane_changing_candidate.curvatures);

  shifted_path.path.points.reserve(zipped_candidates.size());
  shifted_path.shift_length.reserve(zipped_candidates.size());
  for (const auto & [pose, frenet_point, longitudinal_velocity, lateral_velocity, curvature] :
       zipped_candidates) {
    // Find the reference index
    const auto & s = frenet_point.s;
    auto ref_i_itr = std::find_if(
      target_ref_sums.begin(), target_ref_sums.end(),
      [s](const double ref_s) { return ref_s > s; });
    auto ref_i = std::distance(target_ref_sums.begin(), ref_i_itr);

    PathPointWithLaneId point;
    point.point.pose = pose;
    point.point.longitudinal_velocity_mps = static_cast<float>(longitudinal_velocity);
    point.point.lateral_velocity_mps = static_cast<float>(lateral_velocity);
    point.point.heading_rate_rps = static_cast<float>(curvature);
    point.point.pose.position.z = target_lane_ref_path.points[ref_i].point.pose.position.z;
    const auto & current_ids = target_lane_ref_path.points[ref_i].lane_ids;
    point.lane_ids =
      replace_with_sorted_ids(current_ids, sorted_lane_ids, prev_ids, prev_sorted_ids);

    // Add to shifted path
    shifted_path.shift_length.push_back(frenet_point.d);
    shifted_path.path.points.push_back(point);
  }

  if (shifted_path.path.points.empty()) {
    return std::nullopt;
  }

  for (auto & point : shifted_path.path.points) {
    point.point.longitudinal_velocity_mps = std::min(
      point.point.longitudinal_velocity_mps,
      static_cast<float>(shifted_path.path.points.back().point.longitudinal_velocity_mps));
  }

  const auto th_yaw_diff_deg = common_data_ptr->lc_param_ptr->frenet.th_yaw_diff_deg;
  if (
    const auto yaw_diff_opt = exceed_yaw_threshold(
      prepare_segment, shifted_path.path, autoware_utils::deg2rad(th_yaw_diff_deg))) {
    const auto yaw_diff_deg = autoware_utils::rad2deg(yaw_diff_opt.value());
    const auto err_msg = fmt::format(
      "Excessive yaw difference {yaw_diff:2.2f}[deg]. The threshold is {th_yaw_diff:2.2f}[deg].",
      fmt::arg("yaw_diff", yaw_diff_deg), fmt::arg("th_yaw_diff", th_yaw_diff_deg));
    throw std::logic_error(err_msg);
  }

  LaneChangeInfo info;
  info.longitudinal_acceleration = {
    prepare_metric.actual_lon_accel, lane_changing_candidate.longitudinal_accelerations.front()};
  info.velocity = {prepare_metric.velocity, initial_state.longitudinal_velocity};
  info.duration = {
    prepare_metric.duration, lane_changing_candidate.sampling_parameter.target_duration};
  info.length = {prepare_metric.length, lane_changing_candidate.lengths.back()};
  info.lane_changing_start = prepare_segment.points.back().point.pose;
  info.lane_changing_end = lane_changing_candidate.poses.back();

  if (utils::lane_change::is_intersecting_no_lane_change_lines(
        common_data_ptr, info.length, shifted_path.path.points)) {
    throw std::logic_error("Intersect no lane change lines.");
  }

  ShiftLine sl;

  sl.start = lane_changing_candidate.poses.front();
  sl.end = lane_changing_candidate.poses.back();
  sl.start_shift_length = 0.0;
  sl.end_shift_length = initial_state.position.d;
  sl.start_idx = 0UL;
  sl.end_idx = shifted_path.shift_length.size() - 1;

  info.shift_line = sl;
  info.terminal_lane_changing_velocity = lane_changing_candidate.longitudinal_velocities.back();
  info.lateral_acceleration = lane_changing_candidate.lateral_accelerations.front();

  LaneChangePath candidate_path;
  candidate_path.path = utils::combinePath(prepare_segment, shifted_path.path);
  candidate_path.info = info;
  candidate_path.shifted_path = shifted_path;
  candidate_path.frenet_path = trajectory_group;
  candidate_path.type = PathType::FrenetPlanner;

  return candidate_path;
}

void append_target_ref_to_candidate(
  LaneChangePath & frenet_candidate, const double th_curvature_smoothing)
{
  const auto & target_lane_ref_path = frenet_candidate.frenet_path.target_lane_ref_path.points;
  const auto & lc_end_pose = frenet_candidate.info.lane_changing_end;
  const auto lc_end_idx =
    motion_utils::findNearestIndex(target_lane_ref_path, lc_end_pose.position);
  auto & candidate_path = frenet_candidate.path.points;
  const auto & points = target_lane_ref_path;
  if (target_lane_ref_path.size() <= lc_end_idx + 2) {
    for (const auto & pt : points | ranges::views::drop(lc_end_idx + 1)) {
      candidate_path.push_back(pt);
    }
    return;
  }
  const auto add_size = target_lane_ref_path.size() - (lc_end_idx + 1);
  candidate_path.reserve(candidate_path.size() + add_size);
  for (const auto & [p2, p3] : ranges::views::zip(
         points | ranges::views::drop(lc_end_idx + 1),
         points | ranges::views::drop(lc_end_idx + 2))) {
    const auto point1 = autoware_utils::get_point(candidate_path.back().point.pose);
    const auto point2 = autoware_utils::get_point(p2);
    const auto point3 = autoware_utils::get_point(p3);
    const auto curvature = std::abs(autoware_utils::calc_curvature(point1, point2, point3));
    if (curvature < th_curvature_smoothing) {
      candidate_path.push_back(p2);
    }
  }
  candidate_path.push_back(target_lane_ref_path.back());
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
