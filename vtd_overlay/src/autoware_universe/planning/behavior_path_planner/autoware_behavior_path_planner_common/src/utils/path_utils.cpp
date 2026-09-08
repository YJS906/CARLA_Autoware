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

#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/interpolation/spline_interpolation.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils
{
/**
 * @brief calc path arclength on each points from start point to end point.
 */
std::vector<double> calcPathArcLengthArray(
  const PathWithLaneId & path, const size_t start, const size_t end, const double offset)
{
  const auto bounded_start = std::max(start, size_t{0});
  const auto bounded_end = std::min(end, path.points.size());
  std::vector<double> out;
  out.reserve(bounded_end - bounded_start);

  double sum = offset;
  out.push_back(sum);

  for (size_t i = bounded_start + 1; i < bounded_end; ++i) {
    sum += autoware_utils::calc_distance2d(path.points.at(i).point, path.points.at(i - 1).point);
    out.push_back(sum);
  }
  return out;
}

/**
 * @brief resamplePathWithSpline
 */
PathWithLaneId resamplePathWithSpline(
  const PathWithLaneId & path, const double interval, const bool keep_input_points,
  const std::pair<double, double> target_section)
{
  if (path.points.size() < 2) {
    return path;
  }

  std::vector<autoware_planning_msgs::msg::PathPoint> transformed_path(path.points.size());
  for (size_t i = 0; i < path.points.size(); ++i) {
    transformed_path.at(i) = path.points.at(i).point;
  }

  const auto find_almost_same_values =
    [&](const std::vector<double> & vec, double x) -> std::optional<std::vector<size_t>> {
    constexpr double epsilon = 0.2;
    const auto is_close = [&](double v, double x) { return std::abs(v - x) < epsilon; };

    std::vector<size_t> indices;
    if (vec.empty()) {
      return std::nullopt;
    }

    for (size_t i = 0; i < vec.size(); ++i) {
      if (is_close(vec[i], x)) {
        indices.push_back(i);
      }
    }

    if (indices.empty()) {
      return std::nullopt;
    }

    return indices;
  };

  // Get lane ids that are not duplicated
  std::vector<double> s_in;
  std::unordered_set<int64_t> unique_lane_ids;
  const auto s_vec = autoware::motion_utils::calcSignedArcLengthPartialSum(
    transformed_path, 0, transformed_path.size());
  for (size_t i = 0; i < path.points.size(); ++i) {
    const double s = s_vec.at(i);

    for (const auto & lane_id : path.points.at(i).lane_ids) {
      if (!keep_input_points && (unique_lane_ids.find(lane_id) != unique_lane_ids.end())) {
        continue;
      }
      unique_lane_ids.insert(lane_id);

      if (!find_almost_same_values(s_in, s)) {
        s_in.push_back(s);
      }
    }
  }

  std::vector<double> s_out = s_in;

  // sampling from interval distance
  const auto start_s = std::max(target_section.first, 0.0);
  const auto end_s = std::min(target_section.second, s_vec.back());
  for (double s = start_s; s < end_s; s += interval) {
    if (!find_almost_same_values(s_out, s)) {
      s_out.push_back(s);
    }
  }
  if (!find_almost_same_values(s_out, end_s)) {
    s_out.push_back(end_s);
  }

  // Insert Stop Point
  const auto closest_stop_dist =
    autoware::motion_utils::calcDistanceToForwardStopPoint(transformed_path);
  if (closest_stop_dist) {
    const auto close_indices = find_almost_same_values(s_out, *closest_stop_dist);
    if (close_indices) {
      // Update the smallest index
      s_out.at(close_indices->at(0)) = *closest_stop_dist;

      // Remove the rest of the indices in descending order
      for (size_t i = close_indices->size() - 1; i > 0; --i) {
        s_out.erase(s_out.begin() + close_indices->at(i));
      }
    } else {
      s_out.push_back(*closest_stop_dist);
    }
  }

  // spline resample required more than 2 points for yaw angle calculation
  if (s_out.size() < 2) {
    return path;
  }

  std::sort(s_out.begin(), s_out.end());

  return autoware::motion_utils::resamplePath(path, s_out);
}

size_t getIdxByArclength(
  const PathWithLaneId & path, const size_t target_idx, const double signed_arc)
{
  if (path.points.empty()) {
    throw std::runtime_error("[getIdxByArclength] path points must be > 0");
  }

  using autoware_utils::calc_distance2d;
  double sum_length = 0.0;
  if (signed_arc >= 0.0) {
    for (size_t i = target_idx; i < path.points.size() - 1; ++i) {
      const auto next_i = i + 1;
      sum_length += calc_distance2d(path.points.at(i), path.points.at(next_i));
      if (sum_length > signed_arc) {
        return next_i;
      }
    }
    return path.points.size() - 1;
  }
  for (size_t i = target_idx; i > 0; --i) {
    const auto next_i = i - 1;
    sum_length -= calc_distance2d(path.points.at(i), path.points.at(next_i));
    if (sum_length < signed_arc) {
      return next_i;
    }
  }
  return 0;
}

// TODO(murooka) This function should be replaced with autoware::motion_utils::cropPoints
void clipPathLength(
  PathWithLaneId & path, const size_t target_idx, const double forward, const double backward)
{
  if (path.points.size() < 3) {
    return;
  }

  const auto start_idx = utils::getIdxByArclength(path, target_idx, -backward);
  const auto end_idx = utils::getIdxByArclength(path, target_idx, forward);

  const std::vector<PathPointWithLaneId> clipped_points{
    path.points.begin() + start_idx, path.points.begin() + end_idx + 1};

  path.points = clipped_points;
}

PathWithLaneId convertWayPointsToPathWithLaneId(
  const autoware::freespace_planning_algorithms::PlannerWaypoints & waypoints,
  const double velocity, const lanelet::ConstLanelets & lanelets)
{
  PathWithLaneId path;
  path.header = waypoints.header;
  for (size_t i = 0; i < waypoints.waypoints.size(); ++i) {
    const auto & waypoint = waypoints.waypoints.at(i);
    PathPointWithLaneId point{};
    point.point.pose = waypoint.pose.pose;
    // put the lane that contain waypoints in lane_ids.
    bool is_in_lanes = false;
    for (const auto & lane : lanelets) {
      if (autoware::experimental::lanelet2_utils::is_in_lanelet(point.point.pose, lane)) {
        point.lane_ids.push_back(lane.id());
        is_in_lanes = true;
      }
    }
    // If none of them corresponds, assign the previous lane_ids.
    if (!is_in_lanes && i > 0) {
      point.lane_ids = path.points.at(i - 1).lane_ids;
    }

    point.point.longitudinal_velocity_mps = (waypoint.is_back ? -1 : 1) * velocity;
    path.points.push_back(point);
  }
  return path;
}

std::vector<size_t> getReversingIndices(const PathWithLaneId & path)
{
  std::vector<size_t> indices;

  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    if (
      path.points.at(i).point.longitudinal_velocity_mps *
        path.points.at(i + 1).point.longitudinal_velocity_mps <
      0) {
      indices.push_back(i);
    }
  }

  return indices;
}

std::vector<PathWithLaneId> dividePath(
  const PathWithLaneId & path, const std::vector<size_t> & indices)
{
  std::vector<PathWithLaneId> divided_paths;

  if (indices.empty()) {
    divided_paths.push_back(path);
    return divided_paths;
  }

  for (size_t i = 0; i < indices.size(); ++i) {
    PathWithLaneId divided_path;
    divided_path.header = path.header;
    if (i == 0) {
      divided_path.points.insert(
        divided_path.points.end(), path.points.begin(), path.points.begin() + indices.at(i) + 1);
    } else {
      // include the point at indices.at(i - 1) and indices.at(i)
      divided_path.points.insert(
        divided_path.points.end(), path.points.begin() + indices.at(i - 1),
        path.points.begin() + indices.at(i) + 1);
    }
    divided_paths.push_back(divided_path);
  }

  PathWithLaneId divided_path;
  divided_path.header = path.header;
  divided_path.points.insert(
    divided_path.points.end(), path.points.begin() + indices.back(), path.points.end());
  divided_paths.push_back(divided_path);

  return divided_paths;
}

void correctDividedPathVelocity(std::vector<PathWithLaneId> & divided_paths)
{
  for (auto & path : divided_paths) {
    const auto is_driving_forward = autoware::motion_utils::isDrivingForward(path.points);
    // If the number of points in the path is less than 2, don't correct the velocity
    if (!is_driving_forward) {
      continue;
    }

    if (*is_driving_forward) {
      for (auto & point : path.points) {
        point.point.longitudinal_velocity_mps = std::abs(point.point.longitudinal_velocity_mps);
      }
    } else {
      for (auto & point : path.points) {
        point.point.longitudinal_velocity_mps = -std::abs(point.point.longitudinal_velocity_mps);
      }
    }
    path.points.back().point.longitudinal_velocity_mps = 0.0;
  }
}

// only two points is supported
std::vector<double> spline_two_points(
  const std::vector<double> & base_s, const std::vector<double> & base_x, const double begin_diff,
  const double end_diff, const std::vector<double> & new_s)
{
  assert(base_s.size() == 2 && base_x.size() == 2);

  const double h = base_s.at(1) - base_s.at(0);

  const double c = begin_diff;
  const double d = base_x.at(0);
  const double a = (end_diff * h - 2 * base_x.at(1) + c * h + 2 * d) / std::pow(h, 3);
  const double b = (3 * base_x.at(1) - end_diff * h - 2 * c * h - 3 * d) / std::pow(h, 2);

  std::vector<double> res;
  for (const auto & s : new_s) {
    const double ds = s - base_s.at(0);
    res.push_back(d + (c + (b + a * ds) * ds) * ds);
  }

  return res;
}

std::vector<Pose> interpolatePose(
  const Pose & start_pose, const Pose & end_pose, const double resample_interval)
{
  using autoware_utils::calc_azimuth_angle;

  std::vector<Pose> interpolated_poses{};  // output

  const double distance = autoware_utils::calc_distance2d(start_pose.position, end_pose.position);
  const std::vector<double> base_s{0.0, distance};
  const std::vector<double> base_x{start_pose.position.x, end_pose.position.x};
  const std::vector<double> base_y{start_pose.position.y, end_pose.position.y};
  std::vector<double> new_s;

  constexpr double eps = 0.3;  // prevent overlapping
  for (double s = eps; s < distance - eps; s += resample_interval) {
    new_s.push_back(s);
  }

  const std::vector<double> interpolated_x = spline_two_points(
    base_s, base_x, std::cos(tf2::getYaw(start_pose.orientation)),
    std::cos(tf2::getYaw(end_pose.orientation)), new_s);
  const std::vector<double> interpolated_y = spline_two_points(
    base_s, base_y, std::sin(tf2::getYaw(start_pose.orientation)),
    std::sin(tf2::getYaw(end_pose.orientation)), new_s);
  for (size_t i = 0; i < interpolated_x.size(); ++i) {
    Pose pose{};
    pose.position.x = interpolated_x.at(i);
    pose.position.y = interpolated_y.at(i);
    pose.position.z = end_pose.position.z;
    interpolated_poses.push_back(pose);
  }

  // insert orientation
  for (size_t i = 0; i < interpolated_poses.size(); ++i) {
    const double yaw = calc_azimuth_angle(
      interpolated_poses.at(i).position, i < interpolated_poses.size() - 1
                                           ? interpolated_poses.at(i + 1).position
                                           : end_pose.position);
    interpolated_poses.at(i).orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  }

  return interpolated_poses;
}

Pose getUnshiftedEgoPose(const Pose & ego_pose, const ShiftedPath & prev_path)
{
  if (prev_path.path.points.empty()) {
    return ego_pose;
  }

  // un-shifted for current ideal pose
  const auto closest_idx =
    autoware::motion_utils::findNearestIndex(prev_path.path.points, ego_pose.position);

  // NOTE: Considering avoidance by motion, we set unshifted_pose as previous path instead of
  // ego_pose.
  auto unshifted_pose =
    autoware::motion_utils::calcInterpolatedPoint(prev_path.path, ego_pose).point.pose;

  unshifted_pose = autoware_utils::calc_offset_pose(
    unshifted_pose, 0.0, -prev_path.shift_length.at(closest_idx), 0.0);
  unshifted_pose.orientation = ego_pose.orientation;

  return unshifted_pose;
}

// TODO(Horibe) clean up functions: there is a similar code in util as well.
PathWithLaneId calcCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data, const Pose & ref_pose,
  const double longest_dist_to_shift_line, const std::optional<PathWithLaneId> & prev_module_path)
{
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;

  PathWithLaneId centerline_path;

  const auto extra_margin = 10.0;  // Since distance does not consider arclength, but just line.
  const auto backward_length =
    std::max(p.backward_path_length, longest_dist_to_shift_line + extra_margin);

  RCLCPP_DEBUG(
    rclcpp::get_logger("path_utils"),
    "p.backward_path_length = %f, longest_dist_to_shift_line = %f, backward_length = %f",
    p.backward_path_length, longest_dist_to_shift_line, backward_length);

  const lanelet::ConstLanelets current_lanes = [&]() {
    if (!prev_module_path) {
      return utils::calcLaneAroundPose(
        route_handler, ref_pose, p.forward_path_length, backward_length);
    }
    return utils::getCurrentLanesFromPath(*prev_module_path, planner_data);
  }();

  centerline_path = utils::getCenterLinePath(
    *route_handler, current_lanes, ref_pose, backward_length, p.forward_path_length, p);

  centerline_path.header = route_handler->getRouteHeader();

  return centerline_path;
}

PathWithLaneId combinePath(const PathWithLaneId & path1, const PathWithLaneId & path2)
{
  if (path1.points.empty()) {
    return path2;
  }
  if (path2.points.empty()) {
    return path1;
  }

  PathWithLaneId path{};
  path.points.insert(path.points.end(), path1.points.begin(), path1.points.end());

  // skip overlapping point
  path.points.insert(path.points.end(), next(path2.points.begin()), path2.points.end());

  PathWithLaneId filtered_path = path;
  filtered_path.points = autoware::motion_utils::removeOverlapPoints(filtered_path.points);
  return filtered_path;
}

namespace
{
std::optional<double> getLaneletEntryYaw(const lanelet::ConstLanelet & lane)
{
  const auto & centerline = lane.centerline();
  if (centerline.size() < 2) {
    return std::nullopt;
  }
  const auto & p0 = centerline.front();
  const auto & p1 = centerline[std::min<size_t>(2, centerline.size() - 1)];
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

std::optional<double> getLaneletExitYaw(const lanelet::ConstLanelet & lane)
{
  const auto & centerline = lane.centerline();
  if (centerline.size() < 2) {
    return std::nullopt;
  }
  const auto & p1 = centerline.back();
  const auto & p0 = centerline[centerline.size() - std::min<size_t>(3, centerline.size())];
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

std::optional<double> getLaneletChordYaw(const lanelet::ConstLanelet & lane)
{
  const auto & centerline = lane.centerline();
  if (centerline.size() < 2) {
    return std::nullopt;
  }
  const auto & p0 = centerline.front();
  const auto & p1 = centerline.back();
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

double yawDifference(const double yaw1, const double yaw2)
{
  return std::abs(std::atan2(std::sin(yaw1 - yaw2), std::cos(yaw1 - yaw2)));
}

bool isSameManeuver(
  const lanelet::ConstLanelet & route_lane, const lanelet::ConstLanelet & candidate)
{
  const auto route_turn = route_lane.attributeOr("turn_direction", std::string{});
  const auto candidate_turn = candidate.attributeOr("turn_direction", std::string{});
  const auto is_turning = [](const std::string & turn) {
    return turn == "left" || turn == "right";
  };
  return (!is_turning(route_turn) && !is_turning(candidate_turn)) || route_turn == candidate_turn;
}

template <class Centerline>
double centerlineLength(const Centerline & centerline)
{
  double length = 0.0;
  for (size_t i = 1; i < centerline.size(); ++i) {
    length += std::hypot(
      centerline[i].x() - centerline[i - 1].x(), centerline[i].y() - centerline[i - 1].y());
  }
  return length;
}

template <class Centerline>
geometry_msgs::msg::Point interpolateCenterline(
  const Centerline & centerline, const double target_length)
{
  geometry_msgs::msg::Point point;
  if (centerline.empty()) {
    return point;
  }

  double accumulated_length = 0.0;
  for (size_t i = 1; i < centerline.size(); ++i) {
    const auto & p0 = centerline[i - 1];
    const auto & p1 = centerline[i];
    const double segment_length = std::hypot(p1.x() - p0.x(), p1.y() - p0.y());
    if (accumulated_length + segment_length >= target_length && segment_length > 1.0e-6) {
      const double ratio =
        std::clamp((target_length - accumulated_length) / segment_length, 0.0, 1.0);
      point.x = p0.x() + ratio * (p1.x() - p0.x());
      point.y = p0.y() + ratio * (p1.y() - p0.y());
      point.z = p0.z() + ratio * (p1.z() - p0.z());
      return point;
    }
    accumulated_length += segment_length;
  }

  point.x = centerline.back().x();
  point.y = centerline.back().y();
  point.z = centerline.back().z();
  return point;
}

template <class Centerline>
double projectToCenterline(const Centerline & centerline, const geometry_msgs::msg::Point & point)
{
  double best_squared_distance = std::numeric_limits<double>::max();
  double best_length = 0.0;
  double accumulated_length = 0.0;
  for (size_t i = 1; i < centerline.size(); ++i) {
    const auto & p0 = centerline[i - 1];
    const auto & p1 = centerline[i];
    const double dx = p1.x() - p0.x();
    const double dy = p1.y() - p0.y();
    const double squared_length = dx * dx + dy * dy;
    const double segment_length = std::sqrt(squared_length);
    const double ratio =
      squared_length > 1.0e-12
        ? std::clamp(((point.x - p0.x()) * dx + (point.y - p0.y()) * dy) / squared_length, 0.0, 1.0)
        : 0.0;
    const double projected_x = p0.x() + ratio * dx;
    const double projected_y = p0.y() + ratio * dy;
    const double squared_distance =
      std::pow(point.x - projected_x, 2) + std::pow(point.y - projected_y, 2);
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      best_length = accumulated_length + ratio * segment_length;
    }
    accumulated_length += segment_length;
  }
  return best_length;
}

lanelet::BasicLineString3d combineCenterlines(const lanelet::ConstLanelets & lanes)
{
  lanelet::BasicLineString3d combined;
  for (const auto & lane : lanes) {
    for (const auto & point : lane.centerline3d()) {
      const auto basic_point = point.basicPoint();
      if (
        !combined.empty() &&
        std::hypot(combined.back().x() - basic_point.x(), combined.back().y() - basic_point.y()) <
          1.0e-3) {
        continue;
      }
      combined.push_back(basic_point);
    }
  }
  return combined;
}

std::optional<lanelet::ConstLanelet> getPreviousRouteLane(
  const lanelet::ConstLanelet & route_lane, const RouteHandler & route_handler)
{
  const auto previous_lanes = route_handler.getPreviousLanelets(route_lane);
  const auto previous = std::find_if(
    previous_lanes.begin(), previous_lanes.end(),
    [&route_handler](const auto & lane) { return route_handler.isRouteLanelet(lane); });
  if (previous == previous_lanes.end()) {
    return std::nullopt;
  }
  return *previous;
}

bool isCompletedParallelLane(
  const lanelet::ConstLanelet & route_lane, const lanelet::ConstLanelet & candidate)
{
  if (
    candidate.attributeOr(lanelet::AttributeName::Subtype, std::string{}) != "road" ||
    !isSameManeuver(route_lane, candidate)) {
    return false;
  }

  const auto route_entry_yaw = getLaneletEntryYaw(route_lane);
  const auto route_exit_yaw = getLaneletExitYaw(route_lane);
  const auto candidate_entry_yaw = getLaneletEntryYaw(candidate);
  const auto candidate_exit_yaw = getLaneletExitYaw(candidate);
  constexpr double max_parallel_yaw_difference = 5.0 * M_PI / 180.0;
  if (
    !route_entry_yaw || !route_exit_yaw || !candidate_entry_yaw || !candidate_exit_yaw ||
    yawDifference(*route_entry_yaw, *candidate_entry_yaw) > max_parallel_yaw_difference ||
    yawDifference(*route_exit_yaw, *candidate_exit_yaw) > max_parallel_yaw_difference) {
    return false;
  }

  const auto & route_centerline = route_lane.centerline3d();
  const auto & candidate_centerline = candidate.centerline3d();
  if (route_centerline.empty() || candidate_centerline.empty()) {
    return false;
  }
  const auto lateral_separation =
    [](const auto & route_point, const auto & candidate_point, const double route_yaw) {
      const double dx = candidate_point.x() - route_point.x();
      const double dy = candidate_point.y() - route_point.y();
      return -std::sin(route_yaw) * dx + std::cos(route_yaw) * dy;
    };
  const double entry_separation =
    lateral_separation(route_centerline.front(), candidate_centerline.front(), *route_entry_yaw);
  const double exit_separation =
    lateral_separation(route_centerline.back(), candidate_centerline.back(), *route_exit_yaw);
  constexpr double max_separation_change = 1.0;
  return std::abs(std::abs(exit_separation) - std::abs(entry_separation)) <= max_separation_change;
}

lanelet::ConstLanelets getCompletedParallelCorridor(
  const lanelet::ConstLanelet & route_lane, const RouteHandler & route_handler)
{
  lanelet::ConstLanelets corridor{route_lane};
  auto leftmost = route_lane;
  while (const auto left = route_handler.getLeftLanelet(leftmost, true, false)) {
    if (!isCompletedParallelLane(route_lane, *left)) {
      break;
    }
    corridor.insert(corridor.begin(), *left);
    leftmost = *left;
  }
  auto rightmost = route_lane;
  while (const auto right = route_handler.getRightLanelet(rightmost, true, false)) {
    if (!isCompletedParallelLane(route_lane, *right)) {
      break;
    }
    corridor.push_back(*right);
    rightmost = *right;
  }
  return corridor;
}

struct ParallelSuccessorPair
{
  lanelet::ConstLanelet route_lane;
  lanelet::ConstLanelet sibling_lane;
};

std::optional<ParallelSuccessorPair> getParallelSuccessors(
  const lanelet::ConstLanelet & route_lane, const lanelet::ConstLanelet & candidate,
  const RouteHandler & route_handler)
{
  for (const auto & route_next : route_handler.getNextLanelets(route_lane)) {
    if (!route_handler.isRouteLanelet(route_next)) {
      continue;
    }
    const auto downstream_corridor = getCompletedParallelCorridor(route_next, route_handler);
    for (const auto & candidate_next : route_handler.getNextLanelets(candidate)) {
      const bool is_in_downstream_corridor = std::any_of(
        downstream_corridor.begin(), downstream_corridor.end(),
        [&](const auto & lane) { return lane.id() == candidate_next.id(); });
      if (is_in_downstream_corridor) {
        return ParallelSuccessorPair{route_next, candidate_next};
      }
    }
  }
  return std::nullopt;
}

bool rejoinsCompletedRouteCorridor(
  const lanelet::ConstLanelet & route_lane, const lanelet::ConstLanelet & candidate,
  const RouteHandler & route_handler)
{
  return getParallelSuccessors(route_lane, candidate, route_handler).has_value();
}

lanelet::ConstLanelets getTransitionCorridor(
  const lanelet::ConstLanelet & route_lane, const RouteHandler & route_handler)
{
  const auto previous_route_lane = getPreviousRouteLane(route_lane, route_handler);
  if (!previous_route_lane) {
    return {route_lane};
  }

  const auto route_exit_yaw = getLaneletExitYaw(route_lane);
  const auto route_centerline = route_lane.centerline3d();
  if (!route_exit_yaw || route_centerline.empty()) {
    return {route_lane};
  }
  const auto & route_end = route_centerline.back();
  lanelet::ConstLanelets candidates{route_lane};
  // Only branches reachable from the actual route predecessor belong to this
  // transition. Walking every lane in the predecessor's lateral corridor can
  // import a branch from another incoming road whose successor has a different
  // maneuver, producing a disconnected outer bound at the next route lane.
  for (const auto & next : route_handler.getNextLanelets(*previous_route_lane)) {
    if (
      next.id() == route_lane.id() ||
      next.attributeOr(lanelet::AttributeName::Subtype, std::string{}) != "road" ||
      !isSameManeuver(route_lane, next)) {
      continue;
    }
    const auto next_centerline = next.centerline3d();
    const auto next_exit_yaw = getLaneletExitYaw(next);
    const bool has_parallel_exit =
      next_exit_yaw &&
      yawDifference(*route_exit_yaw, *next_exit_yaw) <= 5.0 * M_PI / 180.0;
    // Split lanelets can overlap at their entrance and fan out with different exit yaws. Keep the
    // sibling only when its successor is part of the same completed parallel route corridor. This
    // preserves a continuous outer bound without importing an unrelated branch.
    const bool rejoins_route_corridor =
      rejoinsCompletedRouteCorridor(route_lane, next, route_handler);
    if (next_centerline.empty() || (!has_parallel_exit && !rejoins_route_corridor)) {
      continue;
    }
    const auto & next_end = next_centerline.back();
    const double dx = next_end.x() - route_end.x();
    const double dy = next_end.y() - route_end.y();
    const double longitudinal_offset =
      std::cos(*route_exit_yaw) * dx + std::sin(*route_exit_yaw) * dy;
    const double lateral_offset =
      -std::sin(*route_exit_yaw) * dx + std::cos(*route_exit_yaw) * dy;
    if (std::abs(longitudinal_offset) > 5.0 || std::abs(lateral_offset) > 20.0) {
      continue;
    }
    if (std::none_of(candidates.begin(), candidates.end(), [&](const auto & candidate) {
          return candidate.id() == next.id();
        })) {
      candidates.push_back(next);
    }
  }

  const auto lateral_offset = [&](const auto & lane) {
    const auto lane_centerline = lane.centerline3d();
    const auto & lane_end = lane_centerline.back();
    const double dx = lane_end.x() - route_end.x();
    const double dy = lane_end.y() - route_end.y();
    return -std::sin(*route_exit_yaw) * dx + std::cos(*route_exit_yaw) * dy;
  };
  std::sort(candidates.begin(), candidates.end(), [&](const auto & a, const auto & b) {
    return lateral_offset(a) > lateral_offset(b);
  });

  const auto route_it = std::find_if(
    candidates.begin(), candidates.end(),
    [&](const auto & candidate) { return candidate.id() == route_lane.id(); });
  size_t first = std::distance(candidates.begin(), route_it);
  size_t last = first;
  constexpr double max_lane_spacing = 5.0;
  while (first > 0 && std::abs(
                        lateral_offset(candidates.at(first - 1)) -
                        lateral_offset(candidates.at(first))) <= max_lane_spacing) {
    --first;
  }
  while (last + 1 < candidates.size() &&
         std::abs(lateral_offset(candidates.at(last)) - lateral_offset(candidates.at(last + 1))) <=
           max_lane_spacing) {
    ++last;
  }
  return {candidates.begin() + first, candidates.begin() + last + 1};
}

std::vector<DrivableLanes> generateSplitDrivableLanes(
  const lanelet::ConstLanelets & route_lanes, const RouteHandler & route_handler)
{
  auto drivable_lanes = generateDrivableLanes(route_lanes);
  for (size_t i = 0; i < route_lanes.size(); ++i) {
    // Drivable road space is not limited to fully developed parallel lanes. A legal adjacent
    // lane may start/end at zero width or curve differently at its taper. Keep its real map
    // boundary; the finite path bound and maneuver footprint checks handle the local width.
    // The stricter completed-parallel test is still used by split path-shifting heuristics.
    auto lateral_lanes = expandLaneletCorridor({route_lanes.at(i)}, route_handler);
    const auto transition_lanes = getTransitionCorridor(route_lanes.at(i), route_handler);
    if (transition_lanes.size() > lateral_lanes.size()) {
      // A split corridor can also have a taper reachable only by a lateral edge. Replacing it
      // with direct successors alone would lose such a lane again.
      lateral_lanes = expandLaneletCorridor(transition_lanes, route_handler);
    }
    auto & lanes = drivable_lanes.at(i);
    lanes.left_lane = lateral_lanes.front();
    lanes.right_lane = lateral_lanes.back();
    lanes.middle_lanes.clear();
    if (lateral_lanes.size() > 2) {
      lanes.middle_lanes.assign(std::next(lateral_lanes.begin()), std::prev(lateral_lanes.end()));
    }
  }
  return drivable_lanes;
}

struct SplitTransition
{
  lanelet::ConstLanelets route_lanes;
  lanelet::ConstLanelets sibling_lanes;
};

struct SmoothLaneTransition
{
  ShiftLine shift_line;
  lanelet::ConstLanelets current_lanes;
  lanelet::ConstLanelets target_lanes;
};

std::string laneIdsToString(const lanelet::ConstLanelets & lanes)
{
  std::ostringstream stream;
  stream << '[';
  for (size_t i = 0; i < lanes.size(); ++i) {
    if (i != 0) {
      stream << ',';
    }
    stream << lanes.at(i).id();
  }
  stream << ']';
  return stream.str();
}

const char * turnSignalToString(const uint8_t command)
{
  if (command == TurnIndicatorsCommand::ENABLE_LEFT) {
    return "LEFT";
  }
  if (command == TurnIndicatorsCommand::ENABLE_RIGHT) {
    return "RIGHT";
  }
  return "NONE";
}

std::optional<SplitTransition> getStraightSplitTransition(
  const lanelet::ConstLanelet & route_lane, const RouteHandler & route_handler)
{
  const auto previous_route_lane = getPreviousRouteLane(route_lane, route_handler);
  if (!previous_route_lane) {
    return std::nullopt;
  }
  const auto incoming_yaw = getLaneletExitYaw(*previous_route_lane);
  const auto route_entry_yaw = getLaneletEntryYaw(route_lane);
  const auto route_exit_yaw = getLaneletExitYaw(route_lane);
  const auto route_chord_yaw = getLaneletChordYaw(route_lane);
  if (!incoming_yaw || !route_entry_yaw || !route_exit_yaw || !route_chord_yaw) {
    return std::nullopt;
  }

  constexpr double minimum_improvement = 5.0 * M_PI / 180.0;
  const double route_straightness =
    yawDifference(*route_entry_yaw, *incoming_yaw) + yawDifference(*route_chord_yaw, *incoming_yaw);
  if (route_straightness < minimum_improvement) {
    return std::nullopt;
  }

  std::optional<lanelet::ConstLanelet> straight_sibling;
  std::optional<ParallelSuccessorPair> parallel_successors;
  double smallest_straightness = route_straightness;
  for (const auto & candidate : route_handler.getNextLanelets(*previous_route_lane)) {
    if (
      candidate.id() == route_lane.id() ||
      candidate.attributeOr(lanelet::AttributeName::Subtype, std::string{}) != "road" ||
      !isSameManeuver(route_lane, candidate)) {
      continue;
    }
    const auto candidate_entry_yaw = getLaneletEntryYaw(candidate);
    const auto candidate_exit_yaw = getLaneletExitYaw(candidate);
    const auto candidate_chord_yaw = getLaneletChordYaw(candidate);
    if (!candidate_entry_yaw || !candidate_exit_yaw || !candidate_chord_yaw) {
      continue;
    }
    const bool has_parallel_exit =
      yawDifference(*route_exit_yaw, *candidate_exit_yaw) <= 5.0 * M_PI / 180.0;
    const auto candidate_successors = getParallelSuccessors(route_lane, candidate, route_handler);
    if (!has_parallel_exit && !candidate_successors) {
      continue;
    }

    const double candidate_straightness = yawDifference(*candidate_entry_yaw, *incoming_yaw) +
                                          yawDifference(*candidate_chord_yaw, *incoming_yaw);
    const auto route_centerline = route_lane.centerline3d();
    const auto candidate_centerline = candidate.centerline3d();
    if (
      route_centerline.empty() || candidate_centerline.empty() ||
      candidate_straightness + minimum_improvement >= smallest_straightness) {
      continue;
    }
    const auto & route_end = route_centerline.back();
    const auto & candidate_end = candidate_centerline.back();
    const double dx = candidate_end.x() - route_end.x();
    const double dy = candidate_end.y() - route_end.y();
    const double longitudinal_offset = std::cos(*incoming_yaw) * dx + std::sin(*incoming_yaw) * dy;
    const double lateral_offset = -std::sin(*incoming_yaw) * dx + std::cos(*incoming_yaw) * dy;
    if (
      std::abs(longitudinal_offset) > 5.0 || std::abs(lateral_offset) < 1.0 ||
      std::abs(lateral_offset) > 6.0) {
      continue;
    }
    straight_sibling = candidate;
    parallel_successors = candidate_successors;
    smallest_straightness = candidate_straightness;
  }
  if (!straight_sibling) {
    return std::nullopt;
  }

  SplitTransition transition{{route_lane}, {*straight_sibling}};
  if (parallel_successors) {
    transition.route_lanes.push_back(parallel_successors->route_lane);
    transition.sibling_lanes.push_back(parallel_successors->sibling_lane);
  }
  return transition;
}

std::vector<SmoothLaneTransition> delaySplitLaneShift(
  PathWithLaneId & path, const RouteHandler & route_handler)
{
  const auto route_lanes = get_lanelet_sequence_from_path(path, route_handler);
  std::vector<SmoothLaneTransition> smooth_transitions;
  bool modified = false;
  for (const auto & route_lane : route_lanes) {
    const auto transition = getStraightSplitTransition(route_lane, route_handler);
    if (!transition) {
      continue;
    }

    const auto route_centerline = combineCenterlines(transition->route_lanes);
    const auto sibling_centerline = combineCenterlines(transition->sibling_lanes);
    if (route_centerline.size() < 2 || sibling_centerline.size() < 2) {
      continue;
    }
    const double route_length = centerlineLength(route_centerline);
    const double sibling_length = centerlineLength(sibling_centerline);
    if (route_length < 1.0e-3 || sibling_length < 1.0e-3) {
      continue;
    }
    const auto & route_end = route_centerline.back();
    const auto & sibling_end = sibling_centerline.back();
    const double final_separation =
      std::hypot(route_end.x() - sibling_end.x(), route_end.y() - sibling_end.y());
    double completed_split_length = 0.0;
    for (double s = 0.0; s <= route_length; s += 1.0) {
      const auto route_point = interpolateCenterline(route_centerline, s);
      const auto sibling_point =
        interpolateCenterline(sibling_centerline, s / route_length * sibling_length);
      if (
        std::hypot(route_point.x - sibling_point.x, route_point.y - sibling_point.y) >=
        0.9 * final_separation) {
        completed_split_length = s;
        break;
      }
    }

    constexpr double post_split_margin = 2.0;
    constexpr double maximum_lane_shift_length = 20.0;
    constexpr double minimum_lane_shift_length = 12.0;
    constexpr double end_margin = 10.0;
    const double shift_start = completed_split_length + post_split_margin;
    const double lane_shift_length =
      std::min(maximum_lane_shift_length, route_length - end_margin - shift_start);
    if (lane_shift_length < minimum_lane_shift_length) {
      continue;
    }
    const double shift_end = shift_start + lane_shift_length;

    std::optional<size_t> shift_start_idx;
    std::optional<size_t> shift_end_idx;
    double closest_start_distance = std::numeric_limits<double>::max();
    double closest_end_distance = std::numeric_limits<double>::max();
    bool transition_modified = false;
    for (size_t i = 0; i < path.points.size(); ++i) {
      auto & path_point = path.points.at(i);
      const bool is_in_transition = std::any_of(
        transition->route_lanes.begin(), transition->route_lanes.end(), [&](const auto & lane) {
          return std::find(path_point.lane_ids.begin(), path_point.lane_ids.end(), lane.id()) !=
                 path_point.lane_ids.end();
        });
      if (!is_in_transition) {
        continue;
      }
      auto & position = path_point.point.pose.position;
      const double route_s = projectToCenterline(route_centerline, position);
      const double start_distance = std::abs(route_s - shift_start);
      if (start_distance < closest_start_distance) {
        closest_start_distance = start_distance;
        shift_start_idx = i;
      }
      const double end_distance = std::abs(route_s - shift_end);
      if (end_distance < closest_end_distance) {
        closest_end_distance = end_distance;
        shift_end_idx = i;
      }
      if (route_s >= shift_end) {
        continue;
      }
      const auto sibling_point =
        interpolateCenterline(sibling_centerline, route_s / route_length * sibling_length);
      const double ratio = std::clamp((route_s - shift_start) / lane_shift_length, 0.0, 1.0);
      const double smooth_ratio = ratio * ratio * ratio * (10.0 + ratio * (-15.0 + 6.0 * ratio));
      position.x = sibling_point.x + smooth_ratio * (position.x - sibling_point.x);
      position.y = sibling_point.y + smooth_ratio * (position.y - sibling_point.y);
      modified = true;
      transition_modified = true;
    }

    if (
      transition_modified && shift_start_idx && shift_end_idx &&
      *shift_start_idx < *shift_end_idx) {
      SmoothLaneTransition smooth_transition;
      smooth_transition.shift_line.start_idx = *shift_start_idx;
      smooth_transition.shift_line.end_idx = *shift_end_idx;
      smooth_transition.current_lanes = transition->sibling_lanes;
      smooth_transition.target_lanes = transition->route_lanes;
      smooth_transitions.push_back(std::move(smooth_transition));
    }
  }
  if (modified) {
    autoware::motion_utils::insertOrientation(path.points, true);
  }

  for (auto & transition : smooth_transitions) {
    auto & shift_line = transition.shift_line;
    shift_line.start = path.points.at(shift_line.start_idx).point.pose;
    shift_line.end = path.points.at(shift_line.end_idx).point.pose;
    shift_line.start_shift_length =
      autoware::experimental::lanelet2_utils::get_arc_coordinates(
        transition.current_lanes, shift_line.start)
        .distance;
    shift_line.end_shift_length =
      autoware::experimental::lanelet2_utils::get_arc_coordinates(
        transition.current_lanes, shift_line.end)
        .distance;
  }
  return smooth_transitions;
}

void addRightTurnInnerMargin(PathWithLaneId & path, const RouteHandler & route_handler)
{
  if (path.points.size() < 2) {
    return;
  }

  const auto route_lanes = get_lanelet_sequence_from_path(path, route_handler);
  const auto arc_lengths = calcPathArcLengthArray(path, 0, path.points.size(), 0.0);
  std::vector<double> lateral_offsets(path.points.size(), 0.0);
  constexpr double right_turn_margin = 0.25;

  for (const auto & route_lane : route_lanes) {
    if (route_lane.attributeOr("turn_direction", std::string{}) != "right") {
      continue;
    }

    std::optional<size_t> first_index;
    std::optional<size_t> last_index;
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto & lane_ids = path.points.at(i).lane_ids;
      if (std::find(lane_ids.begin(), lane_ids.end(), route_lane.id()) == lane_ids.end()) {
        continue;
      }
      if (!first_index) {
        first_index = i;
      }
      last_index = i;
    }
    if (!first_index || !last_index || *first_index == *last_index) {
      continue;
    }

    const double section_length = arc_lengths.at(*last_index) - arc_lengths.at(*first_index);
    if (section_length <= 0.0) {
      continue;
    }
    const double transition_length = std::min(5.0, section_length / 3.0);
    for (size_t i = *first_index; i <= *last_index; ++i) {
      const double from_start = arc_lengths.at(i) - arc_lengths.at(*first_index);
      const double to_end = arc_lengths.at(*last_index) - arc_lengths.at(i);
      const double ratio =
        std::clamp(std::min({from_start, to_end, transition_length}) / transition_length, 0.0, 1.0);
      const double smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio);
      lateral_offsets.at(i) =
        std::max(lateral_offsets.at(i), right_turn_margin * smooth_ratio);
    }
  }

  bool modified = false;
  for (size_t i = 0; i < path.points.size(); ++i) {
    const double offset = lateral_offsets.at(i);
    if (offset <= 0.0) {
      continue;
    }
    auto & pose = path.points.at(i).point.pose;
    const double yaw = tf2::getYaw(pose.orientation);
    pose.position.x -= offset * std::sin(yaw);
    pose.position.y += offset * std::cos(yaw);
    modified = true;
  }
  if (modified) {
    autoware::motion_utils::insertOrientation(path.points, true);
  }
}
}  // namespace

BehaviorModuleOutput getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  PathWithLaneId reference_path{};

  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_odometry->pose.pose;
  const auto p = planner_data->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  // calculate path with backward margin to avoid end points' instability by spline interpolation
  constexpr double extra_margin = 10.0;
  const double backward_length = p.backward_path_length + extra_margin;
  const auto current_lanes_with_backward_margin =
    route_handler->getLaneletSequence(current_lane, backward_length, p.forward_path_length);
  const auto no_shift_pose = autoware::experimental::lanelet2_utils::get_closest_center_pose(
    current_lane, autoware::experimental::lanelet2_utils::from_ros(current_pose));
  reference_path = getCenterLinePath(
    *route_handler, current_lanes_with_backward_margin, no_shift_pose, backward_length,
    p.forward_path_length, p);

  if (reference_path.points.empty()) {
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("path_utils"), clock, 5000, "Empty reference path detected.");
    BehaviorModuleOutput output;
    return output;
  }

  const auto smooth_transitions = delaySplitLaneShift(reference_path, *route_handler);
  addRightTurnInnerMargin(reference_path, *route_handler);

  // clip backward length
  // NOTE: In order to keep backward_path_length at least, resampling interval is added to the
  // backward.
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      reference_path.points, no_shift_pose, p.ego_nearest_dist_threshold,
      p.ego_nearest_yaw_threshold);
  reference_path.points = autoware::motion_utils::cropPoints(
    reference_path.points, no_shift_pose.position, current_seg_idx, p.forward_path_length,
    p.backward_path_length + p.input_path_interval);

  const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);
  auto drivable_lanes = generateSplitDrivableLanes(drivable_lanelets, *route_handler);

  const auto & dp = planner_data->drivable_area_expansion_parameters;

  const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  BehaviorModuleOutput output;
  output.path = reference_path;
  output.reference_path = reference_path;
  output.drivable_area_info.drivable_lanes = drivable_lanes;

  for (const auto & transition : smooth_transitions) {
    auto shift_line = transition.shift_line;
    shift_line.start_idx =
      autoware::motion_utils::findNearestIndex(reference_path.points, shift_line.start.position);
    shift_line.end_idx =
      autoware::motion_utils::findNearestIndex(reference_path.points, shift_line.end.position);

    TurnSignalInfo candidate_signal;
    if (shift_line.start_idx < shift_line.end_idx) {
      shift_line.start = reference_path.points.at(shift_line.start_idx).point.pose;
      shift_line.end = reference_path.points.at(shift_line.end_idx).point.pose;
      ShiftedPath shifted_path{
        reference_path, std::vector<double>(reference_path.points.size(), 0.0)};
      shifted_path.shift_length.at(shift_line.start_idx) = shift_line.start_shift_length;
      shifted_path.shift_length.at(shift_line.end_idx) = shift_line.end_shift_length;
      const double current_shift_length =
        autoware::experimental::lanelet2_utils::get_arc_coordinates(
          transition.current_lanes, current_pose)
          .distance;
      constexpr bool is_driving_forward = true;
      constexpr bool egos_lane_is_shifted = true;
      constexpr bool override_ego_stopped_check = false;
      constexpr bool is_pull_out = false;
      constexpr bool is_lane_change = true;
      const auto [signal, is_ignored] =
        planner_data->turn_signal_decider.getBehaviorTurnSignalInfo(
          shifted_path, shift_line, transition.current_lanes, route_handler, p,
          planner_data->self_odometry, p.vehicle_info, current_shift_length, is_driving_forward,
          egos_lane_is_shifted, override_ego_stopped_check, is_pull_out, is_lane_change);
      (void)is_ignored;
      candidate_signal = signal;
    }

    const double relative_shift_length =
      shift_line.end_shift_length - shift_line.start_shift_length;
    const auto current_lane_ids = laneIdsToString(transition.current_lanes);
    const auto target_lane_ids = laneIdsToString(transition.target_lanes);
    RCLCPP_DEBUG(
      rclcpp::get_logger("path_utils"),
      "lane-transition turn signal: source=delaySplitLaneShift module=reference_path "
      "start_shift_length=%.3f end_shift_length=%.3f relative_shift_length=%.3f selected=%s "
      "current_lane_ids=%s target_lane_ids=%s",
      shift_line.start_shift_length, shift_line.end_shift_length, relative_shift_length,
      turnSignalToString(candidate_signal.turn_signal.command), current_lane_ids.c_str(),
      target_lane_ids.c_str());

    if (
      output.turn_signal_info.turn_signal.command == TurnIndicatorsCommand::NO_COMMAND &&
      (candidate_signal.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT ||
       candidate_signal.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT)) {
      output.turn_signal_info = candidate_signal;
    }
  }

  return output;
}

BehaviorModuleOutput createGoalAroundPath(const std::shared_ptr<const PlannerData> & planner_data)
{
  BehaviorModuleOutput output;

  const auto & route_handler = planner_data->route_handler;
  const auto & modified_goal = planner_data->prev_modified_goal;

  const Pose goal_pose = modified_goal ? modified_goal->pose : route_handler->getGoalPose();

  lanelet::ConstLanelet goal_lane;
  const auto shoulder_goal_lanes = route_handler->getShoulderLaneletsAtPose(goal_pose);
  if (!shoulder_goal_lanes.empty()) goal_lane = shoulder_goal_lanes.front();
  const auto is_failed_getting_lanelet =
    shoulder_goal_lanes.empty() && !route_handler->getGoalLanelet(&goal_lane);
  if (is_failed_getting_lanelet) {
    return output;
  }

  constexpr double backward_length = 1.0;
  const auto arc_coord =
    autoware::experimental::lanelet2_utils::get_arc_coordinates({goal_lane}, goal_pose);
  const double s_start = std::max(arc_coord.length - backward_length, 0.0);
  const double s_end = arc_coord.length;

  auto reference_path = route_handler->getCenterLinePath({goal_lane}, s_start, s_end);

  const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);
  const auto drivable_lanes = generateDrivableLanes(drivable_lanelets);

  const auto & dp = planner_data->drivable_area_expansion_parameters;

  const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  // Insert zero velocity to each point in the path.
  for (auto & point : reference_path.points) {
    point.point.longitudinal_velocity_mps = 0.0;
  }

  output.path = reference_path;
  output.reference_path = reference_path;
  output.drivable_area_info.drivable_lanes = drivable_lanes;

  return output;
}

}  // namespace autoware::behavior_path_planner::utils
