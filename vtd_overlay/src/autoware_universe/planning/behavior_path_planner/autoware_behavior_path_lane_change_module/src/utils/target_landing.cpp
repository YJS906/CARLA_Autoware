// Copyright 2026 Selfcar contributors
// SPDX-License-Identifier: Apache-2.0

#include "autoware/behavior_path_lane_change_module/utils/target_landing.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace autoware::behavior_path_planner::utils::lane_change
{
namespace
{
constexpr double epsilon = 1e-6;
// These are evidence tolerances, not extra vehicle clearance or allowed road departure.
constexpr double width_deficit = 0.1;
constexpr double sample_step = 0.5;

std::optional<double> boundary_crossing(
  const lanelet::ConstLineString3d & boundary, const double x, const double y, const double cos_yaw,
  const double sin_yaw)
{
  std::optional<double> crossing;
  for (size_t i = 1; i < boundary.size(); ++i) {
    const auto & a = boundary[i - 1];
    const auto & b = boundary[i];
    const double ax = (a.x() - x) * cos_yaw + (a.y() - y) * sin_yaw;
    const double bx = (b.x() - x) * cos_yaw + (b.y() - y) * sin_yaw;
    if (!std::isfinite(ax) || !std::isfinite(bx)) return std::nullopt;
    if (std::abs(bx - ax) < epsilon) {
      // A boundary segment along the cross-section has no unique intersection.
      if (std::abs(ax) < epsilon) return std::nullopt;
      continue;
    }
    const double ratio = -ax / (bx - ax);
    if (ratio < -epsilon || ratio > 1.0 + epsilon) continue;
    const double lateral = -(a.x() + ratio * (b.x() - a.x()) - x) * sin_yaw +
                           (a.y() + ratio * (b.y() - a.y()) - y) * cos_yaw;
    if (!std::isfinite(lateral)) return std::nullopt;
    // Hairpins/overlapping boundary branches cannot prove a narrow landing here.
    if (crossing && std::abs(*crossing - lateral) > epsilon) return std::nullopt;
    crossing = lateral;
  }
  return crossing;
}

std::optional<NarrowTargetLanding> section_width(
  const lanelet::ConstLanelets & lanes, const geometry_msgs::msg::Pose & pose, const double cos_yaw,
  const double sin_yaw, const double offset)
{
  const double x = pose.position.x + offset * cos_yaw;
  const double y = pose.position.y + offset * sin_yaw;
  std::optional<NarrowTargetLanding> widest;
  for (const auto & lane : lanes) {
    const auto left = boundary_crossing(lane.leftBound(), x, y, cos_yaw, sin_yaw);
    const auto right = boundary_crossing(lane.rightBound(), x, y, cos_yaw, sin_yaw);
    // Both finite side boundaries must bracket this point. In particular, never use a
    // lanelet's artificial front/rear closing edge as evidence that a road is too narrow.
    if (!left || !right || *left < 0.0 || *right > 0.0) continue;
    const double width = *left - *right;
    // At a shared endpoint/overlap, an adequate target continuation defeats a narrow
    // section from the other lanelet. Only the target sequence is inspected, not neighbors.
    if (!widest || width > widest->width) {
      widest = NarrowTargetLanding{lane.id(), width, offset};
    }
  }
  return widest;
}
}  // namespace

std::optional<NarrowTargetLanding> find_narrow_target_landing(
  const lanelet::ConstLanelets & target_lanes, const geometry_msgs::msg::Pose & landing,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle)
{
  const double rear = vehicle.min_longitudinal_offset_m;
  const double front = vehicle.max_longitudinal_offset_m;
  const double width = vehicle.vehicle_width_m;
  const auto & q = landing.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (
    target_lanes.empty() || !std::isfinite(rear) || !std::isfinite(front) ||
    !std::isfinite(width) || width <= width_deficit || front <= rear || !std::isfinite(norm) ||
    std::abs(norm - 1.0) > 0.01 || !std::isfinite(landing.position.x) ||
    !std::isfinite(landing.position.y)) {
    return std::nullopt;
  }
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), norm - 2.0 * (q.y * q.y + q.z * q.z));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const auto samples = static_cast<size_t>(std::ceil((front - rear) / sample_step));
  std::optional<double> narrow_start;
  for (size_t i = 0; i <= samples; ++i) {
    const double offset = std::min(front, rear + i * sample_step);
    const auto section = section_width(target_lanes, landing, cos_yaw, sin_yaw, offset);
    if (!section || section->width + width_deficit >= width) {
      narrow_start.reset();
      continue;
    }
    if (!narrow_start) narrow_start = offset;
    // Require consecutive measurements spanning 0.5 m, not a lone seam or map vertex.
    if (offset - *narrow_start >= sample_step - epsilon) return section;
  }
  return std::nullopt;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
