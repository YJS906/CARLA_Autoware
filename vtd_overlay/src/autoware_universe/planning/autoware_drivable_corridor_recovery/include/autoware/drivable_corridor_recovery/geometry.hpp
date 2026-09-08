// Copyright 2026 Selfcar developers
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE__DRIVABLE_CORRIDOR_RECOVERY__GEOMETRY_HPP_
#define AUTOWARE__DRIVABLE_CORRIDOR_RECOVERY__GEOMETRY_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <limits>
#include <optional>
#include <vector>

namespace autoware::drivable_corridor
{
using Pose = geometry_msgs::msg::Pose;
using Point = geometry_msgs::msg::Point;
using Polygon = autoware_utils::Polygon2d;
using Corridor = std::vector<Polygon>;
using Vehicle = autoware::vehicle_info_utils::VehicleInfo;

// One contract for early lane-change requests AND the final recovery guard. The positive
// margin also bounds the sub-millimetre rotational chord error between dense sweep samples.
inline constexpr double kFootprintMargin = 0.03;
inline constexpr double kSampleDistance = 0.2;
inline constexpr double kSampleYaw = 0.025;

struct CheckResult
{
  bool valid{false};
  bool safe{false};
  double first_violation_arc{std::numeric_limits<double>::infinity()};
  double outside_area{0.0};
};

double yaw(const Pose & pose);
double angleDifference(double a, double b);
bool validPose(const Pose & pose);
Pose interpolatePose(const Pose & a, const Pose & b, double ratio);
Polygon footprint(const Pose & pose, const Vehicle & vehicle);
Corridor fromBounds(const std::vector<Point> & left, const std::vector<Point> & right);
double outsideArea(const Polygon & footprint, const Corridor & corridor);

// All vehicle corners and the hull swept between dense poses must be covered. No road-bound
// relaxation is applied. Recovery may reuse ONLY the initial, already occupied footprint,
// without entering any new area outside the corridor. Its outside area must not grow, and it
// must fully re-enter within max_reentry_distance and before the end of this finite path.
CheckResult checkPath(
  const std::vector<Pose> & poses, const Corridor & corridor, const Vehicle & vehicle,
  const std::optional<Polygon> & initially_occupied = std::nullopt,
  double max_reentry_distance = 0.0);
}  // namespace autoware::drivable_corridor
#endif
