// Copyright 2026 Selfcar developers
// Licensed under the Apache License, Version 2.0.
#include "autoware/drivable_corridor_recovery/geometry.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autoware::drivable_corridor
{
namespace bg = boost::geometry;
namespace
{
constexpr double kAreaTolerance = 1e-6;
Corridor remainder(const Polygon & shape, const Corridor & corridor)
{
  Corridor remaining{shape};
  for (const auto & allowed : corridor) {
    Corridor next;
    for (const auto & part : remaining) bg::difference(part, allowed, next);
    remaining = std::move(next);
    if (remaining.empty()) break;
  }
  return remaining;
}
double area(const Corridor & parts)
{
  double total = 0.0;
  for (const auto & part : parts) total += std::abs(bg::area(part));
  return total;
}
bool validCorridor(const Corridor & corridor)
{
  return !corridor.empty() && std::all_of(corridor.begin(), corridor.end(), [](const auto & p) {
    return bg::is_valid(p) && std::isfinite(bg::area(p)) && std::abs(bg::area(p)) > 1e-6;
  });
}
}  // namespace

double yaw(const Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
double angleDifference(const double a, const double b)
{
  return std::remainder(a - b, 2.0 * std::acos(-1.0));
}
bool validPose(const Pose & p)
{
  const auto & q = p.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3;
}
Pose interpolatePose(const Pose & a, const Pose & b, const double t)
{
  Pose p;
  p.position.x = a.position.x + t * (b.position.x - a.position.x);
  p.position.y = a.position.y + t * (b.position.y - a.position.y);
  p.position.z = a.position.z + t * (b.position.z - a.position.z);
  p.orientation = autoware_utils::create_quaternion_from_yaw(yaw(a) + t * angleDifference(yaw(b), yaw(a)));
  return p;
}
Polygon footprint(const Pose & pose, const Vehicle & v)
{
  if (
    !validPose(pose) || !std::isfinite(v.wheel_base_m) || v.wheel_base_m <= 0.0 ||
    !std::isfinite(v.vehicle_width_m) || v.vehicle_width_m <= 0.0 ||
    !std::isfinite(v.front_overhang_m) || v.front_overhang_m < 0.0 ||
    !std::isfinite(v.rear_overhang_m) || v.rear_overhang_m < 0.0) {
    throw std::invalid_argument("invalid corridor footprint input");
  }
  const double front = v.wheel_base_m + v.front_overhang_m + kFootprintMargin;
  const double rear = -v.rear_overhang_m - kFootprintMargin;
  const double left = v.wheel_tread_m / 2.0 + v.left_overhang_m + kFootprintMargin;
  const double right = -v.wheel_tread_m / 2.0 - v.right_overhang_m - kFootprintMargin;
  Polygon p;
  const double c = std::cos(yaw(pose)), s = std::sin(yaw(pose));
  for (const auto & xy : std::vector<std::pair<double, double>>{
         {front, left}, {front, right}, {rear, right}, {rear, left}, {front, left}}) {
    p.outer().emplace_back(
      pose.position.x + xy.first * c - xy.second * s,
      pose.position.y + xy.first * s + xy.second * c);
  }
  bg::correct(p);
  return p;
}
Corridor fromBounds(const std::vector<Point> & left, const std::vector<Point> & right)
{
  if (left.size() < 2 || right.size() < 2) return {};
  Polygon p;
  for (const auto & point : left) p.outer().emplace_back(point.x, point.y);
  for (auto it = right.rbegin(); it != right.rend(); ++it) p.outer().emplace_back(it->x, it->y);
  bg::correct(p);
  if (!validCorridor({p})) return {};
  return {p};
}
double outsideArea(const Polygon & shape, const Corridor & corridor)
{
  if (!validCorridor(corridor) || !bg::is_valid(shape)) return std::numeric_limits<double>::infinity();
  return area(remainder(shape, corridor));
}
CheckResult checkPath(
  const std::vector<Pose> & poses, const Corridor & corridor, const Vehicle & vehicle,
  const std::optional<Polygon> & initially_occupied, const double max_reentry_distance)
{
  CheckResult result;
  if (poses.size() < 2 || !validCorridor(corridor)) return result;
  if (initially_occupied && !bg::is_valid(*initially_occupied)) return result;
  if (!std::all_of(poses.begin(), poses.end(), validPose)) return result;
  try {
    auto previous = footprint(poses.front(), vehicle);
    double previous_area = outsideArea(previous, corridor);
    bool reentered = previous_area <= kAreaTolerance;
    Corridor allowed = corridor;
    if (initially_occupied) allowed.push_back(*initially_occupied);
    result.valid = true;
    const auto reject = [&](const double arc, const double outside) {
      auto rejected = result;
      rejected.first_violation_arc = arc;
      rejected.outside_area = outside;
      return rejected;
    };
    if (previous_area > kAreaTolerance &&
        (!initially_occupied || outsideArea(previous, allowed) > kAreaTolerance)) {
      return reject(0.0, previous_area);
    }
    double arc = 0.0;
    for (size_t i = 1; i < poses.size(); ++i) {
      const auto & a = poses[i - 1];
      const auto & b = poses[i];
      const double distance = std::hypot(b.position.x - a.position.x, b.position.y - a.position.y);
      const auto steps = static_cast<size_t>(std::max(
        {1.0, std::ceil(distance / kSampleDistance),
         std::ceil(std::abs(angleDifference(yaw(b), yaw(a))) / kSampleYaw)}));
      if (steps > 10000) return CheckResult{};
      for (size_t j = 1; j <= steps; ++j) {
        const auto current = footprint(interpolatePose(a, b, static_cast<double>(j) / steps), vehicle);
        Polygon cloud = previous, sweep;
        cloud.outer().insert(cloud.outer().end(), current.outer().begin(), current.outer().end());
        bg::convex_hull(cloud, sweep);
        bg::correct(sweep);
        const double outside = outsideArea(current, corridor);
        const double sample_arc = arc + distance * static_cast<double>(j) / steps;
        if (outsideArea(sweep, corridor) > kAreaTolerance) {
          if (
            !initially_occupied || reentered || sample_arc > max_reentry_distance ||
            outside > previous_area + kAreaTolerance || outsideArea(sweep, allowed) > kAreaTolerance) {
            return reject(arc + distance * static_cast<double>(j - 1) / steps, outside);
          }
        }
        reentered = reentered || outside <= kAreaTolerance;
        previous = current;
        previous_area = outside;
      }
      arc += distance;
    }
    if (!reentered) return reject(0.0, previous_area);
    result.safe = true;
  } catch (const std::exception &) {
    return CheckResult{};
  }
  return result;
}
}  // namespace autoware::drivable_corridor
