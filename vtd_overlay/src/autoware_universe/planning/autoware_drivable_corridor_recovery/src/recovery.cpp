// Copyright 2026 Selfcar developers
// Licensed under the Apache License, Version 2.0.
#include "autoware/drivable_corridor_recovery/recovery.hpp"
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace autoware::drivable_corridor
{
namespace
{
using Coefficients = std::array<double, 6>;
Coefficients quintic(double p0, double d0, double dd0, double p1, double d1, double dd1)
{
  const double a = p1 - p0 - d0 - dd0 / 2.0;
  const double b = d1 - d0 - dd0;
  const double c = dd1 - dd0;
  return {p0, d0, dd0 / 2.0, 10.0*a - 4.0*b + c/2.0,
          -15.0*a + 7.0*b - c, 6.0*a - 3.0*b + c/2.0};
}
double value(const Coefficients & c, double t, size_t derivative)
{
  double sum = 0.0;
  for (size_t i = derivative; i < c.size(); ++i) {
    double factor = c[i];
    for (size_t j = 0; j < derivative; ++j) factor *= static_cast<double>(i - j);
    sum += factor * std::pow(t, static_cast<int>(i - derivative));
  }
  return sum;
}
}  // namespace

std::vector<Pose> poses(const std::vector<TrajectoryPoint> & points)
{
  std::vector<Pose> result;
  result.reserve(points.size());
  for (const auto & p : points) result.push_back(p.pose);
  return result;
}
std::optional<RecoveryCandidate> makeCandidate(
  const Pose & ego, const double steering, const std::vector<TrajectoryPoint> & ref,
  const size_t end_index, const double tangent_scale, const double velocity_limit,
  const Vehicle & vehicle)
{
  if (
    ref.size() < 3 || end_index < 1 || end_index + 1 >= ref.size() || !validPose(ego) ||
    !std::isfinite(steering) || std::abs(steering) > vehicle.max_steer_angle_rad ||
    !std::isfinite(velocity_limit) || velocity_limit <= 0.0) return std::nullopt;
  const auto & end = ref[end_index].pose;
  const double distance = std::hypot(end.position.x - ego.position.x, end.position.y - ego.position.y);
  if (distance < 3.0 || distance > 35.0 || !validPose(end)) return std::nullopt;
  const double length = distance * tangent_scale;
  const double a0 = yaw(ego), a1 = yaw(end);
  const double k0 = std::tan(steering) / vehicle.wheel_base_m;
  const double ds_end = std::hypot(
    ref[end_index+1].pose.position.x - ref[end_index-1].pose.position.x,
    ref[end_index+1].pose.position.y - ref[end_index-1].pose.position.y);
  if (ds_end < 1e-3) return std::nullopt;
  const double k1 = angleDifference(yaw(ref[end_index+1].pose), yaw(ref[end_index-1].pose)) / ds_end;
  const auto x = quintic(
    ego.position.x, length*std::cos(a0), -k0*length*length*std::sin(a0),
    end.position.x, length*std::cos(a1), -k1*length*length*std::sin(a1));
  const auto y = quintic(
    ego.position.y, length*std::sin(a0), k0*length*length*std::cos(a0),
    end.position.y, length*std::sin(a1), k1*length*length*std::cos(a1));
  const size_t count = static_cast<size_t>(std::ceil(2.0*length / kSampleDistance));
  RecoveryCandidate result;
  result.reference_end_index = end_index;
  double previous_steer = steering;
  for (size_t i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / count;
    const double dx = value(x,t,1), dy = value(y,t,1);
    const double norm = std::hypot(dx,dy);
    if (norm < 0.1*length || !std::isfinite(norm)) return std::nullopt;
    const double k = (dx*value(y,t,2) - dy*value(x,t,2)) / (norm*norm*norm);
    const double steer = std::atan(vehicle.wheel_base_m*k);
    if (!std::isfinite(k) || std::abs(steer) > vehicle.max_steer_angle_rad) return std::nullopt;
    TrajectoryPoint p;
    p.pose.position.x = value(x,t,0);
    p.pose.position.y = value(y,t,0);
    p.pose.position.z = ego.position.z + t*(end.position.z - ego.position.z);
    p.pose.orientation = autoware_utils::create_quaternion_from_yaw(std::atan2(dy,dx));
    p.front_wheel_angle_rad = static_cast<float>(steer);
    double limit = std::min(velocity_limit, std::sqrt(0.7 / std::max(std::abs(k), 1e-6)));
    if (!result.points.empty()) {
      const auto & prev = result.points.back().pose.position;
      const double ds = std::hypot(p.pose.position.x-prev.x,p.pose.position.y-prev.y);
      if (ds < 1e-4 || std::abs(angleDifference(yaw(p.pose),yaw(result.points.back().pose))) > 0.1) return std::nullopt;
      limit = std::min(limit, 0.3*ds / std::max(std::abs(steer-previous_steer),1e-6));
    }
    if (limit < 0.3) return std::nullopt;
    p.longitudinal_velocity_mps = static_cast<float>(limit);
    result.points.push_back(p);
    previous_steer = steer;
  }
  result.points.front().pose = ego;
  result.points.back().pose = end;
  return result;
}
}  // namespace autoware::drivable_corridor
