// Copyright 2026 Selfcar developers
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE__DRIVABLE_CORRIDOR_RECOVERY__RECOVERY_HPP_
#define AUTOWARE__DRIVABLE_CORRIDOR_RECOVERY__RECOVERY_HPP_
#include "autoware/drivable_corridor_recovery/geometry.hpp"
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

namespace autoware::drivable_corridor
{
using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;
struct RecoveryCandidate
{
  std::vector<TrajectoryPoint> points;
  size_t reference_end_index{0};
};
// Planar quintic Hermite boundary conditions preserve actual pose and steering curvature.
// This function never changes road bounds or raises an input velocity/stop.
std::optional<RecoveryCandidate> makeCandidate(
  const Pose & ego, double steering, const std::vector<TrajectoryPoint> & reference,
  size_t end_index, double tangent_scale, double velocity_limit, const Vehicle & vehicle);
std::vector<Pose> poses(const std::vector<TrajectoryPoint> & points);
}  // namespace autoware::drivable_corridor
#endif
