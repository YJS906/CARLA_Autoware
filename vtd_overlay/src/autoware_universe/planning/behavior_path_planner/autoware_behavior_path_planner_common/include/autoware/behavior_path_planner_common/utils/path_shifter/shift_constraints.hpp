// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software distributed under the
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__SHIFT_CONSTRAINTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__SHIFT_CONSTRAINTS_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <cmath>
#include <limits>

namespace autoware::behavior_path_planner::utils
{
// Spatial seed for PathShifter's four-knot spline (peak lateral second derivative 8L/S^2).
// Reducing speed must not shrink a full-width shift into an unsteerable curve. This is only
// a seed on a straight reference; the generated path still needs its actual curvature checked.
inline double minimumGeometricShiftLength(
  const double shift, const autoware::vehicle_info_utils::VehicleInfo & vehicle)
{
  const double curvature = vehicle.calcMaxCurvature();
  if (!std::isfinite(shift) || !std::isfinite(curvature) || curvature <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(8.0 * std::abs(shift) / curvature);
}
}  // namespace autoware::behavior_path_planner::utils

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__SHIFT_CONSTRAINTS_HPP_
