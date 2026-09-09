#ifndef VTD_ROS2_BRIDGE__VEHICLE_GEOMETRY_HPP_
#define VTD_ROS2_BRIDGE__VEHICLE_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

namespace vtd_ros2_bridge
{

// Simulator-only size heuristic, not model metadata or a measured axle offset.
// Larger objects use the same CAR label; size alone cannot separate buses/trucks.
struct VehicleGeometryParameters
{
  bool enabled{true};
  double min_length_m{2.514};
  double min_width_m{1.480};
  double min_height_m{1.302};
  double forward_offset_length_ratio{1.0 / 3.0};

  bool valid() const
  {
    return std::isfinite(min_length_m) && min_length_m > 0.0 &&
           std::isfinite(min_width_m) && min_width_m > 0.0 &&
           std::isfinite(min_height_m) && min_height_m > 0.0 &&
           std::isfinite(forward_offset_length_ratio) &&
           forward_offset_length_ratio >= 0.0 && forward_offset_length_ratio <= 1.0;
  }
};

inline bool is_vehicle_size(
  const double length, const double width, const double height,
  const VehicleGeometryParameters & parameters)
{
  // Only absorb float32 wire rounding at the exact catalog minima (one micrometre).
  constexpr double tolerance_m = 1.0e-6;
  return parameters.enabled && std::isfinite(length) && std::isfinite(width) &&
         std::isfinite(height) && length > 0.0 && width > 0.0 && height > 0.0 &&
         length + tolerance_m >= parameters.min_length_m &&
         width + tolerance_m >= parameters.min_width_m &&
         height + tolerance_m >= parameters.min_height_m;
}

// Observed simulator pedestrian sizes, confirmed by the scenario owner. Match complete
// length/width/height triples, not an unrestricted range of all sub-vehicle objects.
struct PedestrianSizeParameters
{
  bool enabled{true};
  double dimension_tolerance_m{0.01};
  std::vector<double> dimensions_lwh_m{
    0.55, 0.63, 1.625, 0.50, 0.60, 1.425, 0.60, 0.70, 1.80, 0.50, 0.60, 1.35};

  bool valid() const
  {
    return std::isfinite(dimension_tolerance_m) && dimension_tolerance_m >= 0.0 &&
           !dimensions_lwh_m.empty() && dimensions_lwh_m.size() % 3U == 0U &&
           std::all_of(dimensions_lwh_m.begin(), dimensions_lwh_m.end(), [](const double value) {
             return std::isfinite(value) && value > 0.0;
           });
  }
};

inline bool is_pedestrian_size(
  const double length, const double width, const double height,
  const PedestrianSizeParameters & parameters)
{
  if (
    !parameters.enabled || !parameters.valid() || !std::isfinite(length) ||
    !std::isfinite(width) || !std::isfinite(height) || length <= 0.0 || width <= 0.0 ||
    height <= 0.0) {
    return false;
  }
  // Include float32 rounding at an exact tolerance boundary.
  const double tolerance = parameters.dimension_tolerance_m + 1.0e-6;
  const auto & dimensions = parameters.dimensions_lwh_m;
  for (std::size_t i = 0U; i < dimensions.size(); i += 3U) {
    if (
      std::abs(length - dimensions[i]) <= tolerance &&
      std::abs(width - dimensions[i + 1U]) <= tolerance &&
      std::abs(height - dimensions[i + 2U]) <= tolerance) {
      return true;
    }
  }
  return false;
}

}  // namespace vtd_ros2_bridge

#endif  // VTD_ROS2_BRIDGE__VEHICLE_GEOMETRY_HPP_
