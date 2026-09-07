#pragma once

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <cstdint>
#include <istream>
#include <unordered_map>
#include <vector>

namespace vtd_ros2_bridge
{
using HlvtdTrafficLightIdMap = std::unordered_map<std::int32_t, std::vector<std::int64_t>>;

// Strictly accepts the 9910 approach-ID schema, never a raw RDB lamp-ID map.
// Throws on malformed/ambiguous/empty maps instead of silently losing signals.
HlvtdTrafficLightIdMap load_hlvtd_traffic_light_id_map(std::istream & input);

// API.xlsx sheet1!H20: 3=green, 4=left, 5=green+left. A left-only signal
// must not be converted into a green circle (which would release straight lanes).
std::vector<autoware_perception_msgs::msg::TrafficLightElement> hlvtd_traffic_light_elements(
  std::uint8_t state);
}  // namespace vtd_ros2_bridge
