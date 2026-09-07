#include "vtd_ros2_bridge/hlvtd_traffic_light_mapping.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

namespace vtd_ros2_bridge
{
namespace
{
std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

std::int64_t positive_id(const std::string & text)
{
  const auto value = trim(text);
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("Traffic-light IDs must be positive decimal integers");
  }
  const auto id = std::stoll(value);
  if (id <= 0) throw std::invalid_argument("Traffic-light ID must be positive");
  return id;
}
}  // namespace

HlvtdTrafficLightIdMap load_hlvtd_traffic_light_id_map(std::istream & input)
{
  HlvtdTrafficLightIdMap result;
  std::unordered_map<std::int64_t, std::int32_t> group_owners;
  bool header_seen = false;
  std::string line;
  while (std::getline(input, line)) {
    line = trim(line);
    if (line.empty() || line.front() == '#') continue;
    if (!header_seen) {
      if (line != "hlvtd_id,autoware_group_id") {
        throw std::invalid_argument(
          "Expected hlvtd_id,autoware_group_id: a raw RDB lamp-ID map cannot be used for 9910");
      }
      header_seen = true;
      continue;
    }
    const auto comma = line.find(',');
    if (comma == std::string::npos || line.find(',', comma + 1) != std::string::npos) {
      throw std::invalid_argument("Expected exactly two columns in the 9910 traffic-light map");
    }
    const auto api_id = positive_id(line.substr(0, comma));
    const auto group_id = positive_id(line.substr(comma + 1));
    if (api_id > std::numeric_limits<std::int32_t>::max()) {
      throw std::invalid_argument("9910 traffic-light ID exceeds int32");
    }
    const auto key = static_cast<std::int32_t>(api_id);
    const auto owner = group_owners.find(group_id);
    if (owner != group_owners.end() && owner->second != key) {
      throw std::invalid_argument("A traffic-light group is assigned to different 9910 approaches");
    }
    group_owners[group_id] = key;
    result[key].push_back(group_id);
  }
  if (input.bad() || !header_seen || result.empty()) {
    throw std::invalid_argument("Missing, unreadable or empty 9910 traffic-light map");
  }
  for (auto & [id, groups] : result) {
    (void)id;
    std::sort(groups.begin(), groups.end());
    groups.erase(std::unique(groups.begin(), groups.end()), groups.end());
  }
  return result;
}

std::vector<autoware_perception_msgs::msg::TrafficLightElement> hlvtd_traffic_light_elements(
  const std::uint8_t state)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;
  const auto element = [](
                         const std::uint8_t color, const std::uint8_t shape,
                         const std::uint8_t status = Element::SOLID_ON) {
    Element value;
    value.color = color;
    value.shape = shape;
    value.status = status;
    value.confidence = color == Element::UNKNOWN ? 0.0F : 1.0F;
    return value;
  };
  switch (state) {
    case 1U:
      return {element(Element::RED, Element::CIRCLE)};
    case 2U:
      return {element(Element::AMBER, Element::CIRCLE)};
    case 3U:
      return {element(Element::GREEN, Element::CIRCLE)};
    case 4U:
      return {element(Element::GREEN, Element::LEFT_ARROW)};
    case 5U:
      return {
        element(Element::GREEN, Element::CIRCLE), element(Element::GREEN, Element::LEFT_ARROW)};
    case 6U:
      // Preserve the existing flashing-amber representation; the API does not
      // provide separate red/amber flashing states.
      return {element(Element::AMBER, Element::CIRCLE, Element::FLASHING)};
    default:
      return {element(Element::UNKNOWN, Element::UNKNOWN, Element::UNKNOWN)};
  }
}
}  // namespace vtd_ros2_bridge
