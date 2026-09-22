// Offline native Town05 map validation using the installed Autoware Lanelet2 parser.
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  lanelet::ErrorMessages errors;
  lanelet::projection::UtmProjector projector(lanelet::Origin({0.0, 0.0}));
  lanelet::LaneletMapPtr map = lanelet::load(argv[1], projector, &errors);
  for (const auto & error : errors) std::cerr << error << '\n';
  if (!errors.empty()) return 1;
  for (auto point : map->pointLayer) {
    point.x() = point.attribute("local_x").asDouble().value();
    point.y() = point.attribute("local_y").asDouble().value();
  }
  lanelet::utils::overwriteLaneletsCenterline(map, 5.0, true);
  std::size_t road_count = 0, crosswalk_lanelets = 0, invalid_centerlines = 0;
  std::size_t traffic_lights = 0, crosswalks = 0, stop_signs = 0;
  std::set<lanelet::Id> attached;
  for (const auto & lane : map->laneletLayer) {
    const auto subtype = lane.attributeOr("subtype", std::string{});
    road_count += subtype == "road";
    crosswalk_lanelets += subtype == "crosswalk";
    for (const auto & point : lane.centerline()) {
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
        ++invalid_centerlines;
        break;
      }
    }
    for (const auto & reg : lane.regulatoryElements()) attached.insert(reg->id());
  }
  for (const auto & element : map->regulatoryElementLayer) {
    if (!attached.count(element->id())) throw std::runtime_error("Detached regulatory element");
    if (const auto light = std::dynamic_pointer_cast<lanelet::autoware::AutowareTrafficLight>(element)) {
      ++traffic_lights;
      if (!light->stopLine() || light->trafficLights().empty()) {
        throw std::runtime_error("Traffic light lacks physical light geometry or stop line");
      }
    } else if (const auto crosswalk = std::dynamic_pointer_cast<lanelet::autoware::Crosswalk>(element)) {
      ++crosswalks;
      if (crosswalk->crosswalkAreas().size() != 1 || crosswalk->crosswalkLanelet().id() == 0) {
        throw std::runtime_error("Crosswalk lacks its typed polygon or lanelet");
      }
    } else if (const auto sign = std::dynamic_pointer_cast<lanelet::TrafficSign>(element)) {
      ++stop_signs;
      if (sign->type() != "stop_sign" || sign->refLines().size() != 1) {
        throw std::runtime_error("Stop sign lacks its typed sign or stopping line");
      }
    } else {
      throw std::runtime_error("Unexpected or untyped regulatory element");
    }
  }
  std::cout << "{\"road_lanelets\":" << road_count
            << ",\"crosswalk_lanelets\":" << crosswalk_lanelets
            << ",\"traffic_light_groups\":" << traffic_lights
            << ",\"crosswalk_regulations\":" << crosswalks
            << ",\"stop_sign_lane_regulations\":" << stop_signs
            << ",\"nonfinite_centerlines\":" << invalid_centerlines << "}\n";
  return road_count == 486 && crosswalk_lanelets == 66 && traffic_lights == 54 &&
         crosswalks == 66 && stop_signs == 8 && invalid_centerlines == 0 ? 0 : 1;
}
