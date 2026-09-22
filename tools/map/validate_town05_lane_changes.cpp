// Read-only proof that the deployed Autoware routing graph honors native tags.
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <cmath>
#include <iostream>
#include <set>

int main(int argc, char ** argv) {
  if (argc != 2) return 2;
  lanelet::ErrorMessages errors;
  lanelet::projection::UtmProjector projector(lanelet::Origin({0.0, 0.0}));
  lanelet::LaneletMapPtr map = lanelet::load(argv[1], projector, &errors);
  if (!errors.empty()) { for (const auto & e : errors) std::cerr << e << '\n'; return 1; }
  for (auto point : map->pointLayer) {
    point.x() = point.attribute("local_x").asDouble().value();
    point.y() = point.attribute("local_y").asDouble().value();
  }
  lanelet::utils::overwriteLaneletsCenterline(map, 5.0, true);
  std::size_t invalid = 0;
  for (const auto & lane : map->laneletLayer) for (const auto & p : lane.centerline()) {
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) ++invalid;
  }
  autoware::route_handler::RouteHandler handler(map);
  const auto lane = map->laneletLayer.get(18052);
  const auto neighbors = handler.getLaneChangeableNeighbors(lane);
  std::set<lanelet::Id> ids;
  std::cout << "{\"changeable_neighbors\":[";
  bool first = true;
  for (const auto & n : neighbors) { if (!first) std::cout << ','; std::cout << n.id(); first=false; ids.insert(n.id()); }
  auto graph = handler.getRoutingGraphPtr();
  const auto left = graph->left(lane);
  const auto back = graph->right(map->laneletLayer.get(18355));
  const auto right = graph->right(lane);
  std::size_t tagged_edges = 0, connected_edges = 0;
  for (const auto & candidate : map->laneletLayer) {
    for (const bool is_left : {true, false}) {
      const auto bound = is_left ? candidate.leftBound() : candidate.rightBound();
      if (bound.attributeOr("carla:lane_marking_source", std::string{}) !=
          "carla_town05_lane_change_v1") continue;
      ++tagged_edges;
      const auto next = is_left ? graph->left(candidate) : graph->right(candidate);
      if (next && (is_left ? next->rightBound().id() : next->leftBound().id()) == bound.id()) {
        ++connected_edges;
      }
    }
  }
  std::cout << "],\"left_change\":" << (left ? left->id() : 0)
            << ",\"return_right\":" << (back ? back->id() : 0)
            << ",\"outer_right_change\":" << (right ? right->id() : 0)
            << ",\"tagged_directional_edges\":" << tagged_edges
            << ",\"connected_directional_edges\":" << connected_edges
            << ",\"nonfinite_centerline_points\":" << invalid << "}\n";
  return invalid == 0 && ids.count(18355) && left && left->id() == 18355 &&
         back && back->id() == 18052 && !right && tagged_edges > 0 &&
         connected_edges == tagged_edges ? 0 : 1;
}
