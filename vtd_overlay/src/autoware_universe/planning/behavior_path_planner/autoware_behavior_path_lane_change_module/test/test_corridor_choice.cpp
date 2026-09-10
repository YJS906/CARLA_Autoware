// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
// Standalone, no ROS graph or simulator: compile with -I../include.
#include "autoware/behavior_path_lane_change_module/utils/corridor_choice.hpp"

#include <cassert>
#include <iostream>
using namespace autoware::behavior_path_planner::lane_change::tactical;
int main()
{
  const double curvature = std::tan(0.7) / 2.944;
  const auto make = [](double lateral, std::vector<Interval> free, bool reason) {
    return Corridor{lateral, std::move(free), reason, {{0, 60}}, {{0, 60}}};
  };
  // Bag-derived corridor situation at the 2->1 handoff: two cars abreast ahead,
  // third-lane car nearly alongside. A safe later 2->3 connection may become available.
  std::vector<Corridor> lanes{
    make(3.2, {{0, 15.0}, {30, 60}}, true), make(0, {{0, 15.5}, {30, 60}}, false),
    make(-3.2, {{5.0, 60}}, false)};
  const auto stay = score(lanes, 1, 1, curvature);
  const auto back = score(lanes, 1, 0, curvature);
  assert(stay.progress == 60 && back.progress == 15.0);
  assert(dominatedReturn(stay, back, 5.04));
  // Even with an original blocker in the intermediate lane, safe 3->2 staging must not
  // depend on a pre-proven 2->1 path. Only premature FINAL return is deferred.
  assert(!dominatedReturn(stay, back, 5.04, true));
  // No permission after a solid boundary: do not invent the second connection.
  lanes[1].to_right = {{0, 10}};
  assert(!dominatedReturn(score(lanes, 1, 1, curvature), back, 5.04));
  // No known final 2->1 route: 3->2 still gets a finite score, never a safety veto.
  lanes[1].to_left.clear();
  lanes[2].free = {{0, 60}};
  auto stage = score(lanes, 2, 1, curvature);
  assert(stage.known && stage.progress == 15.5);
  assert(!dominatedReturn(score(lanes, 2, 2, curvature), stage, 5.04));
  // Once the original blocking reason goes away, ordinary mission preference resumes.
  lanes[0].reason_ahead = false;
  assert(!dominatedReturn(stay, score(lanes, 1, 0, curvature), 5.04));
  assert(!dominatedReturn({}, back, 5.04));
  assert(!dominatedReturn(stay, {}, 5.04));
  const auto intervals = freeIntervals({{10, 20}, {15, 25}, {-5, -1}, {40, 50}}, 60);
  assert(intervals.size() == 3 && intervals[1].begin == 25 && intervals[1].end == 40);
  // Never teleport the stay option through an occupied interval at the ego pose.
  lanes[1].free = {{10, 60}};
  assert(score(lanes, 1, 1, curvature).progress == 0);
  assert(!score(lanes, 1, 0, 0).known);
  std::cout << "corridor choice: focused checks passed\n";
}
