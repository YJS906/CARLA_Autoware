// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CORRIDOR_CHOICE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CORRIDOR_CHOICE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace autoware::behavior_path_planner::lane_change::tactical
{
// Rear-axle longitudinal coordinates. These intervals are ranking hints, NOT swept-path
// certificates. Each actual maneuver still requires the existing full safety checks.
struct Interval
{
  double begin;
  double end;
};
struct Corridor
{
  double lateral{};
  std::vector<Interval> free;
  bool reason_ahead{};
  std::vector<Interval> to_left;
  std::vector<Interval> to_right;
};
struct Score
{
  bool known{false};
  double immediate{};
  double progress{};
  bool reason_ahead{false};
};

inline std::vector<Interval> freeIntervals(std::vector<Interval> blocked, double end)
{
  std::sort(blocked.begin(), blocked.end(), [](const auto & a, const auto & b) {
    return a.begin < b.begin;
  });
  std::vector<Interval> result;
  double cursor = 0.0;
  for (const auto & interval : blocked) {
    if (interval.end <= cursor || interval.begin >= end) continue;
    if (interval.begin > cursor) result.push_back({cursor, std::min(end, interval.begin)});
    cursor = std::max(cursor, interval.end);
  }
  if (cursor < end) result.push_back({cursor, end});
  return result;
}

inline double shiftLength(double lateral, double curvature)
{
  // Nominal geometric S length, not the simulator's halved search seed. Speed, curvature,
  // lane bounds and prediction can still make the estimated connection unavailable.
  return std::sqrt(8.0 * std::abs(lateral) / curvature);
}

inline double reachable(
  const std::vector<Corridor> & lanes, std::size_t lane, double start, int hops, double curvature)
{
  const auto & source = lanes.at(lane);
  const auto it = std::find_if(source.free.begin(), source.free.end(), [&](const auto & interval) {
    return interval.begin <= start && start <= interval.end;
  });
  if (it == source.free.end()) return start;
  double result = it->end;
  if (hops == 0) return result;
  for (const int direction : {-1, 1}) {
    const auto next = static_cast<int>(lane) + direction;
    if (next < 0 || next >= static_cast<int>(lanes.size())) continue;
    const auto & target = lanes.at(next);
    const double length = shiftLength(target.lateral - source.lateral, curvature);
    const auto & legal = direction < 0 ? source.to_left : source.to_right;
    for (const auto & interval : target.free) {
      for (const auto & permission : legal) {
        const double finish = std::max({start, interval.begin, permission.begin}) + length;
        if (finish > std::min({it->end, interval.end, permission.end})) continue;
        result = std::max(result, reachable(lanes, next, finish, hops - 1, curvature));
      }
    }
  }
  return result;
}

inline Score score(
  const std::vector<Corridor> & lanes, std::size_t current, std::size_t target, double curvature)
{
  if (
    current >= lanes.size() || target >= lanes.size() || !std::isfinite(curvature) ||
    curvature <= 0.0 || lanes[target].free.empty())
    return {};
  const auto & lane = lanes[target];
  const double start =
    current == target ? 0.0 : shiftLength(lane.lateral - lanes[current].lateral, curvature);
  // An unproven first connection MUST NOT eliminate a candidate. Score its first reachable
  // pocket conservatively; only the real path generator decides whether it can get there.
  const auto & pocket = lane.free.front();
  if (current == target && pocket.begin > 0.0) return {true, 0.0, 0.0, lane.reason_ahead};
  const double landing = std::max(start, pocket.begin);
  const double progress =
    landing <= pocket.end ? reachable(lanes, target, landing, current == target ? 2 : 1, curvature)
                          : pocket.end;
  return {true, pocket.begin <= 0.0 ? pocket.end : 0.0, progress, lane.reason_ahead};
}

inline bool dominatedReturn(
  const Score & stay, const Score & target, double vehicle_length, bool intermediate_stage = false)
{
  // Only defer a return into a still-observed original blockage when keeping the current
  // corridor provides materially more progress without sacrificing immediate stopping room.
  // Unknown future connections never veto a safe staging move.
  return !intermediate_stage && stay.known && target.known && target.reason_ahead &&
         stay.immediate + 1.0 >= target.immediate &&
         stay.progress > target.progress + vehicle_length;
}
}  // namespace autoware::behavior_path_planner::lane_change::tactical
#endif
