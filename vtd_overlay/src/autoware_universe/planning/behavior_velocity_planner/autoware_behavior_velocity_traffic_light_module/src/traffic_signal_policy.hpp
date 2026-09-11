// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAFFIC_SIGNAL_POLICY_HPP_
#define TRAFFIC_SIGNAL_POLICY_HPP_

#include <autoware_perception_msgs/msg/traffic_light_group.hpp>

#include <algorithm>
#include <cmath>
#include <optional>

namespace autoware::behavior_velocity_planner::traffic_signal_policy
{
enum class Signal { NORMAL, UNKNOWN, FLASHING_AMBER };

inline Signal classify(const autoware_perception_msgs::msg::TrafficLightGroup & signal)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;
  if (signal.elements.empty()) return Signal::NORMAL;  // Missing data is not an UNKNOWN report.
  if (std::all_of(signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == Element::UNKNOWN;
      })) {
    return Signal::UNKNOWN;
  }
  // Do not override a simultaneous red, solid amber or directional signal.
  if (std::all_of(signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == Element::UNKNOWN ||
               (element.color == Element::AMBER && element.shape == Element::CIRCLE &&
                element.status == Element::FLASHING);
      })) {
    return Signal::FLASHING_AMBER;
  }
  return Signal::NORMAL;
}

struct FlashingStopParameters
{
  double duration{1.0};   // seconds of continuous measured standstill
  double velocity{0.01};  // m/s
  double distance{1.0};   // absolute distance to the planned base_link stop pose, m
};

// One completed stop per approach to a physical signal. A red phase still follows
// the normal stop logic; UNKNOWN/movement after completion must not re-arm this latch.
class FlashingStop
{
public:
  bool update(
    const bool flashing, const double now, const double distance, const double speed,
    const FlashingStopParameters & parameters, const double max_update_gap)
  {
    if (last_update_ && now < *last_update_) reset();  // simulation clock restarted
    if (last_update_ && now - *last_update_ > max_update_gap) stopped_since_.reset();
    last_update_ = now;
    if (
      !flashing || !std::isfinite(distance) || !std::isfinite(speed) ||
      std::abs(distance) > parameters.distance || std::abs(speed) > parameters.velocity) {
      stopped_since_.reset();
    } else if (!completed_) {
      if (!stopped_since_) stopped_since_ = now;
      if (now - *stopped_since_ >= parameters.duration) completed_ = true;
    }
    return flashing && !completed_;
  }

  void reset()
  {
    stopped_since_.reset();
    last_update_.reset();
    completed_ = false;
  }

private:
  std::optional<double> stopped_since_;
  std::optional<double> last_update_;
  bool completed_{false};
};
}  // namespace autoware::behavior_velocity_planner::traffic_signal_policy

#endif  // TRAFFIC_SIGNAL_POLICY_HPP_
