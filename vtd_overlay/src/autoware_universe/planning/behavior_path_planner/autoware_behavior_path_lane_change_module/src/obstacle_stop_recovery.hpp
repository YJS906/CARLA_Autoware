// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef AUTOWARE_BEHAVIOR_PATH_LANE_CHANGE_MODULE_OBSTACLE_STOP_RECOVERY_HPP_
#define AUTOWARE_BEHAVIOR_PATH_LANE_CHANGE_MODULE_OBSTACLE_STOP_RECOVERY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>

#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace autoware::behavior_path_planner
{
// Uses ROS time: pausing the simulation must not complete the wait.
class ObstacleStopRecoveryTimer
{
public:
  bool update(double now, bool stopped_by_obstacle, double duration, double maximum_gap)
  {
    if (
      !std::isfinite(now) || !std::isfinite(duration) || duration <= 0.0 ||
      !std::isfinite(maximum_gap) || maximum_gap <= 0.0 || !stopped_by_obstacle) {
      reset();
      return false;
    }
    if (!started_ || now < last_ || now - last_ > maximum_gap) since_ = now;
    last_ = now;
    started_ = true;
    return now - since_ >= duration;
  }

  void reset() { started_ = false; }

private:
  double since_{0.0};
  double last_{0.0};
  bool started_{false};
};

// Sends the existing RTC ACTIVATE command. It deliberately does not rewrite is_safe.
class ObstacleStopRecovery
{
public:
  using Factors = autoware_internal_planning_msgs::msg::PlanningFactorArray;
  using Commands = tier4_rtc_msgs::srv::CooperateCommands;
  using UUID = unique_identifier_msgs::msg::UUID;

  static void declareParameters(rclcpp::Node & node);
  ObstacleStopRecovery(rclcpp::Node & node, const std::string & module_name);
  void update(
    bool waiting, bool valid, bool blocked_by_signal, double ego_speed, const UUID & uuid,
    bool already_activated, bool force_deactivated);
  void reset();
  void resetApproval();
  double signalQueueDistance() const;
  bool enabled() const;
  // Read-only stop feedback, independent of automatic RTC approval.
  bool hasActiveStop();

private:
  void onFactors(const Factors::ConstSharedPtr & message);
  double parameter(const std::string & key) const;

  rclcpp::Node & node_;
  const std::string module_name_;
  rclcpp::Subscription<Factors>::SharedPtr subscription_;
  rclcpp::Client<Commands>::SharedPtr client_;
  std::optional<rclcpp::Client<Commands>::FutureAndRequestId> pending_;
  double request_time_{0.0};
  std::optional<UUID> requested_uuid_;
  std::mutex mutex_;
  Factors::ConstSharedPtr factors_;
  std::optional<double> received_at_;
  bool stream_interrupted_{false};
  ObstacleStopRecoveryTimer timer_;
};
}  // namespace autoware::behavior_path_planner

#endif
