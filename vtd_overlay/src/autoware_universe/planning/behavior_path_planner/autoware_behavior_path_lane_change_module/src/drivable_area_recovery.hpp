// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef AUTOWARE_BEHAVIOR_PATH_LANE_CHANGE_MODULE_DRIVABLE_AREA_RECOVERY_HPP_
#define AUTOWARE_BEHAVIOR_PATH_LANE_CHANGE_MODULE_DRIVABLE_AREA_RECOVERY_HPP_

#include "obstacle_stop_recovery.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <std_msgs/msg/header.hpp>

namespace autoware::behavior_path_planner
{
// Feedback only: no RTC command, completion transition, velocity override, or object exclusion.
class DrivableAreaRecovery
{
public:
  using Factors = autoware_internal_planning_msgs::msg::PlanningFactorArray;
  using Path = autoware_internal_planning_msgs::msg::PathWithLaneId;
  static void declareParameters(rclcpp::Node & node);
  explicit DrivableAreaRecovery(rclcpp::Node & node);
  bool shouldReplan(
    bool eligible, double speed, const geometry_msgs::msg::Pose & ego, const Path & path);
  bool hasActiveStop(const geometry_msgs::msg::Pose & ego, const Path & path);
  void onReplanned();
  void reset();

private:
  bool activeLocked(double now, const geometry_msgs::msg::Pose & ego, const Path & path) const;
  double parameter(const std::string & key) const;
  rclcpp::Node & node_;
  rclcpp::Subscription<Factors>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr reset_publisher_;
  std::mutex mutex_;
  Factors::ConstSharedPtr factors_;
  std::optional<double> received_at_;
  std::optional<double> last_attempt_;
  std::optional<double> accept_after_;
  bool interrupted_{false};
  ObstacleStopRecoveryTimer timer_;
};
}  // namespace autoware::behavior_path_planner
#endif
