// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0
#include "drivable_area_recovery.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

namespace autoware::behavior_path_planner
{
namespace
{
const std::string prefix = "lane_change.drivable_area_recovery.";
using Factor = autoware_internal_planning_msgs::msg::PlanningFactor;
bool hasBoundaryStop(const DrivableAreaRecovery::Factors & message)
{
  return std::any_of(message.factors.begin(), message.factors.end(), [](const auto & f) {
    return f.module == "path_optimizer" && f.behavior == Factor::STOP &&
           f.detail == "outside_drivable_area" && f.is_driving_forward;
  });
}
}  // namespace

void DrivableAreaRecovery::declareParameters(rclcpp::Node & node)
{
  const auto declare = [&](const std::string & key, auto value) {
    if (!node.has_parameter(prefix + key)) node.declare_parameter(prefix + key, value);
  };
  declare("enabled", false);
  declare("duration", 10.0);
  declare("retry_interval", 1.0);
  declare("message_timeout", 0.5);
  declare("stop_distance", 2.0);
}

DrivableAreaRecovery::DrivableAreaRecovery(rclcpp::Node & node) : node_(node)
{
  reset_publisher_ = node_.create_publisher<std_msgs::msg::Header>(
    "/planning/path_optimizer/reset_previous", rclcpp::QoS(1).reliable());
  subscription_ = node_.create_subscription<Factors>(
    "/planning/planning_factors/path_optimizer", rclcpp::QoS(1).reliable(),
    [this](const Factors::ConstSharedPtr message) {
      const auto now = node_.now().seconds();
      std::lock_guard<std::mutex> lock(mutex_);
      if (
        !hasBoundaryStop(*message) ||
        (received_at_ &&
         (now < *received_at_ || now - *received_at_ > parameter("message_timeout")))) {
        interrupted_ = true;
      }
      received_at_ = now;
      factors_ = message;
    });
}

double DrivableAreaRecovery::parameter(const std::string & key) const
{
  return node_.get_parameter(prefix + key).as_double();
}

bool DrivableAreaRecovery::activeLocked(
  double now, const geometry_msgs::msg::Pose & ego, const Path & path) const
{
  if (
    !node_.get_parameter(prefix + "enabled").as_bool() || !factors_ || !received_at_ ||
    path.points.size() < 2 || path.header.frame_id != factors_->header.frame_id)
    return false;
  const auto stamp = rclcpp::Time(factors_->header.stamp).seconds();
  const auto timeout = parameter("message_timeout");
  const auto max_distance = parameter("stop_distance");
  if (
    !std::isfinite(timeout) || timeout <= 0.0 || !std::isfinite(max_distance) ||
    max_distance < 0.0 || now < *received_at_ || now - *received_at_ > timeout || now < stamp ||
    now - stamp > timeout || (accept_after_ && stamp <= *accept_after_))
    return false;
  for (const auto & factor : factors_->factors) {
    if (
      factor.module != "path_optimizer" || factor.behavior != Factor::STOP ||
      factor.detail != "outside_drivable_area" || !factor.is_driving_forward)
      continue;
    for (const auto & point : factor.control_points) {
      if (
        !std::isfinite(point.distance) || point.distance < -1.0 || point.distance > max_distance ||
        !std::isfinite(point.velocity) || std::abs(point.velocity) > 1e-3 ||
        !std::isfinite(point.pose.position.x) || !std::isfinite(point.pose.position.y))
        continue;
      // A fresh stop must also belong to this running maneuver near the current vehicle.
      if (autoware_utils::calc_distance2d(ego, point.pose) > max_distance + 1.0) continue;
      const auto index =
        autoware::motion_utils::findNearestSegmentIndex(path.points, point.pose.position);
      const auto & a = path.points[index].point.pose.position;
      const auto & b = path.points[index + 1].point.pose.position;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared <= 1e-12) continue;
      const double u = std::clamp(
        ((point.pose.position.x - a.x) * dx + (point.pose.position.y - a.y) * dy) / length_squared,
        0.0, 1.0);
      if (
        std::hypot(point.pose.position.x - a.x - u * dx, point.pose.position.y - a.y - u * dy) <=
        1.0)
        return true;
    }
  }
  return false;
}

bool DrivableAreaRecovery::hasActiveStop(const geometry_msgs::msg::Pose & ego, const Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return activeLocked(node_.now().seconds(), ego, path);
}

bool DrivableAreaRecovery::shouldReplan(
  bool eligible, double speed, const geometry_msgs::msg::Pose & ego, const Path & path)
{
  const auto now = node_.now().seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  if (accept_after_ && now < *accept_after_) {
    accept_after_.reset();
    timer_.reset();
    last_attempt_.reset();
  }
  if (interrupted_) timer_.reset();
  interrupted_ = false;
  const bool active =
    eligible && std::isfinite(speed) && std::abs(speed) <= 0.1 && activeLocked(now, ego, path);
  if (!active) last_attempt_.reset();
  if (!timer_.update(now, active, parameter("duration"), parameter("message_timeout")))
    return false;
  const auto interval = parameter("retry_interval");
  if (!std::isfinite(interval) || interval <= 0.0) return false;
  if (last_attempt_ && now >= *last_attempt_ && now - *last_attempt_ < interval) return false;
  last_attempt_ = now;
  return true;
}

void DrivableAreaRecovery::onReplanned()
{
  std_msgs::msg::Header request;
  request.stamp = node_.now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    request.frame_id = factors_ ? factors_->header.frame_id : "map";
    accept_after_ = rclcpp::Time(request.stamp).seconds();
    factors_.reset();
    received_at_.reset();
    last_attempt_.reset();
    timer_.reset();
  }
  // Reset only for a path produced after this replacement, not a queued old input.
  reset_publisher_->publish(request);
}

void DrivableAreaRecovery::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  factors_.reset();
  received_at_.reset();
  last_attempt_.reset();
  accept_after_.reset();
  interrupted_ = false;
  timer_.reset();
}
}  // namespace autoware::behavior_path_planner
