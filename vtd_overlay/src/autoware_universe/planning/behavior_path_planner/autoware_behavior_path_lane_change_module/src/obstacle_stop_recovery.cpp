// Copyright 2026 selfcar contributors
// SPDX-License-Identifier: Apache-2.0

#include "obstacle_stop_recovery.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

namespace autoware::behavior_path_planner
{
namespace
{
const std::string prefix = "lane_change.obstacle_stop_recovery.";
using Factor = autoware_internal_planning_msgs::msg::PlanningFactor;

bool hasStop(const ObstacleStopRecovery::Factors & message, double maximum_distance)
{
  return std::any_of(message.factors.begin(), message.factors.end(), [&](const auto & factor) {
    return factor.module == "obstacle_stop" && factor.behavior == Factor::STOP &&
           std::any_of(
             factor.control_points.begin(), factor.control_points.end(), [&](const auto & p) {
               return std::isfinite(p.distance) && p.distance <= maximum_distance &&
                      std::isfinite(p.velocity) && std::abs(p.velocity) < 1e-3;
             });
  });
}
}  // namespace

void ObstacleStopRecovery::declareParameters(rclcpp::Node & node)
{
  const auto declare = [&](const std::string & key, auto value) {
    if (!node.has_parameter(prefix + key)) node.declare_parameter(prefix + key, value);
  };
  declare("enabled", false);
  // Only one direction may issue automatic force commands at a time.
  declare("module", std::string("lane_change_left"));
  declare("duration", 5.0);
  declare("stopped_velocity", 0.1);
  declare("message_timeout", 0.5);
  declare("stop_distance", 2.0);
  declare("signal_queue_distance", 100.0);
}

ObstacleStopRecovery::ObstacleStopRecovery(rclcpp::Node & node, const std::string & module_name)
: node_(node), module_name_(module_name)
{
  subscription_ = node_.create_subscription<Factors>(
    "/planning/planning_factors/obstacle_stop", rclcpp::QoS(1).reliable(),
    [this](const Factors::ConstSharedPtr message) { onFactors(message); });
  client_ = node_.create_client<Commands>("/planning/cooperate_commands/" + module_name_);
}

double ObstacleStopRecovery::parameter(const std::string & key) const
{
  return node_.get_parameter(prefix + key).as_double();
}

bool ObstacleStopRecovery::enabled() const
{
  return node_.get_parameter(prefix + "enabled").as_bool() &&
         node_.get_parameter(prefix + "module").as_string() == module_name_;
}

double ObstacleStopRecovery::signalQueueDistance() const
{
  return parameter("signal_queue_distance");
}

void ObstacleStopRecovery::onFactors(const Factors::ConstSharedPtr & message)
{
  const auto now = node_.now().seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    !hasStop(*message, parameter("stop_distance")) ||
    (received_at_ && (now < *received_at_ || now - *received_at_ > parameter("message_timeout")))) {
    stream_interrupted_ = true;
  }
  received_at_ = now;
  factors_ = message;
}

bool ObstacleStopRecovery::hasActiveStop()
{
  const double now = node_.now().seconds();
  const double timeout = parameter("message_timeout");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!factors_ || !received_at_ || !std::isfinite(timeout) || timeout <= 0.0) return false;
  const double age = now - rclcpp::Time(factors_->header.stamp).seconds();
  return now >= *received_at_ && now - *received_at_ <= timeout && age >= 0.0 && age <= timeout &&
         hasStop(*factors_, parameter("stop_distance"));
}

void ObstacleStopRecovery::resetApproval()
{
  timer_.reset();
  requested_uuid_.reset();
  if (pending_) client_->remove_pending_request(*pending_);
  pending_.reset();
}

void ObstacleStopRecovery::reset()
{
  resetApproval();
  std::lock_guard<std::mutex> lock(mutex_);
  factors_.reset();
  received_at_.reset();
  stream_interrupted_ = false;
}

void ObstacleStopRecovery::update(
  bool waiting, bool valid, bool blocked_by_signal, double ego_speed, const UUID & uuid,
  bool already_activated, bool force_deactivated)
{
  const auto now = node_.now().seconds();
  if (pending_) {
    if (pending_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto response = pending_->get();
      const bool accepted =
        !response->responses.empty() && std::all_of(
                                          response->responses.begin(), response->responses.end(),
                                          [](const auto & r) { return r.success; });
      RCLCPP_WARN(
        node_.get_logger(), "[obstacle_stop_recovery] %s RTC force approval %s",
        module_name_.c_str(), accepted ? "accepted" : "rejected");
      if (!accepted) requested_uuid_.reset();
      pending_.reset();
    } else if (now < request_time_ || now - request_time_ > 1.0) {
      client_->remove_pending_request(*pending_);
      pending_.reset();
      requested_uuid_.reset();
    }
  }

  const auto timeout = parameter("message_timeout");
  const auto speed_limit = parameter("stopped_velocity");
  const auto stop_distance = parameter("stop_distance");
  const auto queue_distance = signalQueueDistance();
  bool active = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_interrupted_) timer_.reset();
    stream_interrupted_ = false;
    if (factors_ && received_at_) {
      const auto age = now - rclcpp::Time(factors_->header.stamp).seconds();
      active = now >= *received_at_ && now - *received_at_ <= timeout && age >= 0.0 &&
               age <= timeout && hasStop(*factors_, stop_distance);
    }
  }
  const bool eligible =
    enabled() && waiting && !already_activated && !force_deactivated && !blocked_by_signal &&
    std::isfinite(ego_speed) && std::isfinite(speed_limit) && speed_limit >= 0.0 &&
    std::abs(ego_speed) <= speed_limit && std::isfinite(stop_distance) && stop_distance >= 0.0 &&
    std::isfinite(queue_distance) && queue_distance > 0.0 && active;
  const bool elapsed = timer_.update(now, eligible, parameter("duration"), timeout);
  if (
    !elapsed || !valid || pending_ || (requested_uuid_ && *requested_uuid_ == uuid) ||
    !client_->service_is_ready() || (now >= request_time_ && now - request_time_ < 1.0)) {
    return;
  }
  auto request = std::make_shared<Commands::Request>();
  request->stamp = node_.now();
  tier4_rtc_msgs::msg::CooperateCommand command;
  command.uuid = uuid;
  command.module.type = module_name_ == "lane_change_left"
                          ? tier4_rtc_msgs::msg::Module::LANE_CHANGE_LEFT
                          : tier4_rtc_msgs::msg::Module::LANE_CHANGE_RIGHT;
  command.command.type = tier4_rtc_msgs::msg::Command::ACTIVATE;
  request->commands.push_back(command);
  pending_ = client_->async_send_request(request);
  request_time_ = now;
  requested_uuid_ = uuid;
  RCLCPP_WARN(
    node_.get_logger(),
    "[obstacle_stop_recovery] %s: obstacle_stop >= %.1fs, valid=true; "
    "requesting RTC ACTIVATE regardless of safe (downstream stop checks remain active)",
    module_name_.c_str(), parameter("duration"));
}
}  // namespace autoware::behavior_path_planner
