// Copyright 2026 Selfcar developers
// Licensed under the Apache License, Version 2.0.
#include "autoware/drivable_corridor_recovery/recovery.hpp"

#include <autoware/behavior_path_planner_common/utils/path_safety_checker/trajectory_collision.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::drivable_corridor
{
namespace safety = autoware::behavior_path_planner::utils::path_safety_checker;
using Path = autoware_planning_msgs::msg::Path;
using Trajectory = autoware_planning_msgs::msg::Trajectory;
using Objects = autoware_perception_msgs::msg::PredictedObjects;
using Odometry = nav_msgs::msg::Odometry;
using Steering = autoware_vehicle_msgs::msg::SteeringReport;
using Mode = autoware_adapi_v1_msgs::msg::OperationModeState;
using Route = autoware_planning_msgs::msg::LaneletRoute;
using Markers = visualization_msgs::msg::MarkerArray;

namespace
{
double distance(const Pose & a, const Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}
std::optional<size_t> closest(const std::vector<TrajectoryPoint> & points, const Pose & pose)
{
  double best = 3.0;
  std::optional<size_t> index;
  for (size_t i = 0; i < points.size(); ++i) {
    if (std::abs(angleDifference(yaw(points[i].pose), yaw(pose))) > 1.046) continue;
    const double d = distance(points[i].pose, pose);
    if (d < best) { best = d; index = i; }
  }
  return index;
}
std::vector<TrajectoryPoint> trajectory(const Path & path)
{
  std::vector<TrajectoryPoint> result;
  for (const auto & p : path.points) {
    TrajectoryPoint t;
    t.pose = p.pose;
    t.longitudinal_velocity_mps = p.longitudinal_velocity_mps;
    result.push_back(t);
  }
  return result;
}
std::vector<TrajectoryPoint> forward(
  const std::vector<TrajectoryPoint> & points, const Pose & ego, const double length)
{
  const auto index = closest(points, ego);
  if (!index || *index + 1 >= points.size()) return {};
  std::vector<TrajectoryPoint> result{points[*index]};
  result.front().pose = ego;
  double arc = 0.0;
  for (size_t i = *index + 1; i < points.size(); ++i) {
    arc += distance(result.back().pose, points[i].pose);
    if (arc > length) break;
    result.push_back(points[i]);
  }
  return result;
}
bool finitePoints(const std::vector<TrajectoryPoint> & points)
{
  return points.size() >= 2 && std::all_of(points.begin(), points.end(), [](const auto & p) {
    return validPose(p.pose) && std::isfinite(p.longitudinal_velocity_mps) &&
           p.longitudinal_velocity_mps >= 0.0;
  });
}
void stopFrom(std::vector<TrajectoryPoint> & points, const double stop_arc)
{
  double arc = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i) arc += distance(points[i-1].pose, points[i].pose);
    if (arc + 1e-6 >= stop_arc) points[i].longitudinal_velocity_mps = 0.0;
  }
}
}  // namespace

class RecoveryNode : public rclcpp::Node
{
public:
  explicit RecoveryNode(const rclcpp::NodeOptions & options)
  : Node("drivable_corridor_recovery", options),
    vehicle_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
    collision_(safety::loadTrajectoryCollisionParameters(*this))
  {
    enabled_ = declare_parameter<bool>("recovery.enabled", false);
    max_velocity_ = declare_parameter<double>("recovery.max_velocity", 1.5);
    max_entry_velocity_ = declare_parameter<double>("recovery.max_entry_velocity", 2.0);
    timeout_ = declare_parameter<double>("recovery.input_timeout", 0.5);
    lookahead_ = declare_parameter<double>("recovery.lookahead_distance", 80.0);
    reentry_distance_ = declare_parameter<double>("recovery.max_reentry_distance", 5.0);
    stop_margin_ = declare_parameter<double>("recovery.stop_margin", 2.0);
    search_budget_ms_ = declare_parameter<double>("recovery.search_time_budget_ms", 60.0);
    if (
      !std::isfinite(max_velocity_) || max_velocity_ <= 0.0 || max_velocity_ > 2.0 ||
      !std::isfinite(max_entry_velocity_) || max_entry_velocity_ < max_velocity_ ||
      max_entry_velocity_ > 3.0 || !std::isfinite(timeout_) || timeout_ <= 0.0 || timeout_ > 1.0 ||
      !std::isfinite(lookahead_) || lookahead_ < 35.0 || lookahead_ > 150.0 ||
      !std::isfinite(reentry_distance_) || reentry_distance_ <= 0.0 || reentry_distance_ > 5.0 ||
      !std::isfinite(stop_margin_) || stop_margin_ < 1.0 ||
      !std::isfinite(search_budget_ms_) || search_budget_ms_ <= 0.0 || search_budget_ms_ > 100.0) {
      throw std::invalid_argument("invalid bounded corridor recovery parameters");
    }
    output_ = create_publisher<Trajectory>("~/output/trajectory", 1);
    status_ = create_publisher<std_msgs::msg::String>("~/status", 1);
    wall_ = create_publisher<Markers>("~/virtual_wall", 1);
    raw_sub_ = create_subscription<Trajectory>("~/input/trajectory", 2, [this](Trajectory::ConstSharedPtr p) {
      pending_ = p;
      processPending();
    });
    path_sub_ = create_subscription<Path>("~/input/reference_path", 5, [this](Path::ConstSharedPtr p) {
      references_.push_back(p);
      while (references_.size() > 10) references_.pop_front();
      processPending();
    });
    odom_sub_ = create_subscription<Odometry>("~/input/odometry", rclcpp::SensorDataQoS(),
      [this](Odometry::ConstSharedPtr p) { odom_ = p; });
    objects_sub_ = create_subscription<Objects>("~/input/objects", rclcpp::SensorDataQoS(),
      [this](Objects::ConstSharedPtr p) { objects_ = p; });
    steering_sub_ = create_subscription<Steering>("~/input/steering", rclcpp::SensorDataQoS(),
      [this](Steering::ConstSharedPtr p) { steering_ = p; });
    mode_sub_ = create_subscription<Mode>("~/input/operation_mode", rclcpp::QoS(1).transient_local(),
      [this](Mode::ConstSharedPtr p) { mode_ = p; });
    route_sub_ = create_subscription<Route>("~/input/route", rclcpp::QoS(1).transient_local(),
      [this](Route::ConstSharedPtr p) {
        if (!route_ || route_->uuid != p->uuid) { active_.reset(); occupied_.reset(); }
        route_ = p;
      });
    raw_wall_sub_ = create_subscription<Markers>("~/input/virtual_wall", 1,
      [this](Markers::ConstSharedPtr p) { raw_wall_ = p; });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      if (pending_ && !fresh(pending_->header.stamp)) {
        publishStop(*pending_, "waiting_for_matching_reference");
        pending_.reset();
      } else if (last_output_ && !fresh(last_output_->header.stamp)) {
        publishStop(*last_output_, "stale_trajectory");
      }
    });
  }

private:
  bool fresh(const builtin_interfaces::msg::Time & stamp) const
  {
    const double age = (now() - rclcpp::Time(stamp)).seconds();
    return age >= -0.05 && age <= timeout_;
  }
  void publish(Trajectory output, const std::string & state, const bool clear_wall = false)
  {
    std_msgs::msg::String status;
    status.data = state;
    status_->publish(status);
    if (clear_wall) {
      Markers clear;
      visualization_msgs::msg::Marker marker;
      marker.action = visualization_msgs::msg::Marker::DELETEALL;
      clear.markers.push_back(marker);
      wall_->publish(clear);
    } else if (raw_wall_) {
      wall_->publish(*raw_wall_);
    }
    last_output_ = output;
    output_->publish(output);
  }
  void publishStop(Trajectory output, const std::string & reason)
  {
    for (auto & p : output.points) p.longitudinal_velocity_mps = 0.0;
    publish(std::move(output), reason);
  }
  // Input zeros (traffic lights, stop lines, goal, other behavior stops) are never lifted.
  // Also reject a recovery that jumps forward over a stop between sparse reference points.
  bool applyReferenceVelocity(std::vector<TrajectoryPoint> & points, const Path & reference) const
  {
    const auto ref = trajectory(reference);
    if (!finitePoints(ref) || points.empty()) return false;
    auto nearest = closest(ref, points.front().pose);
    if (!nearest) return false;
    size_t previous = *nearest;
    bool stopped = false;
    for (auto & p : points) {
      double best = std::numeric_limits<double>::infinity();
      size_t selected = previous;
      const size_t end = std::min(ref.size(), previous + 30);
      for (size_t i = previous; i < end; ++i) {
        const double d = distance(p.pose, ref[i].pose);
        if (d < best) { best = d; selected = i; }
      }
      if (best > 2.0) return false;
      double limit = max_velocity_;
      for (size_t i = previous; i <= selected; ++i) {
        limit = std::min(limit, static_cast<double>(ref[i].longitudinal_velocity_mps));
      }
      stopped = stopped || limit <= 1e-4;
      p.longitudinal_velocity_mps = stopped ? 0.0F :
        std::min(p.longitudinal_velocity_mps, static_cast<float>(limit));
      previous = selected;
    }
    return true;
  }
  bool objectsClear(const std::vector<TrajectoryPoint> & points, const double checked_length) const
  {
    if (!objects_ || !fresh(objects_->header.stamp) || objects_->header.frame_id != odom_->header.frame_id) return false;
    // Conservative reachable-distance gate for moving actors, including every predicted branch.
    // Downstream predicted-path/cruise/stop checks remain enabled as well.
    const double horizon = checked_length / std::max(0.3, max_velocity_) + 2.0;
    const auto checked = forward(points, odom_->pose.pose, checked_length);
    if (checked.size() < 2) return false;
    for (const auto & o : objects_->objects) {
      const auto & twist = o.kinematics.initial_twist_with_covariance.twist.linear;
      double speed = std::hypot(twist.x, twist.y);
      if (!std::isfinite(speed)) return false;
      for (const auto & prediction : o.kinematics.predicted_paths) {
        const double dt = rclcpp::Duration(prediction.time_step).seconds();
        if (prediction.path.size() > 1 && (!std::isfinite(dt) || dt <= 0.0)) return false;
        for (size_t i = 1; i < prediction.path.size() && i*dt <= horizon; ++i) {
          if (!validPose(prediction.path[i]) || !validPose(prediction.path[i-1])) return false;
          speed = std::max(speed, distance(prediction.path[i], prediction.path[i-1]) / dt);
        }
      }
      if (speed <= collision_.stationary_velocity) continue;
      double nearest_distance = std::numeric_limits<double>::infinity();
      const auto & object_pose = o.kinematics.initial_pose_with_covariance.pose;
      if (!validPose(object_pose)) return false;
      for (const auto & p : checked) nearest_distance = std::min(nearest_distance, distance(p.pose, object_pose));
      const double radius = 0.5*std::hypot(o.shape.dimensions.x,o.shape.dimensions.y) +
        vehicle_.wheel_base_m + vehicle_.front_overhang_m + 1.0;
      if (!std::isfinite(radius) || nearest_distance <= radius + speed*horizon) return false;
    }
    autoware_internal_planning_msgs::msg::PathWithLaneId path;
    for (const auto & p : points) {
      autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
      point.point.pose = p.pose;
      point.point.longitudinal_velocity_mps = p.longitudinal_velocity_mps;
      path.points.push_back(point);
    }
    if (autoware::motion_utils::calcArcLength(path.points) < checked_length) return false;
    return safety::checkStaticTrajectory(
      path, *objects_, vehicle_, odom_->pose.pose, 0.0, checked_length, collision_).is_safe();
  }
  bool prepareOutput(
    RecoveryCandidate candidate, const Path & reference, const Corridor & corridor, Trajectory & output)
  {
    auto segment = forward(candidate.points, odom_->pose.pose, 40.0);
    if (segment.size() < 2 || !checkPath(poses(segment), corridor, vehicle_, occupied_, reentry_distance_).safe) return false;
    if (distance(segment.front().pose, candidate.points.front().pose) > 0.4 && !active_) return false;
    const auto ref = trajectory(reference);
    if (candidate.reference_end_index + 1 >= ref.size()) return false;
    if (distance(segment.back().pose, ref[candidate.reference_end_index].pose) > 0.3) return false;
    const double maneuver_length = autoware::motion_utils::calcArcLength(segment);
    const double checked_length = maneuver_length + collision_.stop_margin + max_velocity_*max_velocity_ / 2.0;
    // Keep the exact upstream tail and stop positions. The fallback only replaces a bounded prefix.
    segment.insert(segment.end(),ref.begin()+candidate.reference_end_index+1,ref.end());
    if (!applyReferenceVelocity(segment, reference) || segment.front().longitudinal_velocity_mps <= 1e-4) return false;
    if (!objectsClear(segment,checked_length)) return false;
    const auto checked = forward(segment,odom_->pose.pose,std::min(lookahead_,autoware::motion_utils::calcArcLength(segment)-5.0));
    const auto final_check = checkPath(poses(checked),corridor,vehicle_,occupied_,reentry_distance_);
    if (!final_check.valid) return false;
    if (!final_check.safe) {
      if (final_check.first_violation_arc < checked_length) return false;
      stopFrom(segment,std::max(0.0,final_check.first_violation_arc-stop_margin_));
    }
    // Never publish positive velocity into an unchecked continuation past the current horizon.
    stopFrom(segment, std::max(0.0,autoware::motion_utils::calcArcLength(checked)-stop_margin_));
    output.header = reference.header;
    output.points = std::move(segment);
    active_ = std::move(candidate);
    return true;
  }
  void processPending()
  {
    if (!pending_) return;
    const auto found = std::find_if(references_.rbegin(),references_.rend(),[this](const auto & p) {
      return p->header.stamp == pending_->header.stamp && p->header.frame_id == pending_->header.frame_id;
    });
    if (found == references_.rend()) return;
    const auto raw = pending_;
    pending_.reset();
    const auto & reference = **found;
    if (!enabled_) { publish(*raw,"disabled"); return; }
    if (!odom_ || !fresh(odom_->header.stamp) || !fresh(reference.header.stamp) ||
        odom_->header.frame_id != reference.header.frame_id || !validPose(odom_->pose.pose) ||
        !finitePoints(raw->points)) { publishStop(*raw,"invalid_or_stale_input"); return; }
    const auto corridor = fromBounds(reference.left_bound,reference.right_bound);
    if (corridor.empty()) { publishStop(*raw,"invalid_drivable_corridor"); return; }
    const double speed = std::abs(odom_->twist.twist.linear.x);
    if (!std::isfinite(speed)) { publishStop(*raw,"invalid_velocity"); return; }
    const double input_length = autoware::motion_utils::calcArcLength(raw->points);
    const auto nominal = forward(raw->points,odom_->pose.pose,std::min(lookahead_,input_length-5.0));
    const auto check = checkPath(poses(nominal),corridor,vehicle_);
    if (!check.valid) { publishStop(*raw,"invalid_nominal_geometry"); return; }
    const bool autonomous = mode_ && mode_->mode == Mode::AUTONOMOUS && mode_->is_autoware_control_enabled;
    if (!active_ && check.safe) { occupied_.reset(); publish(*raw,"nominal_clear"); return; }
    if (active_ && distance(odom_->pose.pose,active_->points.back().pose) < 1.0 && check.safe) {
      active_.reset(); occupied_.reset(); publish(*raw,"rejoined_nominal",true); return;
    }
    if (!autonomous || !steering_ || !fresh(steering_->stamp) ||
        !objects_ || !fresh(objects_->header.stamp) || speed > max_entry_velocity_) {
      active_.reset();
      auto output = *raw;
      if (!check.safe) {
        // Arrive slowly before the predicted violation; the existing stop is never removed.
        for (auto & p : output.points) p.longitudinal_velocity_mps = std::min(
          p.longitudinal_velocity_mps,static_cast<float>(max_velocity_));
      }
      publish(std::move(output),"corridor_blocked_waiting_for_low_speed_or_fresh_inputs");
      return;
    }
    const auto current_footprint = footprint(odom_->pose.pose,vehicle_);
    if (outsideArea(current_footprint,corridor) > 1e-6 && !occupied_) {
      if (speed > 0.1) { publishStop(*raw,"stop_before_initially_outside_recovery"); return; }
      // Set once per excursion, not once per frame: a moving sequence of enlarged envelopes
      // would allow creeping off the road. The corridor itself is never enlarged.
      occupied_ = current_footprint;
    }
    Trajectory recovered;
    if (active_ && prepareOutput(*active_,reference,corridor,recovered)) {
      publish(std::move(recovered),"low_speed_recovery_active",true); return;
    }
    active_.reset();
    const auto ref = trajectory(reference);
    const auto start = closest(ref,odom_->pose.pose);
    if (!start || !finitePoints(ref) || ref[*start].longitudinal_velocity_mps <= 1e-4) {
      publishStop(*raw,"upstream_stop_or_invalid_reference"); return;
    }
    const auto begin = std::chrono::steady_clock::now();
    double arc = 0.0, next_length = 6.0;
    for (size_t end = *start+1; end+1 < ref.size(); ++end) {
      arc += distance(ref[end-1].pose,ref[end].pose);
      if (arc > 30.0 || ref[end].longitudinal_velocity_mps <= 1e-4) break;
      if (arc < next_length) continue;
      next_length += 2.0;
      for (const double scale : {1.0,0.8,1.2}) {
        if (std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count() > search_budget_ms_) {
          publishStop(*raw,"no_verified_recovery_within_budget"); return;
        }
        auto candidate = makeCandidate(odom_->pose.pose,steering_->steering_tire_angle,ref,end,scale,max_velocity_,vehicle_);
        if (candidate && prepareOutput(*candidate,reference,corridor,recovered)) {
          publish(std::move(recovered),"low_speed_recovery_selected",true); return;
        }
      }
    }
    publishStop(*raw,"no_safe_in_corridor_recovery");
  }

  Vehicle vehicle_;
  safety::TrajectoryCollisionParameters collision_;
  bool enabled_;
  double max_velocity_,max_entry_velocity_,timeout_,lookahead_,reentry_distance_,stop_margin_,search_budget_ms_;
  std::deque<Path::ConstSharedPtr> references_;
  Trajectory::ConstSharedPtr pending_;
  std::optional<Trajectory> last_output_;
  Odometry::ConstSharedPtr odom_;
  Objects::ConstSharedPtr objects_;
  Steering::ConstSharedPtr steering_;
  Mode::ConstSharedPtr mode_;
  Route::ConstSharedPtr route_;
  Markers::ConstSharedPtr raw_wall_;
  std::optional<RecoveryCandidate> active_;
  std::optional<Polygon> occupied_;
  rclcpp::Publisher<Trajectory>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<Markers>::SharedPtr wall_;
  rclcpp::Subscription<Trajectory>::SharedPtr raw_sub_;
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Objects>::SharedPtr objects_sub_;
  rclcpp::Subscription<Steering>::SharedPtr steering_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<Route>::SharedPtr route_sub_;
  rclcpp::Subscription<Markers>::SharedPtr raw_wall_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};
}  // namespace autoware::drivable_corridor
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::drivable_corridor::RecoveryNode)
