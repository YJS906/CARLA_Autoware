#include "vtd_ros2_bridge/hlvtd_control_client.hpp"
#include "vtd_ros2_bridge/hlvtd_traffic_light_mapping.hpp"
#include "vtd_ros2_bridge/msg/vtd_ego_state.hpp"
#include "vtd_ros2_bridge/msg/vtd_object_array.hpp"
#include "vtd_ros2_bridge/msg/vtd_traffic_light_array.hpp"
#include "vtd_ros2_bridge/rdb_codec.hpp"
#include "vtd_ros2_bridge/vehicle_geometry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <VtdToolkit/viRDBIcd.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace vtd_ros2_bridge
{

namespace
{

using autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using autoware_control_msgs::msg::Control;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_internal_planning_msgs::msg::VelocityLimit;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_vehicle_msgs::msg::ControlModeReport;
using autoware_vehicle_msgs::msg::GearCommand;
using autoware_vehicle_msgs::msg::GearReport;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::HazardLightsReport;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using autoware_vehicle_msgs::msg::VelocityReport;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using vtd_ros2_bridge::msg::VtdEgoState;
using vtd_ros2_bridge::msg::VtdObject;
using vtd_ros2_bridge::msg::VtdObjectArray;
using vtd_ros2_bridge::msg::VtdTrafficLight;
using vtd_ros2_bridge::msg::VtdTrafficLightArray;

constexpr double kNanosecondsPerSecond = 1.0e9;
// Keep the verified rear-axle reference while restoring the API-only transport.
// A display-only forward offset adds offset * yaw_rate to odometry lateral velocity.
constexpr double kEgoTfForwardOffsetM = 0.0;
constexpr std::size_t kApiMaxObjects = 30U;

builtin_interfaces::msg::Time sim_stamp(const double seconds)
{
  builtin_interfaces::msg::Time stamp;
  const auto nanoseconds =
    static_cast<std::int64_t>(std::llround(std::max(0.0, seconds) * kNanosecondsPerSecond));
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

std::uint8_t autoware_object_class(
  const VtdObject & object, const VehicleGeometryParameters & vehicle_parameters,
  const PedestrianSizeParameters & pedestrian_parameters)
{
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  if (is_pedestrian_size(object.length, object.width, object.height, pedestrian_parameters)) {
    return Classification::PEDESTRIAN;
  }
  return is_vehicle_size(object.length, object.width, object.height, vehicle_parameters)
           ? Classification::CAR
           : Classification::UNKNOWN;
}

// Treat size-classified vehicles as rear-axle referenced and approximate the bbox center
// along the object's own heading. Preserve raw API coordinates and the vertical convention.
// Detection, range filtering and synthetic pointcloud must all use this same conversion.
std::array<float, 3> autoware_box_center(
  const VtdObject & object, const VehicleGeometryParameters & parameters,
  const PedestrianSizeParameters & pedestrian_parameters)
{
  // Use the same classification as detection so a pedestrian can never receive a car offset,
  // even if the user later configures overlapping pedestrian and vehicle size thresholds.
  const bool is_car = autoware_object_class(object, parameters, pedestrian_parameters) ==
                      autoware_perception_msgs::msg::ObjectClassification::CAR;
  const double offset = is_car ? object.length * parameters.forward_offset_length_ratio : 0.0;
  return {
    static_cast<float>(object.x + offset * std::cos(object.heading)),
    static_cast<float>(object.y + offset * std::sin(object.heading)),
    object.z + 0.5F * object.height};
}

std::string rdb_name(const char * data, const std::size_t capacity)
{
  return std::string(data, ::strnlen(data, capacity));
}

template <typename T>
void append_value(DiagnosticStatus & status, const std::string & key, const T & value)
{
  KeyValue item;
  item.key = key;
  item.value = std::to_string(value);
  status.values.push_back(std::move(item));
}

void append_value(DiagnosticStatus & status, const std::string & key, const std::string & value)
{
  KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}

}  // namespace

class VtdBridgeNode : public rclcpp::Node
{
public:
  VtdBridgeNode() : Node("vtd_bridge")
  {
    control_host_ = declare_parameter<std::string>("control.host", "127.0.0.1");
    control_port_ = declare_parameter<int>("control.port", 9910);
    control_reconnect_delay_ms_ = declare_parameter<int>("control.reconnect_delay_ms", 1000);
    control_send_period_ms_ = declare_parameter<int>("control.send_period_ms", 40);
    ego_player_id_ = declare_parameter<int>("ego_player_id", -1);
    ego_name_ = declare_parameter<std::string>("ego_name", "Ego");
    camera_id_ = declare_parameter<int>("camera_id", -1);
    lidar_emitter_id_ = declare_parameter<int>("lidar_emitter_id", -1);
    shm_key_ = declare_parameter<int>("shm.key", 0);
    shm_check_mask_ = declare_parameter<int>("shm.check_mask", RDB_SHM_BUFFER_FLAG_IG);
    optix_return_index_ = declare_parameter<int>("shm.optix_return_index", 0);
    optix_camera_id_ = declare_parameter<int>("shm.optix_camera_id", -1);

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_link");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical_link");
    map_offset_x_ = declare_parameter<double>("map_offset.x", 0.0);
    map_offset_y_ = declare_parameter<double>("map_offset.y", 0.0);
    map_offset_z_ = declare_parameter<double>("map_offset.z", 0.0);
    map_yaw_offset_ = declare_parameter<double>("map_offset.yaw", 0.0);
    flatten_z_ = declare_parameter<bool>("flatten_z", true);
    publish_clock_ = declare_parameter<bool>("publish_clock", true);
    anchor_clock_to_system_time_ = declare_parameter<bool>("clock.anchor_to_system_time", true);
    clock_reset_threshold_sec_ = declare_parameter<double>("clock.reset_threshold_sec", 0.5);
    clock_restart_gap_sec_ = declare_parameter<double>("clock.restart_gap_sec", 0.05);
    publish_tf_ = declare_parameter<bool>("publish_map_to_base_tf", true);
    flip_image_vertical_ = declare_parameter<bool>("flip_image_vertical", false);

    control_timeout_sec_ = declare_parameter<double>("control_timeout_sec", 0.5);
    watchdog_deceleration_ = declare_parameter<double>("watchdog_deceleration", -2.0);
    min_acceleration_ = declare_parameter<double>("limits.min_acceleration", -6.0);
    max_acceleration_ = declare_parameter<double>("limits.max_acceleration", 3.0);
    max_steering_angle_ = declare_parameter<double>("limits.max_steering_angle", 0.7);
    report_autonomous_mode_ = declare_parameter<bool>("report_autonomous_mode", true);
    report_commanded_gear_ = declare_parameter<bool>("report_commanded_gear", true);
    report_commanded_lights_ = declare_parameter<bool>("report_commanded_lights", true);
    default_autoware_gear_ = declare_parameter<int>("default_autoware_gear", GearCommand::DRIVE);
    command_.gear = default_autoware_gear_;

    publish_empty_occupancy_grid_ = declare_parameter<bool>("publish_empty_occupancy_grid", true);
    publish_empty_obstacle_pointcloud_ =
      declare_parameter<bool>("publish_empty_obstacle_pointcloud", true);
    perception_object_max_range_m_ =
      declare_parameter<double>("perception.object_max_range_m", 200.0);
    const auto min_object_height =
      declare_parameter<double>("perception.object_min_height_m", 0.1);
    if (!std::isfinite(min_object_height) || min_object_height < 0.0 ||
        min_object_height > std::numeric_limits<float>::max()) {
      throw std::invalid_argument("Invalid perception.object_min_height_m");
    }
    // VTD dimensions are float32: compare in the same precision so 0.1m is excluded too.
    perception_object_min_height_m_ = static_cast<float>(min_object_height);
    const std::string geometry_ns = "perception.vehicle_geometry.";
    vehicle_geometry_.enabled = declare_parameter<bool>(geometry_ns + "enabled", true);
    vehicle_geometry_.min_length_m =
      declare_parameter<double>(geometry_ns + "min_length_m", vehicle_geometry_.min_length_m);
    vehicle_geometry_.min_width_m =
      declare_parameter<double>(geometry_ns + "min_width_m", vehicle_geometry_.min_width_m);
    vehicle_geometry_.min_height_m =
      declare_parameter<double>(geometry_ns + "min_height_m", vehicle_geometry_.min_height_m);
    vehicle_geometry_.forward_offset_length_ratio = declare_parameter<double>(
      geometry_ns + "forward_offset_length_ratio", vehicle_geometry_.forward_offset_length_ratio);
    if (!vehicle_geometry_.valid()) {
      throw std::invalid_argument("Invalid perception.vehicle_geometry parameters");
    }
    const std::string pedestrian_ns = "perception.pedestrian_classification.";
    pedestrian_classification_.enabled =
      declare_parameter<bool>(pedestrian_ns + "enabled", true);
    pedestrian_classification_.dimension_tolerance_m = declare_parameter<double>(
      pedestrian_ns + "dimension_tolerance_m", pedestrian_classification_.dimension_tolerance_m);
    pedestrian_classification_.dimensions_lwh_m = declare_parameter<std::vector<double>>(
      pedestrian_ns + "dimensions_lwh_m", pedestrian_classification_.dimensions_lwh_m);
    if (!pedestrian_classification_.valid()) {
      throw std::invalid_argument("Invalid perception.pedestrian_classification parameters");
    }
    occupancy_grid_resolution_ = declare_parameter<double>("occupancy_grid.resolution", 0.5);
    occupancy_grid_width_ = declare_parameter<int>("occupancy_grid.width", 400);
    occupancy_grid_height_ = declare_parameter<int>("occupancy_grid.height", 400);
    occupancy_grid_period_sec_ =
      declare_parameter<double>("occupancy_grid.publish_period_sec", 0.5);

    camera_fx_ = declare_parameter<double>("camera.fallback_fx", 0.0);
    camera_fy_ = declare_parameter<double>("camera.fallback_fy", 0.0);
    camera_cx_ = declare_parameter<double>("camera.fallback_cx", 0.0);
    camera_cy_ = declare_parameter<double>("camera.fallback_cy", 0.0);

    const auto odometry_topic =
      declare_parameter<std::string>("topics.odometry", "/localization/kinematic_state");
    const auto acceleration_topic =
      declare_parameter<std::string>("topics.acceleration", "/localization/acceleration");
    const auto localization_initialization_state_topic = declare_parameter<std::string>(
      "topics.localization_initialization_state", "/localization/initialization_state");
    const auto occupancy_grid_topic =
      declare_parameter<std::string>("topics.occupancy_grid", "/perception/occupancy_grid_map/map");
    const auto obstacle_pointcloud_topic = declare_parameter<std::string>(
      "topics.obstacle_pointcloud", "/perception/obstacle_segmentation/pointcloud");
    const auto pointcloud_topic =
      declare_parameter<std::string>("topics.pointcloud", "/sensing/lidar/top/pointcloud_raw");
    const auto image_topic =
      declare_parameter<std::string>("topics.image", "/sensing/camera/camera0/image_raw");
    const auto camera_info_topic =
      declare_parameter<std::string>("topics.camera_info", "/sensing/camera/camera0/camera_info");
    const auto ego_state_topic =
      declare_parameter<std::string>("topics.ego_state", "/vtd/ego_state");
    const auto objects_topic = declare_parameter<std::string>("topics.objects", "/vtd/objects");
    const auto traffic_lights_topic =
      declare_parameter<std::string>("topics.traffic_lights", "/vtd/traffic_lights");
    const auto autoware_traffic_lights_topic = declare_parameter<std::string>(
      "topics.autoware_traffic_lights", "/simulator/input/traffic_signals");
    const auto road_speed_limit_source_topic = declare_parameter<std::string>(
      "topics.road_speed_limit_source",
      "/planning/scenario_planning/lane_driving/behavior_planning/"
      "path_with_lane_id");
    const auto rviz_velocity_limit_topic = declare_parameter<std::string>(
      "topics.rviz_velocity_limit", "/planning/scenario_planning/applied_velocity_limit");
    const auto default_traffic_light_id_map =
      ament_index_cpp::get_package_share_directory("vtd_ros2_bridge") +
      "/config/hlvtd_traffic_light_id_map.csv";
    traffic_light_id_map_file_ =
      declare_parameter<std::string>("traffic_light.id_map_file", default_traffic_light_id_map);
    publish_unmapped_traffic_light_ids_ =
      declare_parameter<bool>("traffic_light.publish_unmapped_ids", false);
    if (publish_unmapped_traffic_light_ids_) {
      throw std::invalid_argument(
        "traffic_light.publish_unmapped_ids must be false: 9910 approach IDs are not map group "
        "IDs");
    }
    const auto control_topic =
      declare_parameter<std::string>("topics.control_command", "/control/command/control_cmd");
    const auto gear_topic =
      declare_parameter<std::string>("topics.gear_command", "/control/command/gear_cmd");
    const auto turn_indicators_topic = declare_parameter<std::string>(
      "topics.turn_indicators_command", "/control/command/turn_indicators_cmd");
    const auto hazard_lights_topic = declare_parameter<std::string>(
      "topics.hazard_lights_command", "/control/command/hazard_lights_cmd");

    odometry_pub_ = create_publisher<Odometry>(odometry_topic, rclcpp::QoS(10));
    acceleration_pub_ =
      create_publisher<AccelWithCovarianceStamped>(acceleration_topic, rclcpp::QoS(10));
    localization_initialization_state_pub_ = create_publisher<LocalizationInitializationState>(
      localization_initialization_state_topic, rclcpp::QoS(1).transient_local().reliable());
    occupancy_grid_pub_ = create_publisher<OccupancyGrid>(
      occupancy_grid_topic, rclcpp::QoS(1).transient_local().reliable());
    obstacle_pointcloud_pub_ =
      create_publisher<PointCloud2>(obstacle_pointcloud_topic, rclcpp::QoS(1).reliable());
    velocity_pub_ =
      create_publisher<VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS(10));
    steering_pub_ =
      create_publisher<SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS(10));
    gear_pub_ = create_publisher<GearReport>("/vehicle/status/gear_status", rclcpp::QoS(10));
    turn_indicators_pub_ = create_publisher<TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", rclcpp::QoS(10));
    hazard_lights_pub_ =
      create_publisher<HazardLightsReport>("/vehicle/status/hazard_lights_status", rclcpp::QoS(10));
    control_mode_pub_ =
      create_publisher<ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS(10));
    pointcloud_pub_ = create_publisher<PointCloud2>(pointcloud_topic, rclcpp::SensorDataQoS());
    image_pub_ = create_publisher<Image>(image_topic, rclcpp::SensorDataQoS());
    camera_info_pub_ = create_publisher<CameraInfo>(camera_info_topic, rclcpp::SensorDataQoS());
    ego_state_pub_ = create_publisher<VtdEgoState>(ego_state_topic, rclcpp::QoS(10));
    objects_pub_ = create_publisher<VtdObjectArray>(objects_topic, rclcpp::QoS(10));
    detected_objects_pub_ = create_publisher<DetectedObjects>(
      "/perception/object_recognition/detection/objects", rclcpp::QoS(1).reliable());
    traffic_lights_pub_ =
      create_publisher<VtdTrafficLightArray>(traffic_lights_topic, rclcpp::QoS(10));
    autoware_traffic_lights_pub_ =
      create_publisher<TrafficLightGroupArray>(autoware_traffic_lights_topic, rclcpp::QoS(10));
    rviz_velocity_limit_pub_ = create_publisher<VelocityLimit>(
      rviz_velocity_limit_topic, rclcpp::QoS(1).transient_local().reliable());
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
    if (publish_clock_) {
      clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    control_sub_ = create_subscription<Control>(
      control_topic, rclcpp::QoS(10),
      [this](const Control::SharedPtr message) { on_control(*message); });
    gear_sub_ = create_subscription<GearCommand>(
      gear_topic, rclcpp::QoS(10),
      [this](const GearCommand::SharedPtr message) { on_gear(*message); });
    turn_indicators_sub_ = create_subscription<TurnIndicatorsCommand>(
      turn_indicators_topic, rclcpp::QoS(10),
      [this](const TurnIndicatorsCommand::SharedPtr message) { on_turn_indicators(*message); });
    hazard_lights_sub_ = create_subscription<HazardLightsCommand>(
      hazard_lights_topic, rclcpp::QoS(10),
      [this](const HazardLightsCommand::SharedPtr message) { on_hazard_lights(*message); });
    road_speed_limit_sub_ = create_subscription<PathWithLaneId>(
      road_speed_limit_source_topic, rclcpp::QoS(1).reliable(),
      [this](const PathWithLaneId::ConstSharedPtr message) {
        if (message->points.empty()) {
          return;
        }
        double ego_x = 0.0;
        double ego_y = 0.0;
        {
          std::lock_guard<std::mutex> lock(ego_pose_mutex_);
          if (!ego_pose_received_) {
            return;
          }
          ego_x = ego_x_;
          ego_y = ego_y_;
        }
        const auto squared_distance = [ego_x, ego_y](const auto & point) {
          const auto dx = point.point.pose.position.x - ego_x;
          const auto dy = point.point.pose.position.y - ego_y;
          return dx * dx + dy * dy;
        };
        const auto nearest = std::min_element(
          message->points.begin(), message->points.end(),
          [&squared_distance](const auto & lhs, const auto & rhs) {
            return squared_distance(lhs) < squared_distance(rhs);
          });
        const auto & current_lane_ids = nearest->lane_ids;
        float road_speed_limit = 0.0F;
        for (const auto & point : message->points) {
          const bool same_lane =
            current_lane_ids.empty() ||
            std::any_of(
              point.lane_ids.begin(), point.lane_ids.end(),
              [&current_lane_ids](const auto lane_id) {
                return std::find(current_lane_ids.begin(), current_lane_ids.end(), lane_id) !=
                       current_lane_ids.end();
              });
          if (same_lane) {
            road_speed_limit =
              std::max(road_speed_limit, std::abs(point.point.longitudinal_velocity_mps));
          }
        }
        if (road_speed_limit <= 0.0F) {
          return;
        }
        VelocityLimit output;
        output.stamp = message->header.stamp;
        output.max_velocity = road_speed_limit;
        output.sender = "behavior_path/current_road_speed_limit";
        rviz_velocity_limit_pub_->publish(output);
      });

    control_client_ = std::make_unique<HlvtdControlClient>(
      control_host_, control_port_,
      [this](const bool connected) {
        on_hlvtd_connection(connected);
        RCLCPP_INFO(
          get_logger(), "HLVTD DATA/CONTROL channel %s (%s:%d)",
          connected ? "connected" : "disconnected", control_host_.c_str(), control_port_);
      },
      std::chrono::milliseconds(std::max(1, control_reconnect_delay_ms_)),
      [this](const HlvtdParticipantData & data) { on_hlvtd_participant_data(data); });

    diagnostics_timer_ =
      create_wall_timer(std::chrono::seconds(1), [this]() { publish_diagnostics(); });
    control_send_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, control_send_period_ms_)),
      [this]() { send_control(); });

    load_traffic_light_id_map();

    control_client_->start();

    RCLCPP_INFO(
      get_logger(),
      "VTD bridge ready: HLVTD DATA/CONTROL=%s:%d, LiDAR=UDP/9912, "
      "raw RDB TCP disabled",
      control_host_.c_str(), control_port_);
    RCLCPP_WARN(
      get_logger(),
      "API-only compatibility: steering status is command-based, not "
      "measured feedback; object center X/Y use participant coordinates directly");
  }

  ~VtdBridgeNode() override
  {
    if (control_client_) {
      control_client_->stop();
    }
  }

private:
  struct CommandState
  {
    bool received{false};
    float acceleration{0.0F};
    float steering_angle{0.0F};
    int gear{GearCommand::DRIVE};
    std::uint8_t turn_indicators{TurnIndicatorsCommand::DISABLE};
    std::uint8_t hazard_lights{HazardLightsCommand::DISABLE};
    std::chrono::steady_clock::time_point last_received{};
  };

  struct VehicleState
  {
    float steering_angle{0.0F};
    std::uint8_t vtd_gear{RDB_GEAR_BOX_POS_D};
    std::uint32_t light_mask{RDB_VEHICLE_LIGHT_OFF};
  };

  struct TimeUpdate
  {
    double ros_time{};
    bool session_reset{};
    double previous_raw_time{};
  };

  struct ParticipantTimeUpdate
  {
    double ros_time{};
    double dt{};
    bool derivative_valid{};
  };

  struct ParticipantMotionState
  {
    bool valid{};
    bool velocity_valid{};
    double x{};
    double y{};
    double z{};
    double heading{};
    double longitudinal_velocity{};
    double lateral_velocity{};
    double heading_rate{};
  };

  static double system_time_seconds()
  {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
      .count();
  }

  TimeUpdate update_time_mapping(const double raw_sim_time, const std::uint32_t frame_no)
  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    if (!time_mapping_initialized_) {
      const double offset =
        anchor_clock_to_system_time_ ? system_time_seconds() - raw_sim_time : 0.0;
      sim_time_offset_.store(offset);
      last_raw_state_sim_time_ = raw_sim_time;
      last_ros_state_time_ = raw_sim_time + offset;
      last_raw_state_frame_ = frame_no;
      time_mapping_initialized_ = true;
      return {last_ros_state_time_, false, raw_sim_time};
    }

    const double previous_raw_time = last_raw_state_sim_time_;
    const bool time_moved_back =
      raw_sim_time + clock_reset_threshold_sec_ < last_raw_state_sim_time_;
    const bool frame_moved_back = frame_no < last_raw_state_frame_;
    const bool session_reset = time_moved_back || frame_moved_back;
    if (session_reset) {
      const double minimum_next_time =
        last_ros_state_time_ + std::max(clock_restart_gap_sec_, 1.0e-6);
      const double restart_anchor = anchor_clock_to_system_time_
                                      ? std::max(minimum_next_time, system_time_seconds())
                                      : minimum_next_time;
      sim_time_offset_.store(restart_anchor - raw_sim_time);
    }

    double ros_time = raw_sim_time + sim_time_offset_.load();
    if (ros_time < last_ros_state_time_) {
      ros_time = last_ros_state_time_;
    }
    last_raw_state_sim_time_ = raw_sim_time;
    last_ros_state_time_ = ros_time;
    last_raw_state_frame_ = frame_no;
    return {ros_time, session_reset, previous_raw_time};
  }

  builtin_interfaces::msg::Time message_stamp(const double raw_sim_time) const
  {
    return sim_stamp(raw_sim_time + sim_time_offset_.load());
  }

  static double normalize_angle(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void on_hlvtd_connection(const bool connected)
  {
    if (connected) {
      return;
    }
    std::lock_guard<std::mutex> lock(time_mutex_);
    participant_time_initialized_ = false;
    participant_motion_ = ParticipantMotionState{};
    state_frame_received_ = false;
  }

  ParticipantTimeUpdate next_participant_time()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(time_mutex_);
    if (!participant_time_initialized_) {
      const double minimum_next =
        time_mapping_initialized_ ? last_ros_state_time_ + std::max(clock_restart_gap_sec_, 1.0e-6)
                                  : 0.0;
      last_ros_state_time_ =
        anchor_clock_to_system_time_ ? std::max(minimum_next, system_time_seconds()) : minimum_next;
      last_participant_receive_time_ = steady_now;
      participant_time_initialized_ = true;
      time_mapping_initialized_ = true;
      return {last_ros_state_time_, 0.0, false};
    }

    const double dt =
      std::chrono::duration<double>(steady_now - last_participant_receive_time_).count();
    last_participant_receive_time_ = steady_now;
    if (std::isfinite(dt) && dt > 0.0) {
      last_ros_state_time_ += dt;
    }
    return {last_ros_state_time_, dt, std::isfinite(dt) && dt >= 1.0e-4 && dt <= 1.0};
  }

  static bool finite_ego(const HlvtdParticipantData & data)
  {
    return std::isfinite(data.ego_x) && std::isfinite(data.ego_y) && std::isfinite(data.ego_z) &&
           std::isfinite(data.ego_heading) && std::isfinite(data.ego_pitch) &&
           std::isfinite(data.ego_roll);
  }

  static bool finite_object(const HlvtdParticipantObject & object)
  {
    return std::isfinite(object.x) && std::isfinite(object.y) && std::isfinite(object.z) &&
           std::isfinite(object.heading) && std::isfinite(object.speed) &&
           std::isfinite(object.length) && std::isfinite(object.width) &&
           std::isfinite(object.height);
  }

  void on_hlvtd_participant_data(const HlvtdParticipantData & data)
  {
    if (!finite_ego(data)) {
      ++parse_errors_;
      return;
    }

    const auto timing = next_participant_time();
    const auto stamp = sim_stamp(timing.ros_time);
    last_sim_time_.store(timing.ros_time);
    state_frame_received_ = true;
    last_frame_no_.fetch_add(1U);
    if (publish_clock_) {
      rosgraph_msgs::msg::Clock clock;
      clock.clock = stamp;
      clock_pub_->publish(clock);
      ++clock_updates_;
    }

    publish_hlvtd_ego_state(data, stamp, timing);
    ++ego_updates_;

    std::vector<VtdObject> objects;
    objects.reserve(kHlvtdObjectCount);
    for (const auto & source : data.objects) {
      // The fixed array has no count field. An all-zero ID denotes an unused
      // slot in the supplied participant interface.
      if (source.id == 0U) {
        continue;
      }
      if (
        !finite_object(source) || source.length <= 0.0F || source.width <= 0.0F ||
        source.height <= 0.0F) {
        ++objects_dropped_;
        continue;
      }
      VtdObject object;
      object.id = source.id;
      const auto position = map_position(source.x, source.y, source.z);
      object.x = static_cast<float>(position[0]);
      object.y = static_cast<float>(position[1]);
      object.z = flatten_z_ ? 0.0F : static_cast<float>(position[2]);
      object.heading = static_cast<float>(source.heading + map_yaw_offset_);
      object.speed = source.speed;
      object.length = source.length;
      object.width = source.width;
      object.height = source.height;
      objects.push_back(object);
    }

    std::vector<VtdTrafficLight> traffic_lights;
    last_traffic_light_id_.store(data.traffic_light.id);
    last_traffic_light_state_.store(data.traffic_light.state);
    if (data.traffic_light.id != 0) {
      if (data.traffic_light.state <= 6U) {
        VtdTrafficLight light;
        light.id = data.traffic_light.id;
        light.state = data.traffic_light.state;
        traffic_lights.push_back(light);
      } else {
        ++parse_errors_;
      }
    }
    publish_api_arrays(stamp, std::move(objects), std::move(traffic_lights));
  }

  void reset_session_state(const double previous_raw_time, const double new_raw_time)
  {
    selected_ego_id_.store(-1);
    last_occupancy_grid_sim_time_.store(-1.0);
    last_api_publish_frame_.store(std::numeric_limits<std::uint32_t>::max());
    last_control_queue_frame_.store(std::numeric_limits<std::uint32_t>::max());
    watchdog_active_.store(false);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_ = CommandState{};
      command_.gear = default_autoware_gear_;
    }
    {
      std::lock_guard<std::mutex> lock(vehicle_mutex_);
      vehicle_ = VehicleState{};
    }
    {
      std::lock_guard<std::mutex> lock(ego_pose_mutex_);
      ego_pose_received_ = false;
    }
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      cameras_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(api_mutex_);
      pending_objects_.clear();
      traffic_lights_by_id_.clear();
    }
    ++session_resets_;
    RCLCPP_WARN(
      get_logger(),
      "VTD session reset detected (sim time %.3f -> %.3f); "
      "preserving monotonic ROS time and clearing bridge state",
      previous_raw_time, new_raw_time);
  }

  void on_rdb_message(
    const std::string & channel, const std::uint8_t * data, const std::size_t size)
  {
    if (size < sizeof(RDB_MSG_HDR_t)) {
      ++parse_errors_;
      return;
    }
    const auto * message = reinterpret_cast<const RDB_MSG_HDR_t *>(data);
    if (channel == "state/control") {
      const bool first_state_frame = !state_frame_received_.exchange(true);
      const auto previous_frame = last_frame_no_.exchange(message->frameNo);
      if (first_state_frame || previous_frame != message->frameNo) {
        const auto time_update = update_time_mapping(message->simTime, message->frameNo);
        if (time_update.session_reset) {
          reset_session_state(time_update.previous_raw_time, message->simTime);
        }
        last_sim_time_.store(message->simTime);
        if (publish_clock_) {
          rosgraph_msgs::msg::Clock clock;
          clock.clock = sim_stamp(time_update.ros_time);
          clock_pub_->publish(clock);
          ++clock_updates_;
        }
        std::lock_guard<std::mutex> lock(api_mutex_);
        pending_objects_.clear();
      }
    }

    std::string error;
    std::size_t custom_optix_index = 0U;
    const bool valid = parse_rdb_message(
      data, size,
      [this, &channel, &custom_optix_index](
        const RDB_MSG_HDR_t & header, const RdbEntryView & entry) {
        handle_entry(channel, header, entry, custom_optix_index);
      },
      &error);
    if (!valid) {
      ++parse_errors_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejected RDB message: %s", error.c_str());
    }
  }

  void handle_entry(
    const std::string & channel, const RDB_MSG_HDR_t & message, const RdbEntryView & entry,
    std::size_t & custom_optix_index)
  {
    switch (entry.header->pkgId) {
      case RDB_PKG_ID_OBJECT_STATE:
        handle_object_states(message, entry);
        break;
      case RDB_PKG_ID_TRAFFIC_LIGHT:
        handle_traffic_lights(entry);
        break;
      case RDB_PKG_ID_VEHICLE_SYSTEMS:
        handle_vehicle_systems(entry);
        break;
      case RDB_PKG_ID_DRIVETRAIN:
        handle_drivetrain(entry);
        break;
      case RDB_PKG_ID_CAMERA:
        handle_camera_config(entry);
        break;
      case RDB_PKG_ID_IMAGE:
        handle_images(message, entry);
        break;
      case RDB_PKG_ID_RAY:
        handle_rays(message, entry);
        break;
      case RDB_PKG_ID_CUSTOM_OPTIX_START:
        handle_optix_lidar(message, entry, custom_optix_index);
        break;
      case RDB_PKG_ID_END_OF_FRAME:
        if (channel == "state/control") {
          const auto previous_api_frame = last_api_publish_frame_.exchange(message.frameNo);
          if (previous_api_frame != message.frameNo) {
            publish_api_arrays(message);
          }
          const auto previous_control_frame = last_control_queue_frame_.exchange(message.frameNo);
          if (previous_control_frame != message.frameNo) {
            send_control();
          }
        }
        break;
      default:
        break;
    }
  }

  bool is_ego(const RDB_OBJECT_STATE_BASE_t & object)
  {
    if (ego_player_id_ >= 0) {
      return object.id == static_cast<std::uint32_t>(ego_player_id_);
    }
    const auto name = rdb_name(object.name, RDB_SIZE_OBJECT_NAME);
    if (!ego_name_.empty()) {
      if (name == ego_name_) {
        selected_ego_id_.store(static_cast<int>(object.id));
        return true;
      }
      return false;
    }
    if (object.category != RDB_OBJECT_CATEGORY_PLAYER) {
      return false;
    }
    int expected = -1;
    selected_ego_id_.compare_exchange_strong(expected, static_cast<int>(object.id));
    return selected_ego_id_.load() == static_cast<int>(object.id);
  }

  void handle_object_states(const RDB_MSG_HDR_t & message, const RdbEntryView & entry)
  {
    if (
      entry.header->elementSize < sizeof(RDB_OBJECT_STATE_BASE_t) ||
      entry.header->elementSize == 0U) {
      return;
    }
    const auto count = entry.data_size / entry.header->elementSize;
    for (std::size_t index = 0; index < count; ++index) {
      const auto * bytes = entry.data + index * entry.header->elementSize;
      const auto * base = reinterpret_cast<const RDB_OBJECT_STATE_BASE_t *>(bytes);
      const bool extended = (entry.header->flags & RDB_PKG_FLAG_EXTENDED) != 0U &&
                            entry.header->elementSize >= sizeof(RDB_OBJECT_STATE_t);
      const auto * full = reinterpret_cast<const RDB_OBJECT_STATE_t *>(bytes);
      const auto * extension = extended ? &full->ext : nullptr;
      if (is_ego(*base)) {
        publish_ego_state(message, *base, extension);
        ++ego_updates_;
        continue;
      }

      VtdObject object;
      object.id = base->id;
      const auto position = map_position(base->pos.x, base->pos.y, base->pos.z);
      object.x = static_cast<float>(position[0]);
      object.y = static_cast<float>(position[1]);
      object.z = flatten_z_ ? 0.0F : static_cast<float>(position[2]);
      object.heading = static_cast<float>(base->pos.h + map_yaw_offset_);
      object.length = base->geo.dimX;
      object.width = base->geo.dimY;
      object.height = base->geo.dimZ;
      if (extension) {
        const auto speed = body_vector(extension->speed, base->pos.h);
        object.speed = static_cast<float>(
          std::sqrt(speed[0] * speed[0] + speed[1] * speed[1] + speed[2] * speed[2]));
      }
      {
        std::lock_guard<std::mutex> lock(api_mutex_);
        if (pending_objects_.size() < kApiMaxObjects) {
          pending_objects_.push_back(object);
        } else {
          ++objects_dropped_;
        }
      }
    }
  }

  static std::uint8_t api_traffic_light_state(const std::uint8_t phase)
  {
    switch (phase) {
      case RDB_TRLIGHT_PHASE_STOP:
        return 1U;  // red
      case RDB_TRLIGHT_PHASE_STOP_ATTN:
      case RDB_TRLIGHT_PHASE_ATTN:
        return 2U;  // yellow
      case RDB_TRLIGHT_PHASE_GO:
        return 3U;  // green
      case RDB_TRLIGHT_PHASE_GO_EXCL:
        return 5U;  // green + left arrow
      case RDB_TRLIGHT_PHASE_BLINK:
        return 6U;  // flashing
      default:
        return 0U;  // off/unassigned/unknown
    }
  }

  static std::uint8_t api_traffic_light_state_from_signal(
    const std::int32_t signal_type, const std::uint32_t state_mask)
  {
    if (state_mask == 0U) {
      return 0U;
    }
    switch (signal_type) {
      case 1000020:
        return 1U;  // red lamp
      case 1000008:
        return 2U;  // amber lamp
      case 1000012:
        return 3U;  // green lamp or green arrow
      default:
        return 0U;
    }
  }

  static std::uint8_t active_traffic_light_phase(
    const RDB_TRAFFIC_LIGHT_BASE_t & light, const RDB_TRAFFIC_LIGHT_PHASE_t * phases,
    const std::size_t phase_count)
  {
    if (phases == nullptr || phase_count == 0U || !std::isfinite(light.state)) {
      return RDB_TRLIGHT_PHASE_UNKNOWN;
    }
    const float cycle_position = std::clamp(light.state, 0.0F, 1.0F);
    float phase_end = 0.0F;
    std::uint8_t last_valid_phase = RDB_TRLIGHT_PHASE_UNKNOWN;
    for (std::size_t index = 0; index < phase_count; ++index) {
      if (!std::isfinite(phases[index].duration) || phases[index].duration <= 0.0F) {
        continue;
      }
      last_valid_phase = phases[index].type;
      phase_end += phases[index].duration;
      if (cycle_position < phase_end || index + 1U == phase_count) {
        return phases[index].type;
      }
    }
    return last_valid_phase;
  }

  void handle_traffic_lights(const RdbEntryView & entry)
  {
    if (
      entry.header->elementSize < sizeof(RDB_TRAFFIC_LIGHT_BASE_t) ||
      entry.header->elementSize == 0U) {
      return;
    }

    std::vector<VtdTrafficLight> traffic_lights;
    const auto count = entry.data_size / entry.header->elementSize;
    traffic_lights.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      const auto * bytes = entry.data + index * entry.header->elementSize;
      const auto * light = reinterpret_cast<const RDB_TRAFFIC_LIGHT_BASE_t *>(bytes);
      const RDB_TRAFFIC_LIGHT_PHASE_t * phases = nullptr;
      std::size_t phase_count = 0U;
      if (
        (entry.header->flags & RDB_PKG_FLAG_EXTENDED) != 0U &&
        entry.header->elementSize >= sizeof(RDB_TRAFFIC_LIGHT_t)) {
        const auto * full = reinterpret_cast<const RDB_TRAFFIC_LIGHT_t *>(bytes);
        const auto phase_capacity = (entry.header->elementSize - sizeof(RDB_TRAFFIC_LIGHT_t)) /
                                    sizeof(RDB_TRAFFIC_LIGHT_PHASE_t);
        phase_count = std::min<std::size_t>(
          full->ext.noPhases,
          std::min<std::size_t>(
            full->ext.dataSize / sizeof(RDB_TRAFFIC_LIGHT_PHASE_t), phase_capacity));
        phases =
          reinterpret_cast<const RDB_TRAFFIC_LIGHT_PHASE_t *>(bytes + sizeof(RDB_TRAFFIC_LIGHT_t));
      }

      VtdTrafficLight item;
      item.id = light->id;
      const auto signal_type = traffic_light_signal_types_.find(light->id);
      item.state =
        signal_type != traffic_light_signal_types_.end()
          ? api_traffic_light_state_from_signal(signal_type->second, light->stateMask)
          : api_traffic_light_state(active_traffic_light_phase(*light, phases, phase_count));
      traffic_lights.push_back(item);
    }

    std::lock_guard<std::mutex> lock(api_mutex_);
    for (auto & traffic_light : traffic_lights) {
      traffic_lights_by_id_.insert_or_assign(traffic_light.id, std::move(traffic_light));
    }
  }

  void load_traffic_light_id_map()
  {
    std::ifstream input(traffic_light_id_map_file_);
    if (!input) {
      throw std::runtime_error(
        "Cannot open 9910 traffic-light ID map: " + traffic_light_id_map_file_);
    }
    traffic_light_group_ids_ = load_hlvtd_traffic_light_id_map(input);
    std::size_t pair_count = 0U;
    for (const auto & [api_id, group_ids] : traffic_light_group_ids_) {
      (void)api_id;
      pair_count += group_ids.size();
    }
    RCLCPP_INFO(
      get_logger(), "Loaded %zu Autoware traffic-light groups for %zu HLVTD 9910 approach IDs",
      pair_count, traffic_light_group_ids_.size());
  }

  void publish_autoware_traffic_lights(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<VtdTrafficLight> & traffic_lights)
  {
    std::map<std::int64_t, TrafficLightGroup> groups;
    std::uint64_t unmapped_count = 0U;
    for (const auto & light : traffic_lights) {
      std::vector<std::int64_t> group_ids;
      const auto found = traffic_light_group_ids_.find(light.id);
      if (found != traffic_light_group_ids_.end()) {
        group_ids = found->second;
      } else {
        ++unmapped_count;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "HLVTD 9910 traffic-light ID %d has no mapped stop-line group; not publishing a raw ID",
          light.id);
      }
      for (const auto group_id : group_ids) {
        auto & group = groups[group_id];
        group.traffic_light_group_id = group_id;
        if (light.state == 0U) {
          continue;
        }
        const auto elements = hlvtd_traffic_light_elements(light.state);
        for (const auto & candidate : elements) {
          const auto duplicate = std::find_if(
            group.elements.begin(), group.elements.end(), [&candidate](const auto & existing) {
              return existing.color == candidate.color && existing.shape == candidate.shape &&
                     existing.status == candidate.status;
            });
          if (duplicate == group.elements.end()) {
            group.elements.push_back(candidate);
          }
        }
      }
    }

    TrafficLightGroupArray output;
    output.stamp = stamp;
    output.traffic_light_groups.reserve(groups.size());
    for (auto & [group_id, group] : groups) {
      (void)group_id;
      if (group.elements.empty()) {
        group.elements = hlvtd_traffic_light_elements(0U);
      }
      output.traffic_light_groups.push_back(std::move(group));
    }
    traffic_light_items_.store(traffic_lights.size());
    autoware_traffic_light_groups_.store(output.traffic_light_groups.size());
    unmapped_traffic_light_ids_.store(unmapped_count);
    autoware_traffic_lights_pub_->publish(output);
    ++autoware_traffic_light_updates_;
  }

  void publish_vtd_obstacle_pointcloud(
    const std_msgs::msg::Header & header, const std::vector<VtdObject> & objects)
  {
    constexpr float sample_spacing = 0.025F;
    double ego_x = 0.0;
    double ego_y = 0.0;
    double ego_z = 0.0;
    double ego_yaw = 0.0;
    bool ego_pose_received = false;
    {
      std::lock_guard<std::mutex> lock(ego_pose_mutex_);
      ego_x = ego_x_;
      ego_y = ego_y_;
      ego_z = ego_z_;
      ego_yaw = ego_yaw_;
      ego_pose_received = ego_pose_received_;
    }

    std::vector<std::array<float, 4>> points;
    for (const auto & object : objects) {
      if (!ego_pose_received) {
        break;
      }
      if (
        !std::isfinite(object.x) || !std::isfinite(object.y) || !std::isfinite(object.z) ||
        !std::isfinite(object.heading) || !std::isfinite(object.length) ||
        !std::isfinite(object.width) || !std::isfinite(object.height) || object.length <= 0.0F ||
        object.width <= 0.0F || object.height <= 0.0F) {
        continue;
      }
      const float cosine = std::cos(object.heading);
      const float sine = std::sin(object.heading);
      const auto center =
        autoware_box_center(object, vehicle_geometry_, pedestrian_classification_);

      const auto append_edge = [&](
                                 const float x0, const float y0, const float z0, const float x1,
                                 const float y1, const float z1) {
        const float edge_length =
          std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
        const auto samples = std::max<std::size_t>(
          1U, static_cast<std::size_t>(std::ceil(edge_length / sample_spacing)));
        const float ego_cosine = std::cos(static_cast<float>(ego_yaw));
        const float ego_sine = std::sin(static_cast<float>(ego_yaw));
        for (std::size_t index = 0U; index < samples; ++index) {
          const float ratio = (static_cast<float>(index) + 0.5F) / static_cast<float>(samples);
          const float local_x = x0 + ratio * (x1 - x0);
          const float local_y = y0 + ratio * (y1 - y0);
          const float local_z = z0 + ratio * (z1 - z0);
          const float map_x = center[0] + cosine * local_x - sine * local_y;
          const float map_y = center[1] + sine * local_x + cosine * local_y;
          const float delta_x = map_x - static_cast<float>(ego_x);
          const float delta_y = map_y - static_cast<float>(ego_y);
          points.push_back(
            {ego_cosine * delta_x + ego_sine * delta_y, -ego_sine * delta_x + ego_cosine * delta_y,
             center[2] + local_z - static_cast<float>(ego_z), 1.0F});
        }
      };

      const float half_length = 0.5F * object.length;
      const float half_width = 0.5F * object.width;
      const float half_height = 0.5F * object.height;
      for (const float z : {-half_height, half_height}) {
        append_edge(-half_length, -half_width, z, half_length, -half_width, z);
        append_edge(half_length, -half_width, z, half_length, half_width, z);
        append_edge(half_length, half_width, z, -half_length, half_width, z);
        append_edge(-half_length, half_width, z, -half_length, -half_width, z);
      }
      for (const float x : {-half_length, half_length}) {
        for (const float y : {-half_width, half_width}) {
          append_edge(x, y, -half_height, x, y, half_height);
        }
      }
    }

    PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = base_frame_;
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 4U * sizeof(float);
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.fields.resize(4U);
    const std::array<std::string, 4> names{"x", "y", "z", "intensity"};
    for (std::size_t index = 0U; index < cloud.fields.size(); ++index) {
      cloud.fields[index].name = names[index];
      cloud.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
      cloud.fields[index].datatype = PointField::FLOAT32;
      cloud.fields[index].count = 1U;
    }
    cloud.data.resize(points.size() * sizeof(points.front()));
    if (!points.empty()) {
      std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
    }
    obstacle_pointcloud_pub_->publish(cloud);
    ++obstacle_pointcloud_updates_;
  }

  void publish_api_arrays(const RDB_MSG_HDR_t & message)
  {
    std::vector<VtdObject> objects;
    std::vector<VtdTrafficLight> traffic_lights;
    {
      std::lock_guard<std::mutex> lock(api_mutex_);
      objects = pending_objects_;
      traffic_lights.reserve(traffic_lights_by_id_.size());
      for (const auto & [id, traffic_light] : traffic_lights_by_id_) {
        (void)id;
        traffic_lights.push_back(traffic_light);
      }
    }

    publish_api_arrays(
      message_stamp(message.simTime), std::move(objects), std::move(traffic_lights));
  }

  void publish_api_arrays(
    const builtin_interfaces::msg::Time & stamp, std::vector<VtdObject> objects,
    std::vector<VtdTrafficLight> traffic_lights)
  {
    VtdObjectArray object_array;
    object_array.header.stamp = stamp;
    object_array.header.frame_id = map_frame_;

    // /vtd/objects is the unfiltered API feed, while Autoware perception is a
    // sensor-local interface. Do not make every object in the VTD world a
    // predicted object: velocity-planning modules otherwise evaluate distant
    // objects whenever their scene module becomes active.
    std::vector<VtdObject> perception_objects;
    perception_objects.reserve(objects.size());
    double ego_x = 0.0;
    double ego_y = 0.0;
    bool ego_pose_received = false;
    {
      std::lock_guard<std::mutex> lock(ego_pose_mutex_);
      ego_x = ego_x_;
      ego_y = ego_y_;
      ego_pose_received = ego_pose_received_;
    }
    const bool filter_by_range = perception_object_max_range_m_ > 0.0;
    const double max_squared_range =
      perception_object_max_range_m_ * perception_object_max_range_m_;
    for (const auto & object : objects) {
      // Both detections and the synthetic obstacle cloud use this filtered collection.
      if (object.height <= perception_object_min_height_m_) {
        continue;
      }
      if (!filter_by_range) {
        perception_objects.push_back(object);
        continue;
      }
      if (!ego_pose_received) {
        continue;
      }
      const auto center =
        autoware_box_center(object, vehicle_geometry_, pedestrian_classification_);
      const double dx = static_cast<double>(center[0]) - ego_x;
      const double dy = static_cast<double>(center[1]) - ego_y;
      if (dx * dx + dy * dy <= max_squared_range) {
        perception_objects.push_back(object);
      }
    }

    DetectedObjects detected_objects;
    detected_objects.header = object_array.header;
    detected_objects.objects.reserve(perception_objects.size());
    for (const auto & object : perception_objects) {
      const auto center =
        autoware_box_center(object, vehicle_geometry_, pedestrian_classification_);
      DetectedObject detected;
      detected.existence_probability = 1.0F;
      detected.classification.resize(1U);
      detected.classification.front().label =
        autoware_object_class(object, vehicle_geometry_, pedestrian_classification_);
      detected.classification.front().probability = 1.0F;
      detected.kinematics.pose_with_covariance.pose.position.x = center[0];
      detected.kinematics.pose_with_covariance.pose.position.y = center[1];
      detected.kinematics.pose_with_covariance.pose.position.z = flatten_z_ ? 0.0F : center[2];
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, object.heading);
      detected.kinematics.pose_with_covariance.pose.orientation = tf2::toMsg(orientation);
      detected.kinematics.has_position_covariance = false;
      detected.kinematics.orientation_availability =
        autoware_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE;
      detected.kinematics.twist_with_covariance.twist.linear.x = object.speed;
      detected.kinematics.has_twist = true;
      detected.kinematics.has_twist_covariance = false;
      detected.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
      detected.shape.dimensions.x = object.length;
      detected.shape.dimensions.y = object.width;
      detected.shape.dimensions.z = object.height;
      detected_objects.objects.push_back(std::move(detected));
    }
    detected_objects_pub_->publish(detected_objects);
    if (publish_empty_obstacle_pointcloud_) {
      publish_vtd_obstacle_pointcloud(object_array.header, perception_objects);
    }

    object_array.objects = std::move(objects);
    objects_pub_->publish(object_array);
    ++object_updates_;

    VtdTrafficLightArray traffic_array;
    traffic_array.header.stamp = stamp;
    traffic_array.header.frame_id = map_frame_;
    traffic_array.traffic_lights = traffic_lights;
    traffic_lights_pub_->publish(traffic_array);
    ++traffic_light_updates_;

    publish_autoware_traffic_lights(stamp, traffic_lights);
  }

  std::array<double, 3> map_position(const double x, const double y, const double z) const
  {
    const double c = std::cos(map_yaw_offset_);
    const double s = std::sin(map_yaw_offset_);
    return {map_offset_x_ + c * x - s * y, map_offset_y_ + s * x + c * y, map_offset_z_ + z};
  }

  static std::array<double, 3> body_vector(const RDB_COORD_t & vector, const float vehicle_heading)
  {
    if (vector.type != RDB_COORD_TYPE_INERTIAL) {
      return {vector.x, vector.y, vector.z};
    }
    const double c = std::cos(vehicle_heading);
    const double s = std::sin(vehicle_heading);
    return {c * vector.x + s * vector.y, -s * vector.x + c * vector.y, vector.z};
  }

  void publish_ego_state(
    const RDB_MSG_HDR_t & message, const RDB_OBJECT_STATE_BASE_t & base,
    const RDB_OBJECT_STATE_EXT_t * extension)
  {
    const auto stamp = message_stamp(message.simTime);
    auto api_position = map_position(base.pos.x, base.pos.y, base.pos.z);
    if (flatten_z_) {
      api_position[2] = 0.0;
    }
    auto position = api_position;
    const double map_heading = base.pos.h + map_yaw_offset_;
    position[0] += kEgoTfForwardOffsetM * std::cos(map_heading);
    position[1] += kEgoTfForwardOffsetM * std::sin(map_heading);
    {
      std::lock_guard<std::mutex> lock(ego_pose_mutex_);
      ego_x_ = position[0];
      ego_y_ = position[1];
      ego_z_ = position[2];
      ego_yaw_ = map_heading;
      ego_pose_received_ = true;
    }
    tf2::Quaternion orientation;
    orientation.setRPY(base.pos.r, base.pos.p, map_heading);

    Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = map_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = position[0];
    odometry.pose.pose.position.y = position[1];
    odometry.pose.pose.position.z = position[2];
    odometry.pose.pose.orientation = tf2::toMsg(orientation);
    odometry.pose.covariance[0] = 0.01;
    odometry.pose.covariance[7] = 0.01;
    odometry.pose.covariance[14] = 0.01;
    odometry.pose.covariance[21] = 0.001;
    odometry.pose.covariance[28] = 0.001;
    odometry.pose.covariance[35] = 0.001;

    AccelWithCovarianceStamped acceleration;
    acceleration.header.stamp = stamp;
    acceleration.header.frame_id = base_frame_;
    VelocityReport velocity;
    velocity.header.stamp = stamp;
    velocity.header.frame_id = base_frame_;

    VtdEgoState api_ego;
    api_ego.header.stamp = stamp;
    api_ego.header.frame_id = map_frame_;
    api_ego.ego_x = static_cast<float>(api_position[0]);
    api_ego.ego_y = static_cast<float>(api_position[1]);
    api_ego.ego_z = static_cast<float>(api_position[2]);
    api_ego.ego_heading = static_cast<float>(map_heading);
    api_ego.ego_pitch = base.pos.p;
    api_ego.ego_roll = base.pos.r;
    ego_state_pub_->publish(api_ego);

    if (extension) {
      const auto speed = body_vector(extension->speed, base.pos.h);
      const auto accel = body_vector(extension->accel, base.pos.h);
      odometry.twist.twist.linear.x = speed[0];
      odometry.twist.twist.linear.y = speed[1];
      odometry.twist.twist.linear.z = speed[2];
      odometry.twist.twist.angular.x = extension->speed.r;
      odometry.twist.twist.angular.y = extension->speed.p;
      odometry.twist.twist.angular.z = extension->speed.h;
      acceleration.accel.accel.linear.x = accel[0];
      acceleration.accel.accel.linear.y = accel[1];
      acceleration.accel.accel.linear.z = accel[2];
      acceleration.accel.accel.angular.x = extension->accel.r;
      acceleration.accel.accel.angular.y = extension->accel.p;
      acceleration.accel.accel.angular.z = extension->accel.h;
      velocity.longitudinal_velocity = static_cast<float>(speed[0]);
      velocity.lateral_velocity = static_cast<float>(speed[1]);
      velocity.heading_rate = extension->speed.h;
    }
    odometry.twist.covariance[0] = 0.01;
    odometry.twist.covariance[7] = 0.01;
    odometry.twist.covariance[14] = 0.01;
    odometry.twist.covariance[21] = 0.001;
    odometry.twist.covariance[28] = 0.001;
    odometry.twist.covariance[35] = 0.001;
    acceleration.accel.covariance[0] = 0.1;
    acceleration.accel.covariance[7] = 0.1;
    acceleration.accel.covariance[14] = 0.1;
    acceleration.accel.covariance[21] = 0.1;
    acceleration.accel.covariance[28] = 0.1;
    acceleration.accel.covariance[35] = 0.1;

    odometry_pub_->publish(odometry);
    acceleration_pub_->publish(acceleration);
    velocity_pub_->publish(velocity);

    LocalizationInitializationState localization_state;
    localization_state.stamp = stamp;
    localization_state.state = LocalizationInitializationState::INITIALIZED;
    localization_initialization_state_pub_->publish(localization_state);

    if (
      publish_empty_occupancy_grid_ && occupancy_grid_resolution_ > 0.0 &&
      occupancy_grid_width_ > 0 && occupancy_grid_height_ > 0) {
      const double previous = last_occupancy_grid_sim_time_.load();
      if (
        previous < 0.0 || message.simTime < previous ||
        message.simTime - previous >= occupancy_grid_period_sec_) {
        last_occupancy_grid_sim_time_.store(message.simTime);
        OccupancyGrid grid;
        grid.header.stamp = stamp;
        grid.header.frame_id = map_frame_;
        grid.info.map_load_time = stamp;
        grid.info.resolution = static_cast<float>(occupancy_grid_resolution_);
        grid.info.width = static_cast<std::uint32_t>(occupancy_grid_width_);
        grid.info.height = static_cast<std::uint32_t>(occupancy_grid_height_);
        grid.info.origin.position.x = position[0] - 0.5 * occupancy_grid_resolution_ *
                                                      static_cast<double>(occupancy_grid_width_);
        grid.info.origin.position.y = position[1] - 0.5 * occupancy_grid_resolution_ *
                                                      static_cast<double>(occupancy_grid_height_);
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(
          static_cast<std::size_t>(occupancy_grid_width_) *
            static_cast<std::size_t>(occupancy_grid_height_),
          0);
        occupancy_grid_pub_->publish(grid);
        ++occupancy_grid_updates_;
      }
    }

    VehicleState vehicle;
    CommandState command;
    {
      std::lock_guard<std::mutex> lock(vehicle_mutex_);
      vehicle = vehicle_;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command = command_;
    }
    SteeringReport steering;
    steering.stamp = stamp;
    steering.steering_tire_angle = vehicle.steering_angle;
    steering_pub_->publish(steering);

    GearReport gear;
    gear.stamp = stamp;
    if (report_commanded_gear_) {
      gear.report = static_cast<std::uint8_t>(command.gear);
    } else {
      gear.report = autoware_gear(vehicle.vtd_gear);
    }
    gear_pub_->publish(gear);

    TurnIndicatorsReport turn_indicators;
    turn_indicators.stamp = stamp;
    HazardLightsReport hazard_lights;
    hazard_lights.stamp = stamp;
    if (report_commanded_lights_) {
      const bool hazard_enabled = command.hazard_lights == HazardLightsCommand::ENABLE;
      hazard_lights.report =
        hazard_enabled ? HazardLightsReport::ENABLE : HazardLightsReport::DISABLE;
      turn_indicators.report =
        hazard_enabled ? TurnIndicatorsReport::DISABLE : command.turn_indicators;
    } else {
      const bool left = (vehicle.light_mask & RDB_VEHICLE_LIGHT_INDICATOR_L) != 0U;
      const bool right = (vehicle.light_mask & RDB_VEHICLE_LIGHT_INDICATOR_R) != 0U;
      const bool hazard_enabled =
        (vehicle.light_mask & RDB_VEHICLE_LIGHT_EMERGENCY) != 0U || (left && right);
      hazard_lights.report =
        hazard_enabled ? HazardLightsReport::ENABLE : HazardLightsReport::DISABLE;
      turn_indicators.report =
        hazard_enabled
          ? TurnIndicatorsReport::DISABLE
          : (left ? TurnIndicatorsReport::ENABLE_LEFT
                  : (right ? TurnIndicatorsReport::ENABLE_RIGHT : TurnIndicatorsReport::DISABLE));
    }
    turn_indicators_pub_->publish(turn_indicators);
    hazard_lights_pub_->publish(hazard_lights);

    ControlModeReport mode;
    mode.stamp = stamp;
    mode.mode = report_autonomous_mode_ ? ControlModeReport::AUTONOMOUS : ControlModeReport::MANUAL;
    control_mode_pub_->publish(mode);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = position[0];
      transform.transform.translation.y = position[1];
      transform.transform.translation.z = position[2];
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publish_hlvtd_ego_state(
    const HlvtdParticipantData & data, const builtin_interfaces::msg::Time & stamp,
    const ParticipantTimeUpdate & timing)
  {
    auto api_position = map_position(data.ego_x, data.ego_y, data.ego_z);
    if (flatten_z_) {
      api_position[2] = 0.0;
    }
    const double map_heading = data.ego_heading + map_yaw_offset_;
    auto position = api_position;
    position[0] += kEgoTfForwardOffsetM * std::cos(map_heading);
    position[1] += kEgoTfForwardOffsetM * std::sin(map_heading);

    ParticipantMotionState previous;
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      previous = participant_motion_;
    }
    ParticipantMotionState current;
    current.valid = true;
    current.x = position[0];
    current.y = position[1];
    current.z = position[2];
    current.heading = map_heading;
    double longitudinal_acceleration = 0.0;
    if (previous.valid && timing.derivative_valid) {
      const double map_vx = (current.x - previous.x) / timing.dt;
      const double map_vy = (current.y - previous.y) / timing.dt;
      const double map_vz = (current.z - previous.z) / timing.dt;
      const double c = std::cos(map_heading);
      const double s = std::sin(map_heading);
      current.longitudinal_velocity = c * map_vx + s * map_vy;
      current.lateral_velocity = -s * map_vx + c * map_vy;
      current.heading_rate = normalize_angle(map_heading - previous.heading) / timing.dt;
      current.velocity_valid = true;
      if (previous.velocity_valid) {
        longitudinal_acceleration =
          (current.longitudinal_velocity - previous.longitudinal_velocity) / timing.dt;
      }
      (void)map_vz;
    }
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      participant_motion_ = current;
    }
    {
      std::lock_guard<std::mutex> lock(ego_pose_mutex_);
      ego_x_ = position[0];
      ego_y_ = position[1];
      ego_z_ = position[2];
      ego_yaw_ = map_heading;
      ego_pose_received_ = true;
    }

    tf2::Quaternion orientation;
    orientation.setRPY(data.ego_roll, data.ego_pitch, map_heading);

    Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = map_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = position[0];
    odometry.pose.pose.position.y = position[1];
    odometry.pose.pose.position.z = position[2];
    odometry.pose.pose.orientation = tf2::toMsg(orientation);
    odometry.pose.covariance[0] = 0.01;
    odometry.pose.covariance[7] = 0.01;
    odometry.pose.covariance[14] = 0.01;
    odometry.pose.covariance[21] = 0.001;
    odometry.pose.covariance[28] = 0.001;
    odometry.pose.covariance[35] = 0.001;
    odometry.twist.twist.linear.x = current.longitudinal_velocity;
    odometry.twist.twist.linear.y = current.lateral_velocity;
    odometry.twist.twist.angular.z = current.heading_rate;
    odometry.twist.covariance[0] = 0.01;
    odometry.twist.covariance[7] = 0.01;
    odometry.twist.covariance[14] = 0.01;
    odometry.twist.covariance[21] = 0.001;
    odometry.twist.covariance[28] = 0.001;
    odometry.twist.covariance[35] = 0.001;

    AccelWithCovarianceStamped acceleration;
    acceleration.header.stamp = stamp;
    acceleration.header.frame_id = base_frame_;
    acceleration.accel.accel.linear.x = longitudinal_acceleration;
    acceleration.accel.covariance[0] = 0.1;
    acceleration.accel.covariance[7] = 0.1;
    acceleration.accel.covariance[14] = 0.1;
    acceleration.accel.covariance[21] = 0.1;
    acceleration.accel.covariance[28] = 0.1;
    acceleration.accel.covariance[35] = 0.1;

    VelocityReport velocity;
    velocity.header.stamp = stamp;
    velocity.header.frame_id = base_frame_;
    velocity.longitudinal_velocity = static_cast<float>(current.longitudinal_velocity);
    velocity.lateral_velocity = static_cast<float>(current.lateral_velocity);
    velocity.heading_rate = static_cast<float>(current.heading_rate);

    VtdEgoState api_ego;
    api_ego.header.stamp = stamp;
    api_ego.header.frame_id = map_frame_;
    api_ego.ego_x = static_cast<float>(api_position[0]);
    api_ego.ego_y = static_cast<float>(api_position[1]);
    api_ego.ego_z = static_cast<float>(api_position[2]);
    api_ego.ego_heading = static_cast<float>(map_heading);
    api_ego.ego_pitch = data.ego_pitch;
    api_ego.ego_roll = data.ego_roll;
    ego_state_pub_->publish(api_ego);
    odometry_pub_->publish(odometry);
    acceleration_pub_->publish(acceleration);
    velocity_pub_->publish(velocity);

    LocalizationInitializationState localization_state;
    localization_state.stamp = stamp;
    localization_state.state = LocalizationInitializationState::INITIALIZED;
    localization_initialization_state_pub_->publish(localization_state);

    if (
      publish_empty_occupancy_grid_ && occupancy_grid_resolution_ > 0.0 &&
      occupancy_grid_width_ > 0 && occupancy_grid_height_ > 0) {
      const double previous_grid_time = last_occupancy_grid_sim_time_.load();
      if (
        previous_grid_time < 0.0 ||
        timing.ros_time - previous_grid_time >= occupancy_grid_period_sec_) {
        last_occupancy_grid_sim_time_.store(timing.ros_time);
        OccupancyGrid grid;
        grid.header.stamp = stamp;
        grid.header.frame_id = map_frame_;
        grid.info.map_load_time = stamp;
        grid.info.resolution = static_cast<float>(occupancy_grid_resolution_);
        grid.info.width = static_cast<std::uint32_t>(occupancy_grid_width_);
        grid.info.height = static_cast<std::uint32_t>(occupancy_grid_height_);
        grid.info.origin.position.x = position[0] - 0.5 * occupancy_grid_resolution_ *
                                                      static_cast<double>(occupancy_grid_width_);
        grid.info.origin.position.y = position[1] - 0.5 * occupancy_grid_resolution_ *
                                                      static_cast<double>(occupancy_grid_height_);
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(
          static_cast<std::size_t>(occupancy_grid_width_) *
            static_cast<std::size_t>(occupancy_grid_height_),
          0);
        occupancy_grid_pub_->publish(grid);
        ++occupancy_grid_updates_;
      }
    }

    CommandState command;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command = command_;
    }
    SteeringReport steering;
    steering.stamp = stamp;
    // The supplied VTD -> participant interface has no measured steering
    // field. Report the latest accepted command instead of using an
    // undocumented RDB vehicle-system value.
    steering.steering_tire_angle = command.received ? command.steering_angle : 0.0F;
    steering_pub_->publish(steering);

    GearReport gear;
    gear.stamp = stamp;
    gear.report = static_cast<std::uint8_t>(command.gear);
    gear_pub_->publish(gear);

    const bool hazard_enabled = command.hazard_lights == HazardLightsCommand::ENABLE;
    TurnIndicatorsReport turn_indicators;
    turn_indicators.stamp = stamp;
    turn_indicators.report =
      hazard_enabled ? TurnIndicatorsReport::DISABLE : command.turn_indicators;
    turn_indicators_pub_->publish(turn_indicators);
    HazardLightsReport hazard_lights;
    hazard_lights.stamp = stamp;
    hazard_lights.report =
      hazard_enabled ? HazardLightsReport::ENABLE : HazardLightsReport::DISABLE;
    hazard_lights_pub_->publish(hazard_lights);

    ControlModeReport mode;
    mode.stamp = stamp;
    mode.mode = report_autonomous_mode_ ? ControlModeReport::AUTONOMOUS : ControlModeReport::MANUAL;
    control_mode_pub_->publish(mode);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = position[0];
      transform.transform.translation.y = position[1];
      transform.transform.translation.z = position[2];
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void handle_vehicle_systems(const RdbEntryView & entry)
  {
    for_each_element<RDB_VEHICLE_SYSTEMS_t>(entry, [this](const auto & systems) {
      const int player_id = ego_player_id_ >= 0 ? ego_player_id_ : selected_ego_id_.load();
      if (player_id >= 0 && systems.playerId != static_cast<std::uint32_t>(player_id)) {
        return;
      }
      std::lock_guard<std::mutex> lock(vehicle_mutex_);
      vehicle_.steering_angle = systems.steering;
      vehicle_.light_mask = systems.lightMask;
    });
  }

  void handle_drivetrain(const RdbEntryView & entry)
  {
    if (entry.header->elementSize < sizeof(RDB_DRIVETRAIN_BASE_t)) {
      return;
    }
    const auto count = entry.data_size / entry.header->elementSize;
    for (std::size_t index = 0; index < count; ++index) {
      const auto * drivetrain = reinterpret_cast<const RDB_DRIVETRAIN_BASE_t *>(
        entry.data + index * entry.header->elementSize);
      const int player_id = ego_player_id_ >= 0 ? ego_player_id_ : selected_ego_id_.load();
      if (player_id >= 0 && drivetrain->playerId != static_cast<std::uint32_t>(player_id)) {
        continue;
      }
      std::lock_guard<std::mutex> lock(vehicle_mutex_);
      vehicle_.vtd_gear = drivetrain->gear;
    }
  }

  template <typename T, typename Callback>
  static void for_each_element(const RdbEntryView & entry, Callback callback)
  {
    if (entry.header->elementSize < sizeof(T) || entry.header->elementSize == 0U) {
      return;
    }
    const auto count = entry.data_size / entry.header->elementSize;
    for (std::size_t index = 0; index < count; ++index) {
      callback(*reinterpret_cast<const T *>(entry.data + index * entry.header->elementSize));
    }
  }

  void handle_camera_config(const RdbEntryView & entry)
  {
    for_each_element<RDB_CAMERA_t>(entry, [this](const RDB_CAMERA_t & camera) {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      cameras_[camera.id] = camera;
    });
  }

  void handle_images(const RDB_MSG_HDR_t & message, const RdbEntryView & entry)
  {
    if (entry.header->elementSize < sizeof(RDB_IMAGE_t) || entry.header->elementSize == 0U) {
      return;
    }
    const auto count = entry.data_size / entry.header->elementSize;
    for (std::size_t index = 0; index < count; ++index) {
      const auto * element = entry.data + index * entry.header->elementSize;
      const auto * image = reinterpret_cast<const RDB_IMAGE_t *>(element);
      if (camera_id_ >= 0 && image->cameraId != static_cast<std::uint16_t>(camera_id_)) {
        continue;
      }
      const auto payload_capacity = entry.header->elementSize - sizeof(RDB_IMAGE_t);
      if (image->imgSize > payload_capacity || image->height == 0U || image->width == 0U) {
        ++parse_errors_;
        continue;
      }
      publish_image(message, *image, element + sizeof(RDB_IMAGE_t));
    }
  }

  std::optional<std::string> image_encoding(const RDB_IMAGE_t & image) const
  {
    switch (image.pixelFormat) {
      case RDB_PIX_FORMAT_RGB_24:
      case RDB_PIX_FORMAT_RGB8:
        return "rgb8";
      case RDB_PIX_FORMAT_RGBA8:
        return "rgba8";
      case RDB_PIX_FORMAT_BW_8:
      case RDB_PIX_FORMAT_RED8:
        return "mono8";
      case RDB_PIX_FORMAT_DEPTH_16:
      case RDB_PIX_FORMAT_DEPTH16:
      case RDB_PIX_FORMAT_RED16:
        return "16UC1";
      case RDB_PIX_FORMAT_DEPTH_32:
      case RDB_PIX_FORMAT_DEPTH32:
      case RDB_PIX_FORMAT_RED32F:
        return "32FC1";
      default:
        return std::nullopt;
    }
  }

  void publish_image(
    const RDB_MSG_HDR_t & message, const RDB_IMAGE_t & rdb_image, const std::uint8_t * payload)
  {
    const auto encoding = image_encoding(rdb_image);
    if (!encoding) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Unsupported VTD image pixel format %u (pixel size %u)",
        rdb_image.pixelFormat, rdb_image.pixelSize);
      return;
    }
    if (rdb_image.imgSize % rdb_image.height != 0U) {
      ++parse_errors_;
      return;
    }

    Image image;
    image.header.stamp = message_stamp(message.simTime);
    image.header.frame_id = camera_frame_;
    image.width = rdb_image.width;
    image.height = rdb_image.height;
    image.encoding = *encoding;
    image.is_bigendian = false;
    image.step = rdb_image.imgSize / rdb_image.height;
    image.data.resize(rdb_image.imgSize);
    if (!flip_image_vertical_) {
      std::memcpy(image.data.data(), payload, rdb_image.imgSize);
    } else {
      for (std::uint32_t row = 0; row < image.height; ++row) {
        std::memcpy(
          image.data.data() + row * image.step, payload + (image.height - row - 1U) * image.step,
          image.step);
      }
    }
    image_pub_->publish(image);

    CameraInfo info;
    info.header = image.header;
    info.width = image.width;
    info.height = image.height;
    info.distortion_model = "plumb_bob";
    info.d.assign(5U, 0.0);
    RDB_CAMERA_t config{};
    bool have_config = false;
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      const auto found = cameras_.find(rdb_image.cameraId);
      if (found != cameras_.end()) {
        config = found->second;
        have_config = true;
      }
    }
    const double fx = have_config ? config.focalX : camera_fx_;
    const double fy = have_config ? config.focalY : camera_fy_;
    const double cx = have_config
                        ? config.principalX
                        : (camera_cx_ > 0.0 ? camera_cx_ : 0.5 * static_cast<double>(image.width));
    const double cy = have_config
                        ? config.principalY
                        : (camera_cy_ > 0.0 ? camera_cy_ : 0.5 * static_cast<double>(image.height));
    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(info);
    ++image_frames_;
  }

  void handle_rays(const RDB_MSG_HDR_t & message, const RdbEntryView & entry)
  {
    if (entry.header->elementSize < sizeof(RDB_RAY_t) || entry.header->elementSize == 0U) {
      return;
    }
    struct PointXYZI
    {
      float x;
      float y;
      float z;
      float intensity;
    };
    std::vector<PointXYZI> points;
    const auto count = entry.data_size / entry.header->elementSize;
    points.reserve(count);
    int coordinate_type = -1;
    for (std::size_t index = 0; index < count; ++index) {
      const auto * ray =
        reinterpret_cast<const RDB_RAY_t *>(entry.data + index * entry.header->elementSize);
      if (ray->type != RDB_RAY_TYPE_HIT || !std::isfinite(ray->length) || ray->length <= 0.0F) {
        continue;
      }
      if (
        lidar_emitter_id_ >= 0 && ray->emitterId != static_cast<std::uint32_t>(lidar_emitter_id_)) {
        continue;
      }
      if (coordinate_type < 0) {
        coordinate_type = ray->ray.type;
      }
      if (coordinate_type != ray->ray.type) {
        continue;
      }
      const double horizontal = ray->ray.h;
      const double elevation = ray->ray.p;
      double x = ray->ray.x + ray->length * std::cos(elevation) * std::cos(horizontal);
      double y = ray->ray.y + ray->length * std::cos(elevation) * std::sin(horizontal);
      double z = ray->ray.z + ray->length * std::sin(elevation);
      if (coordinate_type == RDB_COORD_TYPE_INERTIAL) {
        const auto mapped = map_position(x, y, z);
        x = mapped[0];
        y = mapped[1];
        z = mapped[2];
      }
      points.push_back(
        PointXYZI{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), 0.0F});
    }
    if (points.empty()) {
      return;
    }

    PointCloud2 cloud;
    cloud.header.stamp = message_stamp(message.simTime);
    if (coordinate_type == RDB_COORD_TYPE_INERTIAL) {
      cloud.header.frame_id = map_frame_;
    } else if (coordinate_type == RDB_COORD_TYPE_PLAYER) {
      cloud.header.frame_id = base_frame_;
    } else {
      cloud.header.frame_id = lidar_frame_;
    }
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = sizeof(PointXYZI);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(4U);
    const std::array<std::string, 4> names{"x", "y", "z", "intensity"};
    for (std::size_t index = 0; index < cloud.fields.size(); ++index) {
      cloud.fields[index].name = names[index];
      cloud.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
      cloud.fields[index].datatype = PointField::FLOAT32;
      cloud.fields[index].count = 1U;
    }
    cloud.data.resize(cloud.row_step);
    std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
    pointcloud_pub_->publish(cloud);
    ++pointcloud_frames_;
  }

  void handle_optix_lidar(
    const RDB_MSG_HDR_t & message, const RdbEntryView & entry, std::size_t & return_index)
  {
    if (entry.header->elementSize < sizeof(RDB_IMAGE_t) || entry.header->elementSize == 0U) {
      return;
    }
    const auto count = entry.data_size / entry.header->elementSize;
    for (std::size_t index = 0; index < count; ++index, ++return_index) {
      const auto * element = entry.data + index * entry.header->elementSize;
      const auto * image = reinterpret_cast<const RDB_IMAGE_t *>(element);
      if (
        optix_return_index_ >= 0 && return_index != static_cast<std::size_t>(optix_return_index_)) {
        continue;
      }
      if (
        optix_camera_id_ >= 0 && image->cameraId != static_cast<std::uint16_t>(optix_camera_id_)) {
        continue;
      }
      const auto capacity = entry.header->elementSize - sizeof(RDB_IMAGE_t);
      if (
        image->pixelFormat != RDB_PIX_FORMAT_RGBA32F || image->imgSize > capacity ||
        image->imgSize % (4U * sizeof(float)) != 0U) {
        continue;
      }
      publish_optix_cloud(message, *image, element + sizeof(RDB_IMAGE_t));
    }
  }

  void publish_optix_cloud(
    const RDB_MSG_HDR_t & message, const RDB_IMAGE_t & image, const std::uint8_t * payload)
  {
    struct PointXYZI
    {
      float x;
      float y;
      float z;
      float intensity;
    };
    std::vector<PointXYZI> points;
    const auto pixel_count = image.imgSize / (4U * sizeof(float));
    points.reserve(pixel_count);
    for (std::size_t index = 0; index < pixel_count; ++index) {
      std::array<float, 4> pixel{};
      std::memcpy(pixel.data(), payload + index * sizeof(pixel), sizeof(pixel));
      if (!std::isfinite(pixel[0]) || !std::isfinite(pixel[1]) || !std::isfinite(pixel[2])) {
        continue;
      }
      std::uint32_t packed{};
      std::memcpy(&packed, &pixel[3], sizeof(packed));
      const float intensity = static_cast<float>((packed >> 16U) & 0xffffU) / 65535.0F;
      if (intensity <= 0.0F && pixel[0] == 0.0F && pixel[1] == 0.0F && pixel[2] == 0.0F) {
        continue;
      }
      const auto mapped = map_position(pixel[0], pixel[1], pixel[2]);
      points.push_back(
        PointXYZI{
          static_cast<float>(mapped[0]), static_cast<float>(mapped[1]),
          static_cast<float>(mapped[2]), intensity});
    }
    if (points.empty()) {
      return;
    }

    PointCloud2 cloud;
    cloud.header.stamp = message_stamp(message.simTime);
    cloud.header.frame_id = map_frame_;
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = sizeof(PointXYZI);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(4U);
    const std::array<std::string, 4> names{"x", "y", "z", "intensity"};
    for (std::size_t index = 0; index < cloud.fields.size(); ++index) {
      cloud.fields[index].name = names[index];
      cloud.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
      cloud.fields[index].datatype = PointField::FLOAT32;
      cloud.fields[index].count = 1U;
    }
    cloud.data.resize(cloud.row_step);
    std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
    pointcloud_pub_->publish(cloud);
    ++pointcloud_frames_;
  }

  void on_control(const Control & control)
  {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_.received = true;
      command_.acceleration = std::clamp(
        control.longitudinal.acceleration, static_cast<float>(min_acceleration_),
        static_cast<float>(max_acceleration_));
      command_.steering_angle = std::clamp(
        control.lateral.steering_tire_angle, static_cast<float>(-max_steering_angle_),
        static_cast<float>(max_steering_angle_));
      command_.last_received = std::chrono::steady_clock::now();
    }
  }

  void on_gear(const GearCommand & gear)
  {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_.gear = gear.command;
    }
  }

  void on_turn_indicators(const TurnIndicatorsCommand & indicators)
  {
    const auto command = indicators.command == TurnIndicatorsCommand::NO_COMMAND
                           ? TurnIndicatorsCommand::DISABLE
                           : indicators.command;
    if (
      command != TurnIndicatorsCommand::DISABLE && command != TurnIndicatorsCommand::ENABLE_LEFT &&
      command != TurnIndicatorsCommand::ENABLE_RIGHT) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring invalid turn-indicator command: %u",
        static_cast<unsigned int>(indicators.command));
      return;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_.turn_indicators = command;
    }
  }

  void on_hazard_lights(const HazardLightsCommand & hazard_lights)
  {
    const auto command = hazard_lights.command == HazardLightsCommand::NO_COMMAND
                           ? HazardLightsCommand::DISABLE
                           : hazard_lights.command;
    if (command != HazardLightsCommand::DISABLE && command != HazardLightsCommand::ENABLE) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring invalid hazard-lights command: %u",
        static_cast<unsigned int>(hazard_lights.command));
      return;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_.hazard_lights = command;
    }
  }

  void send_control()
  {
    if (!control_client_) {
      return;
    }
    CommandState command;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command = command_;
    }
    bool timed_out = !command.received;
    float acceleration = static_cast<float>(watchdog_deceleration_);
    float steering = 0.0F;
    if (command.received) {
      const double age =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - command.last_received)
          .count();
      timed_out = age > control_timeout_sec_;
      acceleration = static_cast<float>(timed_out ? watchdog_deceleration_ : command.acceleration);
      steering = timed_out ? 0.0F : command.steering_angle;
    }
    std::uint8_t turn_signal = 0U;
    if (command.hazard_lights != HazardLightsCommand::ENABLE) {
      if (command.turn_indicators == TurnIndicatorsCommand::ENABLE_LEFT) {
        turn_signal = 1U;
      } else if (command.turn_indicators == TurnIndicatorsCommand::ENABLE_RIGHT) {
        turn_signal = 2U;
      }
    }

    if (control_client_->send_command(steering, acceleration, turn_signal)) {
      watchdog_active_ = timed_out;
    }
  }

  static std::uint8_t autoware_gear(const std::uint8_t vtd_report)
  {
    switch (vtd_report) {
      case RDB_GEAR_BOX_POS_P:
        return GearReport::PARK;
      case RDB_GEAR_BOX_POS_R:
      case RDB_GEAR_BOX_POS_R1:
      case RDB_GEAR_BOX_POS_R2:
      case RDB_GEAR_BOX_POS_R3:
        return GearReport::REVERSE;
      case RDB_GEAR_BOX_POS_N:
        return GearReport::NEUTRAL;
      default:
        return GearReport::DRIVE;
    }
  }

  void publish_diagnostics()
  {
    DiagnosticArray array;
    array.header.stamp = now();
    DiagnosticStatus status;
    status.name = "vtd_ros2_bridge/HLVTD_interface";
    status.hardware_id = "HLVTD";
    const bool control_connected = control_client_ && control_client_->connected();
    const bool data_received = state_frame_received_.load();
    status.level =
      control_connected && data_received ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
    status.message = control_connected && data_received ? "9910 DATA/CONTROL active"
                                                        : "9910 disconnected or no DATA received";
    if (control_connected && data_received && unmapped_traffic_light_ids_.load() > 0U) {
      status.level = DiagnosticStatus::WARN;
      status.message = "9910 active, but traffic-light ID has no mapped stop-line group";
    }
    append_value(status, "control_host", control_host_);
    append_value(status, "control_port", control_port_);
    append_value(status, "control_connected", control_connected ? 1 : 0);
    append_value(status, "participant_data_received", data_received ? 1 : 0);
    append_value(
      status, "control_rx_bytes", control_client_ ? control_client_->received_bytes() : 0U);
    append_value(
      status, "participant_data_packets_decoded",
      control_client_ ? control_client_->decoded_data_packets() : 0U);
    append_value(
      status, "participant_data_packets_skipped",
      control_client_ ? control_client_->skipped_data_packets() : 0U);
    append_value(
      status, "control_packets_queued", control_client_ ? control_client_->queued_commands() : 0U);
    append_value(
      status, "control_packets_sent", control_client_ ? control_client_->sent_commands() : 0U);
    append_value(
      status, "control_packets_overwritten",
      control_client_ ? control_client_->overwritten_commands() : 0U);
    append_value(status, "raw_rdb_tcp_enabled", 0);
    append_value(status, "traffic_light_rdb_enabled", 0);
    append_value(status, "traffic_light_source", std::string{"HLVTD_9910"});
    append_value(status, "traffic_light_id_namespace", std::string{"HLVTD_approach"});
    append_value(status, "traffic_light_id_map_file", traffic_light_id_map_file_);
    append_value(status, "traffic_light_mapped_approaches", traffic_light_group_ids_.size());
    append_value(status, "traffic_light_last_id", last_traffic_light_id_.load());
    append_value(status, "traffic_light_last_state", last_traffic_light_state_.load());
    append_value(status, "steering_feedback_measured", 0);
    append_value(status, "steering_source", std::string{"accepted_command"});
    append_value(status, "object_geometry_uses_rdb", 0);
    append_value(
      status, "object_center_source",
      vehicle_geometry_.enabled ? std::string{"size_classified_vehicle_length_ratio"}
                                : std::string{"participant_xy_direct"});
    append_value(
      status, "object_classification_source",
      pedestrian_classification_.enabled
        ? std::string{"size_templates_pedestrian_then_car_else_unknown"}
        : (vehicle_geometry_.enabled ? std::string{"size_threshold_car_else_unknown"}
                                     : std::string{"all_unknown"}));
    append_value(
      status, "pedestrian_classification_enabled", pedestrian_classification_.enabled ? 1 : 0);
    append_value(
      status, "pedestrian_dimension_tolerance_m",
      pedestrian_classification_.dimension_tolerance_m);
    append_value(
      status, "pedestrian_size_template_count", pedestrian_classification_.dimensions_lwh_m.size() / 3U);
    append_value(status, "vehicle_min_length_m", vehicle_geometry_.min_length_m);
    append_value(status, "vehicle_min_width_m", vehicle_geometry_.min_width_m);
    append_value(status, "vehicle_min_height_m", vehicle_geometry_.min_height_m);
    append_value(
      status, "vehicle_forward_offset_length_ratio", vehicle_geometry_.forward_offset_length_ratio);
    append_value(status, "ego_updates", ego_updates_.load());
    append_value(status, "object_updates", object_updates_.load());
    append_value(status, "traffic_light_updates", traffic_light_updates_.load());
    append_value(status, "autoware_traffic_light_updates", autoware_traffic_light_updates_.load());
    append_value(status, "traffic_light_items", traffic_light_items_.load());
    append_value(status, "autoware_traffic_light_groups", autoware_traffic_light_groups_.load());
    append_value(status, "unmapped_traffic_light_ids", unmapped_traffic_light_ids_.load());
    append_value(status, "objects_dropped", objects_dropped_.load());
    append_value(status, "pointcloud_frames", pointcloud_frames_.load());
    append_value(status, "image_frames", image_frames_.load());
    append_value(status, "occupancy_grid_updates", occupancy_grid_updates_.load());
    append_value(status, "obstacle_pointcloud_updates", obstacle_pointcloud_updates_.load());
    append_value(status, "clock_updates", clock_updates_.load());
    append_value(status, "session_resets", session_resets_.load());
    append_value(status, "sim_time_offset_sec", sim_time_offset_.load());
    append_value(
      status, "control_packets", control_client_ ? control_client_->sent_commands() : 0U);
    append_value(status, "parse_errors", parse_errors_.load());
    append_value(status, "watchdog_active", watchdog_active_.load() ? 1 : 0);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      append_value(status, "turn_indicators_command", command_.turn_indicators);
      append_value(status, "hazard_lights_command", command_.hazard_lights);
    }
    {
      std::lock_guard<std::mutex> lock(vehicle_mutex_);
      append_value(status, "vtd_light_mask", vehicle_.light_mask);
    }
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  std::string control_host_;
  int control_port_{};
  int control_reconnect_delay_ms_{};
  int control_send_period_ms_{};
  int ego_player_id_{};
  std::string ego_name_;
  int camera_id_{};
  int lidar_emitter_id_{};
  int shm_key_{};
  int shm_check_mask_{};
  int optix_return_index_{};
  int optix_camera_id_{};
  std::atomic<int> selected_ego_id_{-1};

  std::string map_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string camera_frame_;
  double map_offset_x_{};
  double map_offset_y_{};
  double map_offset_z_{};
  double map_yaw_offset_{};
  bool flatten_z_{};
  bool publish_clock_{};
  bool anchor_clock_to_system_time_{};
  double clock_reset_threshold_sec_{};
  double clock_restart_gap_sec_{};
  bool publish_tf_{};
  bool flip_image_vertical_{};
  std::string traffic_light_id_map_file_;
  bool publish_unmapped_traffic_light_ids_{};

  double control_timeout_sec_{};
  double watchdog_deceleration_{};
  double min_acceleration_{};
  double max_acceleration_{};
  double max_steering_angle_{};
  bool report_autonomous_mode_{};
  bool report_commanded_gear_{};
  bool report_commanded_lights_{};
  int default_autoware_gear_{GearCommand::DRIVE};
  bool publish_empty_occupancy_grid_{};
  bool publish_empty_obstacle_pointcloud_{};
  double perception_object_max_range_m_{};
  float perception_object_min_height_m_{};
  VehicleGeometryParameters vehicle_geometry_;
  PedestrianSizeParameters pedestrian_classification_;
  double occupancy_grid_resolution_{};
  int occupancy_grid_width_{};
  int occupancy_grid_height_{};
  double occupancy_grid_period_sec_{};
  double camera_fx_{};
  double camera_fy_{};
  double camera_cx_{};
  double camera_cy_{};

  std::unique_ptr<HlvtdControlClient> control_client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<LocalizationInitializationState>::SharedPtr
    localization_initialization_state_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr obstacle_pointcloud_pub_;
  rclcpp::Publisher<VelocityReport>::SharedPtr velocity_pub_;
  rclcpp::Publisher<SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<GearReport>::SharedPtr gear_pub_;
  rclcpp::Publisher<TurnIndicatorsReport>::SharedPtr turn_indicators_pub_;
  rclcpp::Publisher<HazardLightsReport>::SharedPtr hazard_lights_pub_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<VtdEgoState>::SharedPtr ego_state_pub_;
  rclcpp::Publisher<VtdObjectArray>::SharedPtr objects_pub_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr detected_objects_pub_;
  rclcpp::Publisher<VtdTrafficLightArray>::SharedPtr traffic_lights_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr autoware_traffic_lights_pub_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr rviz_velocity_limit_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<Control>::SharedPtr control_sub_;
  rclcpp::Subscription<GearCommand>::SharedPtr gear_sub_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr turn_indicators_sub_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr hazard_lights_sub_;
  rclcpp::Subscription<PathWithLaneId>::SharedPtr road_speed_limit_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr control_send_timer_;

  std::mutex command_mutex_;
  CommandState command_;
  std::mutex vehicle_mutex_;
  VehicleState vehicle_;
  std::mutex ego_pose_mutex_;
  double ego_x_{};
  double ego_y_{};
  double ego_z_{};
  double ego_yaw_{};
  bool ego_pose_received_{};
  std::mutex camera_mutex_;
  std::unordered_map<std::uint16_t, RDB_CAMERA_t> cameras_;
  std::mutex api_mutex_;
  std::vector<VtdObject> pending_objects_;
  std::map<std::int32_t, VtdTrafficLight> traffic_lights_by_id_;
  HlvtdTrafficLightIdMap traffic_light_group_ids_;
  std::unordered_map<std::int32_t, std::int32_t> traffic_light_signal_types_;

  std::mutex time_mutex_;
  bool time_mapping_initialized_{};
  double last_raw_state_sim_time_{};
  double last_ros_state_time_{};
  std::uint32_t last_raw_state_frame_{};
  bool participant_time_initialized_{};
  std::chrono::steady_clock::time_point last_participant_receive_time_{};
  ParticipantMotionState participant_motion_{};
  std::atomic<double> sim_time_offset_{0.0};
  std::atomic<double> last_sim_time_{0.0};
  std::atomic<double> last_occupancy_grid_sim_time_{-1.0};
  std::atomic<std::uint32_t> last_frame_no_{0U};
  std::atomic<std::uint32_t> last_api_publish_frame_{std::numeric_limits<std::uint32_t>::max()};
  std::atomic<std::uint32_t> last_control_queue_frame_{std::numeric_limits<std::uint32_t>::max()};
  std::atomic<bool> state_frame_received_{false};
  std::atomic<std::uint64_t> ego_updates_{0U};
  std::atomic<std::uint64_t> object_updates_{0U};
  std::atomic<std::uint64_t> traffic_light_updates_{0U};
  std::atomic<std::uint64_t> autoware_traffic_light_updates_{0U};
  std::atomic<std::uint64_t> traffic_light_items_{0U};
  std::atomic<std::uint64_t> autoware_traffic_light_groups_{0U};
  std::atomic<std::uint64_t> unmapped_traffic_light_ids_{0U};
  std::atomic<std::int32_t> last_traffic_light_id_{0};
  std::atomic<std::uint8_t> last_traffic_light_state_{0U};
  std::atomic<std::uint64_t> objects_dropped_{0U};
  std::atomic<std::uint64_t> pointcloud_frames_{0U};
  std::atomic<std::uint64_t> image_frames_{0U};
  std::atomic<std::uint64_t> occupancy_grid_updates_{0U};
  std::atomic<std::uint64_t> obstacle_pointcloud_updates_{0U};
  std::atomic<std::uint64_t> clock_updates_{0U};
  std::atomic<std::uint64_t> session_resets_{0U};
  std::atomic<std::uint64_t> parse_errors_{0U};
  std::atomic<bool> watchdog_active_{false};
};

}  // namespace vtd_ros2_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vtd_ros2_bridge::VtdBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
