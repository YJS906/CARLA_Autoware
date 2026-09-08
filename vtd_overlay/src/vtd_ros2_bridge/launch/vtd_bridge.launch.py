from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("vtd_ros2_bridge"))
        / "config"
        / "vtd_bridge.param.yaml"
    )
    default_lidar_config = str(
        Path(get_package_share_directory("vtd_ros2_bridge"))
        / "config"
        / "vtd_lidar.param.yaml"
    )
    config = LaunchConfiguration("config")
    lidar_config = LaunchConfiguration("lidar_config")
    host = LaunchConfiguration("hlvtd_host")
    control_host = LaunchConfiguration("control_host")
    control_port = LaunchConfiguration("control_port")
    lidar_udp_bind = LaunchConfiguration("lidar_udp_bind")
    lidar_udp_port = LaunchConfiguration("lidar_udp_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("lidar_config", default_value=default_lidar_config),
            DeclareLaunchArgument("hlvtd_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("control_host", default_value=host),
            DeclareLaunchArgument("control_port", default_value="9910"),
            DeclareLaunchArgument("lidar_udp_bind", default_value="0.0.0.0"),
            DeclareLaunchArgument("lidar_udp_port", default_value="9912"),
            # The repository bridge launcher supplies host-persistent state, map
            # and CSV mounts. Plain native launches can opt in with these paths.
            DeclareLaunchArgument("csv_preview_enabled", default_value="false"),
            DeclareLaunchArgument(
                "csv_preview_script", default_value="/opt/selfcar-tools/csv_route_preview.py"
            ),
            DeclareLaunchArgument("csv_preview_state", default_value=""),
            DeclareLaunchArgument("csv_preview_map", default_value=""),
            DeclareLaunchArgument("csv_preview_csv", default_value=""),
            ExecuteProcess(
                cmd=[
                    "python3", LaunchConfiguration("csv_preview_script"),
                    "--state", LaunchConfiguration("csv_preview_state"),
                    "--map", LaunchConfiguration("csv_preview_map"),
                    "--csv", LaunchConfiguration("csv_preview_csv"),
                ],
                name="csv_route_preview",
                output="screen",
                condition=IfCondition(LaunchConfiguration("csv_preview_enabled")),
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="vtd_ros2_bridge",
                executable="vtd_bridge_node",
                name="vtd_bridge",
                output="screen",
                parameters=[
                    config,
                    {
                        "control.host": control_host,
                        "control.port": ParameterValue(
                            control_port, value_type=int
                        ),
                    },
                ],
            ),
            Node(
                package="vtd_ros2_bridge",
                executable="vtd_lidar_node",
                name="vtd_lidar",
                output="screen",
                parameters=[
                    lidar_config,
                    {
                        "lidar_udp.bind_address": lidar_udp_bind,
                        "lidar_udp.port": ParameterValue(
                            lidar_udp_port, value_type=int
                        ),
                    },
                ],
            ),
        ]
    )
