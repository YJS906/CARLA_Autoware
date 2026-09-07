#!/usr/bin/env python3
"""Apply the VTD fixed-route planner wiring to the installed tier4 launch package."""

from pathlib import Path


ROOT = Path("/opt/autoware/tier4_planning_launch/share/tier4_planning_launch/launch")
PATCHED_LAUNCH_FILES = (
    "planning.launch.xml",
    "scenario_planning/scenario_planning.launch.xml",
    "scenario_planning/lane_driving.launch.xml",
    "scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml",
    "scenario_planning/lane_driving/motion_planning/motion_planning.launch.xml",
)


def replace_once(path: Path, old: str, new: str) -> None:
    text = path.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"expected one launch fragment in {path}, found {count}: {old[:80]!r}")
    path.write_text(text.replace(old, new, 1))


def patch_planning() -> None:
    path = ROOT / "planning.launch.xml"
    replace_once(
        path,
        '  <arg name="launch_remaining_distance_time_calculator" default="false"/>\n',
        '  <arg name="launch_remaining_distance_time_calculator" default="false"/>\n'
        '  <arg name="launch_fixed_route_obstacle_bypass_planner" default="false"/>\n',
    )
    replace_once(
        path,
        '        <arg name="input_pointcloud_topic_name" value="$(var input_pointcloud_topic_name)"/>\n'
        '      </include>\n'
        '    </group>\n\n'
        '    <!-- diffusion planner module -->',
        '        <arg name="input_pointcloud_topic_name" value="$(var input_pointcloud_topic_name)"/>\n'
        '        <arg name="launch_fixed_route_obstacle_bypass_planner" value="$(var launch_fixed_route_obstacle_bypass_planner)"/>\n'
        '      </include>\n'
        '    </group>\n\n'
        '    <!-- diffusion planner module -->',
    )
    replace_once(
        path,
        '    <!-- planning validator -->\n',
        '    <!-- The final validator uses the trajectory times produced by the global smoother. -->\n'
        '    <group if="$(eval &quot;\'$(var planning_setting)\'==\'rule_based\' and \'$(var launch_fixed_route_obstacle_bypass_planner)\'==\'true\'&quot;)">\n'
        '      <include file="$(find-pkg-share autoware_fixed_route_obstacle_bypass_planner)/launch/fixed_route_obstacle_bypass.launch.xml">\n'
        '        <arg name="launch_planner" value="false"/>\n'
        '        <arg name="launch_final_validator" value="true"/>\n'
        '        <arg name="exclusive_mode_verified" value="true"/>\n'
        '        <arg name="input_objects" value="$(var input_objects_topic_name)"/>\n'
        '      </include>\n'
        '    </group>\n\n'
        '    <!-- planning validator -->\n',
    )
    replace_once(
        path,
        '      <let name="validator_input_trajectory" value="/planning/scenario_planning/velocity_smoother/trajectory" if="$(eval &quot;\'$(var planning_setting)\'==\'rule_based\'&quot;)"/>\n',
        '      <let name="validator_input_trajectory" value="/planning/scenario_planning/velocity_smoother/trajectory" if="$(eval &quot;\'$(var planning_setting)\'==\'rule_based\' and \'$(var launch_fixed_route_obstacle_bypass_planner)\'==\'false\'&quot;)"/>\n'
        '      <let name="validator_input_trajectory" value="/planning/scenario_planning/fixed_route_final_validator/trajectory" if="$(eval &quot;\'$(var planning_setting)\'==\'rule_based\' and \'$(var launch_fixed_route_obstacle_bypass_planner)\'==\'true\'&quot;)"/>\n',
    )


def patch_scenario_planning() -> None:
    path = ROOT / "scenario_planning/scenario_planning.launch.xml"
    replace_once(
        path,
        '  <arg name="is_simulation"/>\n',
        '  <arg name="is_simulation"/>\n'
        '  <arg name="launch_fixed_route_obstacle_bypass_planner" default="false"/>\n',
    )
    replace_once(
        path,
        '        <arg name="input_pointcloud_topic_name" value="$(var input_pointcloud_topic_name)"/>\n'
        '      </include>\n'
        '    </group>\n'
        '    <!-- parking -->',
        '        <arg name="input_pointcloud_topic_name" value="$(var input_pointcloud_topic_name)"/>\n'
        '        <arg name="launch_fixed_route_obstacle_bypass_planner" value="$(var launch_fixed_route_obstacle_bypass_planner)"/>\n'
        '      </include>\n'
        '    </group>\n'
        '    <!-- parking -->',
    )


def patch_lane_driving() -> None:
    path = ROOT / "scenario_planning/lane_driving.launch.xml"
    replace_once(
        path,
        '  <arg name="input_pointcloud_topic_name"/>\n',
        '  <arg name="input_pointcloud_topic_name"/>\n'
        '  <arg name="launch_fixed_route_obstacle_bypass_planner" default="false"/>\n',
    )
    old = '          <arg name="input_pointcloud_topic_name" value="$(var input_pointcloud_topic_name)"/>\n'
    new = old + '          <arg name="launch_fixed_route_obstacle_bypass_planner" value="$(var launch_fixed_route_obstacle_bypass_planner)"/>\n'
    text = path.read_text()
    if text.count(old) != 2:
        raise RuntimeError(f"expected two lane-driving child include fragments in {path}")
    path.write_text(text.replace(old, new))


def gated(module_arg: str) -> str:
    return (
        '$(eval &quot;\'$(var '
        + module_arg
        + ')\'==\'true\' and \'$(var launch_fixed_route_obstacle_bypass_planner)\'==\'false\'&quot;)'
    )


def patch_behavior_planning() -> None:
    path = ROOT / "scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml"
    replace_once(
        path,
        '  <arg name="is_simulation"/>\n',
        '  <arg name="is_simulation"/>\n'
        '  <arg name="launch_fixed_route_obstacle_bypass_planner" default="false"/>\n',
    )
    for module_arg in (
        "launch_static_obstacle_avoidance",
        "launch_avoidance_by_lane_change_module",
        "launch_dynamic_obstacle_avoidance",
        "launch_bidirectional_traffic_module",
        "launch_goal_planner_module",
        "launch_start_planner_module",
        "launch_sampling_planner_module",
        "launch_external_request_lane_change_right_module",
        "launch_external_request_lane_change_left_module",
    ):
        replace_once(path, f'if="$(var {module_arg})"', f'if="{gated(module_arg)}"')
    # These behavior-velocity modules derive stops entirely from detected
    # objects or occupancy. Regulatory writers (traffic lights, stop lines,
    # walkway and merge-from-private) remain active.
    for module_arg in (
        "launch_blind_spot_module",
        "launch_no_stopping_area_module",
        "launch_occlusion_spot_module",
    ):
        replace_once(path, f'if="$(var {module_arg})"', f'if="{gated(module_arg)}"')
    replace_once(
        path,
        '  <node_container pkg="$(var container_package)" exec="$(var container_executable)" name="behavior_planning_container" namespace="" args="" output="both">\n',
        '  <!-- Select a valid parameter file before entering composable-node XML. -->\n'
        '  <let name="fixed_route_behavior_velocity_param_path" value="$(var behavior_velocity_planner_no_drivable_lane_module_param_path)" unless="$(var launch_fixed_route_obstacle_bypass_planner)"/>\n'
        '  <let name="fixed_route_behavior_velocity_param_path" value="$(find-pkg-share autoware_fixed_route_obstacle_bypass_planner)/config/class_free_behavior_velocity_override.param.yaml" if="$(var launch_fixed_route_obstacle_bypass_planner)"/>\n'
        '  <let name="fixed_route_behavior_path_param_path" value="$(var behavior_path_planner_common_param_path)" unless="$(var launch_fixed_route_obstacle_bypass_planner)"/>\n'
        '  <let name="fixed_route_behavior_path_param_path" value="$(find-pkg-share autoware_fixed_route_obstacle_bypass_planner)/config/class_free_behavior_path_override.param.yaml" if="$(var launch_fixed_route_obstacle_bypass_planner)"/>\n\n'
        '  <node_container pkg="$(var container_package)" exec="$(var container_executable)" name="behavior_planning_container" namespace="" args="" output="both">\n',
    )
    replace_once(
        path,
        '      <param from="$(var behavior_velocity_planner_no_drivable_lane_module_param_path)"/>\n',
        '      <param from="$(var behavior_velocity_planner_no_drivable_lane_module_param_path)"/>\n'
        '      <!-- Keep regulatory modules, but remove UNKNOWN/pointcloud obstacle stop ownership. -->\n'
        '      <param from="$(var fixed_route_behavior_velocity_param_path)"/>\n',
    )
    replace_once(
        path,
        '        <param from="$(var behavior_path_planner_common_param_path)"/>\n',
        '        <param from="$(var behavior_path_planner_common_param_path)"/>\n'
        '        <!-- Route lane changes remain enabled; UNKNOWN collision ownership moves downstream. -->\n'
        '        <param from="$(var fixed_route_behavior_path_param_path)"/>\n',
    )


def patch_lane_change_trajectory_safety() -> None:
    path = ROOT / "scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml"
    replace_once(
        path,
        '        <param from="$(var behavior_path_planner_common_param_path)"/>\n',
        '        <param from="$(var behavior_path_planner_common_param_path)"/>\n'
        '        <!-- Share swept-footprint and stop margins with motion velocity planning. -->\n'
        '        <param from="$(var motion_velocity_planner_param_path)"/>\n'
        '        <param from="$(var motion_velocity_planner_obstacle_stop_module_param_path)"/>\n',
    )


def patch_motion_planning() -> None:
    path = ROOT / "scenario_planning/lane_driving/motion_planning/motion_planning.launch.xml"
    replace_once(
        path,
        '  <arg name="interface_output_topic" default="/planning/scenario_planning/lane_driving/trajectory"/>\n',
        '  <arg name="interface_output_topic" default="/planning/scenario_planning/lane_driving/trajectory"/>\n'
        '  <arg name="launch_fixed_route_obstacle_bypass_planner" default="false"/>\n',
    )
    for module_arg in (
        "launch_obstacle_stop_module",
        "launch_obstacle_slow_down_module",
        "launch_obstacle_cruise_module",
        "launch_dynamic_obstacle_stop_module",
        "launch_out_of_lane_module",
        "launch_obstacle_velocity_limiter_module",
        "launch_run_out_module",
        "launch_boundary_departure_prevention_module",
        "launch_road_user_stop_module",
    ):
        replace_once(path, f'if="$(var {module_arg})"', f'if="{gated(module_arg)}"')
    replace_once(
        path,
        '  <!-- plan slowdown or stops on the final trajectory -->\n',
        '  <!-- Class-free path and speed candidates, enabled only with legacy writers excluded. -->\n'
        '  <group if="$(var launch_fixed_route_obstacle_bypass_planner)">\n'
        '    <include file="$(find-pkg-share autoware_fixed_route_obstacle_bypass_planner)/launch/fixed_route_obstacle_bypass.launch.xml">\n'
        '      <arg name="launch_planner" value="true"/>\n'
        '      <arg name="launch_final_validator" value="false"/>\n'
        '      <arg name="exclusive_mode_verified" value="true"/>\n'
        '      <arg name="input_objects" value="$(var input_objects_topic_name)"/>\n'
        '    </include>\n'
        '  </group>\n\n'
        '  <let name="motion_velocity_input_trajectory" value="path_optimizer/trajectory" unless="$(var launch_fixed_route_obstacle_bypass_planner)"/>\n'
        '  <let name="motion_velocity_input_trajectory" value="fixed_route_obstacle_bypass/trajectory" if="$(var launch_fixed_route_obstacle_bypass_planner)"/>\n\n'
        '  <!-- plan slowdown or stops on the final trajectory -->\n',
    )
    replace_once(
        path,
        '        <remap from="~/input/trajectory" to="path_optimizer/trajectory"/>\n',
        '        <remap from="~/input/trajectory" to="$(var motion_velocity_input_trajectory)"/>\n',
    )
    replace_once(
        path,
        '  <group if="$(var launch_surround_obstacle_checker)">\n',
        f'  <group if="{gated("launch_surround_obstacle_checker")}">\n',
    )


def validate_patched_launch_files() -> None:
    # xmllint only checks XML syntax. The ROS frontend additionally validates
    # which attributes are legal for actions such as <param>.
    from launch.frontend import Parser

    for relative_path in PATCHED_LAUNCH_FILES:
        entity, parser = Parser.load(str(ROOT / relative_path))
        parser.parse_description(entity)


def main() -> None:
    patch_planning()
    patch_scenario_planning()
    patch_lane_driving()
    patch_behavior_planning()
    patch_lane_change_trajectory_safety()
    patch_motion_planning()
    validate_patched_launch_files()


if __name__ == "__main__":
    main()
