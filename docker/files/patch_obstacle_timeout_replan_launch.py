#!/usr/bin/env python3
"""Wire the VTD-only object filter without changing the original perception topic."""

from pathlib import Path


def patch(path):
    original = path.read_text()
    if "selfcar_obstacle_timeout_replan" in original:
        return
    text = original
    anchor = '  <arg name="input_objects_topic_name"/>\n'
    assert text.count(anchor) == 1, "planning input argument changed"
    text = text.replace(anchor, anchor + '''  <arg name="enable_obstacle_timeout_replan" default="$(var is_simulation)"/>
  <let name="use_obstacle_timeout_replan" value="$(eval &quot;'$(var enable_obstacle_timeout_replan)'=='true' and '$(var is_simulation)'=='true' and '$(var planning_setting)'=='rule_based' and '$(var launch_fixed_route_obstacle_bypass_planner)'=='false'&quot;)"/>
  <let name="recovery_planning_objects" value="$(var input_objects_topic_name)"/>
  <let name="recovery_planning_objects" value="/planning/obstacle_timeout_replan/objects" if="$(var use_obstacle_timeout_replan)"/>
''', 1)
    anchor = '    <push-ros-namespace namespace="planning"/>\n'
    assert text.count(anchor) == 1, "planning namespace changed"
    text = text.replace(anchor, anchor + '''    <!-- Timed UUID exclusion is confined to the planning object input. -->
    <group if="$(var use_obstacle_timeout_replan)">
      <include file="$(find-pkg-share selfcar_obstacle_timeout_replan)/launch/obstacle_timeout_replan.launch.xml">
        <arg name="input_objects" value="$(var input_objects_topic_name)"/>
        <arg name="output_objects" value="$(var recovery_planning_objects)"/>
      </include>
    </group>
''', 1)
    for include in (
        "$(find-pkg-share tier4_planning_launch)/launch/scenario_planning/scenario_planning.launch.xml",
        "$(find-pkg-share autoware_planning_validator)/launch/planning_validator.launch.xml",
    ):
        start = text.index('<include file="' + include + '">')
        end = text.index("</include>", start)
        block = text[start:end]
        old = '<arg name="input_objects_topic_name" value="$(var input_objects_topic_name)"/>'
        assert block.count(old) == 1, "planning child object input changed"
        block = block.replace(old, '<arg name="input_objects_topic_name" value="$(var recovery_planning_objects)"/>')
        text = text[:start] + block + text[end:]
    # Parse before touching the installed launch file.
    import xml.etree.ElementTree as ET
    ET.fromstring(text)
    path.write_text(text)


def patch_control(path):
    text = path.read_text()
    if "aeb_object_filter" in text:
        return
    anchor = '  <arg name="input_pointcloud_topic_name"/>\n'
    assert text.count(anchor) == 1, "control pointcloud argument changed"
    text = text.replace(anchor, anchor + '''  <arg name="enable_aeb_obstacle_exclusion" default="$(var use_sim_time)"/>
  <let name="use_aeb_obstacle_exclusion" value="$(eval &quot;'$(var enable_aeb_obstacle_exclusion)'=='true' and '$(var use_sim_time)'=='true'&quot;)"/>
  <let name="aeb_pointcloud_topic" value="$(var input_pointcloud_topic_name)"/>
  <let name="aeb_pointcloud_topic" value="/control/aeb_object_filter/pointcloud" if="$(var use_aeb_obstacle_exclusion)"/>
''', 1)
    anchor = '      <group if="$(var launch_autonomous_emergency_braking)">\n'
    assert text.count(anchor) == 1, "AEB launch group changed"
    text = text.replace(anchor, anchor + '''        <group if="$(var use_aeb_obstacle_exclusion)">
          <include file="$(find-pkg-share selfcar_obstacle_timeout_replan)/launch/aeb_object_filter.launch.xml">
            <arg name="input_pointcloud" value="$(var input_pointcloud_topic_name)"/>
            <arg name="input_objects" value="$(var input_objects_topic_name)"/>
            <arg name="output_pointcloud" value="$(var aeb_pointcloud_topic)"/>
          </include>
        </group>
''', 1)
    anchor = '          <composable_node pkg="autoware_autonomous_emergency_braking"'
    start = text.index(anchor)
    end = text.index('</composable_node>', start)
    block = text[start:end]
    old = '<remap from="~/input/pointcloud" to="$(var input_pointcloud_topic_name)"/>'
    assert block.count(old) == 1, "AEB pointcloud remap changed"
    text = text[:start] + block.replace(old, '<remap from="~/input/pointcloud" to="$(var aeb_pointcloud_topic)"/>') + text[end:]
    import xml.etree.ElementTree as ET
    ET.fromstring(text)
    path.write_text(text)


if __name__ == "__main__":
    patch(Path("/opt/autoware/tier4_planning_launch/share/tier4_planning_launch/launch/planning.launch.xml"))
    patch_control(Path("/opt/autoware/tier4_control_launch/share/tier4_control_launch/launch/control.launch.xml"))
