#!/usr/bin/env python3
"""Install the tracked CARLA configuration into the runtime image only."""
import argparse
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET


def install_ground_truth_launch(root):
    """Keep one tracking source and the original prediction/pointcloud pipeline."""
    component = root / "opt/autoware/autoware_launch/share/autoware_launch/launch/components/tier4_perception_component.launch.xml"
    perception = root / "opt/autoware/tier4_perception_launch/share/tier4_perception_launch/launch/perception.launch.xml"
    for path in (component, perception):
        tree = ET.parse(path)
        launch = tree.getroot()
        if launch.find("arg[@name='carla_perception_mode']") is None:
            argument = ET.Element("arg", name="carla_perception_mode", default="sensor")
            for choice in ("sensor", "ground_truth"):
                ET.SubElement(argument, "choice", value=choice)
            launch.insert(0, argument)
        if path == component:
            includes = [node for node in launch.iter("include")
                        if node.get("file", "").endswith("/launch/perception.launch.xml")]
            if len(includes) != 1:
                raise RuntimeError("Unexpected perception component layout")
            if includes[0].find("arg[@name='carla_perception_mode']") is None:
                ET.SubElement(includes[0], "arg", name="carla_perception_mode",
                              value="$(var carla_perception_mode)")
        else:
            for namespace in ("detection", "tracking"):
                groups = [node for node in launch.iter("group")
                          if node.find(f"push-ros-namespace[@namespace='{namespace}']") is not None]
                if len(groups) != 1:
                    raise RuntimeError(f"Unexpected {namespace} launch layout")
                condition = '$(eval "\'$(var carla_perception_mode)\' == \'ground_truth\'")'
                if groups[0].get("if") or groups[0].get("unless") not in (None, condition):
                    raise RuntimeError(f"Unexpected {namespace} launch condition")
                groups[0].set("unless", condition)
        ET.indent(tree, space="  ")
        tree.write(path, encoding="UTF-8", xml_declaration=True)


def install(config, root):
    def copy(source, destination):
        source = config / source
        target = root / destination.lstrip("/")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    launch_share = "/opt/autoware/autoware_launch/share/autoware_launch"
    bridge_share = "/opt/carla_overlay/share/autoware_carla_interface"
    copy("control/trajectory_follower/longitudinal/pid.param.yaml",
         launch_share + "/config/control/trajectory_follower/longitudinal/pid.param.yaml")
    for name in ("autoware.launch.xml", "e2e_simulator.launch.xml"):
        ET.parse(config / "launch" / name)
        copy("launch/" + name, launch_share + "/launch/" + name)
    ET.parse(config / "launch/autoware_carla_interface.launch.xml")
    copy("launch/autoware_carla_interface.launch.xml", bridge_share + "/launch/autoware_carla_interface.launch.xml")
    copy("diagnostics/autoware-carla-direct.yaml", launch_share + "/config/system/diagnostics/autoware-carla-direct.yaml")
    for source in (config / "planning").rglob("*.yaml"):
        relative = source.relative_to(config)
        copy(relative, launch_share + "/config/" + relative.as_posix())
    copy("vehicle/vehicle_info.param.yaml", "/opt/autoware/sample_vehicle_description/share/sample_vehicle_description/config/vehicle_info.param.yaml")
    for source in (config / "sensors").glob("*.yaml"):
        copy(source.relative_to(config), "/opt/autoware/carla_sensor_kit_description/share/carla_sensor_kit_description/config/" + source.name)

    # GT-specific AEB input is additive to the untouched real LiDAR cloud.
    copy("control/autonomous_emergency_braking/ground_truth.param.yaml",
         launch_share + "/config/control/autonomous_emergency_braking/ground_truth.param.yaml")

    # VTD's UUID exclusion assumes ground-level synthetic object boxes. CARLA
    # uses sensor perception; pass its full obstacle cloud to AEB instead.
    control = root / "opt/autoware/tier4_control_launch/share/tier4_control_launch/launch/control.launch.xml"
    install_ground_truth_launch(root)
    control_component = root / "opt/autoware/autoware_launch/share/autoware_launch/launch/components/tier4_control_component.launch.xml"
    component_tree = ET.parse(control_component)
    component_launch = component_tree.getroot()
    if component_launch.find("arg[@name='carla_perception_mode']") is None:
        component_launch.insert(0, ET.Element("arg", name="carla_perception_mode", default="sensor"))
    includes = [node for node in component_launch.iter("include")
                if node.get("file", "").endswith("/launch/control.launch.xml")]
    if len(includes) != 1:
        raise RuntimeError("Unexpected control component layout")
    if includes[0].find("arg[@name='carla_perception_mode']") is None:
        ET.SubElement(includes[0], "arg", name="carla_perception_mode", value="$(var carla_perception_mode)")
    ET.indent(component_tree, space="  ")
    component_tree.write(control_component, encoding="UTF-8", xml_declaration=True)
    tree = ET.parse(control)
    launch = tree.getroot()
    exclusion = launch.find("arg[@name='enable_aeb_obstacle_exclusion']")
    if exclusion is None or exclusion.get("default") not in ("false", "$(var use_sim_time)"):
        raise RuntimeError("Unexpected AEB launch layout; review CARLA cloud wiring")
    exclusion.set("default", "false")
    if launch.find("arg[@name='carla_perception_mode']") is None:
        launch.insert(0, ET.Element("arg", name="carla_perception_mode", default="sensor"))
    aeb_nodes = launch.findall(".//composable_node[@name='autonomous_emergency_braking']")
    aeb_groups = [node for node in launch.iter("group")
                  if node.get("if") == "$(var launch_autonomous_emergency_braking)"]
    if len(aeb_nodes) != 1 or len(aeb_groups) != 1:
        raise RuntimeError("Unexpected AEB node layout")
    if aeb_groups[0].find("let[@name='carla_aeb_override']") is None:
        # Sensor mode re-applies its own base file: no sensor-mode parameter is changed.
        aeb_groups[0].insert(0, ET.Element("let", name="carla_aeb_override",
                                          value="$(var aeb_param_path)"))
        override = ET.Element("let", name="carla_aeb_override",
                              value="$(find-pkg-share autoware_launch)/config/control/autonomous_emergency_braking/ground_truth.param.yaml")
        override.set("if", '$(eval "\'$(var carla_perception_mode)\' == \'ground_truth\'")')
        aeb_groups[0].insert(1, override)
        base = aeb_nodes[0].find("param[@from='$(var aeb_param_path)']")
        if base is None:
            raise RuntimeError("Missing AEB base parameter file")
        index = list(aeb_nodes[0]).index(base)
        aeb_nodes[0].insert(index + 1, ET.Element("param", {"from": "$(var carla_aeb_override)"}))
    ET.indent(tree, space="  ")
    tree.write(control, encoding="UTF-8", xml_declaration=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("/opt/carla-config"))
    parser.add_argument("--root", type=Path, default=Path("/"))
    args = parser.parse_args()
    install(args.config, args.root)
