#!/usr/bin/env python3
"""Install the tracked CARLA configuration into the runtime image only."""
import argparse
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET


def install(config, root):
    def copy(source, destination):
        source = config / source
        target = root / destination.lstrip("/")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    launch_share = "/opt/autoware/autoware_launch/share/autoware_launch"
    bridge_share = "/opt/carla_overlay/share/autoware_carla_interface"
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

    # VTD's UUID exclusion assumes ground-level synthetic object boxes. CARLA
    # uses sensor perception; pass its full obstacle cloud to AEB instead.
    control = root / "opt/autoware/tier4_control_launch/share/tier4_control_launch/launch/control.launch.xml"
    old = '<arg name="enable_aeb_obstacle_exclusion" default="$(var use_sim_time)"/>'
    new = '<arg name="enable_aeb_obstacle_exclusion" default="false"/>'
    content = control.read_text()
    if old in content:
        content = content.replace(old, new)
    elif new not in content:
        raise RuntimeError("Unexpected AEB launch layout; review CARLA cloud wiring")
    ET.fromstring(content)
    control.write_text(content)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("/opt/carla-config"))
    parser.add_argument("--root", type=Path, default=Path("/"))
    args = parser.parse_args()
    install(args.config, args.root)
