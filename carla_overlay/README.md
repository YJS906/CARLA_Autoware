# CARLA adapter overlay

This tracked package snapshots the local Autoware 0.52.0 CARLA interface and
adds the CARLA 0.9.16 adaptation fixes. Image builds must use this directory,
not the developer's ignored `src/` tree.

Build `autoware_carla_interface` on top of `/opt/autoware` and the existing
`/opt/selfcar_overlay`; source its install last. Install these CARLA-only
profiles into the corresponding runtime package shares:

| Repository file | Destination within runtime package share |
| --- | --- |
| `config/carla/vehicle/vehicle_info.param.yaml` | `sample_vehicle_description/config/vehicle_info.param.yaml` |
| `config/carla/sensors/sensor_kit_calibration.yaml` | `carla_sensor_kit_description/config/sensor_kit_calibration.yaml` |
| `config/carla/sensors/sensors_calibration.yaml` | `carla_sensor_kit_description/config/sensors_calibration.yaml` |

Also install the repository's `config/carla/launch/autoware_carla_interface.launch.xml`
into this overlay's package share after building, because the launcher contains
the CARLA map/direct-localization arguments. `carla_localization_ready` and the
current `spectator_follow` are included as executables in the overlay.

The vehicle profile is measured from `vehicle.toyota.prius` in CARLA 0.9.16:
wheelbase 2.819189 m, bounding box 4.513523 × 2.006814 × 1.524833 m, maximum front
steering 70°. Wheel width is descriptive (0.235 m), since CARLA's physics API
does not expose it. Another ego model requires a matching vehicle profile.
Sensors are calibrated in ROS axes at the rear-axle ground projection; the
bridge measures that reference from the spawned vehicle before creating sensors.

The bridge converts radians to normalized CARLA steering, accounts for the
speed-dependent steering limit, translates velocity to the rear axle, and
publishes ROS-handed velocity/yaw rate in SI units. The CARLA steering-curve
speed axis is km/h; the simulator applies its ratio internally. See
[CARLA 0.9.16 NW vehicle implementation, lines 296–301](https://github.com/carla-simulator/carla/blob/0.9.16/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/WheeledVehicleMovementComponentNW.cpp#L296).

Drive/reverse/neutral/park are mapped to CARLA controls. Direction changes wait
until the vehicle stops. Initial/no command, nonfinite commands, and stale
commands apply the service brake. `command_timeout_sec` defaults to 0.5 seconds;
both received wall time and ROS simulation stamp are checked. The synthetic
`AUTONOMOUS` control-mode report still means that the ROS bridge owns the vehicle;
Autoware's command gate controls engagement.

Run regression tests without a CARLA server or vehicle movement from the
repository root:

```bash
docker run --rm --network none --entrypoint bash \
  -e PYTHONDONTWRITEBYTECODE=1 -v "$PWD:/workspace:ro" \
  selfcar-2026-carla:local -lc '
    source /opt/ros/jazzy/setup.bash
    source /opt/autoware/setup.bash
    source /opt/selfcar_overlay/setup.bash
    export PYTHONPATH=/workspace/carla_overlay/src/autoware_carla_interface/src:$PYTHONPATH
    python3 -m unittest discover -s /workspace/carla_overlay/src/autoware_carla_interface/test -v
  '
```

These tests cover conversions, profile/frame consistency, ROS message callbacks,
gear interlocks and timeout braking with fake actors. They do not validate
closed-loop tracking, braking distance, or emergency responses in a moving
scenario; those require separately controlled driving tests.
