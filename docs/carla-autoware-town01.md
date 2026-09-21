# CARLA Town01 + final Autoware

This setup runs the final Autoware image that was tested with VTD against
CARLA 0.9.16. The VTD bridge is replaced by `autoware_carla_interface`;
the local planner overlay and tracked runtime parameters remain in use.

## Installed assets

- CARLA map: `Town01`
- Autoware map: `/home/a/autoware_data/maps/Town01`
- Autoware image: `selfcar-2026-carla:local`
- Compose file: `/home/a/carla_pp/docker/carla/compose.yaml`
- Background traffic: 30 bridge-managed vehicles and 30 requested walkers
- Sensor mode: one front camera plus LiDAR, IMU, and GNSS

## Run

Start CARLA first, then run from the repository root:

```bash
./scripts/carla/carla_autoware start
```

Useful commands:

```bash
./scripts/carla/carla_autoware status
./scripts/carla/carla_autoware logs
./scripts/carla/carla_autoware stop
```

When RViz finishes loading, use **Init by GNSS**, set a **Goal Pose**, wait
for a trajectory, and press **Engage**. The CARLA spectator follows the
`ego_vehicle`; it does not take control of it.

The supplied Town01 Lanelet2 map has no complete Autoware traffic-light
regulatory elements, so traffic-light recognition is disabled in this setup.
Lane following, routing, vehicles, pedestrians, and LiDAR obstacle perception
can be tested. Full traffic-light behavior requires editing the Lanelet2 map.
