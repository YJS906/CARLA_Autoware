# HL_FMA VTD scenario in CARLA

`HL_FMA_VTD_YJS01.xml` is a VTD 2025.2 scenario, not an ASAM OpenSCENARIO
file. `carla_vtd_scenario` reads the VTD file directly and adapts it to CARLA
0.9.16 at runtime.

It converts VTD coordinates as follows:

```text
CARLA x = VTD x
CARLA y = -VTD y
CARLA yaw = -degrees(VTD direction)
```

The adapter loads the matching OpenDRIVE, spawns the 71 NPC vehicles, 32
characters, and 15 roadside objects, executes proximity-triggered speed/lane
and character-path actions, and maps the VTD controller programs that also
exist in the OpenDRIVE file to CARLA traffic lights. VTD's OSGB visual model is
not readable by CARLA, so CARLA builds the road surface from OpenDRIVE and does
not show the original VTD buildings.

## Inspect without CARLA

```bash
cd /home/a/carla_pp
./carla_vtd_scenario --dry-run
```

## Preview in CARLA

Start the server:

```bash
/home/a/carla_run -quality-level=Low
```

In a second terminal, load the map, spawn a preview ego, and run the scenario:

```bash
cd /home/a/carla_pp
./carla_vtd_scenario --load-map --spawn-ego
```

Stop the adapter with `Ctrl-C`; it removes the actors it created.

## Run with Autoware

Start the adapter first. It loads the map and then waits for the bridge ego:

```bash
cd /home/a/carla_pp
./carla_vtd_scenario --load-map --wait-for-ego
```

While it waits, no scenario actors are started; the terminal prints
`Waiting for Autoware ego role 'ego_vehicle'`. For an immediate visual check,
use `--spawn-ego` instead. After the map has already been loaded, omit
`--load-map` to avoid rebuilding the navigation mesh:

```bash
./carla_vtd_scenario --spawn-ego
```

In the Autoware container, launch `autoware_carla_interface` with the generated
map already selected and the VTD ego pose converted to CARLA coordinates:

```bash
ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml \
  carla_map:=OpenDriveMap \
  spawn_point:="237.698070,-142.085686,37.2,0.0,0.0,73.293666" \
  use_traffic_manager:=False
```

The CARLA interface owns the synchronous simulation clock. The scenario
adapter observes those ticks and does not call `world.tick()`, so the two
processes do not compete for clock ownership.

The current adapter keeps actor placement, pedestrian paths, proximity
triggers, speed commands, lane-change commands, and controller phase timing.
CARLA vehicle models replace VTD-specific models, bicycles use pedestrian
actors, and CARLA Traffic Manager chooses NPC lane-following routes after
spawn. Those substitutions mean trajectories and collision geometry are not
pixel-for-pixel identical to VTD.

## Verified result

With CARLA 0.9.16, the VTD ego pose projected to the generated driving lane
with a 0.39 m lateral error. A live smoke test spawned all 71 NPC vehicles, all
32 characters, and all 15 objects, then removed all adapter-owned actors on
exit. The corrected OpenDRIVE contains 214 signal controllers that also occur
in the VTD file: 213 have runnable phase programs and one (`116`) has no phase.
The remaining 98 VTD controllers do not occur in this corrected OpenDRIVE and
therefore keep CARLA's generated behavior.
