# Reproducible VTD build and run

For the 2026-09-12 backup, including the bridge and map, use
[the existing-PC update guide](update-existing-pc-20260912.md).

This repository's VTD path uses a digest-pinned Autoware CUDA/Jazzy base image
and builds the tracked source overlay into `/opt/selfcar_overlay`. It does not
mount source files or `.so` files from a developer home directory.

## New machine bootstrap

The host needs Ubuntu 24.04, Docker with BuildKit/Buildx, NVIDIA Container
Toolkit, and an X11 display. Obtain the external Autoware maps, ML models and
VTD installation separately; their expected locations are recorded in
`.env.example`.

The VTD bridge build reads only
`Develop/Communication/VtdApi/lib_cxx11/include/VtdToolkit/viRDBIcd.h` and
`Develop/Communication/Common/viRDBTypes.h` from `VTD_INSTALL_DIR`. The runtime
bridge additionally mounts the VTD installation for its simulator connection.

```bash
git clone https://github.com/DCAM1/selfcar_2026_.git
cd selfcar_2026_
git checkout backup-20260912-intersection-exit
cp .env.example .env
# Edit .env: set AUTOWARE_MAP_DIR, AUTOWARE_ML_MODELS_DIR,
# VTD_MAP_DIR, VTD_MAP_RELATIVE_PATH, VTD_INSTALL_DIR and DISPLAY.
./scripts/verify-reproducibility.sh
./scripts/build-vtd-image.sh
./vtd_bridge
./autoware routes/route_example.csv
```

`./autoware` checks that the external map/model directories and the selected
VTD map exist, enables temporary X11 access, runs the planning simulator with
the existing launch arguments, and removes the X11 access rule on exit. When a
CSV path is supplied, it loads that route as soon as the map and VTD
localization are ready.

## CSV routes

Route files require `seq,x,y` columns. Optional `z`, `yaw`, and `lanelet_id`
columns are accepted; `yaw` is a map-matching hint in radians and
`lanelet_id` is a persistent manual override. Rows are sorted by `seq`, which
must be unique and consecutive from 1. Sequence 1 is the expected vehicle
start, the rows in between are checkpoints, and the last sequence is the goal.

The setter never sends the raw CSV coordinates directly. For every row it
finds nearby road lanelets, compares their directed centerlines with the CSV
heading, and uses the routing graph to select one forward-connected sequence
across all rows. Each point is projected onto the selected centerline and its
yaw is replaced with the lanelet direction. Goal validation also checks that
the centered vehicle has lateral clearance inside the selected goal lanelet.

Start Autoware and load the example route automatically:

```bash
./autoware routes/route_example.csv
```

The VTD ego pose supplies Autoware's actual route start. The setter reports the
distance to corrected sequence 1 but does not block submission on that distance
by default. Add `--check-start --start-tolerance 5` when that check is wanted.
It sends corrected sequences 2 through N-1 as waypoints and corrected sequence
N as the goal. An `UNSET` route uses `/api/routing/set_route_points`; a `SET`
route is replaced without clearing it through `/api/routing/change_route_points`.

To replace a route on an Autoware instance that is already running, use a
second terminal:

```bash
./set_route /home/a/route_example.csv
```

The default RViz configuration displays four transient-local previews:

- `/debug/csv/raw_checkpoints`: red X markers for official CSV coordinates.
- `/debug/csv/candidate_lanelets`: yellow lanelet outlines.
- `/debug/csv/selected_lanelets`: the selected connected route in green.
- `/debug/csv/corrected_checkpoints`: blue waypoint dots/arrows and a blue
  star for the final goal.

The four arrays are republished every second by a `csv_route_preview` process
inside the bridge container started by `./vtd_bridge`. The host retains the input CSV and the
exact marker arrays (including interactive lanelet overrides), so closing the
setter terminal or restarting Autoware/RViz does not lose the preview. Marker
lifetime is unlimited, and replay does not depend on simulation time advancing.
The setter exits after the route API call; success or failure does not stop the
preview. If map matching fails, the raw CSV points remain visible without a
fabricated matched route. Direction and goal validation are informational by
default; use `--strict-validation` to make them block route submission.

Every bridge/preview process start rebuilds the preview from
`AUTOWARE_CSV_PREVIEW_CSV` (default: `$HOME/route_example.csv`), replacing the previous
saved visualization even when the filename is unchanged. This refresh is
**visualization only**: it does not call the route API or wait for Autoware, localization,
engagement or a running simulation clock. Raw points are saved/published before map matching;
matching failure leaves those points visible. A missing or invalid CSV preserves the last
saved preview and reports the refresh failure. During the run, explicit CSV loads and user
overrides remain visible; the startup refresh runs only once. An explicit load during startup
cannot be overwritten by the background match result. Starting the bridge again reloads the
configured CSV, including when earlier overrides were saved.

`./autoware` (or the `autoware_run` wrapper) without a CSV argument restores or
initializes **only the visualization**. It never resubmits the saved driving route.
An explicit CSV argument still requests route submission as before. State is
stored under `${XDG_STATE_HOME:-$HOME/.local/state}/selfcar/csv-preview` by default;
set `AUTOWARE_CSV_PREVIEW_DIR` in `.env` to override it. The state is separate from
the image and keyed by map path. The map hash is recorded as provenance; startup refresh
matches against the current map. `--dry-run` and `--no-preview`
do not update saved state. Missing/unreadable bootstrap CSV is reported; it does not prevent
restoring an existing valid saved preview or stop the bridge's other functions.

Autoware and `set_route` reuse the bridge publisher instead of starting a duplicate.
Without a managed bridge they can start the standalone `selfcar-csv-route-preview`
fallback. Bridge startup stops that owned fallback before starting its own publisher,
without deleting the saved data. Autoware/RViz can be stopped and restarted while the bridge
keeps publishing. Stopping the bridge stops its publisher; the saved data survives the next start.

Preview without changing Autoware:

```bash
./set_route official.csv --preview-only
```

To override an ambiguous row, request one or more sequence numbers and click
the desired road with RViz's **Publish Point** tool. The result is saved with
corrected coordinates, lanelet yaw, and fixed `lanelet_id` values:

```bash
./set_route official.csv \
  --override-seq 5 \
  --output-csv corrected_route.csv
```

If `--output-csv` is omitted during an override, the wrapper writes
`corrected_route.csv` in the current directory. Existing output files are
protected unless `--force-output` is supplied. Run `./set_route --help` for
search radius, direction threshold, preview, start-tolerance, and validation
options. CSV files are runtime inputs and do not require an image rebuild.
The launchers read-only mount the tracked CSV Python helpers; the bridge launcher also mounts
its tracked launch file. These changes do not require a C++/image rebuild. Restart the owning
bridge (or standalone preview worker) after changing a running Python process. Starting Autoware
does not restart the bridge. `VTD_BRIDGE_DETACHED=1 ./vtd_bridge [HLVTD_HOST]` starts it without
attaching the terminal; the normal foreground launcher attaches to the same container.

For a native `ros2 launch vtd_ros2_bridge vtd_bridge.launch.py`, CSV preview is opt-in: pass
`csv_preview_enabled:=true`, `csv_preview_script`, `csv_preview_state`, `csv_preview_map` and
`csv_preview_csv` paths. Keep `csv_route_preview.py` and `set_route_from_csv.py` together.
Direct Compose/native launches do not automatically discover host CSV/state paths; the
repository `./vtd_bridge` wrapper provides and mounts them.

## Overlay build

`scripts/build-vtd-image.sh` validates the immutable base-image digest and the
VTD RDB header, then invokes Buildx with the VTD installation as a read-only
named build context. The Dockerfile sources ROS 2 Jazzy and the pinned
`/opt/autoware` installation before running:

```bash
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The overlay build runs packages sequentially with one compiler job. This keeps
peak memory usage safe on the 16 GB target PC; the first build takes longer,
while later builds reuse Docker's layer cache.

The custom package prefixes are expected to resolve to
`/opt/selfcar_overlay` after that setup file is sourced.

To rebuild after changing a tracked package, edit only its complete package
under `vtd_overlay/src/`, run `./scripts/verify-reproducibility.sh`, and then
run `./scripts/build-vtd-image.sh` again. Do not copy generated `build/`,
`install/`, or `.so` files into the repository.

## Optional VTD bridge

The bridge source is built into the same image. Its runtime still needs the
external VTD installation:

```bash
./vtd_bridge
```

`./vtd_bridge` checks `.env`, `VTD_INSTALL_DIR`, and the locally built image,
then starts the bridge through Docker Compose. The HLVTD host defaults to
`HLVTD_HOST` from `.env`, or `127.0.0.1` when the variable is omitted. A
one-time address can be supplied without editing files:

```bash
./vtd_bridge 192.168.0.20
```

The launcher refuses to start while another VTD bridge container is running,
which prevents duplicate ROS topic and TF publishers.

The bridge container mounts only `/opt/vtd` from `VTD_INSTALL_DIR` and the X11
socket. It does not mount the bridge workspace or a host-built library.

## External assets

The following are intentionally outside Git:

| Asset | Environment variable | Expected content |
| --- | --- | --- |
| Autoware maps | `AUTOWARE_MAP_DIR` | map directories used by the base demos |
| VTD map set | `VTD_MAP_DIR` | `VTD_MAP_RELATIVE_PATH`, currently `HL_FMA_VTD_LivingLab_topology_fixed` |
| ML models | `AUTOWARE_ML_MODELS_DIR` | Autoware model artifacts |
| VTD installation | `VTD_INSTALL_DIR` | `Develop/Communication/VtdApi/lib_cxx11/include/VtdToolkit/viRDBIcd.h` and the VTD runtime |

The current machine's map and VTD installation locations, current file names,
and the recovered local-only inputs are recorded in
[docs/reproducibility-audit.md](reproducibility-audit.md).
