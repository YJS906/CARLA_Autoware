# VTD source overlay

This directory contains complete ROS 2 packages whose source was modified in
the local Autoware workspaces. It does not contain `build/`, `install/`, `log/`
or prebuilt shared libraries.

The Autoware packages are based on `autoware_universe` commit
`6e477c645efec33f7909095eea684474e97f5e3d` (tag `0.52.0`). The local changes
were recovered from the clean nested workspace at commit
`b10300980e9009e61721808de13dbc27b414f1a3`. Their package-level provenance
and changed-file lists are in [origin.yaml](origin.yaml).

The external-request lane-change plugin is also built from that exact upstream
snapshot. It derives from the modified lane-change classes, so it must be rebuilt
with them; loading the underlay's prebuilt plugin would mix incompatible C++ layouts.

The `vtd_ros2_bridge` package is local-only and requires the external VTD RDB
headers during the Docker build. `scripts/build-vtd-image.sh` provides those
headers to BuildKit as a read-only named context; they are not copied into the
image or Git repository.

The resulting install space is `/opt/selfcar_overlay`. Runtime setup order is:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/autoware/setup.bash
source /opt/selfcar_overlay/setup.bash
```

To inspect the packages without building the full image:

```bash
colcon list --base-paths vtd_overlay/src
```
