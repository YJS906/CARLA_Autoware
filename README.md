# Autoware - the world's leading open-source software project for autonomous driving

<p align="center">
    <img src="https://user-images.githubusercontent.com/63835446/158918717-58d6deaf-93fb-47f9-891d-e242b02cba7b.png"
        alt="Autoware_RViz" width="75%" />
</p>

<!--- License -->
<p align="center">
    <a href="https://github.com/autowarefoundation/autoware/blob/main/LICENSE">
        <img src="https://img.shields.io/github/license/autowarefoundation/autoware?style=flat&label=License"
            alt="License" /></a>
</p>

<!--- Social Media -->
<p align="center">
    <a href="https://discord.gg/Q94UsPvReQ">
        <img src="https://img.shields.io/discord/953808765935816715?logo=discord&logoColor=white&style=flat&label=Autoware%20Discord"
            alt="Autoware Discord"></a>
    <a href="https://twitter.com/intent/follow?screen_name=AutowareFdn">
        <img src="https://img.shields.io/twitter/follow/AutowareFdn?logo=x&logoColor=white&style=flat"
            alt="Autoware Twitter / X"></a>
    <a href="https://www.linkedin.com/company/the-autoware-foundation/">
        <img src="https://img.shields.io/badge/Linkedin-Autoware%20Foundation-0a66c2?logo=linkedin&logoColor=white&style=flat"
            alt="Autoware Linkedin"></a>
</p>

Autoware is the world's leading open-source autonomous driving framework. Autoware provides a comprehensive, production-ready software stack designed to accelerate the commercial deployment of autonomous vehicles across diverse platforms and use cases.

## 🚀 Get Started

<p align="center">
    <a href="https://autowarefoundation.github.io/autoware-documentation/main/installation">
        <img src="https://img.shields.io/badge/📥_Installation-Get_Autoware_Running-2ea44f?style=for-the-badge"
            alt="Installation" /></a>
    &nbsp;&nbsp;
    <a href="https://autowarefoundation.github.io/autoware-documentation/main/demos/">
        <img src="https://img.shields.io/badge/⚡_Quick_Start-Try_the_Demo-1f6feb?style=for-the-badge"
            alt="Quick Start" /></a>
</p>

> **New to Autoware?**
>
> 1. **[Install Autoware](https://autowarefoundation.github.io/autoware-documentation/main/installation)** → Set up your environment and build the stack from source.
> 2. **[Run the Quick Start demo](https://autowarefoundation.github.io/autoware-documentation/main/demos/)** → Drive a simulated vehicle in just a few minutes.

## Selfcar 2026 VTD runtime

**2026-09-12 최신 백업(브릿지·지도 포함), 구버전 PC 업데이트:**
[Codex 인수인계 및 설치 가이드](docs/update-existing-pc-20260912.md).
고정 태그: `backup-20260912-intersection-exit`.

This branch preserves the local VTD Autoware changes as complete ROS 2 source
packages under `vtd_overlay/src/` and the runtime configuration under
`config/vtd/`. The custom overlay is built into an immutable image; host-built
`.so` files and host source/configuration mounts are not used. The base image
is fixed to
`ghcr.io/autowarefoundation/autoware@sha256:86d12e0f2504b058b54faead3a6113b5777c778899fc7a7c362e787808fdac24`.

The tracked custom overlay packages are `autoware_mission_planner_universe`,
the behavior-path planner, planner-common, lane-change and static-obstacle
modules, `autoware_behavior_velocity_traffic_light_module`, and the local
`vtd_ros2_bridge`. They are based on `autoware_universe` 0.52.0 at
`6e477c645efec33f7909095eea684474e97f5e3d`; the launch configuration baseline
is `autoware_launch` 0.52.0 at
`f942598d44b5769353167c76b784323d5c14c8c7`.

On a new Ubuntu 24.04 + ROS 2 Jazzy/Docker host, obtain the external map,
model, and VTD installation assets, then run:

```bash
git clone https://github.com/DCAM1/selfcar_2026_.git
cd selfcar_2026_
git checkout backup-20260912-intersection-exit
cp .env.example .env
# Edit .env with the external asset paths.
./scripts/verify-reproducibility.sh
./scripts/build-vtd-image.sh
./autoware routes/route_example.csv
```

See [docs/build-and-run.md](docs/build-and-run.md),
[docs/reproducibility-audit.md](docs/reproducibility-audit.md), and
[`vtd_overlay/origin.yaml`](vtd_overlay/origin.yaml) for the exact upstream
commits, external asset manifest, package provenance, and optional bridge
runtime.

The optional CSV argument map-matches a complete route after Autoware is ready:
nearby lanelets are selected using direction and forward topology, every point
is projected to the chosen centerline, sequence 1 is checked against the VTD
ego start, intermediate sequences become checkpoints, and the final sequence
becomes an in-lane goal. Use `./set_route FILE.csv` for RViz preview and to set
or safely change the route of an already running instance. Manual RViz lanelet
overrides can be saved as `corrected_route.csv`; see
[docs/build-and-run.md](docs/build-and-run.md#csv-routes).

Edit a package only under `vtd_overlay/src/`, then rerun
`./scripts/build-vtd-image.sh`; use `./scripts/verify-reproducibility.sh` to
check that source/configuration inputs remain tracked. The former direct host
`.so` mounts and host source/configuration mounts have been removed. Pull
requests run the static checks; the full image build is a manual workflow for a
self-hosted NVIDIA runner.

## CARLA 0.9.16 integration

This repository also contains the CARLA runtime layer developed from the final
VTD-tested image. It runs Autoware against Town01, starts bridge-managed NPC
vehicles, adds continuously roaming pedestrians, and follows the ego vehicle
with the CARLA spectator.

Build the runtime image after building `selfcar-2026-vtd:local`:

```bash
docker build \
  --build-arg AUTOWARE_IMAGE=selfcar-2026-vtd:local \
  -t selfcar-2026-carla:local \
  -f docker/carla/Dockerfile .
```

CARLA itself, the Town01 point-cloud/Lanelet2 map, ML models, and generated
TensorRT engines are external runtime assets and are intentionally not stored
in Git. With CARLA 0.9.16 installed under `/home/a/CARLA/0.9.16` and the map
and models under `/home/a/autoware_data`, run:

```bash
./scripts/carla/carla_run -quality-level=Low
./scripts/carla/carla_autoware start
./scripts/carla/carla_autoware status
```

The original VTD scenario adapter and matching OpenDRIVE are retained under
`tools/carla_vtd_scenario.py` and `config/carla/`. The active Town01 setup does
not use that converted scenario. See
[docs/carla-autoware-town01.md](docs/carla-autoware-town01.md) and
[docs/carla-vtd-scenario.md](docs/carla-vtd-scenario.md).

Town01's supplied Lanelet2 map has no traffic-light, stop-line, or crosswalk
regulatory elements. Vehicle and pedestrian perception and obstacle planning
are available, while traffic-rule behavior needs a CARLA-specific regulatory
map and traffic-signal bridge.

## Documentation

To learn more about using or developing Autoware, refer to the [Autoware documentation site](https://autowarefoundation.github.io/autoware-documentation/main/). You can find the source for the documentation in [autowarefoundation/autoware-documentation](https://github.com/autowarefoundation/autoware-documentation).

## Contributing

<p align="left">
    <a href="https://github.com/autowarefoundation/autoware_universe/graphs/contributors">
        <img src="https://img.shields.io/github/contributors/autowarefoundation/autoware_universe?style=flat&label=Autoware%20Universe%20Contributors"
            alt="Autoware Universe Contributors" /></a>
    <a href="https://github.com/autowarefoundation/autoware/graphs/contributors">
        <img src="https://img.shields.io/github/contributors/autowarefoundation/autoware?style=flat&label=Autoware%20Contributors"
            alt="Autoware Contributors" /></a>
</p>

<!--- Commit Activity -->
<p align="left">
    <a href="https://github.com/autowarefoundation/autoware_universe/pulse">
        <img src="https://img.shields.io/github/commit-activity/m/autowarefoundation/autoware_universe?style=flat&label=Autoware%20Universe%20Commit%20Activity"
            alt="Autoware Universe Activity" /></a>
    <a href="https://github.com/autowarefoundation/autoware/pulse">
        <img src="https://img.shields.io/github/commit-activity/m/autowarefoundation/autoware?style=flat&label=Autoware%20Commit%20Activity"
            alt="Autoware Activity" /></a>
</p>

- Make sure to follow the [Contribution Guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/).
- Take a look at Autoware's [various working groups](https://github.com/autowarefoundation/autoware-projects/wiki#working-group-list) to gain an understanding of any work in progress and to see how projects are managed.
- If you have any technical questions, you can start a discussion in the [Q&A category](https://github.com/autowarefoundation/autoware/discussions/categories/q-a) to request help and confirm if a potential issue is a bug or not.

## Useful resources

<!--- CI Reports -->
<p align="left">
    <a href="https://github.com/autowarefoundation/autoware/actions/workflows/health-check.yaml?query=branch%3Amain">
        <img src="https://img.shields.io/github/actions/workflow/status/autowarefoundation/autoware/health-check.yaml?style=flat&label=health-check"
            alt="health-check CI" /></a>
    <a href="https://app.codecov.io/gh/autowarefoundation/autoware_universe">
        <img src="https://img.shields.io/codecov/c/gh/autowarefoundation/autoware_universe?style=flat&label=Coverage&logo=codecov&logoColor=white"
            alt="Code Coverage" /></a>
</p>

- [Autoware Foundation homepage](https://www.autoware.org/)
- [Support guidelines](https://autowarefoundation.github.io/autoware-documentation/main/community/support/)
- [CI metrics](https://autowarefoundation.github.io/autoware-ci-metrics/)
