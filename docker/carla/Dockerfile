# syntax=docker/dockerfile:1.7
ARG AUTOWARE_IMAGE=selfcar-2026-vtd:local
FROM ${AUTOWARE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root
RUN env -u PIP_CONSTRAINT python3 -m pip install --no-cache-dir --break-system-packages carla==0.9.16

# Rebuild changed packages into a separate CARLA overlay. The VTD image and
# its configuration stay available for the original simulator.
COPY vtd_overlay/src /opt/carla-build/src/vtd
COPY carla_overlay/src /opt/carla-build/src/carla
RUN source /opt/ros/jazzy/setup.bash && \
    source /opt/autoware/setup.bash && \
    source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 && \
    colcon --log-base /opt/carla-build/log build \
      --base-paths /opt/carla-build/src \
      --packages-select autoware_mission_planner_universe \
        autoware_behavior_path_static_obstacle_avoidance_module \
        autoware_drivable_corridor_recovery autoware_carla_interface \
      --executor sequential --merge-install \
      --install-base /opt/carla_overlay --build-base /opt/carla-build/build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    find /opt/carla-build/src -type f -not -path '*/__pycache__/*' -print0 | \
      sort -z | xargs -0 sha256sum > /opt/carla-build/source-manifest.sha256 && \
    rm -rf /opt/carla-build/build

COPY config/carla /opt/carla-config
COPY docker/files/install_carla_runtime.py /opt/carla-build/install_carla_runtime.py
RUN python3 /opt/carla-build/install_carla_runtime.py

COPY --chmod=0755 tools/carla_localization_ready.py \
  /opt/carla_overlay/lib/autoware_carla_interface/carla_localization_ready
COPY --chmod=0755 tools/carla_spectator_follow.py \
  /opt/carla_overlay/lib/autoware_carla_interface/spectator_follow
COPY --chmod=0755 tools/carla_traffic_signal_bridge.py /opt/selfcar-tools/carla_traffic_signal_bridge.py

LABEL org.opencontainers.image.title="selfcar_2026 Autoware for CARLA 0.9.16" \
      org.opencontainers.image.description="VTD planner with rebuilt CARLA adapter, Prius calibration and CARLA planning configuration"
