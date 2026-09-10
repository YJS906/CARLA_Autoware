# syntax=docker/dockerfile:1.7
ARG BASE_IMAGE=selfcar-2026-vtd:avoidance-request4-20260911
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY vtd_overlay/src/vtd_ros2_bridge/ /opt/selfcar-build/src/vtd_ros2_bridge/
RUN --mount=type=bind,from=vtd,target=/opt/vtd,ro \
    --mount=type=cache,id=selfcar-bridge-height-build,target=/tmp/bridge-height-build \
    source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && \
    source /opt/selfcar_overlay/setup.bash && \
    export VTD_ROOT=/opt/vtd CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/bridge-height-log build \
      --base-paths /opt/selfcar-build/src/vtd_ros2_bridge \
      --packages-select vtd_ros2_bridge --executor sequential --merge-install \
      --install-base /tmp/bridge-height-install --build-base /tmp/bridge-height-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/bridge-height-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/bridge-height-install/include/ /opt/selfcar_overlay/include/
COPY --from=build /tmp/bridge-height-install/share/ /opt/selfcar_overlay/share/
COPY vtd_overlay/src/vtd_ros2_bridge/ /opt/selfcar-build/src/vtd_ros2_bridge/
LABEL selfcar.bridge.min_object_height="exclude-at-or-below-0.1m"
