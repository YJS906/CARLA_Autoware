# Only the lane-change implementation changes; its public headers and C++ ABI stay unchanged.
# Preserve the already-built intersection recheck and all bridge/configuration changes.
ARG BASE_IMAGE=selfcar-2026-vtd:intersection-recheck-20260910
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp
RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/no-signal-proximity-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module \
      --packages-select autoware_behavior_path_lane_change_module \
      --executor sequential --merge-install --install-base /tmp/no-signal-proximity-install \
      --build-base /tmp/no-signal-proximity-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/no-signal-proximity-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp
LABEL selfcar.planning.traffic_light_proximity_veto="disabled-request-and-waiting-only" \
      selfcar.planning.base_image="2427b6a3ca39fc77b03ea863966aa4303c929805ba61d79d0a44f00a2277d304"
