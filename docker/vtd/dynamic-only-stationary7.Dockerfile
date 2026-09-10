# Separate static confirmation from stopped-dynamic reclassification, preserving the last image.
ARG BASE_IMAGE=selfcar-2026-vtd:stop-hold-stationary7-20260910
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/ /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md

RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/dynamic-only-stationary7-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner \
      --packages-select autoware_behavior_path_avoidance_by_lane_change_module \
      --executor sequential --merge-install --install-base /tmp/dynamic-only-stationary7-install \
      --build-base /tmp/dynamic-only-stationary7-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/dynamic-only-stationary7-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/dynamic-only-stationary7-install/share/ /opt/selfcar_overlay/share/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module

COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml

LABEL selfcar.planning.stationary_reclassification="static-original-3s-stopped-dynamic-7s-shared-motion-history" \
      selfcar.planning.base_image="24ff1e47b8c07026db4068758485226ea802ce82d6fab2580890cbcebb0a4592"
