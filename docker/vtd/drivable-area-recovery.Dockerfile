# syntax=docker/dockerfile:1.7
ARG BASE_IMAGE=selfcar-2026-vtd:aeb-cloud-frame-20260912
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root
COPY vtd_overlay/src/autoware_universe/planning/autoware_path_optimizer/ /opt/selfcar-build/src/autoware_universe/planning/autoware_path_optimizer/
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/ /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/
# Rebuild subclasses against the changed lane-change class layout as well.
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/ /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_external_request_lane_change_module/ /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_external_request_lane_change_module/
RUN --mount=type=cache,target=/root/.cache/ccache \
    source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CCACHE_DIR=/root/.cache/ccache MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 && \
    colcon build --base-paths \
      /opt/selfcar-build/src/autoware_universe/planning/autoware_path_optimizer \
      /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module \
      /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module \
      /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_external_request_lane_change_module \
      --packages-select autoware_path_optimizer autoware_behavior_path_lane_change_module \
        autoware_behavior_path_avoidance_by_lane_change_module autoware_behavior_path_external_request_lane_change_module \
      --executor sequential --build-base /opt/selfcar-build/drivable-area-recovery-build \
      --install-base /opt/selfcar_overlay --merge-install \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
COPY config/vtd/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml /opt/selfcar-config/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml
RUN install -m644 /opt/selfcar-config/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml && \
    install -m644 /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml
