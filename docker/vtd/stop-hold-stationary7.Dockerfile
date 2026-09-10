# Narrow follow-up to return-plan removal. Preserve every unrelated binary/configuration.
ARG BASE_IMAGE=selfcar-2026-vtd:remove-return-plan-20260910
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/data_structs.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/data_structs.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md

RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/stop-hold-stationary7-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner \
      --packages-select autoware_behavior_path_avoidance_by_lane_change_module \
      --executor sequential --merge-install --install-base /tmp/stop-hold-stationary7-install \
      --build-base /tmp/stop-hold-stationary7-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/stop-hold-stationary7-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/stop-hold-stationary7-install/share/ /opt/selfcar_overlay/share/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module

COPY config/vtd/control/trajectory_follower/longitudinal/pid.param.yaml /opt/selfcar-config/control/trajectory_follower/longitudinal/pid.param.yaml
COPY config/vtd/control/trajectory_follower/longitudinal/pid.param.yaml /opt/autoware/autoware_launch/share/autoware_launch/config/control/trajectory_follower/longitudinal/pid.param.yaml
COPY config/vtd/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml /opt/selfcar-config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml
COPY config/vtd/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml

LABEL selfcar.control.residual_crawl="brake-keeping-and-stopped-entry-0.05mps" \
      selfcar.planning.stationary_reclassification="7-consecutive-seconds-immediate-motion-reset" \
      selfcar.planning.obstacle_stop_margin="4.5m" \
      selfcar.planning.base_image="59328dfa33e728af535a9435ec66e90f9c0e918ccd275cf1470dd96e7b6118e5"
