FROM selfcar-2026-vtd:obstacle-stop-recovery-20260909 AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_external_request_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_external_request_lane_change_module
RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/speed-preparation-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner \
      --packages-select autoware_behavior_path_lane_change_module autoware_behavior_path_avoidance_by_lane_change_module autoware_behavior_path_external_request_lane_change_module \
      --executor sequential --merge-install --install-base /tmp/speed-preparation-install \
      --build-base /tmp/speed-preparation-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM selfcar-2026-vtd:obstacle-stop-recovery-20260909
USER root
COPY --from=build /tmp/speed-preparation-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/speed-preparation-install/include/ /opt/selfcar_overlay/include/
COPY --from=build /tmp/speed-preparation-install/share/ /opt/selfcar_overlay/share/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_external_request_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_external_request_lane_change_module
LABEL selfcar.planning.speed_preparation="longitudinal-before-lateral-approval-20260909" \
      selfcar.planning.base_image="2f1e3f57895bc97c48a35e942fb758773fb36cd3fa3fc3789b662d433f7e45f4"
