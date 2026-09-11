FROM selfcar-2026-vtd:avoidance-landing-width-20260911 AS build
SHELL ["/bin/bash", "-c"]
COPY vtd_overlay/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/ /opt/selfcar-build/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/ /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/
RUN --mount=type=cache,target=/root/.cache/ccache \
    source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CCACHE_DIR=/root/.cache/ccache MAKEFLAGS=-j2 && \
    colcon build --base-paths \
      /opt/selfcar-build/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module \
      /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common \
      --packages-select autoware_behavior_velocity_traffic_light_module autoware_behavior_path_planner_common \
      --executor sequential --build-base /opt/selfcar-build/traffic-unknown-flashing-build \
      --install-base /opt/selfcar_overlay --merge-install \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
FROM selfcar-2026-vtd:avoidance-landing-width-20260911
COPY --from=build /opt/selfcar_overlay/lib/libautoware_behavior_velocity_traffic_light_module.so /opt/selfcar_overlay/lib/libautoware_behavior_velocity_traffic_light_module.so
COPY --from=build /opt/selfcar_overlay/lib/libautoware_behavior_path_planner_common.so /opt/selfcar_overlay/lib/libautoware_behavior_path_planner_common.so
COPY --from=build /opt/selfcar_overlay/share/autoware_behavior_velocity_traffic_light_module/ /opt/selfcar_overlay/share/autoware_behavior_velocity_traffic_light_module/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/ /opt/selfcar-build/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/ /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light.param.yaml /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light.param.yaml
