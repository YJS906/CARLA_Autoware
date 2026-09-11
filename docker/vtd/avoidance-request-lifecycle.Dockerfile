FROM selfcar-2026-vtd:backup-20260911-really-better AS build
SHELL ["/bin/bash", "-c"]
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/ /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/
RUN --mount=type=cache,target=/root/.cache/ccache \
    source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CCACHE_DIR=/root/.cache/ccache MAKEFLAGS=-j2 && \
    colcon build --base-paths \
      /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module \
      --packages-select autoware_behavior_path_avoidance_by_lane_change_module \
      --executor sequential --build-base /opt/selfcar-build/avoidance-request-lifecycle-build \
      --install-base /opt/selfcar_overlay --merge-install \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
FROM selfcar-2026-vtd:backup-20260911-really-better
COPY --from=build /opt/selfcar_overlay/lib/libautoware_behavior_path_avoidance_by_lane_change_module.so /opt/selfcar_overlay/lib/libautoware_behavior_path_avoidance_by_lane_change_module.so
COPY --from=build /opt/selfcar_overlay/share/autoware_behavior_path_avoidance_by_lane_change_module/ /opt/selfcar_overlay/share/autoware_behavior_path_avoidance_by_lane_change_module/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/ /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/
