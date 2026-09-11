# syntax=docker/dockerfile:1.7
FROM selfcar-2026-vtd:drivable-area-recovery-20260912
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/ /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/
RUN --mount=type=cache,target=/root/.cache/ccache \
    source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CCACHE_DIR=/root/.cache/ccache MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 && \
    colcon build --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module \
      --packages-select autoware_behavior_path_lane_change_module --executor sequential \
      --build-base /opt/selfcar-build/intersection-exit-build --install-base /opt/selfcar_overlay --merge-install \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
