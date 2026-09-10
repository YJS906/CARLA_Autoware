# Targeted rebuild on the deployed image; do not copy unrelated host overlay rollbacks.
ARG BASE_IMAGE=selfcar-2026-vtd:signal-independent-avoidance-20260910
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/structs/parameters.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/structs/parameters.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/utils/calculation.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/utils/calculation.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/curvature_braking.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/curvature_braking.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/calculation.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/calculation.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/path.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/path.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/low_speed.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/low_speed.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/config/lane_change.param.yaml
RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/lateral-unlimited-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner \
      --packages-select autoware_behavior_path_lane_change_module autoware_behavior_path_avoidance_by_lane_change_module autoware_behavior_path_external_request_lane_change_module \
      --executor sequential --merge-install --install-base /tmp/lateral-unlimited-install \
      --build-base /tmp/lateral-unlimited-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/lateral-unlimited-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/lateral-unlimited-install/include/ /opt/selfcar_overlay/include/
COPY --from=build /tmp/lateral-unlimited-install/share/ /opt/selfcar_overlay/share/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml
LABEL selfcar.planning.lateral_acceleration_limit="disabled" \
      selfcar.planning.lateral_jerk_limit="disabled" \
      selfcar.planning.lateral_jerk_calculation="phase-specific-signed-derivative" \
      selfcar.planning.base_image="db0d9a154ae0220bccadfe9c647e682730a6b4b4c1524997567f8d80cd432779"

