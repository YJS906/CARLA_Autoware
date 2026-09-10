# Incremental build on the verified running image. Preserve its rolled-back static-avoidance
# sources and unrelated bridge/control/map changes; do not recopy the whole host overlay.
ARG BASE_IMAGE=selfcar-2026-vtd:lane-speed-preparation-20260909
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_shifter/shift_constraints.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_shifter/shift_constraints.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_shifter/shift_constraints.hpp /opt/selfcar_overlay/include/autoware/behavior_path_planner_common/utils/path_shifter/shift_constraints.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/data_structs.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/data_structs.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml
RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/approval35-halfshift-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner \
      --packages-select autoware_behavior_path_static_obstacle_avoidance_module autoware_behavior_path_lane_change_module autoware_behavior_path_avoidance_by_lane_change_module autoware_behavior_path_external_request_lane_change_module \
      --executor sequential --merge-install --install-base /tmp/approval35-halfshift-install \
      --build-base /tmp/approval35-halfshift-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/approval35-halfshift-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/approval35-halfshift-install/include/ /opt/selfcar_overlay/include/
COPY --from=build /tmp/approval35-halfshift-install/share/ /opt/selfcar_overlay/share/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_shifter/shift_constraints.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_shifter/shift_constraints.hpp
COPY --from=build /opt/selfcar_overlay/include/autoware/behavior_path_planner_common/utils/path_shifter/shift_constraints.hpp /opt/selfcar_overlay/include/autoware/behavior_path_planner_common/utils/path_shifter/shift_constraints.hpp
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml
LABEL selfcar.planning.approval_distance="35m" \
      selfcar.planning.geometric_shift_length_scale="0.5" \
      selfcar.planning.base_image="f2b1866d7835a39b4a9cef344a3c0a489b8291142d10fc6d06425c177ee4819b"
