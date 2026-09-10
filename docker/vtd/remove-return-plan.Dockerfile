# Remove only ABLC return-plan reservation logic from the current image; do not roll back.
ARG BASE_IMAGE=selfcar-2026-vtd:direct-return-fallback-20260910
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/manager.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/data_structs.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/data_structs.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/config/avoidance_by_lane_change.param.yaml
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md

RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/remove-return-plan-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner \
      --packages-select autoware_behavior_path_avoidance_by_lane_change_module \
      --executor sequential --merge-install --install-base /tmp/remove-return-plan-install \
      --build-base /tmp/remove-return-plan-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/remove-return-plan-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/remove-return-plan-install/share/ /opt/selfcar_overlay/share/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml
COPY config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml
LABEL selfcar.planning.return_plan="removed" \
      selfcar.planning.mission_return_direct_fallback="ordinary-lane-change-only" \
      selfcar.planning.mission_return_search_budget="removed" \
      selfcar.planning.base_image="87cb33010537035b5444a53dd080dd78b3eb51227ea88b957a96955aed1cb027"
