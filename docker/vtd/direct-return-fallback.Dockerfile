# Add only the direct mission-return fallback on the verified lateral-unlimited image.
ARG BASE_IMAGE=selfcar-2026-vtd:lateral-unlimited-20260910
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/CMakeLists.txt /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/CMakeLists.txt
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/scene.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/scene.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/structs/data.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/structs/data.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/utils/utils.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/utils/utils.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/direct_multi_lane.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/direct_multi_lane.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp
RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/direct-return-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner \
      --packages-select autoware_behavior_path_lane_change_module autoware_behavior_path_avoidance_by_lane_change_module autoware_behavior_path_external_request_lane_change_module \
      --executor sequential --merge-install --install-base /tmp/direct-return-install \
      --build-base /tmp/direct-return-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/direct-return-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/direct-return-install/include/ /opt/selfcar_overlay/include/
COPY --from=build /tmp/direct-return-install/share/ /opt/selfcar_overlay/share/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp
LABEL selfcar.planning.mission_return_direct_fallback="enabled" \
      selfcar.planning.mission_return_search_budget="unchanged" \
      selfcar.planning.base_image="b3c074f48927d30f11433736ec97abc77d6de83fee9d77c2f86fe9f3162396a2"
