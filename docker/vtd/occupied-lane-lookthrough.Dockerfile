# Extend avoidance candidate search only. Keep the verified 35 m / half-shift image and its
# static-avoidance rollback, bridge, control, map and other planning modules unchanged.
ARG BASE_IMAGE=selfcar-2026-vtd:approval35-halfshift-20260910
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/scene.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/data_structs.hpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/data_structs.hpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md
RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/occupied-lane-lookthrough-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner \
      --packages-select autoware_behavior_path_avoidance_by_lane_change_module \
      --executor sequential --merge-install --install-base /tmp/occupied-lane-lookthrough-install \
      --build-base /tmp/occupied-lane-lookthrough-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/occupied-lane-lookthrough-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/occupied-lane-lookthrough-install/share/ /opt/selfcar_overlay/share/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module
LABEL selfcar.planning.occupied_lane_lookthrough="true" \
      selfcar.planning.base_image="83e78aeeb9ffb2bad7b9f3784250d94dcf008c196a29256c53543efb6edfeec4"
