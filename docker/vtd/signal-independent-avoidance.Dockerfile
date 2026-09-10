# Remove only the signal-state veto on avoidance candidate planning. Preserve the verified
# occupied-lane lookthrough image, its return scheduling, and all other modules and settings.
ARG BASE_IMAGE=selfcar-2026-vtd:occupied-lane-lookthrough-20260910
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/src/route_policy.cpp
COPY vtd_overlay/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module/README.md
RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 && \
    colcon --log-base /tmp/signal-independent-avoidance-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner \
      --packages-select autoware_behavior_path_avoidance_by_lane_change_module \
      --executor sequential --merge-install --install-base /tmp/signal-independent-avoidance-install \
      --build-base /tmp/signal-independent-avoidance-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/signal-independent-avoidance-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/signal-independent-avoidance-install/share/ /opt/selfcar_overlay/share/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module /opt/selfcar-build/src/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module
LABEL selfcar.planning.signal_state_candidate_veto="false" \
      selfcar.planning.base_image="4032785905c9618fb7139333fda832f89f56dfb712167c73e54fae7f9af81bf8"
