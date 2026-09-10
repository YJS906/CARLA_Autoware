# Scoped rebuild on the exact "슬라럼 잘됨" runtime; do not rebuild other planners or the bridge.
ARG BASE_IMAGE=selfcar-2026-vtd:stepwise-lane-choice-20260910
FROM ${BASE_IMAGE} AS build
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY vtd_overlay/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/ /opt/selfcar-build/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/
RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 && \
    colcon --log-base /tmp/intersection-recheck-log build \
      --base-paths /opt/selfcar-build/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module \
      --packages-select autoware_behavior_velocity_intersection_module \
      --executor sequential --merge-install --install-base /tmp/intersection-recheck-install \
      --build-base /tmp/intersection-recheck-build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
RUN g++ -std=c++17 -pthread \
      -I/opt/selfcar-build/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/include \
      /opt/selfcar-build/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/test/test_post_pass_judge.cpp \
      -lgtest_main -lgtest -o /tmp/test-post-pass-judge && /tmp/test-post-pass-judge

FROM ${BASE_IMAGE}
USER root
COPY --from=build /tmp/intersection-recheck-install/lib/ /opt/selfcar_overlay/lib/
COPY --from=build /tmp/intersection-recheck-install/include/ /opt/selfcar_overlay/include/
COPY --from=build /tmp/intersection-recheck-install/share/ /opt/selfcar_overlay/share/
COPY --from=build /opt/selfcar-build/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/ /opt/selfcar-build/src/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/
LABEL selfcar.planning.intersection="collision-stop-recheck-after-pass-judge" \
      selfcar.planning.base_image="c0fad1d29b016de7abc53e7097519c1275aec15d3428d8db54f144f4ed246a30"
