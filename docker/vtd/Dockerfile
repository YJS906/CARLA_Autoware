# syntax=docker/dockerfile:1.7

ARG AUTOWARE_BASE_IMAGE=ghcr.io/autowarefoundation/autoware@sha256:86d12e0f2504b058b54faead3a6113b5777c778899fc7a7c362e787808fdac24
FROM ${AUTOWARE_BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

USER root
WORKDIR /opt/selfcar-build

# Only the packages that contain local changes are copied into the overlay.
# Unmodified Autoware dependencies remain in the pinned base image.
COPY vtd_overlay/src /opt/selfcar-build/src
COPY config/vtd /opt/selfcar-config
COPY docker/files/cyclonedds.xml /opt/selfcar-config/middleware/cyclonedds.xml
COPY docker/files/default_adapi_vtd.param.yaml /opt/selfcar-config/default_adapi.param.yaml

# The bridge needs VTD's public RDB headers at configure time. The named
# context is read-only and is never copied into the resulting image.
RUN --mount=type=bind,from=vtd,target=/opt/vtd,ro \
    source /opt/ros/jazzy/setup.bash && \
    source /opt/autoware/setup.bash && \
    export VTD_ROOT=/opt/vtd && \
    export CMAKE_BUILD_PARALLEL_LEVEL=1 && \
    export MAKEFLAGS=-j1 && \
    colcon list --base-paths /opt/selfcar-build/src && \
    colcon --log-base /tmp/selfcar_overlay_log build \
      --base-paths /opt/selfcar-build/src \
      --executor sequential \
      --merge-install \
      --install-base /opt/selfcar_overlay \
      --build-base /tmp/selfcar_overlay_build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf /tmp/selfcar_overlay_build /tmp/selfcar_overlay_log

# Wire the fixed-route planner behind one default-off launch argument. The patch script asserts
# every upstream fragment before editing so a changed base-image launch layout fails the build.
COPY docker/files/patch_tier4_planning_launch.py /opt/selfcar-tools/patch_tier4_planning_launch.py
RUN source /opt/ros/jazzy/setup.bash && \
    python3 /opt/selfcar-tools/patch_tier4_planning_launch.py

# Install tracked VTD configuration into the same package-share locations the
# existing Autoware launch files use. This preserves launch arguments while
# making the configuration part of the immutable image layer.
RUN install -Dm644 /opt/selfcar-config/control/autonomous_emergency_braking/autonomous_emergency_braking.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/control/autoware_autonomous_emergency_braking/autonomous_emergency_braking.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/preset/vtd_low_memory_preset.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/preset/vtd_low_memory_preset.yaml && \
    install -Dm644 /opt/selfcar-config/perception/object_recognition/tracking/multi_object_tracker/multi_object_tracker_node.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/perception/object_recognition/tracking/multi_object_tracker/multi_object_tracker_node.param.yaml && \
    install -Dm644 /opt/selfcar-config/perception/object_recognition/tracking/multi_object_tracker/data_association_matrix.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/perception/object_recognition/tracking/multi_object_tracker/data_association_matrix.param.yaml && \
    install -Dm644 /opt/selfcar-config/control/trajectory_follower/lateral/mpc.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml && \
    install -Dm644 /opt/selfcar-config/control/trajectory_follower/longitudinal/pid.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/control/trajectory_follower/longitudinal/pid.param.yaml && \
    install -Dm644 /opt/selfcar-config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/common/autoware_velocity_smoother/JerkFiltered.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/common/autoware_velocity_smoother/JerkFiltered.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/motion_planning/path_smoother/elastic_band_smoother.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/path_smoother/elastic_band_smoother.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/common/common.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/common/common.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/drivable_area_expansion.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/drivable_area_expansion.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/goal_planner/goal_planner.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/goal_planner/goal_planner.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/start_planner/start_planner.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/start_planner/start_planner.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/crosswalk.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/crosswalk.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/intersection.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/intersection.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_cruise.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_cruise.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_slow_down.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_slow_down.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/run_out.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/run_out.param.yaml && \
    install -Dm644 /opt/selfcar-config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/road_user_stop.param.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/road_user_stop.param.yaml && \
    install -Dm644 /opt/selfcar-config/diagnostics/control.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/system/diagnostics/control.yaml && \
    install -Dm644 /opt/selfcar-config/diagnostics/localization.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/system/diagnostics/localization.yaml && \
    install -Dm644 /opt/selfcar-config/diagnostics/system.yaml \
      /opt/autoware/autoware_launch/share/autoware_launch/config/system/diagnostics/system.yaml && \
    install -Dm644 /opt/selfcar-config/rviz/autoware.rviz \
      /opt/autoware/autoware_launch/share/autoware_launch/rviz/autoware.rviz && \
    install -Dm644 /opt/selfcar-config/vehicle/vehicle_info.param.yaml \
      /opt/autoware/sample_vehicle_description/share/sample_vehicle_description/config/vehicle_info.param.yaml && \
    install -Dm644 /opt/selfcar-config/default_adapi.param.yaml \
      /opt/autoware/autoware_default_adapi_universe/share/autoware_default_adapi_universe/config/default_adapi.param.yaml && \
    install -Dm644 /opt/selfcar-config/middleware/cyclonedds.xml /home/aw/cyclonedds.xml

COPY --chmod=755 scripts/set_route_from_csv.py /opt/selfcar-tools/set_route_from_csv.py
COPY --chmod=755 scripts/csv_route_preview.py /opt/selfcar-tools/csv_route_preview.py

LABEL org.opencontainers.image.title="selfcar_2026 VTD Autoware" \
      org.opencontainers.image.description="Autoware VTD overlay built from tracked source" \
      org.opencontainers.image.base.name="${AUTOWARE_BASE_IMAGE}"
