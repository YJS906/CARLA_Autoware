FROM selfcar-2026-vtd:before-aeb-uuid-exclusion-20260912
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root
COPY vtd_overlay/src/selfcar_obstacle_timeout_replan/ /opt/selfcar-build/src/selfcar_obstacle_timeout_replan/
RUN source /opt/ros/jazzy/setup.bash && source /opt/autoware/setup.bash && source /opt/selfcar_overlay/setup.bash && \
    colcon build --base-paths /opt/selfcar-build/src/selfcar_obstacle_timeout_replan \
      --packages-select selfcar_obstacle_timeout_replan --executor sequential \
      --build-base /tmp/aeb-exclusion-build --install-base /opt/selfcar_overlay --merge-install \
      --cmake-args -DBUILD_TESTING=OFF && rm -rf /tmp/aeb-exclusion-build
COPY docker/files/patch_obstacle_timeout_replan_launch.py /opt/selfcar-tools/patch_obstacle_timeout_replan_launch.py
RUN python3 /opt/selfcar-tools/patch_obstacle_timeout_replan_launch.py
