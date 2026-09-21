# CARLA runtime layer for the final VTD-tested Autoware image.
ARG AUTOWARE_IMAGE=selfcar-2026-vtd:local
FROM ${AUTOWARE_IMAGE}

USER root
RUN env -u PIP_CONSTRAINT python3 -m pip install --no-cache-dir --break-system-packages carla==0.9.16

# Expose Autoware's existing traffic-light switch through the e2e simulator
# launch. Official Town01 Lanelet2 data has no complete traffic-light
# regulatory elements, so the CARLA compose setup disables this module.
COPY config/carla/launch/autoware.launch.xml \
  /opt/autoware/autoware_launch/share/autoware_launch/launch/autoware.launch.xml
COPY config/carla/launch/e2e_simulator.launch.xml \
  /opt/autoware/autoware_launch/share/autoware_launch/launch/e2e_simulator.launch.xml
COPY config/carla/launch/autoware_carla_interface.launch.xml \
  /opt/autoware/autoware_carla_interface/share/autoware_carla_interface/launch/autoware_carla_interface.launch.xml

LABEL org.opencontainers.image.title="selfcar_2026 Autoware for CARLA 0.9.16" \
      org.opencontainers.image.description="Final VTD-tested Autoware planner plus the CARLA 0.9.16 Python API"
