# Reachable current-pose error envelope

Shared by the VTD behavior-path static/LC checks and obstacle_stop. Only the temporary
velocity clock used to decay pose error is bounded by measured speed and maximum
acceleration. It does not modify the executable path, velocities, full nominal
collision corridor, lateral margins, stop margins, or moving-object prediction.

Missing/nonfinite motion data uses the original conservative polygon implementation.
The exact current vehicle footprint, including lateral margin, is retained in the
first two polygons. This is a bounded error envelope, not a kinematic recovery planner.

The 0.1-radian static-avoidance path-connection gate remains independent and unchanged.
Build uses the standard overlay colcon discovery; no LD_PRELOAD or runtime patching.
