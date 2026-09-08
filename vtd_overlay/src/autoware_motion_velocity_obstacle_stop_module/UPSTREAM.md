# Upstream provenance

Imported from autowarefoundation/autoware_core tag 1.9.0, commit
f25f83c632c1984ec276c894c41857d4abc0dad8, package
planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module.

Local change: pass measured ego motion to the shared selfcar_trajectory_safety
current-pose envelope helper for nominal-margin and zero-margin filtering polygons.
Collision/stop margins, object filtering policy and stop planning remain unchanged.
