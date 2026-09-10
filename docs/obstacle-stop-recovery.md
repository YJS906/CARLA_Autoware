# Obstacle-stop candidate selection experiment

The VTD configuration enables `lane_change.obstacle_stop_recovery.enabled`.
The package's generic configuration defaults to `false`.

For the selected module (initially `lane_change_left`), a valid candidate receives
the existing RTC `ACTIVATE` command after the vehicle has been stopped by
`obstacle_stop` continuously for 5 seconds of ROS/simulation time. This selects the
candidate even when its normal safety result is `false` or approval is pending.
The original `is_safe` result is retained; RTC reports a force activation.
This applies to ordinary lane change, not static-obstacle-avoidance modules.

The speed-preparation change adds one exclusion: an unsafe candidate carrying
`speed_preparation_target` must finish longitudinal preparation and pass the safety
checks; this experiment does not force-approve that speed-blocked candidate.
See [lane-change-speed-preparation.md](lane-change-speed-preparation.md).

The trigger uses `/planning/planning_factors/obstacle_stop`, not just a generic
stopped-vehicle timer. It requires a STOP control point with zero velocity at or
behind the vehicle, or no more than 2 m ahead, and ego speed at most 0.1 m/s.
That speed threshold includes the approximately 0.016 m/s residual crawl observed
in VTD. A valid candidate must exist when the command is issued. Missing/stale
messages for more than 0.5 s, an empty stop factor, vehicle motion, signal waiting,
an explicit RTC deactivation, disabling the feature, or a ROS time reset clears
the wait. A paused simulator does not accumulate waiting time.

Signal-queue exclusion uses the existing signal interpretation to find a stopping
signal within 100 m ahead along the current lane sequence. This also excludes following
a vehicle before that signal even if ego is far from the stop line. It is a
conservative signal-queue heuristic, not an inference of another driver's intent;
it depends on the map's signal associations and available signal messages.

Only one module name is configured, so both directions cannot be force-approved
by this feature at the same time. The force command is sent once per candidate
RTC UUID; service failure/timeout permits a retry. The code reads the parameters
on each planning cycle, so the enable switch works through `ros2 param set`.

## Selection and subsequent motion

After RTC accepts the command, the existing lane-change state machine promotes
the candidate from waiting to running. Existing force-activation behavior applies,
including its bypass of lane-change cancellation based on the normal unsafe result.
The lane-change static-obstacle stop insertion, downstream `obstacle_stop`,
trajectory validation, vehicle command gate, and emergency braking are unchanged.
Selecting a colliding candidate can therefore still produce a zero-speed output.

Logs use the `[obstacle_stop_recovery]` prefix and distinguish the command request
from RTC acceptance/rejection. No activation is published merely by displaying a
candidate or by receiving a future obstacle stop while driving.

## Disable and rollback

Inside the running Autoware container, with the ROS and overlay setup files sourced:

```bash
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner lane_change.obstacle_stop_recovery.enabled false
```

This immediately prevents **new** automatic force approvals. An RTC approval
already accepted is not revoked mid-maneuver. To restore the pre-experiment runtime,
stop Autoware, restore the saved image tag, and start Autoware again:

```bash
docker tag selfcar-2026-vtd:before-stop-recovery-20260909-2049 selfcar-2026-vtd:local
```

The source/configuration backup is Git commit `9551f20`, pushed with the exact
message `0909 20:49 emergency breaking 수정` and tag
`backup/20260909-2049-emergency-breaking`. The feature is a separate commit on
`feat/obstacle-stop-valid-path-recovery-20260909`; reverting that commit restores
the source without removing the emergency-braking changes. A full source build
continues to use `./scripts/build-vtd-image.sh`.

For a persistent configuration-only rollback when building an image, set
`enabled: false` in
`config/vtd/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml`
and rebuild. The other options live in the same `obstacle_stop_recovery` block.

## Build and validation record

The focused recovery tests passed (8 tests), including force activation through
RTC with `safe=false`. Broader legacy regression runs did not finish cleanly;
their logs are retained in `log/obstacle_stop_recovery_20260909/`. The user requested
stopping tests and proceeding with the build. No complete regression pass or VTD
driving validation is claimed. The production build uses `BUILD_TESTING=OFF`.

The experimental image rebuilds the lane-change and avoidance-by-lane-change
packages from the backed-up workspace plus this change, on the saved local image.
Both package libraries and their matching headers are installed together.
