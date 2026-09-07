# Avoidance by lane change design

This is a sub-module to avoid obstacles by lane change maneuver.

## Purpose / Role

This module is designed as one of the obstacle avoidance features and generates a lane change path if the following conditions are satisfied.

- Exist lane changeable lanelet.
- Exist avoidance target objects on ego driving lane.

![avoidance_by_lane_change](./images/avoidance_by_lane_change.svg)

## Inner-workings / Algorithms

Basically, this module is implemented by reusing the avoidance target filtering logic of the existing [Static Object Avoidance Module](../autoware_behavior_path_static_obstacle_avoidance_module/README.md) and the path generation logic of the [Normal Lane Change Module](../autoware_behavior_path_lane_change_module/README.md). On the other hand, the conditions under which the module is activated differ from those of a normal avoidance module.

Check that the following conditions are satisfied after the filtering process for the avoidance target.

### Number of the avoidance target objects

This module is launched when the number of avoidance target objects on **EGO DRIVING LANE** is greater than `execute_object_num`. If there are no avoidance targets in the ego driving lane or their number is less than the parameter, the obstacle is avoided by normal avoidance behavior (if the normal avoidance module is registered).

![trigger_1](./images/avoidance_by_lc_trigger_1.svg)

### Lane change end point condition

Unlike the normal avoidance module, which specifies the shift line end point, this module does not specify its end point when generating a lane change path. On the other hand, setting `execute_only_when_lane_change_finish_before_object` to `true` will activate this module only if the lane change can be completed before the avoidance target object.

Although setting the parameter to `false` would increase the scene of avoidance by lane change, it is assumed that sufficient lateral margin may not be ensured in some cases because the vehicle passes by the side of obstacles during the lane change.

![trigger_2](./images/avoidance_by_lc_trigger_2.svg)

## Parameters

### VTD shared safety parameters

Per-class `avoidance_by_lane_change.target_object` values update the inherited avoidance map
instead of being ignored by insertion into existing keys. Unoverridden fields are preserved.
Like static avoidance, hard margins cannot fall below the shared `obstacle_stop` nominal margin
plus the upstream optimization budget. The signed-interval necessity check also includes
centerline-straddling obstacles.

### VTD bilateral selection

The VTD configuration uses `execute_object_longitudinal_margin: 10.0`. This is a **minimum
obstacle distance**, not a 10 m perception horizon or an instruction to wait until the obstacle
is within 10 m. The physical minimum avoidance distance and normal lane-change validity checks
still apply.

Before requesting the exclusive planning slot, the module evaluates candidates in **both legal
route-connected directions**. An occupied immediate lane is still considered by the prediction
checker, but does not suppress evaluation of the other side. Every candidate must also pass a
stationary-object swept-footprint check through the maneuver and the obstacle being passed.
This check uses all perceived stationary objects, independently of their avoidance-target
classification, and does not scan the entire mission route.

Among safe candidates, selection is ordered by proximity to the route's preferred lane, then
the number of lateral lane steps, then the previous unapproved target for stability. The obstacle's
left/right offset is only a final tie breaker. The selected direction, lanes, path and safety
debug state are restored together. An approved maneuver retains its direction and target; ongoing
predicted-path and stationary-footprint safety checks remain active.

An unsafe lane-change candidate cannot claim the slot ahead of a safe static-avoidance fallback.
Static avoidance separately checks both passing corridors against stationary objects and checks
the generated path. Neither mechanism grants permission to override traffic stops or collision
braking. `avoidance bilateral:` INFO logs summarize eligible candidates, safe sides and selection;
DEBUG logs include per-lane predicted/static safety results.

Approval uses the lane-change interface's `WAITING_APPROVAL`/`RUNNING` state machine; this module
does not latch the separate legacy `waitApproval()` flag. A safe RTC-auto candidate can therefore
leave the candidate pool and reach the approved output, while unsafe or RTC-manual candidates
remain waiting. The four-argument RTC updates used by waiting, running and abort planning dispatch
through the direction-aware virtual hook. Only the selected side is registered, and explicit
safety/abort state is preserved. A waiting candidate that changes sides removes its old RTC entry.
A candidate with no selected direction expires; the absence of an RTC request cannot approve it.

`test_approval_handoff.cpp` exercises the real entry/state/RTC dispatch and planner-manager
promotion, with geometry and maneuver completion stubbed. It covers auto left/right output,
manual/unsafe waiting, side changes, missing direction and abort-state propagation. These tests
complement the geometry regressions; they do not replace full scenario-driving validation.

| Name                                               | Unit | Type   | Description                                                                                                                              | Default value |
| :------------------------------------------------- | ---- | ------ | ---------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| execute_object_num                                 | [-]  | int    | Number of avoidance target objects on ego driving lane is greater than this value, this module will be launched.                         | 1             |
| execute_object_longitudinal_margin                 | [m]  | double | Only when the forward avoidance target is farther than this early-decision distance, this module gets the first opportunity to launch.   | 0.0           |
| execute_only_when_lane_change_finish_before_object | [-]  | bool   | If this flag set `true`, this module will be launched only when the lane change end point is **NOT** behind the avoidance target object. | true          |
| max_lane_changing_length_scale                     | [-]  | double | Scale applied to the longitudinal lane-changing distance available to this module.                                                     | 1.0           |
| obstacle_velocity_limit_ratio                      | [-]  | double | Maximum approach/candidate velocity as a ratio of the behavior path planner maximum velocity while an obstacle overlaps the path.      | 1.0           |
| disable_lateral_acceleration_limit                 | [-]  | bool   | Temporarily bypass the PathShifter lateral-acceleration limit for this module only.                                                     | false         |
