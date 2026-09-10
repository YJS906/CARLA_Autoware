# Lane-change traffic-light proximity veto disabled

The request/approval-waiting check no longer rejects a lane-change candidate solely because a
traffic light is within `max_prepare_duration * max_vel`. This applies to normal and
avoidance-by-lane-change requests through their shared `is_near_regulatory_element()` check.

Unchanged:

- Crosswalk/intersection proximity checks and their existing stopped-ego exception.
- `lane_change.regulation.traffic_light: true` and the shared regulatory-distance helper.
- Regulatory-distance constraints used for candidate length, route departure and stopped recovery.
- Downstream traffic-light stopping, object collision checks and approval safety checks.
- Preparation duration, global speed limit, vehicle margins and the seven-second observation rule.
- Public headers / C++ ABI, bridge and the preceding intersection recheck change.

The scoped image is built with `docker/vtd/no-signal-proximity.Dockerfile` on
`selfcar-2026-vtd:intersection-recheck-20260910`. Updating the launch image does not modify any
already-running container; it takes effect on the next launch.

Removing this veto allows candidate evaluation, not unconditional approval or passage through a
red light. Other lane-change checks can still reject a candidate.

## Focused verification

The saved 2026-09-10 stationary-vehicle snapshot was checked in network-isolated containers,
without replaying messages to the simulator or starting a driving session. With stopped ego,
the previous image returned `Ego is close to regulatory element.`; the new image returns no
lane-change request error. Both retain the signal distance of 44.2616 m for downstream candidate
constraints and accept the supplied eight-second stopped-object history. Crosswalk/intersection
proximity still rejects the non-stopped case.

All 962 installed overlay files were compared against the intersection-recheck base. Only the
lane-change shared library differs; the normal, avoidance and external-request lane-change
plugins all load. These checks verify the removed veto, not complete maneuver safety or approval.
