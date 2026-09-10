# Intersection collision recheck after pass-judge commitment

Baseline: Git tag `슬라럼-잘됨`, commit `3337157`, runtime image
`sha256:c0fad1d29b016de7abc53e7097519c1275aec15d3428d8db54f144f4ed246a30`.
The complete intersection package is imported from upstream 0.52.0, commit
`6e477c645efec33f7909095eea684474e97f5e3d`.

## Behavior

Object prediction/collision evaluation already ran every cycle after pass-judge commitment.
The old permanent-GO branch returned `OverPassJudge` even when that evaluation found a
collision, so RTC stayed safe and no intersection STOP planning factor was produced.

Both standard and experimental plugins now route the existing collision state machine's
STOP result to `NonOccludedCollisionStop` even after commitment. A reachable collision
stop line is retained; if it is behind ego or infeasible under the existing normal braking
limits, the module requests zero velocity from the closest path point instead of ignoring
the collision. Physical deceleration limits and downstream emergency protection still apply;
this late braking request is not a guarantee that a collision can be avoided.

The existing collision-clear hold (currently 0.5 seconds in VTD config) also applies after
commitment. Once it expires, the normal clear-corridor result allows progress, and subsequent
new collisions can request STOP again. The historical pass-judge flag is not reset.

Object classification, prediction confidence, attention-lane geometry, traffic-control filtering,
occlusion, RTC/manual overrides, bridge acceleration estimation, and lane-change/avoidance
logic are unchanged. This does not add a second collision search or change early returns
for invalid intersection geometry/missing stop lines. It fixes the permanent-GO override,
not every possible reason an intersection check can be skipped.

## Build and deployment

The normal `docker/vtd/Dockerfile` discovers the newly tracked package automatically.
For a scoped rebuild preserving the exact working baseline:

```bash
docker build -f docker/vtd/intersection-recheck.Dockerfile \
  -t selfcar-2026-vtd:intersection-recheck-20260910 .
```

The scoped build runs only the five focused post-pass decision tests, not scenario driving
or bag replay. Select the resulting image for the next normal launch; do not copy libraries
into or restart a running Autoware session. Full closed-loop behavior still requires a
separate simulator run after restart.

## Verification on 2026-09-10

- Scoped package build completed; all five post-pass decision tests passed.
- In a network-isolated container, the package resolved to `/opt/selfcar_overlay` and
  its shared library loaded successfully with all dynamic dependencies resolved.
- Compared 1,149 existing overlay library/header/share and runtime configuration files
  against the baseline: zero changed or missing; 54 intersection-package files added.
- Image: `selfcar-2026-vtd:intersection-recheck-20260910`,
  `sha256:2427b6a3ca39fc77b03ea863966aa4303c929805ba61d79d0a44f00a2277d304`.
- Selected as `selfcar-2026-vtd:local` for the next `./autoware_run` launch.
  Existing Autoware/bridge containers were not modified or restarted.
- Build log and checksum inventories:
  `/home/a/autoware-task-backups/intersection-recheck-20260910.JeTKo8/`.
