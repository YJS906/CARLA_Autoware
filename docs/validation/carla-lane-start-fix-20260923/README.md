# Lane start fix evidence

- `unit-tests.log`: four signed-distance and regression tests.
- `vtd-{baseline,fixed}.log`: identical recorded VTD input, actual old/new planner libraries.
- `carla-{baseline,fixed}.log`: identical current CARLA input, candidate generation and RSS outcome.
- `map-comparison.json`: road lanelet length statistics for both maps.
- `closed-loop-summary.json`: sampled milestones, collision results and clearances for two connected
  drive segments. The second segment inherits the first segment's completed avoidance/return flags;
  it checks only the remaining right turn. It is not a new full avoidance trial.
- `timing-validation.json`: temporary simulation timing change for the continuation.
- `final-world.json`: stopped ego, original timing restored, same world and population.

See [the report](../../carla-lane-start-fix-20260923.md) for limitations.
Raw CDR inputs, harness source/builds, route/actor snapshots, complete drive samples and build logs:
`/home/a/autoware-task-backups/carla-lane-start-fix-20260923/`.
Original successful VTD fixture: `/home/a/autoware-task-backups/request4-left-sweep-20260911.rvtlswqq/`.
