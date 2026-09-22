# Town05 city scenario

This ScenarioRunner scenario keeps 30 Traffic Manager vehicles circulating in
`Town05_Opt`. Traffic obeys lights and signs and keeps a 3 m following gap.

Run it with a ScenarioRunner-spawned ego vehicle:

```bash
/home/a/carla_pp/scenarios/town05/run_scenario.sh
```

If an `ego_vehicle` already exists (for example, one created by the Autoware
bridge), use:

```bash
/home/a/carla_pp/scenarios/town05/run_scenario.sh --wait-for-ego
```

CARLA must already be running on port 2000 with `Town05_Opt` loaded. When
`--wait-for-ego` is used, ScenarioRunner waits for an actor whose `role_name` is
`ego_vehicle`, then places it at the XML start transform.
