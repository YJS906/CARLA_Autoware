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

To run the scenario with Autoware, install the external map assets once, start
Autoware, and then attach ScenarioRunner to the bridge-owned ego vehicle:

```bash
/home/a/carla_pp/scripts/carla/install_town05_map
/home/a/carla_pp/scripts/carla/carla_autoware start-town05
/home/a/carla_pp/scenarios/town05/run_scenario.sh --wait-for-ego
```

The supplied CARLA Lanelet2 map does not contain Autoware traffic-light,
stop-line, or crosswalk regulatory elements. The Town05 launcher therefore uses
CARLA ground-truth localization and leaves traffic-light recognition disabled.
