# Town05 city and proximity-event scenarios

This ScenarioRunner scenario starts ordinary Traffic Manager traffic in
`Town05_Opt`, targeting 30 vehicles by default. It configures NPCs to observe
lights and signs and requests a 3 m following gap. If the Autoware bridge
already created background traffic, the scenario configures it and only adds
the missing vehicles. Available spawn locations may limit the total.

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

## Edit basic settings

Open `town05_city.xml` and change the values near the top:

- `vehicle_count`: target number of ordinary traffic vehicles; only missing
  vehicles are added. Existing vehicles are never removed by this setting, so
  reducing it below the bridge's existing count will not reduce that traffic.
- `following_distance`: requested NPC following distance in metres
- `speed_difference`: percentage slower than the road limit; a negative value is faster
- `automatic_lane_change`: `true` or `false`
- `spawn_clearance`: exclusion radius for new NPC spawn positions around the
  ego, in metres; it does not move or exclude existing vehicles
- `scenario_duration`: run time in simulation seconds

The `ego_vehicle` block controls the ego start position, and the `weather`
block controls clouds, rain, wind, and sun. Stop and rerun the scenario after
saving the XML file. With `--wait-for-ego`, the existing ego is moved back to
the XML start position on each run, so stop the vehicle and put Autoware in
Stop mode before restarting the scenario. The XML model does not replace an
existing bridge-owned ego vehicle.

These traffic settings apply to NPCs, not Autoware's ego speed or route. The
default `town05_city.xml` implements background vehicle traffic and ego collision
checking. The separate `town05_hazards.xml` enables a pedestrian dart-out and a
vehicle cut-in, implemented in `town05_events.py`. These two events are optional;
a scripted sudden-braking event is not included.

Open the settings in the installed editor:

```bash
gedit /home/a/carla_pp/scenarios/town05/town05_city.xml
```

For example, `speed_difference=30` requests 35 km/h on a road with a 50 km/h
limit. It is a desired speed, subject to traffic and signals, not a fixed speed.

## 보행자 돌발 횡단과 차량 끼어들기

`town05_hazards.xml`은 일반 교통에 두 가지 접근 감지 이벤트를 추가합니다.
기존 `town05_city.xml`의 시작 위치와 일반 교통 설정은 유지됩니다.
새 시나리오는 Town05 동쪽 가장자리의 편도 3차로 구간에서 시작하며,
ego는 가장 오른쪽 차로에서 CARLA 좌표의 -y 방향으로 진행합니다.

- 시작점 약 40m 앞: 사람이 오른쪽 인도를 왕복해서 걷다가 ego가 접근하면
  차도를 가로질러 중앙 갓길로 뜁니다. 반대편 차도까지 건너지는 않습니다.
- 시작점 약 120m 앞: 왼쪽 차로에 있는 차량이 ego가 접근하면 출발하고,
  거리가 더 가까워지면 ego 차로로 오른쪽 끼어들기를 합니다.

Autoware와 CARLA `Town05_Opt`가 실행 중이고 `ego_vehicle`이 준비된 상태에서:

1. ego를 정지시키고 Autoware를 **STOP** 상태로 바꿉니다.
2. 기존 시나리오 터미널에서 `Ctrl+C`로 종료합니다.
3. 다음 명령으로 새 시나리오를 실행합니다.

```bash
/home/a/carla_pp/scenarios/town05/run_scenario.sh --hazards --wait-for-ego
```

**ScenarioRunner는 실행할 때마다 기존 ego를 XML의 시작 위치로 이동시킵니다.**
위치 변경 후 RViz에서 위치를 확인하고, 시작 차로를 따라 앞쪽으로 진행하는
경로를 다시 지정하세요. 일반 교통 시나리오로 돌아가려면 같은 종료 절차 후
`--hazards`를 빼고 실행합니다. Autoware 없이 ScenarioRunner가 ego를 생성하게
하려면 `--wait-for-ego`를 뺄 수 있지만, 이 시나리오가 ego를 자동 운전하지는 않습니다.

이벤트는 단순히 가까이 있다는 이유만으로 실행되지 않습니다. ego가 각 이벤트의
**지정된 도로·차로에서, 같은 진행 방향을 향해, 앞으로 주행하며 접근**해야 합니다.
구현은 진행 방향 속도 0.2m/s 이상을 요구합니다. 옆 차로, 반대 방향, 정지 상태,
이벤트 지점을 이미 지나친 상태에서는 시작하지 않습니다. 기본 위치의 대상 차로는
보행자가 road 38 / lane -3, 끼어들기가 road 34 / lane -3입니다.

각 이벤트는 실행당 한 번 발생합니다. 완료 후 배경 교통은 계속 실행되며,
다시 시험하려면 ego를 STOP으로 바꾸고 시나리오를 종료한 뒤 재실행하세요.
종료 시 시나리오가 생성한 이벤트 차량·보행자와 배경 차량을 정리하며,
브리지가 기존에 생성했던 배경 차량은 유지합니다.

### 이벤트 설정 수정

```bash
gedit /home/a/carla_pp/scenarios/town05/town05_hazards.xml
```

각 이벤트의 `enabled="true"`를 `enabled="false"`로 바꾸면 해당 이벤트만 끕니다.
파일 저장 후 위 순서로 시나리오를 다시 실행해야 적용됩니다.
일반 교통의 차량 수와 속도도 이 파일 위쪽에서 수정할 수 있으며,
`vehicle_count`에는 별도로 생성하는 끼어들기 차량과 보행자가 포함되지 않습니다.

| 보행자 설정 | 기본값 | 의미 |
|---|---:|---|
| `trigger_distance` | 25 | ego가 횡단 기준점까지 전방 거리 25m 이내로 접근하면 횡단 시작 |
| `walk_speed` | 1.2 | 인도 보행 속도, m/s |
| `run_speed` | 3.5 | 횡단 속도, m/s; 약 12.6km/h |
| `walk_distance` | 6 | 인도 왕복 구간의 전체 길이, m |
| `side` | `right` | ego 진행 방향 기준으로 보행자를 배치할 인도 방향 |
| `end_x`, `end_y` | XML 참조 | 횡단이 끝나는 중앙 갓길의 CARLA 좌표 |

| 끼어들기 설정 | 기본값 | 의미 |
|---|---:|---|
| `approach_distance` | 35 | ego가 고정된 이벤트 기준점 전방 35m 이내로 오면 옆 차로 차량 출발 |
| `trigger_distance` | 16 | 출발한 차량과 ego의 전방 거리가 16m 이내가 되면 차선 변경 시작 |
| `approach_speed_kmh` | 12 | 차선 변경 전 옆 차로 주행 목표 속도, km/h |
| `speed_kmh` | 25 | 차선 변경 중 목표 속도, km/h |
| `lane_change_distance` | 12 | 차선 변경 경로의 전방 길이, m; 짧을수록 급한 합류 |
| `max_approach_distance` | 20 | 출발 후 옆 차로에서 접근을 기다리는 최대 주행 거리, m |
| `from_side` | `left` | ego 진행 방향 기준으로 끼어드는 차량의 출발 차로 방향 |

예를 들어 보행자가 더 늦게 뛰어나오게 하려면 `trigger_distance="20"`으로,
끼어들기를 더 가까이서 시작하게 하려면 차량의 `trigger_distance="12"`로
수정합니다. 두 `trigger_distance`는 서로 다른 이벤트 안에 있으므로
원하는 이벤트의 값을 바꾸세요. 거리 값은 m이며, 보행자 속도는 m/s,
차량 속도는 km/h입니다. 이 값들은 Autoware ego의 속도 설정을 바꾸지 않습니다.

끼어들기는 정해진 접근 거리 또는 20초 안에 ego가 합류 조건을 만족하지 않으면
건너뛰고 일반 Traffic Manager 주행으로 전환합니다. 완료·건너뜀 여부는
터미널의 `[pedestrian]` 및 `[cut_in]` 로그에서 확인할 수 있습니다.

이벤트의 `x/y/z`는 ego가 접근할 **대상 차로의 기준점**입니다. 보행자 자체의
생성 좌표나 끼어드는 차량의 출발 차로 좌표가 아닙니다. 위치를 옮길 때는
교차로 밖의 차로, 같은 방향의 인접 차로, 인도 및 횡단 종료 지점을 함께
확인해야 합니다. CARLA 좌표를 RViz 좌표로 그대로 입력하면 안 됩니다.

### 검증 범위

오프라인 테스트 8개, 설치된 ScenarioRunner를 사용한 XML 읽기와 코드 import,
실행 스크립트의 `--hazards --list` 검사를 통과했습니다. 실제 CARLA 물리
시뮬레이션에서 두 이벤트를 주행해 검증하지는 않았습니다.
특히 인도 턱과 중앙 갓길의 물리 구조,
주변 차량의 간섭, 합류 경로 추종은 실제 실행으로 확인해야 합니다.

이 시나리오는 보행자와 NPC의 행동을 만들고 ego 충돌을 기록합니다.
Autoware의 객체 인식·감속·회피 기능을 추가하거나 그 기능이 작동함을 보장하지는
않습니다. ego의 대응은 연결된 Autoware 모듈과 지도·인지 설정에 달려 있습니다.
