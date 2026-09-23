# Town05 시내 차량·보행자 밀도 설정

`tools/carla_city_traffic.py`는 실행 중인 Town05에 일반 차량과 AI 보행자를 추가하고 목표 수를 유지한다. Autoware를 재시작하거나 에고 위치·경로·주행 모드를 바꾸지 않는다. CARLA world tick과 Traffic Manager 동기화 설정은 기존 Autoware 브리지가 계속 소유한다.

현재 저장한 설정은 **배경 차량 75대·보행자 250명**이다. 에고는 차량 수에 포함하지 않는다. 2026-09-23 혼잡과 지연을 줄이기 위해 기존 250대·500명에서 차량은 30%, 보행자는 50%로 낮췄다. 아래 성능 표는 변경 전 교통량별 측정 기록이다.

## 실행과 수정

CARLA와 Autoware가 실행 중인 상태에서:

```bash
cd /home/a/carla_pp
./scripts/carla/carla_city_traffic start
./scripts/carla/carla_city_traffic status
```

목표 수와 운전 설정은 [city_dense.json](../config/carla/traffic/city_dense.json)에 저장한다. 실행 중 이 파일을 수정하면 자동으로 읽는다.

```bash
gedit /home/a/carla_pp/config/carla/traffic/city_dense.json
```

| 설정 | 의미 |
|---|---|
| `vehicles` | 에고와 특별 이벤트 차량을 제외한 일반 차량 목표 수 |
| `walkers` | 일반 보행자 목표 수; 기존 보행자도 수에 포함 |
| `vehicle_speed_difference_percent` | 도로 제한 속도보다 느리게 주행할 비율; 30이면 제한의 70% |
| `following_distance_m` | 앞차 간격, 현재 3m |
| `auto_lane_change` | 일반 NPC의 자동 차선 변경; 밀집 교통에서는 false |
| `crossing_factor` | CARLA 보행자 내비게이션의 횡단 설정, 현재 0.15 |
| `ego_clearance_m` | 새 객체 생성 시 에고와의 최소 거리, 현재 30m |
| `walker_speed_min_mps`, `walker_speed_max_mps` | 보행 속도 범위, 현재 1.0~1.6m/s |

차량은 CARLA의 원래 주행 차로 생성 지점을 사용하고 신호·정지 표지·차량·보행자를 무시하지 않도록 설정한다. 보행자는 navmesh 위치와 AI walker controller를 사용한다. 목적지에 도착하거나 오래 멈추면 새 목적지를 지정한다. 객체가 사라지면 여유 있는 생성 위치에서 다시 보충하므로 총수는 순간적으로 목표보다 적을 수 있다. 생성 위치가 막혀 있으면 강제로 겹쳐 놓지 않는다.

다음 명령은 이 도구가 생성한 객체만 정리한다. 기존 차량·사용자 시나리오 객체·에고·경로는 삭제하지 않는다.

```bash
./scripts/carla/carla_city_traffic stop
```

에고가 없는 동안 생성을 대기한다. CARLA의 world 자체가 교체되면 이전 actor ID를 재사용하지 않고 종료하므로 새 world가 준비된 후 `start`를 다시 실행한다. 로그인/PC 부팅 시 자동 시작 서비스는 설치하지 않았다.

## 상태와 검증

실행 상태: `~/.local/state/carla_pp/city-traffic/state.json`.
프로세스 로그: `~/.local/state/carla_pp/city-traffic/controller.log`.

상태에는 실제 수·목표 부족분·소유 객체 ID·이동 수·프레임 처리 속도·생성 실패 수가 들어 있다. AI 보행자는 실제로 걷는데도 CARLA 속도 API가 0을 반환할 수 있으므로, 보행자 이동 통계는 snapshot 위치 변화와 시뮬레이션 시간으로 계산한다. 이 보정은 교통 도구의 **통계**에 적용하며, Autoware 브리지의 객체 속도 입력을 수정하는 기능은 아니다.

읽기 전용 성능·위치 이동 측정:

```bash
/home/a/CARLA/venv-0.9.16/bin/python tools/carla_city_traffic_probe.py \
  --duration 20 --output /tmp/carla-city-traffic-probe.json
```

오프라인 소유권·설정·정리·통계 시험:

```bash
PYTHONDONTWRITEBYTECODE=1 /home/a/CARLA/venv-0.9.16/bin/python \
  tools/tests/test_carla_city_traffic.py -v
```

교통량에 따른 성능은 에고 위치, 주행 중인 Autoware 모듈, GPU 및 표시 품질에 따라 달라진다. 목표 수의 코드상 상한은 차량 300대·보행자 800명이며 성능 보장은 아니다. 교통량을 줄일 때는 이 도구 소유 객체만 줄이므로 기존 객체 수보다 낮게 설정해도 외부 객체를 삭제하지 않는다.

이 도구는 도시 교통을 생성한다. 밀집 교통 전체에 대한 Autoware 회피·정지 성공을 보증하지 않으며, 기존의 소수 객체 조건에서 수행한 GT 주행 검증과 별개다.

## 2026-09-23 실제 확인

CARLA 0.9.16 Town05_Opt, Autoware ground_truth 모드, Epic/1280×720, RTX 4060 Ti 16GB에서 기존 배경 차량 6대·보행자 0명으로 시작했다. 작업 중 에고가 AUTO로 주행하는 것을 확인했으며, 도구가 에고를 정지시키거나 위치·경로를 변경하지 않았다.

| 배경 차량 / 보행자 목표 | 20초 측정의 실제 수 | world 처리 속도 | 시뮬레이션 / 실제 시간 | Autoware 실행 확인 |
|---|---|---:|---:|---|
| 100 / 150 | 100 / 150 | 18.27Hz | 0.91배 | 21/21 통과 |
| 200 / 350 | 200 / 349 | 10.29Hz | 0.51배 | 21/21 통과 |
| 250 / 500 | 250 / 500 | 7.87Hz | 0.39배 | 21/21 통과 |

최종 250/500 표본에서는 20초 실제 시간 동안 계속 존재한 차량 250대 중 143대, 보행자 498명 중 291명이 1m 이상 이동했다. 한 시점에 모든 객체가 움직인다는 뜻은 아니며, 신호 대기·혼잡·보행 경로 정체도 포함한다. 프레임 처리 속도는 표시 FPS의 별도 계측값이 아니라 CARLA world frame 증가량으로 계산했다. 에고 위치와 주행 중 계획 처리량도 바뀌므로 이 표를 교통량만의 엄밀한 성능 실험으로 해석하지 않는다.

부드러운 주행을 우선하려면 JSON의 `vehicles`와 `walkers`를 우선 **100 / 150**으로 낮추면 된다. 실행 중 설정을 읽고 이 도구가 추가한 객체부터 단계적으로 줄인다. 도구의 최대 입력값인 300/800은 실행 검증한 수가 아니다.

작은 규모의 실제 생성·이동·종료에서 도구 소유 객체가 모두 정리되고 기존 차량·에고가 남는 것을 확인했다. 종료 직후 서버 목록이 한 tick 늦게 갱신되는 상태 표시 오류와 AI 보행자의 속도 API 0 문제를 발견해 상태 기록을 보완했다. 관련 오프라인 시험 12개가 통과했다.

원시 측정·설정·ROS 확인은 [검증 기록](validation/carla-city-density-20260923/)에 보존했다. 이번 작업에서는 CARLA/Autoware 이미지와 지도를 변경하지 않았다.
