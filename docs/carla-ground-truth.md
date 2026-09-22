# CARLA 객체 입력 모드

`sensor`는 LiDAR 검출·추적을 사용한다. `ground_truth`는 CARLA의 현재 차량·보행자 상태를 추적 객체로 제공하고, Autoware의 기존 지도 기반 예측·계획·제어를 실행한다. 두 모드 모두 실제 CARLA LiDAR와 장애물 점군을 유지한다.

VTD 방식의 판단·제어 기능 검증에는 `ground_truth`를 사용한다. 이 모드에서 보행자 정지가 성공해도 CenterPoint 검출 성능이 개선됐다는 뜻은 아니다. 감지 범위 기본값은 에고로부터 150m이며, 센서 가림이나 탐지 확률을 재현하지 않는다. 미래 위치나 시나리오 트리거 정보는 전달하지 않는다.

이미지를 빌드하고 지도 준비를 완료한 뒤, 다음과 같이 시작한다. `stop`은 현재 CARLA 동적 객체를 정리하므로 진행 중인 시나리오를 종료한 상태에서 사용한다.

```bash
cd /home/a/carla_pp
./scripts/carla/carla_autoware stop
CARLA_PERCEPTION_MODE=ground_truth ./scripts/carla/carla_autoware start-town05
```

LiDAR 인식 모드로 시작하려면 다음과 같이 실행한다.

```bash
cd /home/a/carla_pp
./scripts/carla/carla_autoware stop
CARLA_PERCEPTION_MODE=sensor ./scripts/carla/carla_autoware start-town05
```

기본값은 `sensor`다. 이미 실행 중인 컨테이너와 요청 모드가 다르면 launcher가 중단하고 재시작 필요를 알린다. 시작 명령은 에고를 AUTO로 전환하지 않는다.

| 처리 단계 | sensor | ground_truth |
|---|---|---|
| LiDAR·장애물 점군·점유지도 | 실행 | 실행 |
| CenterPoint 등 검출·추적 | 실행 | 비활성화 |
| 추적 객체 발행자 | multi_object_tracker | autoware_carla_interface |
| map_based_prediction | 실행 | 실행 |
| 계획·제어 | 실행 | 실행 |
| AEB 점군 입력 | 유지 | 유지 |
| AEB 예측 객체 입력 | 기존 설정 | 추가 활성화 |

브리지가 기존 world tick에서 객체를 발행하므로 별도 ticker가 추가되지 않는다. 객체 UUID는 브리지 세션과 actor ID에 기반하며, 제거된 actor는 다음 목록에서 빠진다. 박스 중심·회전과 속도를 CARLA 좌표계에서 ROS 좌표계로 변환하고 차량·보행자 종류를 전달한다.

CARLA의 보행자 정책은 차량 회피와 구분한다. 초기 GT 시험에서는 보행자를 정확히 입력해도 정적 회피·차선 변경 회피가 정지한 보행자를 우회 대상으로 선택했다. 따라서 [CARLA 회피 설정](../config/carla/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml)에서 다음 정책을 적용한다.

```yaml
avoidance.target_filtering.target_type.pedestrian: false
avoidance.target_filtering.target_type.car: true
avoidance.safety_check.target_type.pedestrian: true
avoidance.safety_check.check_unavoidable_object: true
```

변경한 값은 첫 항목뿐이다. 차량은 회피 대상으로 유지하고, 보행자는 회피 목표에서 제외하면서 충돌 안전 검사에는 계속 포함한다. 따라서 차량 회피 경로를 만들 때도 보행자를 무시하지 않는다. 진입 보행자의 감속·정지는 `run_out`, `road_user_stop`, `obstacle_stop` 등의 실제 판단과 제동으로 검증하며, 보행자가 경로를 막는 동안 최소 2초 정지를 유지해야 통과한다. 설정 변경만으로 정지 성공을 판정하지 않는다.

정적 회피와 차선 변경 회피는 모두 공통 `avoidance.target_filtering` 값을 읽는다. 차선 변경 회피는 초기화 때 복사하므로 값 변경 후 Autoware를 재시작해야 한다. 이 정책은 CARLA의 두 입력 모드에 적용되며, VTD 설정은 바꾸지 않는다. `sensor` 모드에서 보행자 검출이 누락되는 문제 자체를 해결한 것은 아니다.

지도는 [Town05 차선 변경 보완](carla-town05-lane-changes.md)을 적용해야 한다. 객체 입력 모드만 바꿔도 누락된 차선 연결이 자동으로 생기지는 않는다. 실행 중 외부에서 `load_world()`로 맵을 바꾸지 말고 Autoware 브리지를 종료한 뒤 다시 시작한다.

실제 주행 검증 결과는 별도의 날짜별 보고서에 기록한다. 입력 모드 이름, 실행 이미지와 지도 해시를 함께 확인해야 한다.

구현·확인 경로:

- [객체 변환 코드](../carla_overlay/src/autoware_carla_interface/src/autoware_carla_interface/modules/ground_truth_objects.py): 좌표계·박스·분류·UUID·제거 객체 처리.
- [실행 구성 설치 코드](../docker/files/install_carla_runtime.py): 검출·추적 실행 분리, 기존 예측 유지, GT 모드의 AEB 객체 입력 추가.
- [객체 변환 단위 시험](../carla_overlay/src/autoware_carla_interface/test/test_ground_truth_objects.py): CARLA·ROS 변환, 안정적인 ID, 에고·범위 제외와 삭제 객체 확인.
- [읽기 전용 실행 검사](../tools/carla_behavior_runtime_check.py): 브리지 모드, 추적·예측 토픽의 단일 발행자, 센서 검출 노드의 실행 여부, 시각·좌표계, AEB 입력과 운행 상태를 JSON으로 기록. 객체가 없는 장면도 검사할 수 있다.

이미 실행 중인 컨테이너를 검사하려면 다음 명령을 사용한다. 시뮬레이터 제어, tick, AUTO 전환은 수행하지 않는다.

```bash
cd /home/a/carla_pp
docker cp tools/carla_behavior_runtime_check.py selfcar-carla-autoware:/tmp/carla_behavior_runtime_check.py
docker exec selfcar-carla-autoware bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /opt/autoware/setup.bash &&
  source /opt/selfcar_overlay/setup.bash &&
  source /opt/carla_overlay/setup.bash &&
  python3 /tmp/carla_behavior_runtime_check.py --mode ground_truth --duration 5 --output /tmp/carla-runtime-check.json'
docker cp selfcar-carla-autoware:/tmp/carla-runtime-check.json /tmp/carla-runtime-check.json
```

센서 모드 검사에는 `--mode sensor`를 지정한다. 검사 통과는 입력 연결과 실행 상태를 뜻하며, 차량 회피나 보행자 정지 성공은 날짜별 실제 주행 기록으로 별도 확인한다.
