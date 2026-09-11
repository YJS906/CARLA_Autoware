# VTD 장애물 타임아웃 재계획

수정 전 GitHub 백업은 `backup-20260912-almost-final` (`ceed681`, `0912 진짜거의마지막`)이다.

VTD에서 AUTONOMOUS 제어가 활성화되고 목적지 경로가 SET인 동안, 차량 속도가
0.1 m/s 이하로 유지되면 다음 조건을 확인한다. 시간은 ROS 시뮬레이션 시간이다.

| 조건 | 처리 |
| --- | --- |
| 가까운 `obstacle_stop`이 연속 10초이고 신호 대기가 아님 | 현재 감지된 UUID들을 계획 입력에서 제외 |
| 후보가 없거나, 승인에 실패하거나, RTC가 RUNNING이어도 실제로 계속 정지함 | 동일한 10초 조건 적용 |
| 같은 신호에서 빨간불 대기 후 초록불이 되고 10초 이내에 다시 빨간불 | 현재 UUID 제외. 신호등 정지는 유지 |
| 정지 이유가 바뀌어도 연속 20초 정지 | 현재 UUID 제외. 신호등 정지는 유지 |

20초는 신호 대기 시간을 포함한다. 10초 카운트는 신호 대기/장애물 정지 해제 시
초기화된다. 실제로 움직이면 두 카운트가 초기화된다. 시뮬레이션 일시정지는
시간을 누적하지 않고, 시간 역행·입력 단절은 잘못된 만료를 방지하도록 처리한다.
목적지 도착, 수동 모드, 경로 변경, 기능 비활성화 시 대기 상태와 제외 목록을 해제한다.

제외 목록은 이벤트 시점 UUID의 스냅샷이다. 새 UUID가 무조건 제외되지는 않는다.
이미 제외한 UUID는 출발 후에도 보이는 동안 유지해 재정지를 막고, 감지 목록에서
연속 2초 사라지면 목록에서 삭제한다. 새 객체가 남아 정지하면 다시 해당 시간 조건을
판단한다. 20초 조건은 연속 정지 구간당 한 번이며, 10초 조건은 새로 카운트할 수 있다.

## 기존 동작과 연결

새 패키지 `selfcar_obstacle_timeout_replan`을 추가한다. 원래 인지 출력
`/perception/object_recognition/objects`는 보존한다. VTD 시뮬레이션의 기존 rule-based
계획기에 전달하는 객체 입력만 `/planning/obstacle_timeout_replan/objects`로 연결한다.
경로·속도 계획기와 planning validator가 같은 필터 결과를 받는다. 평상시에는 원본
메시지를 그대로 전달하며, 제외할 때에도 나머지 객체의 예측 경로·분류·형상·UUID·
헤더를 변경하지 않는다. 변경된 입력은 기존 계획 주기에서 자동으로 재계획된다.

기존 C++ 차선 선택, RTC 승인, 경계 검사, 신호등 처리 코드는 수정하지 않는다.
기존 신호등 모듈의 STOP factor와 현재 경로에서 선택된 신호를 읽기만 한다.
관련 없는 신호나 UNKNOWN을 초록불 전이로 판단하지 않는다. 초록불과 빨간불 사이의
황색 구간은 실제 초록불 시작 시간을 지우지 않는다.

현재 설정의 **7초는 이전에 움직이던 객체의 정지 장애물 재분류 시간**이다.
`avoidance_by_lane_change.route_priority.stopped_dynamic_min_duration=7.0` 분기는
정지 확인을 끝내는 조건이며, RTC 승인 자체를 강제하지 않는다.
일반 `lane_change_left`의 별도 강제 RTC 승인 설정은
`lane_change.obstacle_stop_recovery.duration=5.0`이다. 두 값과 그 조건은 유지한다.
후보 유무나 RTC 응답을 새 필터의 타이머 초기화 조건으로 사용하지 않는다.

객체 제외 후에도 기존 모듈의 짧은 객체 유실 보상/정지 유지 시간이 적용될 수 있다.
유효한 기하 경로 자체가 없는 문제는 객체 제외만으로 해결되지 않는다. 신호등,
고정 정지선, 차량 제어·비상 정지 명령을 강제로 해제하거나 속도를 주입하지 않는다.
이 변경의 20초 처리는 장애물 UUID 제외를 통한 재계획이다. AEB에도 아래와 같이
동일한 제외 목록을 입력 필터로 적용하며, 기존 AEB 판단과 정지 유지 시간은 유지한다.

## AEB와 제외 목록 공유

계획 노드는 현재 제외된 객체를 원래 UUID와 함께
`/planning/obstacle_timeout_replan/excluded_objects`에 발행한다.
별도 노드 `/control/aeb_object_filter`는 이 목록을 받아 AEB 전용 점군
`/control/aeb_object_filter/pointcloud`를 만든다. 원래 점군 토픽은 보존하고,
control launch에서 AEB의 점군 입력만 전용 토픽으로 연결한다.

VTD 점군에는 UUID가 없으므로 추적 객체와 브릿지의 원래 감지 상자를 시간 보정 후
0.5 m 이내에서 일대일로 대응시킨다. 추적기가 보행자 형상을 원통으로 바꾸더라도
점군 대응에는 원래 감지 상자를 사용한다. 제외된 UUID에 유일하게 대응하는 상자의
모서리 점만 0.1 m 오차 범위로 제거한다. 새 UUID, 대응이 불명확한 감지 객체,
어떤 상자에도 대응하지 않는 점은 유지한다. 제외하지 않은 감지 상자와 겹치는 점도
유지한다. 이 필터는 브릿지가 생성하는 상자 모서리 점군을 대상으로 한다.

추적·감지·제외 목록의 유효 시간은 0.5초다. 점군과 추적 시각 차이는 최대 0.25초,
점군과 감지 시각 차이는 최대 0.1초로 제한한다. 점군 시각의 TF를 최대 0.1초 기다리고,
입력이 오래됐거나 TF·형상·대응을 확인할 수 없으면 원본 점군을 전달한다.
제외 목록이 비어 있거나 기능을 끄면 원본 점군을 그대로 전달한다.

신호등 정지, AEB의 충돌 거리·감속 파라미터·충돌 유지 시간, 다른 모듈의 원래
점군 입력은 바꾸지 않는다. 따라서 제외된 객체의 점군 때문에 AEB가 반복 발동하던
경로를 제거하면서 새 객체와 대응 불명 점군은 AEB가 계속 검사한다.

## 확인 및 해제

상태 토픽에는 각 카운트, 신호 대기 여부, 선택된 신호 ID, 제외 UUID와 마지막
발동 이유가 JSON으로 표시된다.

```bash
ros2 topic echo /planning/obstacle_timeout_replan/status
ros2 topic echo /control/aeb_object_filter/status
ros2 param set /planning/obstacle_timeout_replan enabled false
```

`enabled=false`는 즉시 제외 목록을 지우고 원본 객체를 전달한다. 시작 시
`enable_obstacle_timeout_replan:=false`를 지정하면 원래 객체 토픽을 직접 연결한다.
비시뮬레이션, diffusion planner, 기존 fixed-route bypass 구성은 원래 연결을 유지한다.
AEB 점군 필터만 해제하려면 `ros2 param set /control/aeb_object_filter enabled false`를
사용한다. control launch의 `enable_aeb_obstacle_exclusion:=false`는 AEB를 원래 점군에
직접 연결한다. AEB 필터는 `use_sim_time=true`일 때만 제외 처리를 한다.

## 빌드 및 검증

```bash
docker buildx build --load -f docker/vtd/aeb-uuid-exclusion.Dockerfile \
  -t selfcar-2026-vtd:aeb-uuid-exclusion-20260912 .
```

정책 테스트는 ROS 없이 실행할 수 있다. 실제 토픽 테스트는 운영 ROS와 분리한
컨테이너에서만 명시적으로 활성화한다.

```bash
docker run --rm --network none -e ROS_DOMAIN_ID=220 -e SELFCAR_ISOLATED_TEST=1 \
  --entrypoint bash selfcar-2026-vtd:aeb-uuid-exclusion-20260912 -lc '
    source /opt/ros/jazzy/setup.bash
    source /opt/autoware/setup.bash
    source /opt/selfcar_overlay/setup.bash
    unset CYCLONEDDS_URI LD_PRELOAD
    python3 -m unittest discover \
      -s /opt/selfcar-build/src/selfcar_obstacle_timeout_replan/test -v'
```

실제 VTD 차량 주행, 전체 회귀 테스트, 실행 중인 Autoware 재시작은 이 검증에 포함하지 않는다.
