# Autoware 차량 정지 신호·조건·진단 가이드

작성일: 2026-09-08 / 대상: 이 저장소의 Autoware + VTD 실행 환경

빠른 이동:

- [정지 형태](#2-정지가-발생하는-다섯-가지-형태) · [전체 전달 구조](#3-전체-전달-구조)
- [행동 계획](#4-경로행동-계획-단계의-정지) · [후단 계획](#5-후단-경로속도-계획-단계의-정지) · [외부 속도 제한](#6-속도-제한평활화scenario-선택)
- [Planning Validator](#7-planning-validator-진단과-실제-감속을-구분) · [제어기](#8-trajectory-follower와-종방향-제어기) · [게이트와 외부 요청](#9-vehicle-command-gate운전-모드외부-정지-요청)
- [AEB·MRM](#10-aeb진단mrm을-통한-정지) · [브릿지·시뮬레이터](#11-차량-인터페이스브릿지시뮬레이터)
- [원인 추적 순서](#13-실제-정지-원인을-찾는-순서) · [확인 명령](#14-읽기-전용-확인-명령) · [실제 사례](#15-이번-오른쪽-장애물-정지-사례)

## 1. 범위와 읽는 방법

이 문서는 차량을 **정지시키거나, 정지 상태를 유지하거나, 출발하지 못하게 하는 경로**를 정리한다. `obstacle_stop`뿐 아니라 계획, 승인, 속도 제한, 제어기, 명령 게이트, 진단/MRM, 통신, 시뮬레이터를 포함한다.

여기서 “전체”는 **현재 실행 구성의 정지 경로와 이 구성에서 비활성화한 관련 모듈**을 뜻한다. 모든 Autoware 버전·차량 인터페이스·외부 플러그인의 모든 분기까지 동일하다는 뜻은 아니다. 파라미터와 토픽 이름은 버전 및 launch remap에 따라 달라진다.

- **실행 확인**: 실제 ROS 노드·연결·파라미터 또는 설치된 메시지/헤더에서 확인했다.
- **구성 확인**: 배포 설정과 launch preset으로 확인했다. 모듈이 로드되어도 지도 요소·센서 입력·장면 조건이 없으면 동작하지 않는다.
- **간접 정지**: 직접 제동하는 기능이 아니라, 다른 모듈의 판단을 거쳐 정지할 수 있다.
- **비활성**: 현재 preset에서 꺼져 있다. 토픽 이름이 존재하는 것만으로 활성 여부를 판단하면 안 된다.

문서 작성 시 실행 컨테이너는 `vtd-autoware-run-39ae118b9bba`, 실제 이미지 ID는 `4f1fbfacbe84…`였다. 기본 태그 `selfcar-2026-vtd:local`은 이미 `b724681c3881…`로 바뀌었지만 **실행 중인 컨테이너에는 태그 변경이 소급 적용되지 않는다.** 아래 “실행값”은 이전 실행 이미지의 값이다. 승차감 완화 수정본과의 차이는 12절에 정리했다.

문서 작성 과정에서는 운전 모드, 승인, 파라미터, 지도, 실행 프로세스를 변경하지 않았다.

## 2. 정지가 발생하는 다섯 가지 형태

| 형태 | 직접 나타나는 신호 | 의미 |
|---|---|---|
| 계획 정지 | Path/Trajectory의 특정 지점부터 종방향 속도가 0 | 그 위치에 도달하기 전에 감속하도록 계획 |
| 속도 제한 정지 | `VelocityLimit.max_velocity = 0` | 외부 제한·MRM 등이 전체 속도를 0으로 제한 |
| 제어 정지 | 최종 Control의 제동 가속도, 정지 유지 출력 | traj와 별개로 제어기/게이트가 제동할 수 있음 |
| 출발 차단 | STOP 모드, pause, 승인 대기, 데이터 준비 미완료 | 유효한 경로가 보여도 출발하지 않음 |
| 입력/통신 단절 | traj/control 갱신 중단, watchdog, 시뮬레이션 중지 | 정상적인 정지 계획 없이도 차량이 멈출 수 있음 |

중요한 구분:

- `acceleration < 0`은 감속 명령이지, 항상 완전 정지 명령은 아니다.
- 경로 끝의 속도 0은 목적지 정지일 수 있다. **현재 차량 기준 첫 정지점까지의 거리**를 봐야 한다.
- 후진 경로의 음수 속도는 정지 신호가 아니다. 전진 전용 분석의 `v <= 0` 기준을 후진에 그대로 쓰면 안 된다.
- `safe=false`, `no safe path`, `INSUFFICIENT_DRIVABLE_SPACE`는 판단 결과다. 그것만으로 어느 위치에 제동이 들어갔는지는 알 수 없다.
- `/api/motion/state`의 `STOPPED`는 상태 보고이지 정지 요청이 아니다.
- RViz의 가상 정지 벽과 `/planning/planning_factors/*`는 **설명용 출력**이다. 차량이 이를 직접 구독해서 제동하는 구조가 아니다.

## 3. 전체 전달 구조

```text
지도·경로·자차 상태·객체·점군·신호등
    ↓
Behavior Path Planner ── RTC/안전성/회피/차선 끝/목표 도착
    ↓
Behavior Velocity Planner ── 신호등/교차로/횡단보도/정지선
    ↓
Path Smoother → Path Optimizer ── 경로 최적화/주행가능영역 이탈
    ↓
Motion Velocity Planner ── 장애물 정지/추종/감속/경계 이탈
    ↓
Scenario Selector → Velocity Smoother ← 외부 속도 제한·편안한 MRM 정지
    ↓
Planning Validator ── 검증/이전 경로/soft stop
    ↓ /planning/trajectory
Trajectory Follower ── PID 정지/정지 유지/추종 오류 비상 제동
    ↓ /control/trajectory_follower/control_cmd
Vehicle Command Gate ← engage/pause/외부 비상정지/비상 MRM 명령
    ↓ /control/command/control_cmd
VTD Bridge ── 명령 clamp/수신 timeout watchdog
    ↓ TCP 9910
VTD 차량 모델

진단·AEB·상태 감시 → 진단 그래프/운행 가능 상태 → MRM → 위 경로에 개입
```

앞단의 차선변경 승인은 후단 정지 검사를 면제하지 않는다. 여러 모듈이 동시에 정지를 요구할 수 있으며, 하나를 해소해도 다른 정지가 남을 수 있다.

## 4. 경로·행동 계획 단계의 정지

### 4.1 경로 자체와 Behavior Path Planner

현재 로드된 모듈은 정적 회피, 차선변경 회피, 좌/우 차선변경, 출발, 도착, side shift, bidirectional traffic이다.

| 원인/모듈 | 정지 또는 출발 차단 조건 | 확인할 신호 | 주의점 |
|---|---|---|---|
| 경로 없음/경로 준비 미완료 | 지도·자차 위치·route 미준비, route 생성 실패 | `/api/routing/state`, `/planning/mission_planning/route`, path 갱신 여부 | 빈 경로는 유효한 “정지 경로”와 다르다. 하위 준비 검사/timeout으로 이어질 수 있음 |
| 목적지 도착 | route 목표, goal planner 목표, 경로 종단 | path/traj 마지막 0속도, goal 관련 planning factor | 정상적인 정지 |
| 현재 차선 종단 | 다음 차선으로 못 넘어가고 현재 주행 차선의 유효 구간이 끝남 | lane-change factor의 stop pose, `next lc terminal` 등 detail | 장애물과 떨어져 있어도 정지 가능 |
| 정적 장애물 회피 | 충돌 회피 필요, 안전한 shift 또는 정지 후 재계획 경로 없음 | static avoidance info/debug, 최종 path 속도 | 정적 회피가 못 만든 경로를 후단 obstacle stop이 대신 막을 수 있음 |
| 일반 좌/우 차선변경 | 정적 충돌, 예측 객체/RSS 충돌, 남은 거리 부족, 조향·횡가속도·저크 조건 불충족 | `/planning/cooperate_status/lane_change_left`, `lane_change_right`, 해당 planning factors | 후보 경로가 있어도 안전 경로가 아닐 수 있음 |
| 차선변경 회피 | 회피 대상·목표 차선·경로 복귀/차선변경 조건 불충족 | `avoidance_by_lane_change_left/right` RTC와 debug | 일반 차선변경과 회피 정책 판단이 추가됨 |
| 승인 대기 | RTC manual 모드, 요청 미승인, 실행 조건 미충족 | `safe`, `requested`, `command_status`, `state`, `auto_mode` | AUTO 모드라도 안전 검사를 통과해야 함 |
| 승인 후 위험 발생 | 새 장애물, 앞/뒤 차량, 경로 충돌, 안전성 재평가 실패 | RUNNING 상태 변화, 정지/abort factor, path 속도 | 승인 상태를 보존해도 충돌 제동은 독립적으로 가능 |
| Start planner | 안전한 출발 공간/경로 미확보 또는 승인 대기 | `start_planner` RTC/factors | 출발하지 않는 원인이 종방향 제어기만은 아님 |
| Goal planner | 주차/도로 가장자리 접근 중 위험, 승인 대기, 부분 경로 전환·도착 | `goal_planner` RTC/factors | 주차 전용 scenario가 꺼져 있어도 behavior goal planner는 활성 |
| Bidirectional traffic | 좁은 양방향 구간의 대향 차량과 통과 조건 충돌 | 해당 모듈 debug/path/factors | 지도/장면 조건이 맞을 때 동작 |
| Side shift | 횡오프셋 요청 경로가 후단 충돌/영역 조건과 충돌 | `/planning/path_candidate/*`, path bounds, 후단 factors | side shift 요청 자체를 정지 명령으로 해석하면 안 됨 |

정적 회피에서 구분해야 할 대표 사유:

| 표시 | 의미 | 바로 제동한다는 뜻인가? |
|---|---|---|
| `INSUFFICIENT_DRIVABLE_SPACE` | 계산된 주행가능영역 안에서 요구 회피 공간 부족 | 아니다. 최종 path/traj와 후단 stop 확인 필요 |
| `INSUFFICIENT_LONGITUDINAL_DISTANCE` | 준비·횡이동·복귀에 필요한 종방향 거리 부족 | 동일 |
| `NEED_DECELERATION` | 현재 조건에서는 감속이 필요 | 감속/정지 출력 반영 여부 확인 |
| `LIMIT_DRIVABLE_SPACE_TEMPORARY` | 일시적으로 사용 가능한 회피 공간 제한 | 별도 정지 여부 확인 |
| `INVALID_SHIFT_LINE` | shift 선이 유효하지 않음 | 후보 탈락과 차량 정지를 구분 |
| `OUT_OF_TARGET_AREA`, `ENOUGH_LATERAL_DISTANCE`, `LESS_THAN_EXECUTION_THRESHOLD` | 해당 객체를 회피 대상으로 선택하지 않은 이유 | 보통 그 객체에 대한 회피 미생성 사유 |
| `no safe path` | 평가한 후보 중 안전 경로를 확보하지 못함 | 현재 위치 정지인지 차선 종단 정지인지 factor 거리로 구분 |

현재 수정된 차선변경 계획은 감속 준비 후보를 탐색하고, 완전한 안전 후보가 있으면 승인 대기 중에도 현재 차선에서 감속할 수 있다. **안전 후보가 전혀 없을 때 무조건 앞으로 진행하는 기능은 아니다.**

### 4.2 Behavior Velocity Planner: 교통 규칙·지도 요소

공통 출력은 `/planning/scenario_planning/lane_driving/behavior_planning/path`이며, 설명은 `/planning/planning_factors/<모듈>`과 해당 virtual wall에서 확인한다.

| 모듈 | 현재 구성 | 정지 조건/의미 |
|---|---|---|
| `traffic_light` | 활성 | 진행 방향에 유효한 신호가 통과를 허용하지 않음. 적색, 황색 통과 판단 실패, 해당 신호 상태 미확보/만료 등의 처리를 확인 |
| `stop_line` | 활성 | 지도상 해당 정지선에서 정지 의무 수행. 신호등 적색 여부와 별개 |
| `crosswalk` | 활성 | 횡단보도 이용자와의 충돌/양보 판단, 횡단보도 앞 정지 필요 |
| `walkway` | 활성 | 보도/보행 통행 구간 진입 전 정지 조건 |
| `intersection` | 활성 | 교차 차량 충돌, 교차로 정체, 시야 가림, 안전 확인/승인 대기 |
| `blind_spot` | 활성 | 회전 시 사각 영역 객체와 충돌 위험 |
| `detection_area` | 활성 | 지도에 지정된 검출 영역의 장애물에 따라 진입 전 정지 |
| `virtual_traffic_light` | 활성 | 인프라 승인·가상 신호 상태에 따른 정지/진입 대기. 일반 카메라 신호등과 별개 |
| `no_stopping_area` | 활성 | 진입하면 금지 구간 내부에서 멈출 것으로 판단해 **구간 진입 전에** 정지 |
| `roundabout` | 비활성 | 활성화하면 회전교차로 진입 충돌/양보 판단 |
| `merge_from_private` | 비활성 | 활성화하면 사유지/부도로 합류 전 정지 판단 |
| `occlusion_spot` | 비활성 | 활성화하면 가려진 위험 구간에 대한 예방적 감속 등 |
| `speed_bump` | 비활성 | 주로 감속 기능. 활성화되었다고 항상 0속도를 만드는 것은 아님 |
| `no_drivable_lane` | 비활성 | 활성화하면 주행 불가능 차선 진입 관련 정지 판단 |

신호등 관련 확인 사항:

- 최종 입력: `/perception/traffic_light_recognition/traffic_signals` (`TrafficLightGroupArray`).
- VTD 입력 경로: 9910 수신 → 브릿지 ID 매핑 → `/simulator/input/traffic_signals` → perception 신호 토픽의 연결을 확인한다.
- 지도 정지선/신호 규제 요소 ID와 입력의 `traffic_light_group_id`가 맞아야 해당 신호를 찾을 수 있다.
- 현재 브릿지의 `traffic_light.publish_unmapped_ids=false`: 매핑되지 않은 원시 ID는 Autoware ID로 대체 발행하지 않는다.
- 현재 `traffic_light.tl_state_timeout=1.0s`, `stop_margin=1.0m`, `enable_pass_judge=true`다. “황색이면 무조건 같은 거리에서 정지”는 아니다.
- 지도·신호 매핑 오류는 실제로 녹색이어도 통과 신호를 못 찾는 원인이 될 수 있다. 반드시 해당 모듈 factor와 신호 stamp/ID를 함께 확인한다.

## 5. 후단 경로·속도 계획 단계의 정지

### 5.1 Path Optimizer와 주행가능영역

| 조건 | 신호/설정 | 의미 |
|---|---|---|
| 최적화 경로가 주행가능영역 밖으로 나감 | `outside drivable area` virtual wall, `enable_outside_drivable_area_stop=true` | 경로 상 이탈 위치 전에 정지점 삽입 가능 |
| 입력 경로/경계 형상 불량 | 빈 points, 잘못된 bounds, 끊긴 통로, 가까운 점 검색 실패 | 최적화 실패/이전 결과/출력 중단 가능. 최종 결과로 판별 |
| 최적화 결과의 불연속·큰 변화 | path optimizer 전후 경로와 planning validator 상태 | 별도의 검증/soft stop으로 이어질 수 있음 |

`/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`에서 이미 0속도라면, 뒤의 obstacle stop이 최초 원인이 아닐 수 있다. 반대로 먼 곳의 outside-drivable-area 정지 벽은 현재 위치의 즉시 정지 원인이 아닐 수 있다.

### 5.2 Motion Velocity Planner

기준 노드: `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner`

| 모듈 | 현재 구성 | 정지/감속 조건 | 대표 확인 신호 |
|---|---|---|---|
| `obstacle_stop` | 활성 | 경로를 따라 움직이는 차량의 확장 폴리곤과 객체가 충돌, 또는 예측 진입 조건 충족 | `/planning/planning_factors/obstacle_stop`, `obstacle_stop/debug_markers`, `debug/obstacle_stop/trajectory` |
| `obstacle_cruise` | 활성 | 앞 객체와의 안전 거리/상대 속도에 따라 감속·추종. 정지 객체/정체 상황에서 정지로 이어질 수 있음 | cruise factor, 속도 제한 sender, debug trajectory |
| `obstacle_slow_down` | 활성 | 경로 옆 객체와의 측방 거리에 따라 통과 속도 낮춤 | slow-down factor의 `SLOW_DOWN`, 목표 속도 |
| `out_of_lane` | 활성 | 자차 경로/차체가 다른 차선에 걸치며 그 차선의 교통과 충돌 위험 | out-of-lane factor, virtual walls, debug trajectory |
| `obstacle_velocity_limiter` | 활성 | 전방 투영 경로와 장애물/지도 구조물의 충돌 시간에 따라 속도 제한 | 해당 debug trajectory/virtual walls |
| `boundary_departure_prevention` | 활성 | 도로 경계 접근·이탈 예측, 위치/종방향 추종 이상 가정 | 해당 virtual walls/diagnostics, 경계 정지 옵션 |
| `dynamic_obstacle_stop` | 다음 재실행부터 활성 (2026-09-08 설정 변경) | 해당 모듈 대상 동적 객체의 충돌 위험에 따른 정지. 객체 분류·충돌 필터는 유지하며, 기존 실행 세션에는 적용하지 않음 |
| `run_out` | 비활성 | 활성화하면 갑작스러운 객체 진입/예측 충돌 관련 정지·감속 |
| `road_user_stop` | 비활성 | 활성화하면 도로 이용자 관련 안전 정지 |
| `surround_obstacle_checker` | 비활성 | 활성화하면 정지 상태 주변 장애물로 인한 출발 차단 등 |
| VTD fixed-route bypass planner | 비활성 | 별도 우회 계획기. 현재 기본 정지 판단 주체로 보면 안 됨 |

현재 obstacle velocity limiter의 배포 설정은 `min_adjusted_velocity=2.5m/s`, `min_ttc=1.0s`, `dynamic_source=static_only`다. **이 설정의 감속만 보고 완전 정지 주체라고 단정하면 안 된다.** Boundary departure prevention에는 `stop_before_departure=true`가 설정되어 있다.

### 5.3 obstacle_stop 상세

현재 배포값:

| 항목 | 값 | 의미 |
|---|---:|---|
| 기본 측방 여유 | 0.3m | 차체 경계 바깥으로 더 검사하는 폭 |
| 기본 정지 여유 | 5.0m | 충돌 위치에 대한 정지 위치 여유. 차량 앞단 보정 등과 함께 계산됨 |
| 현재 자세 고려 | 사용 | 실제 자차 자세와 계획 경로 자세의 차이를 검사 영역에 반영 |
| 객체 내부 검사 | UNKNOWN 활성 | VTD 객체를 UNKNOWN으로 정규화하는 구성 |
| 객체 외부 진입 검사 | UNKNOWN 활성 | 현재 바깥에 있어도 들어오는 예측을 검사 |
| 일반 stop의 pointcloud 검사 | 비활성 | 동일 VTD 객체의 점군 중복 정지를 피하는 설정 |
| AEB의 pointcloud 검사 | 활성 | 일반 stop 점군 검사를 꺼도 AEB까지 꺼진 것이 아님 |
| 정지 상태 유지 | on/off·객체 유지·정지점 유지 설정 존재 | 한 프레임에서 안 겹쳐도 즉시 정지가 풀리지 않을 수 있음 |

따라서 다음은 서로 다르다.

1. 장애물이 내 **지도 차선**에 속하는가?
2. 장애물이 **계획 경로를 따라 움직일 실제 차체**에 닿는가?
3. 장애물이 **차체 + 안전 여유 + 자세 오차 범위**에 닿는가?

옆 차선 객체도 3번에 해당하면 정지할 수 있다. 반대로 같은 차선 ID라는 사실만으로 반드시 차체 충돌은 아니다. 객체 크기·방향, 차량 폭·앞뒤 길이, map/TF 변환, 최적화 경로, 경계 margin을 함께 봐야 한다.

## 6. 속도 제한·평활화·scenario 선택

### 6.1 외부 속도 제한

| 토픽 | 의미 |
|---|---|
| `/planning/scenario_planning/max_velocity_candidates` | 모듈별 속도 제한 후보. `sender`로 출처 식별 |
| `/planning/scenario_planning/max_velocity_default` | 기본 외부 속도 제한 입력 |
| `/planning/scenario_planning/max_velocity` | 선택된 속도 제한. **publisher가 여럿인지 확인 필요** |
| `/planning/scenario_planning/applied_velocity_limit` | smoother가 적용한 제한 보고 |
| `/planning/scenario_planning/clear_velocity_limit` | sender를 지정한 제한 해제 요청. 상태 조회 토픽이 아님 |
| `/planning/scenario_planning/external_velocity_limit_selector/debug` | 제한 선택 상황을 확인하는 설명 출력 |

`VelocityLimit`의 핵심 필드는 `max_velocity`, `sender`, `use_constraints`, `constraints`다. `max_velocity=0`이면 정지 원인이 될 수 있다. 제한이 사라졌는지는 단순히 원래 publisher가 없어졌는지가 아니라 selector와 적용 결과에서 확인한다.

이 저장소의 실행 스크립트는 `sender=autoware_run`으로 `/planning/scenario_planning/max_velocity`를 반복 발행한다. selector도 같은 토픽을 발행하므로, 속도 제한 이상에서는 `ros2 topic info --verbose`로 publisher 수와 sender를 반드시 확인한다. **특정 sender만 한 번 보는 것으로 전체 제한 상태를 단정하지 않는다.**

### 6.2 Velocity Smoother

| 조건 | 결과 | 확인 방법 |
|---|---|---|
| 입력 traj에 0속도 정지점 | 도달 가능한 감속 프로파일 생성 | raw → filtered → output 비교 |
| 외부 속도 제한 0 | 전체 정지/감속 프로파일 | max_velocity와 applied_velocity_limit 비교 |
| 횡가속도 제한 | 급한 곡률에서 속도 낮춤 | `trajectory_lateral_acc_filtered` |
| 조향 속도 제한 | 빠르게 조향해야 하는 구간에서 속도 낮춤 | `trajectory_steering_rate_limited` |
| 가감속·저크 제한 | 속도 변화가 완만해짐, 감속 시작 위치/정지 추종에 영향 | forward/backward/merged filtered trajectory |
| engage 금지 거리 | 가까운 정지점 때문에 정지 상태에서 출발용 속도를 넣지 않음 | `stop_dist_to_prohibit_engage=0.5m`, 첫 정지점 거리 |
| 경로 부족·잘못된 입력·입력 미수신 | 계산 실패/출력 미갱신 가능 | 로그, stamp, points 수, 후단 timeout |

횡가속도/저크 필터를 완화하는 것은 obstacle stop, 신호등, MRM을 해제하는 것과 다르다. 현재 곡선 감속 필터에는 `min_curve_velocity=1.5m/s`도 있으므로, “곡선 필터가 켜져 있다”만으로 완전 정지 원인을 설명할 수 없다.

### 6.3 Scenario Selector

선택한 주행/주차 scenario의 trajectory가 없거나 오래되면 정상적인 출력이 이어지지 않을 수 있다. 현재 parking scenario는 비활성이며, selector의 `th_max_message_delay_sec=1.0s`다. 상태 전환·도착·정지 확인과 **데이터 미수신**을 구분한다.

## 7. Planning Validator: 진단과 실제 감속을 구분

입력: `/planning/scenario_planning/velocity_smoother/trajectory` → 출력: `/planning/trajectory`.

현재 로드된 검사기는 LatencyChecker, TrajectoryChecker, IntersectionCollisionChecker, RearCollisionChecker다.

설치 헤더의 `InvalidTrajectoryHandlingType`:

| 값 | 이름 | 처리 의미 |
|---:|---|---|
| 0 | `PUBLISH_AS_IT_IS` | 현재 결과를 그대로 발행 |
| 1 | `USE_PREVIOUS_RESULT` | 이전 유효 결과 사용 |
| 2 | `USE_PREVIOUS_RESULT_WITH_SOFT_STOP` | 이전 유효 결과에 soft stop 처리 |

현재 `default_handling_type=0`이며, 대부분 trajectory 검사의 `handling_type=0`이다. **검증 오류가 있다고 항상 `/planning/trajectory`를 0으로 덮는 것은 아니다.** 다만 오류 진단이 MRM으로 이어지는 경로는 별도다. `trajectory_shift.handling_type=2`는 예외이며, 현재 soft-stop 설정은 감속 `−1.0m/s²`, 저크 크기 `0.3m/s³`다.

| 검사 범주 | 현재 대표 기준 | 확인 신호 |
|---|---|---|
| 입력 지연 | latency 1.0s | trajectory stamp, latency diagnostic |
| 유효성/형상 | points 수·유한값·점 간격·상대 각도 | validation_status, 개별 진단 |
| 곡률·조향 | curvature 1.0, steering 1.414rad, steering rate 10.0rad/s | 해당 검사 상태 |
| 가감속·횡운동 | 종가속 ±9.8m/s², 횡가속 9.8m/s², 횡저크 7.0m/s³ | 해당 검사 상태 |
| 자차와 경로 차이 | yaw 1.5708rad, 종방향 위치 차이 1.0m 등 | yaw/distance/velocity deviation |
| 경로 급변 | 횡 shift 0.5m, 전방 shift 1.0m, 후방 shift 0.1m | trajectory_shift, soft stop 적용 여부 |
| 전방 경로 길이 | 감속도 −3.0m/s²·margin 2.0m를 사용하는 검사 | forward trajectory length |
| 교차로/후방 충돌 | 지도·점군·신호·상대 운동을 이용한 추가 검사 | `/planning/planning_factors/intersection_collision_checker`, `rear_collision_checker` |

위 수치는 **검사 기준**이지 차량이 실제로 낼 수 있는 성능 보증값이 아니다. 검사기별 적용 범위와 출력 반영 여부를 분리해서 읽는다.

준비 데이터에는 trajectory, odometry, acceleration, obstacle pointcloud, route/map이 포함된다. 입력이 없으면 검사 준비 자체가 안 될 수 있다. 디버그 확인 순서:

1. smoother 출력과 최종 trajectory를 같은 시각에 비교한다.
2. `/planning/planning_validator/validation_status`와 `virtual_wall`을 확인한다.
3. handling type과 개별 진단을 확인한다.
4. 최종 trajectory가 변하지 않았다면 MRM/게이트에서 제동하는지 확인한다.

## 8. Trajectory Follower와 종방향 제어기

노드: `/control/trajectory_follower/controller_node_exe`.

| PID 상태 | 값 | 의미 |
|---|---:|---|
| DRIVE | 0 | 주행 속도 추종. 목표가 낮으면 정상 감속 가능 |
| STOPPING | 1 | 정지점에 접근하며 정지 제어 |
| STOPPED | 2 | 정지 상태 유지 |
| EMERGENCY | 3 | 제어기 내부 비상 제동 |

| 원인 | 관련 설정/상태 | 해석 |
|---|---|---|
| 목표 속도 0/가까운 정지점 | target velocity, stop distance | 정상 정지 제어 |
| 큰 추종 오차 | `enable_large_tracking_error_emergency=true` | 계획은 있어도 현재 자세/경로 조건 때문에 비상 제동 가능 |
| 정지점 초과 | `enable_overshoot_emergency=false` 현재값 | 이 특정 비상 분기는 현재 꺼져 있음. 다른 overrun 진단까지 꺼진 것은 아님 |
| 부드러운 정지 | 실행값 `enable_smooth_stop=true` | 정지 방식 선택. 끈다고 STOPPED나 비상 제동이 없어지지 않음 |
| 정지 유지 | `stopped_acc=-3.4m/s²` | 이미 멈춘 뒤에도 음의 명령이 나올 수 있음 |
| 제어기 비상 | `emergency_acc=-5.0m/s²`, `emergency_jerk=-3.0m/s³` | 일반 계획 감속 설정과 별도 |
| 조향 수렴 대기 | `enable_keep_stopped_until_steer_convergence=false` 현재값 | 해당 출발 대기는 현재 꺼져 있음 |
| 데이터 미준비/잘못된 trajectory | 자차 상태·조향·가속·trajectory 등 | 출력 중단 또는 오류 처리 여부를 로그/stamp로 확인 |
| slope/PID/필터 영향 | slope angle, feedforward/PID, jerk limit | 별도 정지 명령 없이 음의 가속도가 나오는 원인이 될 수 있음 |

`/control/trajectory_follower/longitudinal/diagnostic`의 `data` 인덱스는 이 실행 버전의 설치 헤더 기준으로 다음과 같다.

| 인덱스 | 의미 | 인덱스 | 의미 |
|---:|---|---:|---|
| 1 | 현재 속도 | 2 | 목표 속도 |
| 3 | 목표 가속도 | 13 | CONTROL_STATE |
| 14 | PID 적용 가속도 | 15 | 가속도 제한 적용 후 |
| 16 | 저크 제한 적용 후 | 17 | 경사 보상 후 |
| 18 | 최종 발행 가속도 | 22 | FLAG_STOPPING |
| 23 | FLAG_EMERGENCY_STOP | 28 | STOP_DIST |
| 36 | SMOOTH_STOP_MODE | | |

다른 버전에서는 인덱스를 다시 확인해야 한다. 조향 제한은 주로 횡제어 출력에 작용하지만, 큰 추종 오차·검증 오류를 통해 간접적으로 정지를 유발할 수 있다.

## 9. Vehicle Command Gate·운전 모드·외부 정지 요청

Gate는 정상 제어, 외부 조종 명령, 비상 명령을 선택하고 제한한다. 비교할 핵심 토픽은 **게이트 전** `/control/trajectory_follower/control_cmd`와 **게이트 후** `/control/command/control_cmd`다.

| 신호/조건 | 확인 위치 | 정지와의 관계 |
|---|---|---|
| STOP 운전 모드 | `/api/operation_mode/state.mode=1` | 출발/정상 자율 주행 차단 상태 |
| Autoware 제어 비활성 | `is_autoware_control_enabled=false`, 차량 control mode | 경로가 있어도 제어 권한이 없음. 권한 해제 자체를 물리 비상 브레이크와 동일시하지 않음 |
| engage 해제 | `/autoware/engage`, `/api/autoware/get/engage` | 정상 자동 명령 대신 정지 유지 처리 경로 확인 |
| pause | `/control/vehicle_cmd_gate/is_paused` | 정지/출발 대기 처리 |
| stop 요청 | `/control/vehicle_cmd_gate/is_stopped`, `is_start_requested` | stop/start 요청 상태에 따른 게이트 처리 |
| 외부 비상정지 | external emergency stop 서비스, `/api/autoware/get/emergency` | 게이트 비상 제동 |
| 비상 MRM 실행 | `/system/fail_safe/mrm_state`, `/system/emergency/control_cmd` | 정상 제어 대신 비상 제어 선택 |
| 비상 명령 heartbeat 만료 | `system_emergency_heartbeat_timeout=0.5s` | 비상 제어 통신 이상에 대한 fallback/제동 경로 |
| 외부 조종 heartbeat 만료 | `/external/selected/heartbeat` | 현재 `check_external_emergency_heartbeat=false`. 옵션과 선택 모드 확인 필요 |
| AUTO/EXTERNAL 선택 오류 | `/control/current_gate_mode`, `/control/gate_mode_cmd` | 정상 자율 제어가 선택되지 않을 수 있음 |
| 명령 제한 필터 | `is_filter_activated`, `enable_cmd_limit_filter=true` | 가감속·저크·조향 명령 제한. 활성 플래그만으로 완전 정지라고 판단하지 않음 |

현재 gate의 정지 유지 가속도는 `−1.5m/s²`, 자체 비상 가속도는 `−2.4m/s²`, moderate stop 가속도는 `−1.5m/s²`다. 이는 PID의 `−5.0`, MRM의 `−2.5`, 브릿지 watchdog의 `−2.0`과 서로 다른 설정이다.

현재 graph에서 확인한 요청 인터페이스 목록이다. **목록은 설명용이며, 진단할 때 임의로 호출하지 않는다.**

| 서비스 | 요청 의미 |
|---|---|
| `/api/operation_mode/change_to_stop` | STOP 모드 전환 요청 |
| `/api/operation_mode/disable_autoware_control` | Autoware 제어 권한 해제 |
| `/api/autoware/set/engage` | legacy engage 변경 |
| `/api/autoware/set/emergency` | legacy 비상 상태 변경 |
| `/control/vehicle_cmd_gate/external_emergency_stop` | 외부 비상정지 설정 |
| `/control/vehicle_cmd_gate/clear_external_emergency_stop` | 외부 비상정지 해제 요청 |
| `/control/vehicle_cmd_gate/set_pause` | pause 상태 변경 |
| `/control/vehicle_cmd_gate/set_stop` | stop 요청 상태 변경 |
| `/api/motion/accept_start` | 출발 승인. 정지 원인을 강제로 무시하는 API가 아님 |
| `/api/fail_safe/mrm_request/send` | MRM 요청 |
| `/system/mrm/emergency_stop/operate` | 비상 정지 동작 시작/취소 요청 |
| `/system/mrm/comfortable_stop/operate` | 편안한 정지 동작 시작/취소 요청 |

`/api/operation_mode/state` enum은 UNKNOWN=0, STOP=1, AUTONOMOUS=2, LOCAL=3, REMOTE=4다. AUTONOMOUS=2만으로 출발 가능하다고 단정하지 않고 제어 활성, transition, MRM, gate pause/stop을 함께 본다.

운전 모드 전환 관리자의 현재 값은 `check_engage_condition=false`, `input_timeout=3.0s`, `transition_timeout=10.0s`다. 특정 engage 조건 검사를 꺼도 데이터 준비·제어 권한·시스템 안전 상태 검사가 모두 사라지는 것은 아니다.

## 10. AEB·진단·MRM을 통한 정지

### 10.1 AEB와 충돌 검출

- `/control/autonomous_emergency_braking`은 이 graph에서 직접 Control 명령을 발행하지 않는다. 충돌 판단을 `/diagnostics`에 내고, 진단 그래프/운행 가능 상태/MRM 경로를 통해 정지에 연결된다.
- 현재 AEB: 예측 trajectory 사용, IMU 경로 미사용, 점군 사용, predicted objects 미사용. 예측 horizon은 4.5s이며 `collision_keeping_sec=3.0s`다.
- 따라서 일반 obstacle stop이 안전하다고 해도 AEB 점군 판정은 별도로 발생할 수 있다.
- `/control/collision_detector` 역시 직접 제동 토픽을 내지 않고 진단을 발행한다. 현재 동적 객체 입력 사용, 점군 검사 미사용, `collision_distance=0.1m`다.
- collision detector나 control validator의 오류가 실제 MRM을 일으키는지는 **진단 그래프 연결과 운행 가능 상태**까지 확인해야 한다. 노드가 있다는 사실만으로 자동 정지 연결을 보장하지 않는다.

### 10.2 진단 원인의 전체 범주

| 범주 | 대표 원인 | 정지까지의 경로 |
|---|---|---|
| 제어 성능 | 경로 이탈, yaw 오차, 과속, 가속 오차, 정지점 초과 예상/실제 초과, rollback | validator/제어 상태 진단 → 연결된 안전 판단 |
| 경로 검증 | 잘못된 형상, 경로 급변, 충돌 검사, 지연 | 직접 soft stop 또는 진단 → MRM |
| 위치 추정 | localization 상태 불량, TF 누락/오차, 초기화 미완료 | 준비 실패/운행 불가/계획 또는 제어 오류 |
| 입력 freshness | trajectory, control, emergency control, odometry 등의 갱신 중단 | topic monitor/timeout → 안전 판단 또는 bridge watchdog |
| 센서/지도 | 객체·점군·신호등·map 데이터 미준비/오류 | 모듈별 보수적 판단 또는 준비 실패. 모두 즉시 제동하는 것은 아님 |
| 프로세스 | 노드 종료, 중복 노드, executor 지연, CPU 과부하, 통신 장애 | heartbeat/주기 감시/데이터 단절 |
| 운행 가능 상태 | 자율 운행에 필요한 구성 요소의 진단 조건 실패 | `/system/operation_mode/availability` → MRM/전환 차단 |
| 외부 MRM 요청 | API/운영 시스템의 최소위험기동 요청 | 요청 목록/상태 → 해당 MRM 동작 |

진단값은 OK=0, WARN=1, ERROR=2, STALE=3이지만 **모든 WARN/ERROR가 바로 정지를 의미하지 않는다.** 연결된 그래프 조건, 시간 유지, 운전 모드에 따라 결과가 달라진다.

현재 프로젝트의 control 진단 그래프에는 trajectory follower/control command 주기, gate heartbeat, AEB, lane departure, control state가 연결되어 있다. 문서 끝의 진단 설정 파일을 참조한다.

### 10.3 MRM 종류와 출력

| 동작 | 현재 구성/값 | 주 출력 |
|---|---|---|
| Comfortable stop | 사용, 감속 −1.0m/s², jerk −0.3~+0.3m/s³ | 속도 제한 후보/해제 → 계획 속도 감소 |
| Emergency stop | 사용, 목표 감속 −2.5m/s², 목표 jerk −1.5m/s³ | `/system/emergency/control_cmd` → gate |
| Pull over | `use_pull_over=false` | 현재 미사용 |
| Emergency holding | `use_emergency_holding=false` | 현재 해당 유지 옵션 미사용 |
| 정지 후 P단 전환 | `use_parking_after_stopped=false` | 현재 해당 옵션 미사용 |

현재 MRM handler의 availability timeout은 0.5s, emergency recovery timeout은 5.0s다. 진단이 순간 회복해도 상태/회복 대기 때문에 제동이 즉시 끝나지 않을 수 있다.

관찰 토픽:

- `/api/fail_safe/mrm_state`, `/system/fail_safe/mrm_state`
- `/api/fail_safe/mrm_request/list`
- `/system/mrm/emergency_stop/status`, `/system/mrm/comfortable_stop/status`
- `/system/operation_mode/availability`
- `/diagnostics`, `/diagnostics_graph/struct`, `/diagnostics_graph/status`
- `/api/system/diagnostics/struct`, `/api/system/diagnostics/status`

현재 `MrmState.state`는 NORMAL=1, MRM_OPERATING=2, MRM_SUCCEEDED=3, MRM_FAILED=4다. behavior의 숫자만 하드코딩하기보다 설치된 메시지와 description API를 확인한다.

## 11. 차량 인터페이스·브릿지·시뮬레이터

### 11.1 현재 9910 브릿지에서 실제 사용하는 명령

`vtd_bridge_node.cpp`의 `on_control()`은 `/control/command/control_cmd`의 **가속도와 조향각**을 저장한다. `send_control()`이 9910으로 조향, 가속도, 방향지시 신호를 보낸다.

| 항목 | 현재 동작 | 정지 진단의 의미 |
|---|---|---|
| `longitudinal.acceleration` | −6~+3m/s² 범위로 clamp 후 전달 | 실제 VTD 제동 확인에 중요한 값 |
| `longitudinal.velocity` | 이 브릿지의 직접 송신 목표속도로 사용하지 않음 | Control의 velocity=0만 보고 VTD에 제동이 전달됐다고 단정하지 않음 |
| 조향각 | ±0.7rad로 clamp | 추종 오차·경계 접근에 간접 영향 |
| 최초 명령 미수신 | watchdog 감속 −2.0m/s², 조향 0 | Autoware 제어 출력이 한 번도 없어도 정지 방향 명령 가능 |
| 명령 timeout | 마지막 수신 이후 wall time 0.5s 초과 | watchdog 감속 −2.0m/s², 조향 0 |
| 송신 주기 | wall time 40ms | ROS simulation time과 별개 |
| gear | 이 구현에서는 명령 저장/상태 보고에 사용 | `send_command(steering, acceleration, turn_signal)`에 실제 gear 인자가 없으므로 P/N 명령만으로 VTD가 제동한다고 가정하지 않음 |
| emergency flag | 별도 `/control/command/emergency_cmd`를 직접 소비하지 않음 | gate가 최종 Control에 제동을 반영했는지 확인해야 함 |

브릿지 diagnostics의 `watchdog_active`, 송수신 상태와 마지막 명령 수신 시각을 확인한다. **9910 연결 자체가 끊기면 watchdog 명령도 차량까지 전달되지 못할 수 있다.** 이 경우 VTD/Host 측의 별도 timeout 처리까지 확인해야 한다.

### 11.2 Autoware 정지와 혼동하기 쉬운 외부 상태

- VTD Pause/Stop, simulation time 정지, 시나리오 이벤트에 의한 정지.
- Host가 다른 제어 주체를 선택하거나 외부 명령을 수락하지 않는 상태.
- VTD/브릿지 좌표 변환, 차량 크기, steering 응답 모델 불일치.
- 차량 인터페이스에 따라 P/N단, 주차 브레이크, 운전자 brake override, DBW fault, 도어/안전벨트 interlock 등도 출발을 막을 수 있다. **이 VTD 9910 브릿지가 그 모든 기능을 구현한다는 뜻은 아니다.**
- 시뮬레이터 물리 모델의 정지와 ROS odometry의 속도 0 보고를 비교한다. 경로가 멈춘 것인지, 물리 시간이 멈춘 것인지 구분한다.

## 12. 승차감 설정과 안전 정지는 별개

문서 작성 시 실행값과 빌드된 다음 실행 이미지의 대표 차이:

| 설정 | 실행 중 `4f1fbfac…` | 다음 실행 `b724681c…` |
|---|---:|---:|
| PID `enable_smooth_stop` | true | false |
| 공통 계획 최대 가속 | +1.0 | +2.0m/s² |
| 공통/PID longitudinal jerk | −5 / +2 | −10 / +4m/s³ |
| smoother 횡가속 상한 | 0.65 | 1.3m/s² |
| lane-change 횡가속 상한 | 0.99 | 1.98m/s² |
| lane-change 횡저크 상한 | 1.1 | 2.2m/s³ |
| 정적 회피 횡가속 상한 | 0.55 | 1.1m/s² |
| 일반 계획 최대 감속 | −5.0 | −5.0m/s² 유지 |
| obstacle stop 측방/종방향 여유 | 0.3 / 5.0m | 유지 |

다음 실행 이미지에서는 주행 평활화 비용·일부 시간상수도 절반으로 줄였다. 그러나 충돌 검사, MRM, 비상 제동, 운전 모드 전환 안전 조건, 센서 잡음 필터, 차량 모델 한계를 일괄 해제한 것은 아니다. 특히 **PID smooth stop을 꺼도 MRM comfortable stop은 별도 기능으로 남는다.**

## 13. 실제 정지 원인을 찾는 순서

### 13.1 1차 분류

| 관측 | 우선 조사 |
|---|---|
| 최종 traj의 가까운 지점부터 0 | 계획 단계별 첫 정지점과 planning factors |
| smoother 출력은 주행 가능, 최종 traj만 정지/이전 경로 | planning validator |
| 최종 traj는 주행 가능, follower부터 제동 | PID 상태/target/stop distance/추종 오류 |
| follower는 주행 명령, gate 후 제동 | pause/engage/STOP/MRM/emergency/게이트 제한 |
| gate 명령은 주행인데 VTD가 제동 | 브릿지 수신 시각/watchdog/9910/Host |
| 경로·제어 명령 자체가 갱신되지 않음 | 프로세스·QoS·ROS domain·입력 준비·simulation clock |
| 모든 경로가 정상이지만 출발하지 않음 | 제어 권한, 출발 승인, gate stop/pause, VTD 실행 상태 |

### 13.2 단계별 trajectory 비교

아래 순서에서 **0속도 정지점이 처음 생기는 단계**를 찾는다. 같은 프레임/가까운 stamp를 비교하고, ego에 대한 signed arc distance를 계산한다.

1. `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`
2. `/planning/scenario_planning/lane_driving/behavior_planning/path`
3. `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory`
4. `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/<module>/trajectory`
5. `/planning/scenario_planning/lane_driving/trajectory`
6. `/planning/scenario_planning/scenario_selector/trajectory`
7. `/planning/scenario_planning/velocity_smoother/trajectory`
8. `/planning/trajectory`
9. `/control/trajectory_follower/control_cmd`
10. `/control/command/control_cmd`

디버그 trajectory가 모듈별로 발행되어도 해당 출력이 최종 선택되었는지는 최종 trajectory로 재확인한다. 정지점이 여러 개면 가장 가까운 유효 정지점과 각 모듈의 거리/시간을 함께 기록한다.

### 13.3 PlanningFactor·RTC 읽기

`PlanningFactor.behavior`는 UNKNOWN=0, NONE=1, SLOW_DOWN=2, STOP=3, SHIFT_LEFT=4, SHIFT_RIGHT=5, TURN_LEFT=6, TURN_RIGHT=7이다.

확인할 필드:

- `module`, `behavior`, `detail`: 판단 주체와 행동.
- `control_points[].pose`, `distance`, `velocity`: 실제 조작 위치·상대 거리·목표속도.
- `safety_factors.factors[].object_id`, `points`: 선택한 객체/점군 원인.
- `safety_factors.*.is_safe`: 그 판단의 안전 상태. 비어 있는 배열이나 기본값을 과도하게 해석하지 않는다.
- `header.stamp`: 현재 판단인지 확인.

`distance`가 미계산 값·비유한 값이면 숫자만 사용하지 말고 stop pose를 자차 기준 경로에 투영한다. 객체 UUID는 추적기 재시작 등으로 달라질 수 있으므로 이전 장면과 비교할 때 위치·형상·시각도 함께 본다.

RTC에서는 `safe=true`와 승인 `command_status.type=ACTIVATE(1)`를 구분한다. `auto_mode=true`도 안전을 무시하는 강제 실행이 아니다. state는 WAITING_FOR_EXECUTION=0, RUNNING=1, ABORTING=2, SUCCEEDED=3, FAILED=4다.

### 13.4 정지가 풀리지 않는 이유

객체 제거 후에도 다음 상태가 남을 수 있다.

- 객체 추적/예측 유지, obstacle stop의 on/off hysteresis와 정지점 hold.
- AEB 충돌 유지 시간, collision detector 해제 hysteresis.
- selector에 남은 다른 sender의 0속도 제한.
- MRM recovery 대기, 외부 emergency latch, gate pause/stop.
- PID STOPPED 상태의 재출발 조건, 가까운 정지점에 의한 engage 억제.
- RTC 승인/안전성 미확보, 경로 종단 또는 다른 장애물 정지.
- 실제 출력은 갱신됐지만 RViz에 남은 오래된 marker.

## 14. 읽기 전용 확인 명령

호스트에서 먼저 실행 대상을 확인한다. 아래의 컨테이너 이름은 실제 실행 이름으로 바꾼다.

```bash
docker ps --format '{{.Names}}  {{.Image}}  {{.Status}}'
docker inspect vtd-autoware-run-39ae118b9bba --format '{{.Image}}'
docker image inspect selfcar-2026-vtd:local --format '{{.Id}}'
docker exec -it vtd-autoware-run-39ae118b9bba bash
```

컨테이너 안에서:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/autoware/setup.bash
source /opt/selfcar_overlay/setup.bash

ros2 topic echo /api/operation_mode/state --once
ros2 topic echo /api/motion/state --once
ros2 topic echo /api/fail_safe/mrm_state --once
ros2 topic echo /control/vehicle_cmd_gate/is_paused --once
ros2 topic echo /control/vehicle_cmd_gate/is_stopped --once
ros2 topic echo /planning/planning_factors/obstacle_stop --once
ros2 topic echo /planning/scenario_planning/applied_velocity_limit --once
ros2 topic info /planning/scenario_planning/max_velocity --verbose
ros2 topic echo /control/trajectory_follower/longitudinal/diagnostic --once
ros2 topic echo /control/trajectory_follower/control_cmd --once
ros2 topic echo /control/command/control_cmd --once
ros2 topic echo /diagnostics --once
```

추가 조사:

```bash
ros2 topic list -t
ros2 node info /control/vehicle_cmd_gate
ros2 node info /planning/planning_validator
ros2 param dump /planning/planning_validator
ros2 param dump /control/trajectory_follower/controller_node_exe
ros2 param dump /system/mrm_handler
ros2 param dump /vtd_bridge
ros2 topic hz /planning/trajectory
ros2 topic hz /control/command/control_cmd
ros2 interface show autoware_internal_planning_msgs/msg/PlanningFactor
ros2 interface show tier4_rtc_msgs/msg/CooperateStatus
ros2 run tf2_ros tf2_echo map base_link
```

`--once`도 새 메시지가 없으면 기다릴 수 있다. 필요한 경우 `timeout 5 ros2 ...`로 관측 시간을 제한한다. `topic hz`는 Ctrl-C로 끝낸다. publisher의 QoS와 맞지 않으면 수신이 안 될 수 있으므로 `topic info --verbose`를 먼저 확인한다. **한 번의 `/diagnostics` 수신에는 모든 노드의 진단이 들어 있지 않을 수 있다.** 여러 주기 또는 aggregate graph를 본다.

`ros2 param dump`가 비어 있어도 모듈이 없다는 결론을 내리지 않는다. 이번 motion velocity planner는 dump 결과가 비어 있었지만 node info에서 실제 plugin 출력이 확인되었다. 그 경우 launch 파일/설치 YAML과 publisher 정보를 함께 사용한다.

진단 중에는 속도 제한 해제, RTC 강제 승인, emergency 해제, 노드 재시작, 지도 수정 등을 관측 명령과 섞지 않는다. bag 기록·주행 재현은 별도 승인된 작업으로 수행한다.

## 15. 이번 오른쪽 장애물 정지 사례

2026-09-08 19:59 관측 결과이며, 이후 장면이 변하면 다시 측정해야 한다.

| 관측 항목 | 결과 |
|---|---|
| 자차 속도 | 0m/s |
| 선택된 객체까지 거리 | 약 9.19m |
| 객체 위치 | map 기준 약 `(644.525, -318.028)` |
| 계획 경로를 따라간 차체와 객체 최소 간격 | 약 0.2446m |
| 여유 없는 차체 폴리곤 충돌 | 없음 |
| 0.3m 확장 폴리곤 충돌 | 있음. 실제 발행 detection polygon과도 일치 |
| 즉시 정지 주체 | `obstacle_stop` |
| 앞단 회피 실패 표시 | `INSUFFICIENT_DRIVABLE_SPACE` |
| 별도 outside-drivable-area 벽 | 자차로부터 약 78.6m의 유클리드 거리. 즉시 정지 원인과 구분 |

이 사례는 “차체가 부딪힌다”가 아니라 **30cm 측방 여유에 약 5.5cm 못 미친다**는 판정으로 설명된다. 지도 오차가 전혀 없다는 증명은 아니지만, 정지 발생을 설명하는 직접 근거는 폴리곤 여유다.

지도 파일을 바꾸지 않는 해결 방향은, 주행가능영역 안에서 작은 차선 내 횡보정 후보를 만들고 차체·후단 여유·추종 가능성을 다시 검사하는 것이다. 이조차 불가능하면 저속·정적 객체 등에 한정한 검증된 여유 조정 정책을 별도로 설계할 수 있다. **옆 차선 소속이라는 이유로 객체를 무시하거나 모든 충돌 여유를 일괄 제거하는 문제로 취급하지 않는다.** 이 문서 작성에서는 해당 정책을 구현하거나 충돌 설정을 바꾸지 않았다.

## 16. 근거와 유지보수

저장소 근거:

- [VTD 활성/비활성 모듈 preset](../config/vtd/planning/preset/vtd_low_memory_preset.yaml)
- [실행 스크립트](../autoware), [이미지 설정 설치 위치](../docker/vtd/Dockerfile)
- [종방향 PID 설정](../config/vtd/control/trajectory_follower/longitudinal/pid.param.yaml), [MPC 설정](../config/vtd/control/trajectory_follower/lateral/mpc.param.yaml)
- [게이트 설정](../config/vtd/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml)
- [공통 계획 제한](../config/vtd/planning/scenario_planning/common/common.param.yaml), [Velocity smoother](../config/vtd/planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml)
- [Obstacle stop 설정](../config/vtd/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml)
- [AEB 설정](../config/vtd/control/autonomous_emergency_braking/autonomous_emergency_braking.param.yaml)
- [Control 진단 연결](../config/vtd/diagnostics/control.yaml), [System 진단 연결](../config/vtd/diagnostics/system.yaml)
- [브릿지 설정](../vtd_overlay/src/vtd_ros2_bridge/config/vtd_bridge.param.yaml), [브릿지 제어 수신/송신 코드](../vtd_overlay/src/vtd_ros2_bridge/src/vtd_bridge_node.cpp)
- [Obstacle stop 구현](../vtd_overlay/src/autoware_motion_velocity_obstacle_stop_module/src/obstacle_stop_module.cpp)

로컬 근거 보관 위치: `/home/a/autoware-task-backups/stop-signal-reference-20260908.xiAFJk/`. `graph.txt`, 노드별 `.txt`에 parameter dump와 node info, `interfaces.txt`에 설치된 메시지 정의를 저장했다. 현재값 확인에서는 dirty 호스트 소스보다 실행 이미지/실행 파라미터를 우선했다.

설치 헤더 근거는 컨테이너 내부의 다음 경로다.

- `/opt/autoware/autoware_planning_validator/include/autoware/planning_validator/types.hpp`: handling type, 준비 입력.
- `/opt/autoware/autoware_pid_longitudinal_controller/include/autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp`: PID 상태 enum.
- 같은 디렉터리의 `debug_values.hpp`: diagnostic 배열 인덱스.

이미지 변경, 차량 모델 교체, plugin 로드 변경, 지도 규제 요소 변경, parameter runtime 변경 후에는 **활성 목록·토픽 연결·핵심 제한·진단 그래프**를 다시 확인한다. 이 문서는 특정 정지 원인을 감추기 위한 필터 해제 목록이 아니라, 최초 판단과 최종 제동을 추적하기 위한 기준이다.
