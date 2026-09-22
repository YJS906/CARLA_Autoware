# VTD 원본과 CARLA 실행 구성 비교 — 2026-09-22

VTD 코드 기반은 CARLA 실행 이미지에 포함되어 있지만, VTD와 같은 기능이 모두 동작하는 CARLA 전환 완료 상태는 아니다. 경로 생성 프로세스의 실제 충돌, 비활성화된 돌발 대응 모듈, 지도 규제·신호 입력 누락, 차량/센서 좌표/조향 변환 불일치가 확인됐다.

비교 대상은 `/home/a/Downloads/selfcar_2026_`, 같은 위치의 ZIP, `/home/a/carla_pp`, 실행 컨테이너 `selfcar-carla-autoware`, CARLA `Town05_Opt`이다. 파일·이미지·ROS 그래프·파라미터·CARLA 상태를 읽어 비교했다. 이 감사에서는 실행 설정 변경, 컨테이너 재시작, 경로 제출, AUTO 전환, 차량 이동 시험을 하지 않았다. 아래 문서는 점검 결과이며 수정 완료 보고서가 아니다.

## 1. 원본 포함 여부와 이미지 반영 범위

- ZIP의 `vtd_overlay/src`, `config/vtd`, `docker/vtd` 관련 421개 파일은 압축 해제 폴더와 바이트 단위로 같다. ZIP 파일 수정 시각이 최신이라는 이유로 내용도 더 최신이라고 볼 수 없다.
- Downloads 원본의 커스텀 overlay 패키지 7개는 CARLA 소스에 모두 존재한다. 현재 CARLA 소스에는 12개가 추가되어 총 19개다. 원본 overlay 파일 401개 중 343개 동일, 58개 변경, 누락 0개다. 이는 소스 포함 검사이며 동일 동작을 의미하지 않는다.
- CARLA 이미지 `sha256:d0b1c893b49ac37aaea70fca0e51dbb598bd5ee41d9e1d3d6baaeb9a1fd3e15e`는 최종 VTD 이미지 `sha256:ef00a3b3baa134b4ba5eddac52e0e27abc50f1e2edbff62fb0649b21e77e88ec`의 368개 레이어를 그대로 상속하고 6개 레이어를 추가했다.
- 실행 명령은 `/opt/selfcar_overlay/setup.bash`를 읽고 `planning_module_preset:=vtd_low_memory`를 사용한다. 해당 planning/control 소스를 덮는 host mount는 없다. 근거: [compose.yaml](../docker/carla/compose.yaml).
- 현재 호스트 overlay와 이미지에 보관된 build source는 742개 동일, 5개 변경, 11개가 같은 경로에 없다. 의미 있는 차이는 호스트의 `autoware_drivable_corridor_recovery` 패키지가 이미지에 설치되지 않았고, static avoidance의 최신 `approved_path.cpp`/`ApprovedPath` 유지 로직이 이미지 build source에 없다는 것이다. static avoidance 모듈 자체는 설치되어 있다. 이 차이는 Downloads 원본 7개 패키지 누락과는 다른 문제다.

## 2. 기능 설정 비교

아래는 원본과 실제 이미지의 preset 비교다. '켬'은 실행하도록 설정했다는 뜻이며, 지도·인지 입력과 주행 결과까지 검증됐다는 뜻이 아니다.

| 기능 | Downloads VTD 원본 | 현재 CARLA preset | 추가 판단 |
|---|---|---|---|
| 정적 장애물 회피 | 켬 | 켬 | behavior path planner의 launch_modules에도 존재 |
| 회피 목적 차선 변경 | 켬 | 켬 | 같은 목록에서 확인 |
| 좌·우 차선 변경 | 켬 | 켬 | 같은 목록에서 확인 |
| 동적 장애물 회피 | 끔 | 끔 | CARLA 전환 중 새로 빠진 기능은 아님 |
| 장애물 정지·감속·추종 | 켬 | 켬 | 객체 종류별 필터 검토 필요 |
| dynamic obstacle stop | 켬 | 켬 | 구현의 차량 필터가 PEDESTRIAN을 제외 |
| run_out | 켬 | **끔** | 돌발 진입 대응에서 원본과 차이 |
| road_user_stop | 켬 | **끔** | 도로 이용자 정지 대응에서 원본과 차이 |
| blind_spot | 켬 | **끔** | 원본과 차이 |
| 신호등·횡단보도·정지선·교차로 | 켬 | 켬 | Town05의 지도 규제 정보와 신호 입력 누락 |
| 긴급 제동 AEB | 포함 | 실행 중 | pointcloud 수신 확인, 제동 성능 미검증 |

근거: [현재 preset](../config/vtd/planning/preset/vtd_low_memory_preset.yaml), 원본 `Downloads/selfcar_2026_/config/vtd/planning/preset/vtd_low_memory_preset.yaml`, 컨테이너의 설치된 preset, 실제 behavior path planner 파라미터 조회.

`run_out`과 `road_user_stop`을 끈 주석에는 VTD UNKNOWN 객체에 대한 회피 경로와 정지 처리의 충돌을 피하려는 이유가 남아 있다. CARLA는 센서 검출로 객체를 분류하므로 이 설정을 그대로 적용하는 것이 적절한지는 별도 검토해야 한다. 무조건 전부 켜는 것으로 해결됐다고 볼 수 없다.

일반 obstacle_stop은 `ignore_crossing_obstacle: true`이며 UNKNOWN/CAR/PEDESTRIAN을 허용하고 pointcloud 기반 객체 처리는 끈 설정이다. Cruise 설정은 TRUCK/BUS/TRAILER/MOTORCYCLE/BICYCLE를 제외한다. CARLA의 버스·트럭·이륜차에도 의도대로 반응하도록 실제 적용 경로와 종류별 검증이 필요하다. dynamic obstacle stop 모듈 존재만으로 보행자 돌발 횡단 대응이 갖춰졌다고 판단하면 안 된다.

## 3. 현재 주행을 막는 경로 생성 충돌

실행 로그에서 `mission_planner_container` PID 179가 `exit code -11`로 종료됐다. 현재 ROS 그래프에서도 mission planner가 없고 `/planning/mission_planning/route`의 발행자는 0개다. 목적지를 아직 지정하지 않은 정상 대기 상태만으로 설명할 수 없다.

해당 로그 구간에는 RViz Goal Pose 설정 이후 프로세스 종료가 기록되어 있다. 이것만으로 특정 목표 위치가 충돌 원인이라고 확정하지는 않는다.

실제 crash artifact:

```text
/var/crash/_opt_ros_jazzy_lib_rclcpp_components_component_container_mt.1000.crash
Date: Tue Sep 22 15:26:57 2026
Signal: 11
Command: mission_planner_container /planning/mission_planning
```

ProcMaps와 현재 라이브러리를 대조한 결과, 충돌 주소는 `libautoware_lanelet2_extension_lib.so`의 `lanelet::ConstLanelet::centerline3d() const`에 해당한다. 명령은 `mov (%rsi), %rax`, RSI는 `0xffffffffffffffe8`로 유효하지 않은 Lanelet 객체 주소 역참조가 확인된다. 호출 스택이 끊겨 잘못된 객체를 넘긴 호출부와 정확한 원인은 아직 확정하지 못했다. 신호·횡단보도 누락이 이 충돌 원인이라는 증거는 없다.

최우선 작업은 충돌을 분리 재현하여 경로 생성이 정상적으로 살아 있고 유효/무효 Goal Pose에 대해 오류 없이 응답하는지 검증하는 것이다.

## 4. 지도와 신호 연결

| 지도에서 직접 집계 | 현재 Town05 | VTD topology_fixed |
|---|---:|---:|
| 전체 lanelet | 486 | 2,270 |
| road lanelet | 486 | 1,808 |
| crosswalk lanelet | 0 | 195 |
| regulatory_element | 0 | 558 |
| traffic_light 규제 요소 | 0 | 363 |
| crosswalk 규제 요소 | 0 | 195 |
| 신호 규제 요소의 고유 ref_line 정지 경계 | 0 | 363 |

지도 경로는 `/home/a/autoware_data/maps/Town05/lanelet2_map.osm`과 `/home/a/vtd_autoware_maps/HL_FMA_VTD_LivingLab_topology_fixed/lanelet2_map.osm`이다. VTD 지도는 신호 정지선 363개 중 16개만 명시적인 `type=stop_line` 태그가 있으므로 단순 태그 수만으로 정지선을 세면 안 된다.

CARLA Town05 OpenDRIVE에는 동적 신호 54개, 정지 표지 5개, crosswalk object 66개가 존재한다. 현재 Autoware Lanelet2에는 대응 정보가 변환되지 않았다. CARLA 화면의 신호등·횡단보도가 보인다는 사실은 Autoware가 그것을 이해한다는 뜻이 아니다.

- [Town05 시작 스크립트](../scripts/carla/carla_autoware)는 신호 인식을 false로 설정한다.
- 경량 센서 모드에서는 [카메라 신호 인식용 릴레이](../config/carla/launch/autoware_carla_interface.launch.xml)도 생략한다.
- 실제 `/perception/traffic_light_recognition/traffic_signals` 발행자와 관측 메시지는 모두 0개다.
- VTD에는 신호 ID를 Autoware 지도 group ID로 변환하는 브리지가 있다. 현재 CARLA 구성에서는 동등한 상태 발행/ID 매핑 구현을 찾지 못했다.
- Traffic-light planner와 crosswalk planner는 켜져 있어도 현재 지도에서 적용 대상 규제 요소를 찾을 수 없다. 인식 옵션 하나만 켜는 것으로 해결되지 않는다.
- 독립 stop-sign 정지는 TrafficSign/stop_sign 규제 요소가 필요하다. 원본 VTD 지도에도 이 요소는 없어, 원본의 모든 독립 정지 표지 대응이 완전했다고 단정하지 않는다.

필요 작업은 Town05 규제 지도 보강, 실제 CARLA 신호와 지도 ID 연결, 신호 상태 공급, 교차로·횡단보도별 시험이다.

## 5. 차량 및 CARLA 브리지 변환 문제

실행 중인 `carla_ros.py`와 `carla_autoware.py`는 현재 로컬 소스와 해시가 같아 다음 문제는 현재 브리지에 해당한다.

| 항목 | 실제 CARLA ego | Autoware 실행 설정 |
|---|---:|---:|
| 차량 | Toyota Prius | sample_vehicle의 VTD IONIQ6 설정 |
| 휠베이스 | 약 2.819 m | 2.944 m |
| 전체 길이 | 약 4.514 m | 5.044 m |
| 폭 | 약 2.007 m | 1.896 m |
| 높이 | 약 1.525 m | 2.5 m |

CARLA 값은 actor 138의 physics/bounding box를 읽은 값이고, Autoware 값은 설치된 vehicle_info와 실제 planner/AEB 파라미터를 대조했다. 충돌 여유와 회전/제어 모델이 맞지 않는다.

다음은 별도의 코드상 문제다.

1. **센서 위치와 localization 원점:** `carla_ros.py`는 CARLA actor 원점을 pose로 발행하고 `carla_state_publisher.cpp`가 이를 그대로 `base_link` 위치로 사용한다. 센서 생성은 뒤 차축 기준 calibration에서 1.425 m를 빼서 actor에 붙인다. 실제 LiDAR 상대 x는 -0.390 m인데 ROS calibration은 +1.035 m다. 카메라와 IMU/GNSS도 같은 차이가 있다. 현재 TF로 변환된 장애물 위치가 실제 배치에 비해 전방으로 약 1.425 m 이동하는 구조다.
2. **차량 상태 단위·부호:** `carla_ros.py:775-776`은 lateral velocity에 CARLA→ROS 부호 변환을 하지 않고, `Actor.get_angular_velocity()` 값을 rad/s 변환 없이 heading_rate로 보낸다. 해당 CARLA API는 deg/s다. [CARLA 0.9.16 API](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor)
3. **조향 입력:** raw converter는 조향 변환이 꺼져 있어 타이어 각도(rad)를 넘기지만, `carla_ros.py:693`은 이를 CARLA 정규화 steer 값으로 사용한다. 조향 곡선의 속도 입력도 속력 대신 `abs(world_velocity.x)`여서 같은 속도라도 차량 진행 방향에 따라 값이 달라진다. 주행 시험 전에 단위와 실제 최대 조향각에 맞춰야 한다.
4. **기어/명령 끊김 처리:** CARLA 어댑터에 기어 명령 구독이 없고 기어 상태는 DRIVE, 제어 모드는 AUTONOMOUS로 고정된다. 마지막 제어 값을 유지하며 어댑터 자체의 VTD식 0.5초 명령 watchdog 제동도 확인되지 않는다. 다른 Autoware watchdog이 있으므로 현재 차량 폭주가 관측됐다는 뜻은 아니다. 후진·주차·명령 단절 대응의 동등성은 미완성이다.

관련 로컬 소스는 `src/universe/autoware_universe/simulator/autoware_carla_interface/`와 `src/universe/autoware_universe/vehicle/autoware_raw_vehicle_cmd_converter/`다. 차량 모델·rear-axle 원점·센서 calibration·TF를 하나의 기준으로 맞춰야 한다.

## 6. 실제 입력 관측과 시험 한계

12초 동안 제어 명령을 발행하지 않고 구독만 하여 확인했다.

| 입력/출력 | 관측 |
|---|---|
| /clock | 241개, 약 20 Hz |
| ego 속도/조향 상태 | 각각 241개 |
| LiDAR | 121개, 약 10 Hz |
| CenterPoint 검출 | 121개, 차량 2~3개 검출 |
| tracking / predicted objects | 각각 121개, 실제 데이터 연결 확인 |
| AEB 필터 pointcloud | 120개, 실제 입력 연결 확인 |
| traffic_signals | 발행자 0, 메시지 0 |
| mission_planning/route | 발행자 0, 메시지 0 |
| planning/trajectory | 발행자는 있으나 메시지 0 |
| control_cmd / actuation_cmd | 발행자는 있으나 메시지 0 |

즉 CARLA 센서→객체 검출→추적→예측 연결 전체가 끊긴 상태는 아니다. AEB에도 pointcloud가 공급된다. 그러나 경로 생성이 죽어 주행 궤적과 제어 명령이 없고 AEB 로그도 control predicted trajectory를 기다린다. 이 상태로 장애물 정지·보행자 대응·끼어들기 제동 시험을 성공했다고 판단할 수 없다.

원본 VTD는 시뮬레이터의 객체 위치·속도·크기를 직접 입력하는 구조였고, CARLA는 센서 인지 과정을 거친다. 탐지 지연·누락·종류 분류·좌표 오차가 달라져 원본 planner가 포함되어 있어도 결과가 같다고 보장할 수 없다.

점검 시 ego는 기존 시작점에 정지해 있었고 CARLA walker 수는 0이었다. 따라서 새 보행자/끼어들기 시나리오의 물리 주행과 Autoware 반응은 이번 감사에서 시험하지 않았다. 센서 메시지 수신과 실제 제동 성공은 다른 검증 항목이다.

## 7. 수정과 검증 순서

1. mission planner의 SIGSEGV를 해결하고 경로 생성·변경의 회귀 검사를 수행한다.
2. CARLA 실제 차량에 맞춰 vehicle_info, localization/센서 원점, 조향 입력, yaw-rate/속도 부호를 통일한다.
3. 현재 호스트에서 사용할 소스 버전을 확정해 실행 이미지를 다시 만들고 포함 여부를 검증한다.
4. CARLA 전용 planning preset을 분리하여 run_out/road_user_stop/blind_spot과 객체 종류 필터를 검토한다. 기존 VTD 회피와의 충돌까지 시험한다.
5. Town05 규제 지도와 신호 상태 연결을 구현한다.
6. 정적 장애물 정지·회피, 앞차 추종, 차량 끼어들기, 보행자 돌발 횡단, 신호 정지·출발을 각각 시험하고 속도·거리·지연·충돌 여부를 기록한다.

현재 결론은 'VTD 기반 코드가 포함된 CARLA 통합 중간 단계'다. 모든 필요한 기능이 CARLA에 맞게 연결되고 검증된 상태는 아니다.
