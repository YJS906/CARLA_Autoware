# CARLA Town05 수정 및 검증 기록 — 2026-09-22

[VTD/CARLA 비교 점검](carla-vtd-audit-20260922.md)에서 확인한 경로 생성 충돌, 지도 규제 누락, 차량·센서 변환 불일치와 실행 이미지의 소스 반영 차이를 수정했다. **최종 이미지를 빌드·적용하고 실제 ROS 입력, 모듈 로드, 경로·궤적·제어 연결까지 확인했다.** AUTO 전환 및 실제 이동을 수반하는 폐루프 주행 시험은 수행하지 않았다.

## 경로 생성 충돌의 원인과 수정

원래 Town05 지도에는 연속한 경계점의 좌표가 같은 구간이 있었다. 설치된 Lanelet2 중심선 재표본화 코드가 길이 0인 구간을 나누면서 **27개 lanelet의 중심선에 NaN**을 만들었다. RouteHandler는 경로 비용을 비교하기 전에 성공 여부를 설정하므로, NaN 비용의 경로를 선택하지 못해도 `true`와 빈 경로를 반환했다. 이후 경로 구간 생성에서 빈 벡터의 `.back()`을 사용하여 mission planner가 SIGSEGV로 종료되는 것을 분리 재현했다.

- 지도 생성기는 연속 좌표 중복 38개를 제거하며, 원래 도로 형상과 경계 시작·끝 노드 ID를 유지한다.
- mission planner는 성공으로 보고된 빈 경로 및 비유한 중심선을 거부한다. 손상된 지도 결과를 도달 불가능한 목표로 취급하여 다른 차로로 옮기지 않는다.
- 목표 차량 외곽 검사의 앞·뒤 이웃 탐색은 반복문으로 바꾸고, 유한한 양의 길이와 방문 ID를 검사한다. NaN 거리나 순환 연결 때문에 탐색이 무한 재귀에 빠지지 않는다.
- 시작·목표 개수 부족과 빈 최종 구간도 정상적인 경로 거부로 처리한다. 예외를 일괄 무시하는 방식은 사용하지 않았다.

수정 소스: [default_planner.cpp](../vtd_overlay/src/autoware_universe/planning/autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp), [회귀 테스트](../vtd_overlay/src/autoware_universe/planning/autoware_mission_planner_universe/test/test_lanelet2_plugins_default_planner.cpp).

## CARLA 실행 구성 변경

| 범위 | 수정 내용 |
| --- | --- |
| planning preset | CARLA 전용 `carla_city`를 추가하여 `run_out`, `road_user_stop`, `blind_spot`을 켰다. 정적 회피, 회피 목적 차선 변경, 좌·우 차선 변경, 신호·횡단보도·정지선·교차로, 장애물 정지·감속·추종 설정을 유지한다. `dynamic_obstacle_avoidance`는 원본과 같이 꺼져 있다. |
| 객체 필터 | stop/slow_down/cruise의 트럭·버스·트레일러·이륜차 분류를 포함하고, obstacle_stop의 횡단 객체 무시 설정을 해제했다. run_out과 road_user_stop은 보행자·이륜차 등 각 모듈의 대상 분류에 맞췄다. |
| 차량·센서 | CARLA 0.9.16 Prius의 휠베이스 약 2.819 m와 bounding box 약 4.514 × 2.007 × 1.525 m를 반영했다. `base_link`를 후륜축의 지면 투영점으로 통일하고 센서 외부 보정, 속도·각속도 단위와 좌표 부호를 맞췄다. |
| 제어 브리지 | ROS 타이어 각도를 CARLA 정규화 조향으로 변환하고 속도별 조향 한계를 반영한다. 기어·후진·중립·주차를 처리하며, 이동 중 방향 전환은 정지까지 보류한다. 초기·비유한·오래된 명령은 제동 처리한다. |
| AEB | VTD 합성 객체의 UUID 제외 가정을 CARLA에 적용하지 않도록 하여 전체 장애물 점군을 AEB에 전달하는 구성으로 바꿨다. 제동 성능은 주행 미검증이다. |
| 카메라 | spectator는 world 변경 후 world·ego·spectator 핸들을 다시 취득하고, snapshot 없는 actor 및 RPC 오류를 재시도한다. |
| 이미지 | 추적되는 `carla_overlay`와 `vtd_overlay`에서 mission planner, static obstacle avoidance, drivable corridor recovery, CARLA interface 네 패키지를 `/opt/carla_overlay`로 빌드하고 마지막에 source한다. 최신 static avoidance 소스와 누락됐던 recovery 패키지를 빌드 대상에 포함한다. |

설정 파일은 [CARLA planning](../config/carla/planning/), [차량](../config/carla/vehicle/), [센서](../config/carla/sensors/), [브리지 설명](../carla_overlay/README.md), [Dockerfile](../docker/carla/Dockerfile)에 있다. 모듈을 빌드하거나 preset을 켠 사실만으로 런타임 활성화 및 주행 성능이 검증된 것은 아니다. VTD 기본 이미지를 상속하되 CARLA 이미지 안에서 전용 설정을 설치한다.

## Town05 지도와 신호 입력

원래 도로 lanelet 486개를 유지하면서 CARLA 원본 자료에 근거한 신호 그룹 54개(접근 차로 111개), 횡단보도 66개, 교차로 진행 방향 태그 211개를 추가한다. OpenDRIVE에 선언된 정지 표지 5개는 해당 차로의 `TrafficSign/stop_sign` 규제 8개로 연결한다. 신호 브리지는 실제 CARLA 신호 상태를 지도 group ID로 변환하여 `/perception/traffic_light_recognition/traffic_signals`에 발행한다. 지도와 manifest는 SHA256으로 연결한다.

범위상 제한도 남아 있다. 원본 도로 polygon 중 Shapely 검사에서 유효하지 않았던 6개는 도로 형상을 임의로 고치지 않고 보존했다. OpenDRIVE 선언과 일치하지 않는 추가 scene 정지 trigger 25개는 점검 기록만 남기고 규제를 임의 추가하지 않았다. 따라서 전체 지도와 모든 교차로의 주행 검증을 완료한 상태는 아니다. 근거: [지도 생성 결과](../config/carla/maps/Town05_regulatory_report.json).

## 완료한 오프라인 검증

mission planner 패키지의 **16개 gtest가 모두 통과**했다. 빈 체크포인트, NaN 비용으로 인한 빈 경로, 기존 preferred route에 비유한 중심선이 포함된 경우, 앞·뒤의 잘못된 이웃 lanelet, 거부 이후 유효 요청의 복구를 포함한다. 브리지의 변환·메시지·기어·timeout 검증은 가짜 actor를 사용하는 14개 테스트로 확인했다.

최종 이미지에서 브리지 14개와 신호 입력 10개 테스트가 모두 통과했다. 신호 검사는 시간 정지, RPC 실패, actor 교체, 중복 ID, 지도 불일치와 알 수 없는 신호의 RED 처리를 포함한다. 실제 Autoware C++ 지도 로더로 신호 규제 54개, 횡단보도 규제 66개(각각 polygon 1개), 정지 표지 규제 8개, 비유한 중심선 0개를 검증했다.

실제 Town05 OSM을 C++ Lanelet2/Autoware 코드로 읽고 중심선을 생성한 경로 회귀 결과는 다음과 같다. 시작점은 충돌 당시 기록된 ROS 좌표 `(-145.5, +0.9979, yaw=π)`를 사용했다.

| 지도·목표 | 결과 |
| --- | --- |
| 원본 지도 + 문제 발생 목표 `(-268.808, -45.1585, yaw=-1.52857)` | NaN 중심선 27개를 확인하고 빈 결과로 거부, 프로세스 정상 종료 |
| 보정 지도 + 같은 목표 | 중심선 모두 유한함. 목표 차로 진행 방향과 반대인 목표를 거부, 정상 종료 |
| 보정 지도 + lanelet 6183 중심의 유효 목표 `(-264.260333454229, -80.0957277393601, yaw=2.17606833889921 rad)` | 경로 구간 11개 생성, 정상 종료 |
| 보정 지도 + 지도 밖 목표 `(10000, 10000)` | 빈 결과로 거부, 정상 종료 |

문제 발생 목표가 거부되는 것은 그대로 남겨 두었다. 해당 위치는 lanelet 6183 안에 있으나 지정 방향이 그 차로와 거의 반대다. 유효한 차로와 방향을 지정해야 한다.

최초 라이브 검증 스크립트에는 별도의 입력 실수가 있었다. lanelet 6183의 다른 위치에서 확인한 `yaw=1.581`을 위 곡선 구간의 중심 목표에 잘못 사용하여 차량 외곽이 차로를 벗어났고, planner가 정상적으로 목표를 거부했다. 오프라인 테스트는 처음부터 그 위치의 중심선 접선 방향을 계산해 사용했다. 최종 이미지의 `/opt/carla_overlay` 바이너리, 새 Prius 설정, 현재 지도로 다시 분리 확인한 결과, 잘못된 방향은 거부되고 `yaw=2.17607`로 고친 같은 목표는 11개 구간을 생성했다. 차량 외곽 검사를 완화하지 않았다.

검증은 네트워크가 차단된 `selfcar-mission-repair-20260922` 컨테이너, `ROS_DOMAIN_ID=188`에서 실행했으며 실제 CARLA tick이나 차량 제어를 사용하지 않았다. 로그, XML 결과와 재현 코드는 `/tmp/carla-mission-repair-20260922`에 보존했다. 테스트 완료 후 해당 격리 컨테이너는 중지했다.

## 적용 명령과 남은 확인

아래는 최종 이미지와 지도를 적용할 때 사용할 명령이다. Town05 원본 지도 자산, CARLA 0.9.16, 기본 이미지 `selfcar-2026-vtd:local`이 설치된 환경을 전제로 한다. `prepare_town05_map`은 원본 백업을 보존하고 지도·신호 manifest를 함께 생성하며, CARLA에 연결하거나 tick을 진행하지 않는다.

```bash
cd /home/a/carla_pp
./scripts/carla/build_carla_image
./scripts/carla/carla_autoware stop
./scripts/carla/prepare_town05_map
```

실행 중인 기존 Autoware를 종료한 후 CARLA 서버가 열린 상태에서 새 구성을 시작한다. `stop`은 기존 시뮬레이션의 차량·보행자·센서를 정리한다. `start-town05`에는 AUTO 전환 명령이 없다.

```bash
./scripts/carla/carla_autoware start-town05
./scripts/carla/carla_autoware status
```

아래 실행 검증까지 완료했다. 정지선 정지·녹색 출발, 보행자 돌발 횡단, 차량 끼어들기, 정적 회피, AEB 제동 거리와 곡선 추종은 별도의 폐루프 주행 시험 대상이다.

## 최종 이미지 라이브 확인

실제 실행 이미지: `sha256:1d44cf81b364baca1f1db296a8735135a81a2a696f798483f3ddb907bfd45caf`.
지도 SHA256: `413bf0a9e9c08357d284364d8a70d9a1e94fed64c1a8b8854df65b2a214468a8`.
수정한 네 패키지는 모두 `/opt/carla_overlay`에서 선택되며, 이미지의 mission planner 소스 해시도 최종 호스트 소스와 일치한다.

| 실제 확인 | 결과 |
| --- | --- |
| mission planner 및 경로 서비스 | 종료되지 않고 유효 목표에 경로 11개 구간 생성, 잘못된 방향의 목표는 오류 응답으로 거부 |
| 주행 궤적 | 유효 경로 설정 후 15초 동안 `/planning/trajectory` 149개 수신, 모두 비어 있지 않고 속도 값이 유한함 |
| 제어 명령 | 같은 15초 동안 300개 수신. STOP 상태이므로 요청 속도는 0. 경로 삭제 후 별도 3초 관측에서도 actuation 60개 수신(가속 0, 제동 0.6) |
| 검출·추적·예측 | CenterPoint 검출 약 10 Hz. 12초 동안 추적·예측 각각 121개 수신, 실제 차량 객체 포함 |
| 신호 | 54개 그룹 약 10 Hz. 40회 표본에서 모든 그룹의 색상이 CARLA 신호와 일치, 누락·중복 ID 0개 |
| 모듈 | 실제 `launch_modules`와 관련 토픽에서 RunOut, RoadUserStop, BlindSpot 로드 확인 |
| 차량 파라미터 | 세 planner와 AEB에서 Prius 휠베이스 2.81918946 m와 최대 조향각 1.2217304764 rad 확인 |
| 좌표 | 정지 상태 CARLA 후륜 기준 pose와 ROS pose 오차 약 13 μm, 실제 센서 위치와 TF 오차 약 5–11 μm |
| 시험 종료 상태 | ego 이동 거리 0, 속도 0. 시험 경로 삭제 성공 및 UNSET 확인. AUTO 미전환, STOP 유지 |

배경 차량은 실행 중이며 관측 시 보행자는 0명이었다. 돌발 보행자/끼어들기 시나리오의 실제 정지 성공을 의미하지 않는다. RViz sample vehicle의 외형 mesh는 기존 Lexus 표현이며, 위의 물리·계획 치수는 Prius 설정이다.

원시 측정 결과는 [검증 JSON](validation/carla-repair-20260922.json)에 보관했다. 최초 generic detection 토픽 탐색은 실제 발행 경로와 달라 0개였으므로, 별도 CenterPoint/clustering 측정 결과를 사용한다. 경로 미설정 상태의 읽기 전용 관측과 임시 경로 시험의 출력 수신은 별도 구간이다.

원본 지도는 `/home/a/autoware_data/maps/Town05/lanelet2_map.before-carla-repair.osm`, 교체 직전 bundle은 같은 디렉터리의 `backups/carla-map-20260922T071442Z-07aea423`에 보관했다. 이전 이미지는 `selfcar-2026-carla:before-repair-20260922`로 보관했다. 지도 준비 스크립트는 이미 준비된 지도/manifest가 일치하면 변경하지 않으며, 이후 편집으로 서로 달라진 경우 기존 파일을 덮어쓰지 않고 중단한다.
