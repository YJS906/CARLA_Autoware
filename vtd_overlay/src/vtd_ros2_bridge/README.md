# VTD–ROS 2–Autoware bridge

`인터페이스 API.xlsx`에 정의된 HLVTD 데이터를 ROS 2/Autoware 토픽으로
변환하고, Autoware 제어 명령을 Host로 되돌려 보내는 브릿지입니다. 학생 PC와 Host
사이에서는 TCP 9910, 기존 RTSP 8554, UDP 9912만 사용합니다. 2026-09-04의 API-only
수신 방식을 복원했으며, 별도 raw RDB TCP 연결이나 그 입력에 대한 의존성은 없습니다.

## 데이터 흐름

| 방향 | VTD / ROS 입력 | 브릿지 출력 |
|---|---|---|
| HLVTD → ROS | TCP 9910 DATA의 `egoX/Y/Z`, `egoHeading/Pitch/Roll` | `/localization/kinematic_state`, `/localization/acceleration`, `/vehicle/status/velocity_status`, TF `map→base_link`, `/clock` |
| VTD → Autoware | 첫 ego pose 수신 이후 | `/localization/initialization_state = INITIALIZED` |
| VTD bridge → Autoware | VTD ground-truth 주행 모드 | `/perception/occupancy_grid_map/map` (ego 주변 free-space grid) |
| VTD bridge → Autoware | VTD ground-truth 주행 모드 | `/perception/obstacle_segmentation/pointcloud` (VTD 객체 bbox 합성 점군) |
| HLVTD UDP `9912` → ROS | 독립 `vtd_lidar_node`가 `IVHL` 분할 패킷의 world-coordinate float32 XYZ 수신 | `/sensing/lidar/concatenated/pointcloud` (`sensor_msgs/PointCloud2`) |
| HLVTD RTSP `8554` → ROS | 기존 RTSP 수신 경로 | camera image topics |
| HLVTD → Autoware | TCP 9910 DATA의 단일 신호등 `id/state` | `/simulator/input/traffic_signals` → `/perception/traffic_light_recognition/traffic_signals` |
| Autoware planning → RViz | 현재 도로의 맵 최고속도 | `/planning/scenario_planning/applied_velocity_limit` (표시 전용) |
| Autoware → HLVTD | control/turn-indicator command topics | TCP `9910` headerless CONTROL (`steering`, `targetAccel`, `turnSignal`) |

인터페이스 API의 상태 필드는 다음 ROS 토픽으로도 그대로 제공됩니다.

| 인터페이스 API | ROS 토픽 | 메시지 |
|---|---|---|
| `egoX/Y/Z`, `egoHeading/Pitch/Roll` | `/vtd/ego_state` | `vtd_ros2_bridge/msg/VtdEgoState` |
| `objects[].id/x/y/z/heading/speed/length/width/height` | `/vtd/objects` | `vtd_ros2_bridge/msg/VtdObjectArray` |
| `trafficLights[].id/state` | `/vtd/traffic_lights` | `vtd_ros2_bridge/msg/VtdTrafficLightArray` |
| `steering`, `targetAccel`, `turnSignal` | `/control/command/*` | Autoware control and turn-indicator messages |

`Object(Array=30)`은 고정 30슬롯으로 해석하며 ID가 0인 슬롯은 비어 있는
슬롯으로 처리합니다. 신호등은 문서에 정의된 `id`와 `state(0..6)`를 그대로
사용합니다.

Autoware 표준 출력은 `autoware_perception_msgs/msg/TrafficLightGroupArray`입니다.
현재 9910용 기본 매핑은 **`config/hlvtd_traffic_light_id_map.csv`**이며,
`hlvtd_id,autoware_group_id` 두 열을 사용합니다. Host가 선택한 접근 도로의 신호 ID를
현재 Lanelet2 지도의 신호등 regulatory-element ID로 변환합니다.
기존 `config/traffic_light_id_map.csv`는 개별 RDB/OpenDRIVE 램프 ID용 자료이며,
9910에는 사용하지 않습니다. 잘못된 스키마·중복 소유·빈 파일은 시작 시 오류로 처리합니다.

Host 대응표는 `config/hlvtd_traffic_light_approaches.json`에 보관합니다. 설치된
`libHLVTD.so.1.0.0`의 `HlvtdTrafficLightMapping::kEntries` 135개 항목에서 확인한
도로 ID·lane sign·9910 ID·green mode이며, 해당 바이너리 SHA256도 함께 기록했습니다.
이 빌드와 다른 Host 대응표를 사용하는 경우 먼저 계약을 대조해야 합니다.
9910 ID를 OpenDRIVE controller 번호나 개별 램프 번호로 임의 간주하지 않습니다.

새 생성기는 Host의 도로·주행 방향에 해당하는 출구 위치, laneOffset을 포함한 주행
차로 폭, 정지선 위치, CommonRoad 원본 차로 진행 방향을 함께 검사합니다.
OSM 경계 way의 좌표 순서만으로 진행 방향을 추정하지 않습니다. 하나의 정지선 그룹이
여러 접근 도로에 대응하거나 기하 조건이 맞지 않으면 제외합니다. 지도·정지선·경로
자체를 변경하거나, 미확인 ID를 가까운 교차로에 강제로 연결하지 않습니다.

2026-09-07의 고정 지도에서는 120개 Host ID를 280개 사용 중인 지도 그룹에 연결했습니다.
`11`은 도로 `429`, lane sign `+1`의 그룹 `463107/463109/463111/463113`에 대응합니다.
전체 315개 그룹 중 33개는 참조 차로가 없고, 사용 중인 그룹 `463021/463295` 두 개는
대응 기하가 일치하지 않아 제외했습니다. 지도에 대응하는 정지선이 확인되지 않은
Host 접근 ID 15개도 미매핑으로 남깁니다. 제외 목록은 생성기의 audit report에 기록됩니다.

`인터페이스 API.xlsx`의 `sheet1!H20` 정의에 따라 상태를 구분합니다:
`1=적색`, `2=황색`, `3=녹색 원형`, `4=녹색 좌회전 화살표만`,
`5=녹색 원형+좌회전 화살표`, `0=미할당(UNKNOWN)`입니다.
`6=점멸`은 기존 황색 점멸 표현을 유지합니다(API에 점멸 색상 구분이 없음).
좌회전 전용 상태 4에 직진 허용 녹색 원형을 추가하지 않습니다.

`traffic_light.publish_unmapped_ids`는 false여야 합니다. 미매핑 수신 ID는 raw 토픽에는
남기되 Autoware 그룹 ID로 그대로 발행하지 않습니다. 진단은 WARN으로 바뀌고
`traffic_light_last_id/state`, `traffic_light_id_namespace`, `traffic_light_id_map_file`,
`unmapped_traffic_light_ids`로 문제를 표시합니다. 신호 미수신 시 정지/통과 정책은
기존 planner 설정을 유지하므로, 미매핑이 자동 정지를 보장한다는 뜻은 아닙니다.

RViz 속도제한 표시는 전역 외부 상한인 `current_max_velocity` 대신 behavior path에서
ego와 가장 가까운 lane ID를 찾고, 그 lane의 reference-path 최고속도를 표시용
`VelocityLimit` 메시지로 변환한 토픽을 사용합니다. 정지선이나 곡률로 낮아진 목표속도가
아니라 현재 도로의 맵 최고속도를 표시하며, planner나 controller 입력으로 되돌아가지
않습니다.

```bash
/home/a/.venvs/crdesigner/bin/python \
  scripts/generate_hlvtd_traffic_light_id_map.py \
  /home/a/vtd_autoware_maps/HL_FMA_VTD_LivingLab_topology_fixed/HL_FMA_VTD_LivingLab_laneoffset_fixed.xodr \
  /home/a/vtd_autoware_maps/HL_FMA_VTD_LivingLab_topology_fixed/lanelet2_map.osm \
  config/hlvtd_traffic_light_approaches.json \
  --commonroad /home/a/vtd_autoware_maps/HL_FMA_VTD_LivingLab_topology_fixed/map.commonroad.xml \
  --audit-report /tmp/hlvtd_traffic_light_mapping_report.json
```

패키지 디렉터리에서 실행하면 CSV를 표준 출력으로 생성합니다. 감사 결과와 지도 해시를
검토한 뒤 `config/hlvtd_traffic_light_id_map.csv`를 갱신하고 브릿지를 다시 빌드합니다.
지도 또는 Host 계약 변경 시 다시 생성해야 하며, 포트 추가는 필요하지 않습니다.

TCP 9910은 `vtd_bridge_node`가 HLVTD의 양방향 DATA/CONTROL 채널로 사용합니다.
DATA는 고정 1109바이트, CONTROL은 고정 9바이트입니다. TCP partial read와 한
번에 여러 record가 들어오는 경우를 모두 처리하며, 밀려 들어온 DATA는 최신
record만 ROS에 반영합니다. 신호등도 이 DATA의 단일 `id/state`를 사용합니다.
별도 raw RDB 연결, 전역 신호등 목록 수신 및 RDB 형상/조향 대기는 하지 않습니다.
UDP 9912는 LiDAR 전용입니다. LiDAR
UDP 9912는 제어용 `vtd_bridge_node`와 분리된 `vtd_lidar_node`가
`0.0.0.0:9912`에 직접 bind합니다. 브리지는 수신 데이터의 frame ID,
packet index/count, 전체 point count와 point offset을
검증하고 모든 조각을 재조립한 다음 `(0,0,0)` miss point를 제외해 PointCloud2로
발행합니다. 현재 `scenario_simulation` 구성에서는 raw LiDAR 전처리 체인이
실행되지 않으므로, Autoware와 RViz가 실제 구독하는
`/sensing/lidar/concatenated/pointcloud`로 재조립된 프레임을 직접 발행합니다.
구조나 길이가 잘못된 패킷은 PointCloud2로 발행하지 않고
`lidar_udp_rejected_packets`와 `parse_errors`에 기록합니다.

`accelTgt`와 Autoware acceleration은 모두 m/s²이고, `steeringTgt`와 Autoware `steering_tire_angle`은 모두 앞바퀴 등가 조향각(rad)입니다. 따라서 steering wheel ratio나 pedal percentage로 바꾸지 않습니다.

## 빌드와 실행

VTD와 Autoware가 호스트에서 실행 중일 때 다음 한 줄로 빌드 후 브릿지를 실행할 수 있습니다.

```bash
cd /home/a/vtd_ros2_bridge_ws
./src/vtd_ros2_bridge/scripts/run_bridge_docker.sh
```

이 스크립트는 현재 설치된 Autoware Jazzy 이미지, host network, host IPC를 사용합니다. 직접 실행할 경우에는 다음 환경이 필요합니다.

```bash
export VTD_ROOT=/home/a/VIRES/VTD.2025.2
source /opt/ros/jazzy/setup.bash
source /opt/autoware/setup.bash
cd /home/a/vtd_ros2_bridge_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch vtd_ros2_bridge vtd_bridge.launch.py
```

다른 VTD 호스트를 쓰려면 launch 인자로 바꿉니다.

```bash
./src/vtd_ros2_bridge/scripts/run_bridge_docker.sh hlvtd_host:=192.168.0.20
```

제어 설정은 `config/vtd_bridge.param.yaml`, LiDAR 설정은
`config/vtd_lidar.param.yaml`에서 각각 읽습니다. HLVTD LiDAR UDP 수신 주소나
포트를 바꾸려면 다음 launch 인자를 사용합니다.

```bash
ros2 launch vtd_ros2_bridge vtd_bridge.launch.py \
  lidar_udp_bind:=0.0.0.0 lidar_udp_port:=9912
```

## Host 인터페이스 설정

1. TCP 9910은 Host가 listen하고 학생 PC가 하나의 persistent connection을
   만듭니다. 같은 socket에서 Host→학생 DATA와 학생→Host CONTROL이 오갑니다.
2. RTSP 8554는 기존 영상 수신 경로를 사용합니다.
3. UDP 9912는 Host가 학생 PC로 LiDAR를 전송합니다. `vtd_lidar_node`가
   `0.0.0.0:9912`에 bind합니다.
4. 학생 PC에서 Host의 raw RDB 포트에 연결하지 않습니다. Host 내부의 VTD 통신은
   별개이며, 이 브릿지에서는 `traffic_light.rdb_host/port` 설정도 사용하지 않습니다.

## 좌표 동기화

VTD inertial과 ROS는 모두 x-forward/y-left/z-up인 우수 좌표계입니다. 현재 Lanelet2 경로는 2-D 기준으로 사용하므로 `flatten_z: true`가 기본이며, ego TF/odometry와 VTD 객체 pose의 z를 0으로 발행합니다. 객체 높이는 `Shape.dimensions.z`와 합성 obstacle pointcloud에 그대로 보존됩니다.

```yaml
map_offset:
  x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0
flatten_z: true
```

VTD 시나리오 원점과 Lanelet2 원점이 다르면 2-D rigid transform만 보정합니다.

```text
p_autoware = R(map_offset.yaw) · p_vtd + [map_offset.x, map_offset.y, map_offset.z]
```

좌표를 맞출 때는 VTD와 RViz에서 동일한 두 지점의 `(x,y)`를 얻어 yaw를 먼저 계산하고 translation을 계산합니다. y축 부호를 임의로 뒤집으면 좌·우 차선과 조향 부호가 함께 깨지므로 reflection 파라미터는 제공하지 않습니다.

브릿지가 `map→base_link`를 게시하므로 다른 localization 노드가 같은 TF를 동시에 게시하면 안 됩니다. VTD ground truth를 localization으로 쓸 때는 기존 NDT/localization TF publisher를 끄거나 `publish_map_to_base_tf: false`로 설정해야 합니다.

## Autoware 연결 시 주의점

- 이 노드는 `/vehicle/status/control_mode`를 `AUTONOMOUS`로 합성합니다. 실제 VTD 운전자 모드 상태가 아니라 Autoware vehicle interface 계약을 만족시키기 위한 값입니다.
- 첫 Control 메시지 전에는 가속·조향을 덮어쓰지 않고 기본 기어 `D`만 VTD에 전송합니다. 첫 Control 이후 명령이 `control_timeout_sec` 동안 끊기면 `watchdog_deceleration`을 보냅니다.
- API에는 gear feedback이 없으므로 Autoware가 승인한 기어 명령을 `/vehicle/status/gear_status`에 반영합니다.
- 좌/우 방향지시등과 비상등 명령은 각각 `/control/command/turn_indicators_cmd`, `/control/command/hazard_lights_cmd`에서 받아 9910 CONTROL의 `turnSignal`로 전송합니다. 수락 상태는 차량 status 토픽으로 게시합니다.
- Autoware가 미교전 또는 명령 종료를 나타낼 때 보내는 `NO_COMMAND`는 VTD에서 직전 램프 상태가 남지 않도록 `DISABLE`로 정규화합니다.
- 방향지시등 status는 API에 실제 lamp feedback이 없으므로 수락된 명령 상태를 유지합니다.
- `scenario_simulation:=true`에서는 Autoware occupancy-grid 노드가 비활성화되므로, 브릿지가 ego 주변의 빈 점유 격자를 발행해 behavior path planner의 입력 계약을 만족시킵니다. 실제 센서 기반 점유 격자를 연결할 때는 `publish_empty_occupancy_grid: false`로 바꿉니다.
- 같은 모드에서는 VTD 객체 바운딩박스의 모서리를 합성 obstacle pointcloud로 발행합니다. 객체가 없을 때만 빈 cloud가 되며, VTD LiDAR/실제 perception 출력을 별도로 연결하면 `publish_empty_obstacle_pointcloud: false`로 바꿉니다.
- 현재 시나리오는 `<Description ... Control="external" Name="Ego"/>`이므로 기본값은 `ego_player_id: -1`, `ego_name: Ego`입니다. 브릿지가 첫 object-state에서 실제 player ID를 찾은 후 그 ID로 제어합니다.
- Autoware의 steering tire rotation rate는 VTD의 `steeringSpeed`(steering-wheel 계열 값)와 의미가 같다고 보장되지 않아 보내지 않습니다. 앞바퀴 target angle만 사용합니다.
- API에는 timestamp와 ego speed/acceleration이 없으므로 `/clock`은 DATA 수신의
  monotonic wall-time 간격으로 만들고, ego 속도·가속도는 연속 pose를 차분해
  계산합니다. API에 실측 steering/gear/light feedback도 없으므로, 해당 vehicle status는
  이전 API-only 구현처럼 수락한 명령 상태를 게시합니다. 조향 status는 실측값이 아닙니다.
- 제어 subscriber callback은 최신 명령만 저장합니다. 40ms wall timer가 그
  시점의 최신 명령을 depth-1 mailbox에 넣고, 전용 TCP 송신 스레드가 9바이트를
  끝까지 전송합니다. 송신 중 도착한 중간 명령은 최신값으로 덮어씁니다.
- 9910 연결이 끊기면 부분 송신과 pending 명령을 폐기합니다. 재연결 대기 중 새 VTD frame이 들어오면 그 시점의 최신 명령 하나만 유지하며, 연결 복구 후 과거 FIFO를 재생하지 않습니다. `/diagnostics`의 `control_packets_queued`, `control_packets_sent`, `control_packets_overwritten`에서 mailbox 동작을 확인할 수 있습니다.
- 9910이 재연결되어도 ROS 시간은 뒤로 돌리지 않습니다. 새 DATA 세션은 기존
  timestamp 이후에서 다시 시작하므로 TF가 새 데이터를 `TF_OLD_DATA`로 버리지 않습니다.
- `/home/a/autoware_run`은 `scenario_simulation:=true`, `localization_sim_mode:=none`으로 실행되어 `simple_planning_simulator`를 띄우지 않습니다. 따라서 VTD 브릿지가 `/localization/kinematic_state`와 `map→base_link`의 유일한 발행자입니다.
- `/simulator/input/traffic_signals`은 Autoware의 dummy traffic-light passthrough를 거쳐 계획 모듈이 사용하는 `/perception/traffic_light_recognition/traffic_signals`로 전달됩니다.
- 브릿지와 Autoware 컨테이너는 모두 host network/IPC와 `/home/a/autoware/docker/files/cyclonedds.xml`을 사용합니다. 이 설정을 빼면 서로의 토픽이 보이지 않을 수 있습니다.
- `TurnIndicatorsCommand`의 좌/우와 `HazardLightsCommand`의 비상등은 `turnSignal` 0/1/2로 변환되며, 비상등에는 별도 API 값이 없어 `turnSignal=0`을 전송합니다.
- `/vtd/objects`와 `/vtd/traffic_lights`는 9910 DATA를 받을 때마다 갱신됩니다.
  신호등은 이 패킷이 제공하는 한 개뿐이며, 다른 신호등 상태를 추정해서 추가하지 않습니다.
- RoboSense 전용 플러그인이 거리/각도를 vendor packet 형식으로만 구성한 경우에는 이 generic XYZ 변환 대신 해당 모델의 UDP/packet decoder를 추가해야 합니다.

## API-only 복원 범위와 한계

검증한 VTD 차량의 participant pose는 후륜축 기준이므로, 이후 수정한 TF/odometry의
전방 오프셋 0 m는 유지합니다. 다른 차량 모델에 대한 검증 결과를 뜻하지 않습니다.

9910 DATA에는 RDB `geo.offX/offY/offZ`와 실측 조향이 없습니다. 따라서 다음은
실측 피드백과 구분해야 하는 **기존 호환 동작**입니다.

- 객체 bbox 중심은 participant 위치에 heading 방향 `length/2`와 높이 `height/2`를
  더하는 당시 계산을 복원했습니다. 이것은 RDB 형상 오프셋과 일치한다고 보장하지 않습니다.
- 조향 status는 수락한 명령값(명령 수신 전 0)입니다. 진단에
  `steering_feedback_measured=0`, `steering_source=accepted_command`로 표시합니다.
- `raw_rdb_tcp_enabled=0`, `traffic_light_rdb_enabled=0`은 이제 실제 연결 구조와 일치합니다.
- 삭제된 RDB 입력을 기다리며 인지/조향 발행을 보류하던 조건은 없습니다.

이 복원은 과거 통신 계약을 복원한 것이며, 실측 피드백 기반 안전 검증을 대체하지 않습니다.
RDB 피드백을 전제로 한 별도 수동 통합시험은 복원 전 백업에 보관했습니다.

## CSV 경로와 RViz

기존 CSV 로더와 네 개의 RViz MarkerArray 시각화는 유지합니다. 저장소 루트에서:

```bash
./autoware /path/to/route.csv
./set_route /path/to/route.csv --preview-only
```

첫 명령은 Autoware 시작 후 CSV 경로를 설정하고, 두 번째는 실행 중인 RViz에 표시만 합니다.
`/debug/csv/raw_checkpoints`, `candidate_lanelets`, `selected_lanelets`,
`corrected_checkpoints`가 기본 RViz 설정의 `CSV Route Preview` 그룹에 표시됩니다.
자세한 사용법은 저장소의 `docs/build-and-run.md`를 참고하세요.

## 빠른 확인

```bash
ros2 topic hz /localization/kinematic_state
ros2 topic echo /simulator/input/traffic_signals --once
ros2 topic echo /perception/traffic_light_recognition/traffic_signals --once
ros2 topic echo /vehicle/status/velocity_status --once
ros2 topic echo /vehicle/status/turn_indicators_status --once
ros2 topic echo /vehicle/status/hazard_lights_status --once
ros2 topic hz /sensing/lidar/concatenated/pointcloud
ros2 topic hz /sensing/camera/camera0/image_raw
ros2 topic echo /diagnostics --once
```

연결은 되었는데 ego update가 0이면 `/diagnostics`의
`participant_data_packets_decoded`와 `control_rx_bytes`를 확인합니다. 전자는 1109바이트
DATA record 수, 후자는 TCP 9910으로 수신한 총 byte 수입니다.
