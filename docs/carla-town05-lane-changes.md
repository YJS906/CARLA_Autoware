# Town05 차선 변경 지도 보완

`tools/map/repair_town05_lane_changes.py`는 CARLA의 원본 `Town05_Opt.xodr`와 기존 Lanelet2 도로를 대조해 차선 변경 정보를 추가한다. 실제 시뮬레이터에 접속하거나 tick하지 않는 오프라인 도구다. 지도 연결 검증은 실제 차량 회피 성공 판정과 구분한다.

## 적용 정책

- 기존 노드 좌표·way의 점 배열·lanelet ID·관계·신호등·횡단보도·정지선·속도 태그를 그대로 둔다.
- 동일 방향의 두 도로 lanelet이 공유하는 경계만 대상으로 한다. 각 lanelet의 중심을 최대 2m 간격으로 원본 CARLA 도로와 대조하고, 방향·높이·위치를 확인한다.
- 두 lanelet이 같은 OpenDRIVE road/laneSection의 인접한 주행 차로여야 한다. OpenDRIVE에서 두 차로 중 안쪽 lane의 `roadMark`가 공통 경계를 정의한다.
- 해당 native laneSection의 **모든** roadMark가 `type=broken`, `laneChange=both`일 때만 `type=line_thin`, `subtype=dashed`, `lane_change=yes`를 추가한다. 점선이라는 모양만으로 변경을 허용하지 않는다.
- 교차로, 여러 native 도로에 걸친 경계, 실선/점선 혼합, 방향별 허용 또는 불확실한 경계는 분할하지 않고 기존의 변경 불가 상태를 유지한다. 따라서 이 패치는 CARLA의 모든 차선 변경 가능 구간을 완전히 변환한 것은 아니다.
- 기존에 사용자가 작성한 경계 형식·변경 허용 태그는 덮어쓰지 않는다. 이 도구가 이전에 작성한 경계가 이후 수정되었거나 native 조건이 달라지면 오류로 중단한다.

2026-09-22 후보 지도는 도로 경계 798개 중 77개에 태그를 추가했다. 시험 구간의 `18052 ↔ 18355`(공통 경계 `18050`)는 native road38 / lane -3 ↔ -2의 지속적인 점선·양방향 차선 변경 허용에 해당한다. 반대 진행 방향으로 넘어가거나 도로 바깥쪽으로 변경하는 연결은 추가하지 않았다.

## 설치와 백업

`./scripts/carla/prepare_town05_map`는 이미 규제가 추가된 지도라면 **현재 지도**를 입력으로 사용한다. 최초 원본에서 다시 만들지 않으므로 사용자 속도 등 기존 편집을 보존한다. CARLA Autoware와 signal 컨테이너가 실행 중이면 변경을 거부한다. 상위 실행 스크립트가 관리하는 중지·재시작 절차에 따라 설치한다.

기존 파일은 `Town05/backups/carla-map-<UTC>-<id>/`에 저장하며 다음 네 파일을 갱신한다.

- `lanelet2_map.osm`
- `carla_traffic_signals.json` — 갱신된 OSM SHA256
- `carla_map_report.json` — 갱신된 OSM SHA256과 lane-change source
- `carla_lane_change_report.json` — 각 경계의 허용 여부·native 근거·제외 이유

정상 예외 처리 시 이전 파일을 복원한다. 개별 파일 rename은 atomic이지만 네 파일 전체는 하나의 atomic transaction이 아니다. 설치 후 Autoware가 지도를 다시 읽어야 한다.

## 검증 기록

2026-09-22 입력 OSM SHA256:
`413bf0a9e9c08357d284364d8a70d9a1e94fed64c1a8b8854df65b2a214468a8`

후보 OSM SHA256:
`44628aefb80c91e64d8d65c318f257b96182e8365406483f7747a343ea8b1ef8`

native XODR SHA256:
`7edc53b0be840e177598da7a4ed9571146d7b2595ce280d154c90f92e8a48851`

7개 Python 시험을 통과했다. 좌표·관계·규제·사용자 속도 보존, 명시적인 사용자 변경 금지 유지, 재실행 불변성, 아주 짧은 실선 구간도 거부, 반대 방향/다른 road/laneSection 거부를 포함한다.

```bash
PYTHONDONTWRITEBYTECODE=1 /home/a/CARLA/venv-0.9.16/bin/python \
  /home/a/carla_pp/tools/map/test_repair_town05_lane_changes.py -v
```

기존 실행 이미지의 실제 Autoware RouteHandler와 Lanelet2 parser로 후보를 읽어 확인한 결과:

```json
{"changeable_neighbors":[18658,18355,18052],"left_change":18355,"return_right":18052,"outer_right_change":0,"tagged_directional_edges":154,"connected_directional_edges":154,"nonfinite_centerline_points":0}
{"road_lanelets":486,"crosswalk_lanelets":66,"traffic_light_groups":54,"crosswalk_regulations":66,"stop_sign_lane_regulations":8,"nonfinite_centerlines":0}
```

오프라인 installer fixture에서도 업그레이드, 이전 번들 백업, 지도·신호 manifest·지도 report의 SHA 일치, 두 번째 실행 시 파일 변경 없음이 통과했다. live 지도는 이 검증 단계에서 수정하지 않았다.

C++ 검사 프로그램은 `tools/map/CMakeLists.txt`로 빌드한다. 실제 Autoware 이미지 내부에서 ROS와 Autoware/overlay setup을 source한 다음:

```bash
cmake -S /path/to/tools/map -B /tmp/map-validation-build \
  -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++
cmake --build /tmp/map-validation-build -j2
/tmp/map-validation-build/validate_town05_lane_changes /path/to/lanelet2_map.osm
/tmp/map-validation-build/validate_town05_regulatory /path/to/lanelet2_map.osm
```

첫 검사는 추가한 77개 공통 경계의 양쪽 변경 연결 154개가 실제 routing graph에 모두 존재하는지 확인한다. 시험 차로의 좌측 변경과 원래 차로 복귀를 허용하고 우측 도로 밖 변경은 금지하는지도 확인한다. 수정 전 지도에서는 변경 가능 이웃이 `[18052]` 하나이며 좌측/복귀 연결이 모두 0인 대조 결과를 얻었다. 둘째는 지도 규제와 유한한 centerline을 회귀 검사한다. 최종 차량 회피 여부는 배포 후 AUTO 주행 시험으로 별도 판정한다.
