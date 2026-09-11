# Goal Pose 도로 제한속도 저장

Autoware 지도 작업 중 **“저장”**이라고 요청하면 다음 명령 하나를 실행한다.

```bash
/home/a/autoware_goal_save
```

현재 RViz 2D Goal Pose / Autoware Goal Pose가 위치한 도로 구간의 `speed_limit`을
`30 km/h`로 영구 저장한다. 목표 지점의 도로 횡단면에서 옆으로 붙어 있는 모든
차로를 포함하며, 주행 방향이나 Goal 화살표 방향으로 차로를 제외하지 않는다.
각 선택된 lanelet 전체에 속도가 적용된다. 앞뒤로 이어지는 구간이나 중앙분리대,
별도 도로로 떨어진 구간까지 따라가지는 않는다. 경계 좌표의 작은 오차는 0.25 m까지
허용하며, Goal이 도로에서 1 m보다 멀면 저장하지 않고 오류를 낸다.

```bash
# 지도 변경 없이 현재 목표와 선택 대상 확인
/home/a/autoware_goal_save --dry-run
```

지도 파일 저장은 실행 중인 지도 메시지를 바꾸지 않는다. **저장한 제한속도는
다음 지도 로드 / Autoware 재시작부터 적용된다.** 명령이 차량을 움직이거나 자동
모드를 켜거나 Autoware를 재시작하지는 않는다.

## 빠른 실행 방식

- 실행 중인 `vtd-autoware-run-*` 컨테이너와 `VTD_MAP_RELATIVE_PATH`, bind mount를
  확인해 실제 사용하는 호스트 지도를 찾는다.
- `goal_speed_watch.py`가 컨테이너 안에서 읽기 전용으로 Goal Pose를 수신한다.
  기존 목표는 신뢰성/지속성 QoS를 맞춘 route 토픽에서 받고, 이후 RViz 클릭도
  기록한다. 과거 컨테이너나 예전 로그의 목표로 대체하지 않는다.
- 최초 실행 때 지도 좌표 검색 인덱스를 만든다. 이후에는 파일의 inode·크기·수정
  시각이 같을 때 재사용한다. 외부 지도 수정이나 컨테이너 재시작은 자동 감지한다.
- 전체 XML을 재출력하지 않고, 선택된 lanelet의 속도 태그 바이트만 바꾼다.
  기존 좌표, 경계, 연결, 신호등, 정지선은 그대로 보존된다.
- 지도 백업 후 임시 파일 쓰기/fsync/원자적 교체로 저장한다. 이미 30 km/h이면
  지도를 다시 쓰거나 중복 백업을 만들지 않는다.

## 파일과 복원

- 명령: `/home/a/autoware_goal_save`
- 구현: `scripts/save_goal_speed.py`, `scripts/goal_speed_watch.py`
- 상태/인덱스/최근 결과: `~/.local/state/autoware-goal-speed/`
- 변경 전 지도와 변경 기록:
  `~/.local/state/autoware-goal-speed/backups/<저장시각>/lanelet2_map.osm`, `change.json`
- 대화 단축 명령 지침: `/home/a/AGENTS.md`

복원할 때는 해당 `change.json`에 적힌 지도 경로와 백업 경로를 확인해 되돌린다.
다른 수정이 그 뒤에 이루어졌다면 전체 백업을 덮어쓰기 전에 후속 변경을 보존해야 한다.

Local projector의 local_x/local_y 지도용이며 호스트의 기존
`/home/a/.venvs/crdesigner/bin/python`에 설치된 Shapely/lxml을 사용한다.

개발 검증은 `scripts/test_save_goal_speed.py`와 실제 지도 **복사본**으로 수행한다.
`--map FILE --goal X Y --state-dir DIR`로 차량과 분리된 복사본 저장을 확인할 수 있다.

2026-09-11 현재 Goal `(600.527710, -121.476471)`에서 `25596, 25705, 25814`의
3개 차로가 선택됐다. 캐시 사용 미리보기 0.546초, 백업 포함 저장 1.302초,
동일 값 재저장 0.432초였다(현재 PC 측정). 최초 지도 인덱스 생성은 약 4초다.
복사본의 속도 태그 3개 외 모든 바이트와 원본 백업 일치를 확인했고, 전체 XML
파싱과 Lanelet2의 30 km/h 속도 해석, 선택/파일 수정 테스트 8개를 통과했다.
