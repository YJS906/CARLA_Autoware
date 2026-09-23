# Town05 횡단보도 중앙 기준 3m 정지선

2026-09-23에 기존 정지선 중 횡단보도 진입 방향과 대응하는 57개를
횡단보도 폴리곤의 면적 중심에서 진행 방향 반대로 3m인 위치로 평행 이동했다.
이는 횡단보도 가장자리에서 3m라는 의미가 아니다. 횡단보도의 폭과 기울기에 따라
수정한 선과 횡단보도 가장자리 사이의 최단 거리는 약 0.59~1.78m다.

횡단보도 66개의 모든 좌표, 도로 경계, 정지선 길이·방향·높이, way/relation ID,
규제 연결 및 속도 태그는 유지했다. 변경된 지도 요소는 기존 정지선 끝점 114개의
XY 좌표와 그에 대응하는 lat/lon뿐이다. 신호 manifest와 지도 보고서의 해시는
새 지도에 맞게 갱신했다. CARLA의 노면 텍스처나 NPC 정지 위치는 변경하지 않았다.

수정 대상은 신호등 정지선 49개와 정지 표지 정지선 8개이며, 횡단보도 54곳에
대응한다. 진입 방향의 횡단보도와 대응하지 않는 기존 정지선 5개는 유지했다.
나머지 횡단보도 12곳에는 대응하는 기존 정지선이 없어 새 선을 추가하지 않았다.
상세 ID와 이전/이후 좌표는 [수정 기록](validation/carla-stop-lines-20260923/carla_stop_line_report.json)에 있다.

차량 앞부분이 정지선에서 얼마나 떨어져 멈추는지 정하는 planner 파라미터는
변경하지 않았다. 예를 들어 현재 신호등 모듈의 `traffic_light.stop_margin`은
1m다. 이 작업은 지도 정지선의 위치 수정이며 실제 주행 제동 위치 검증은 아니다.

## 재현과 검증

현재 지도를 입력으로 사용하여 다른 디렉터리에 후보를 만든다. 명령을 반복해도
이미 같은 위치에 있는 선의 좌표는 다시 바꾸지 않는다.

```bash
/home/a/CARLA/venv-0.9.16/bin/python tools/map/move_town05_stop_lines.py \
  --map-dir /home/a/autoware_data/maps/Town05 \
  --source config/carla/maps/Town05_regulatory_source.json \
  --output-dir /tmp/town05-stop-lines-candidate --offset 3
```

실제 지도 교체는 Autoware와 신호 브리지를 종료한 뒤 원본 번들을 백업하고,
지도와 manifest를 함께 반영해야 한다. 적용 시 사용한 백업은
`/home/a/autoware_data/maps/Town05/backups/stop-lines-center-3m-20260923-140418`이다.

독립 좌표 검사에서 중심 거리 오차는 1e-6m 미만이며 모든 이동 정지선이 도로와
교차하고 횡단보도와 겹치지 않았다. 실제 Autoware Lanelet2 로더에서 도로 486개,
횡단보도 66개, 신호 그룹 54개, 정지 표지 규제 8개를 확인했고, 기존 차선 변경
연결 154개가 모두 유지됐다. [검증 결과](validation/carla-stop-lines-20260923/independent-validation.json)와
[Autoware 검사 로그](validation/carla-stop-lines-20260923/autoware-validation.log)를 보존했다.
