# Town05 현재 위치 차선 변경 후보 누락 진단

2026-09-23, 에고 약 (-20.85, 186.46), 현재 lanelet 17447, 왼쪽 16629.
전방 정지 차량은 중심 기준 약 11m 앞이며, 회피 모듈의 장애물 종방향
거리는 10.69m였다. 읽기 전용 관측과 로그 수준 임시 조정으로 확인했고
관측 후 로그 수준을 INFO로 돌렸다. 경로, 객체, 운전 모드, planner 파라미터는
변경하지 않았다.

## 확인한 원인

`AvoidanceByLaneChange::getTargetLaneCandidates`는 목표 차로 중심선의 첫 점을
현재의 잘린 reference path 위에 투영해 target_start를 구한다. 현재 왼쪽
차로는 약 787.43m 길이이며 에고는 그 시작점에서 약 759.46m 진행한 위치에
있다. 시작점은 뒤에 있지만 reference path의 다른 미래 구간에 투영되어
**start=235.12m**로 계산된다. 따라서 `target_start <= object.longitudinal`
조건이 거짓이 되어 현재 바로 옆에 있는 차로를 후보에 넣지 않는다.

로그: `direction=LEFT lane=16629 start=235.12 object=10.69 clear=false usable=false`.
`clear=false`는 현재 구현에서 진단용이며, 이 단계에서 후보를 없애는 직접
원인은 `usable=false`다. 일반 차선 변경도 `Ego is far from target lane start.`를
표시하고, 정적 회피는 `INSUFFICIENT_DRIVABLE_SPACE(WAIT AND SEE)`를 표시한다.
planner manager의 승인/후보 모듈 목록은 비어 있다. 기존 경로 자체는 존재한다.

## 누락 여부와 지도 확인

- 실행 중 avoidance-by-lane-change 소스와 CARLA 빌드 당시 복사본, 저장소의
  VTD overlay 소스 SHA256이 모두 `46bbb9e091136dd483d3302c936c0d23df67c1d3025db8fcce4321d1a1f36acf`로 같다.
- 실제 routing graph의 `left(17447)=16629`, `right(16629)=17447`을 확인했다.
- 현재 mission route에도 17447, 16629, 15811이 같은 segment의 후보로 들어 있다.
- 차선 변경 연결 154개가 모두 유지되고, CARLA 원본에서도 현재 차로는 Left,
  왼쪽 차로는 Both 변경을 허용한다.

VTD 회피 기능이나 지도 연결의 누락이 아니라 긴 CARLA lanelet에 대한
시작점 거리 계산 결함이다. 수정 시 이미 옆에서 겹치는 차로는 현재 위치와
차로의 종방향 관계로 판정하고, 실제로 앞에서 시작하는 차로의 제한은 유지해야
한다. 일반 차선 변경의 같은 시작점 계산도 함께 회귀 검사해야 한다.
옆 차로에도 차량이 있으므로 후보 생성 결함을 고친 뒤에도 실제 기동이 충돌
검사를 통과하는지는 별도 확인해야 한다. 이 진단에서는 수정·주행 시험을 하지 않았다.

[후보 제외 로그](validation/carla-route-block-20260923/candidate-rejection.log) ·
[현재 차로 연결 검사](validation/carla-route-block-20260923/graph-validation.log).
원시 관측은 `/home/a/autoware-task-backups/carla-route-block-20260923/`에 보존했다.
