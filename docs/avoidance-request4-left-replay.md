# 요청 최소거리 4m 및 현재 장면 LEFT 경로 재현

2026-09-11: `avoidance_by_lane_change.execute_object_longitudinal_margin`을
7.0에서 4.0m로 변경했다. VTD 설정과 패키지 설정에 함께 반영했다.

## 결과

현재 장면을 저장하여 배포된 실제 회피 차선변경 플러그인의
`updateLaneChangeStatus()` → `getSafePath()`를 실행했다.
새 이미지에 설치된 YAML을 읽은 결과 `request_min=4`, 목표 차선 `204620`,
`direction=LEFT`, `valid=1`, `safe=1`이었다.
선택된 경로의 별도 정적 궤적 검사도 `valid=1`, `object_id 없음`으로 통과했다.
예측/RSS 검사 대상 8개도 모두 안전으로 판정됐다.

| 요청 최소거리 | polygon_expansion_scale | LEFT 유효 경로 | 정적 검사 | 예측/RSS 검사 |
|---|---|---|---|---|
| 4.0m | 0.5 (현재 설정) | 생성 | 통과 | 통과 |
| 4.0m | 0.4 | 생성 | 통과 | 통과 |
| 4.0m | 0.3 | 생성 | 통과 | 통과 |
| 4.0m | 0.2 | 생성 | 통과 | 통과 |
| 4.0m | 0.1 | 생성 | 통과 | 통과 |
| 4.0m | 0.05 | 생성 | 통과 | 통과 |
| 4.0m | 0.01 | 생성 | 통과 | 통과 |

추가 축소 없이 현재 0.5에서 이미 통과하므로 **배포 값은 0.5를 유지**한다.
정차 물체 프로필의 `lateral_distance_max_threshold=1.0`,
`longitudinal_distance_min_threshold=3.0`도 유지한다.
배율 적용 후 해당 프로필의 좌우 추가분은 각각 0.5m이며, 정차 rectangle 정책의
최소 종방향 추가분은 앞/뒤 각각 0.75m다. 실제 RSS 계산값이 최소거리보다 크면
그 계산값에 0.5배를 적용한다.
차체 크기, 장애물 footprint, 실제 충돌 검사, 조향 한계를 변경하지 않았다.

선택 경로의 준비시간은 1.5초, 차선변경 시간은 약 8.85초다.
현재 목표는 인접한 한 차선이므로 앞서 설정한 다중 차선 준비시간 1초의 적용 대상이 아니다.
`multi_lane_prepare_duration=1.0`은 그대로 유지했다.

## 재현 방법과 이력 차이

- 실행 중 컨테이너에서 지도, 경로, 자차 위치/속도/가속도, 장애물 및 예측 궤적,
  신호, 운행 상태를 CDR로 저장했다. 자차와 객체의 캡처 시각은 동일하다.
- 자차 위치는 약 `(897.085, -463.040)`, 속도는 0m/s다.
- 별도 `--network none` 컨테이너에서 실제 빌드된 플러그인에 입력했다.
  현재 정차 장면에 맞게 동일 장면의 시각을 진행시켜 정차 장애물 확인 이력을 만들고,
  자차의 장시간 정지 상태를 맞췄다. 실제 주행이나 제어 출력 테스트는 아니다.
- 실제 실행 중 로그는 `object=6.77 request_distance=7.00`을 보였으며
  `left=not_evaluated`가 유지됐다. DEBUG 수집 후 로그 레벨은 INFO로 복구했다.
- 새 재현 인스턴스에는 실행 중 누적된 장애물 envelope 이력이 없어
  같은 장애물의 longitudinal 값이 8.14481m로 초기화된다.
  따라서 새 인스턴스에서는 7m도 요청 조건을 통과한다.
- 이 차이를 별도로 확인하기 위해 요청 판단에 쓰는 longitudinal 스칼라만
  실제 로그의 6.77m로 맞춘 회귀 검사를 추가했다.
  이 조건에서 7m는 경로 평가에 진입하지 않고, 4m는 LEFT 경로와 모든 충돌검사를 통과했다.
  해당 검사는 실제 누적 envelope 전체를 복원한 테스트가 아니며, 장애물의 물리적 형상,
  위치, 예측 궤적이나 충돌 판정을 수정하지 않았다. 이 조정은 진단 실행 파일에만 있다.

## 빌드와 연결

파라미터만 변경되어 기존 빌드된 플러그인 위에 설정 이미지를 빌드했다.
`docker/vtd/avoidance-request4.Dockerfile`은 실제 launch 설정, overlay 패키지 설정,
이미지 내 소스 설정에 새 값을 설치한다.
`translated-curve.Dockerfile`에도 동일 설정 설치를 추가해 이후 재빌드에 반영했다.

```bash
docker build -f docker/vtd/avoidance-request4.Dockerfile \
  -t selfcar-2026-vtd:avoidance-request4-20260911 .
docker tag selfcar-2026-vtd:avoidance-request4-20260911 selfcar-2026-vtd:local
```

이미지 ID: `sha256:921dc10975c9e9de204cccde86f23739d220d9cfc4aa8c8faaaf628c1ab1b947`.
실행 중인 컨테이너는 유지했으므로 현재 세션은 여전히 7m이고,
다음 `./autoware_run`부터 4m가 적용된다. 이 모듈은 해당 파라미터를 초기화 시 읽으며
실시간 갱신을 구현하지 않아 ROS 파라미터 서비스만 호출하는 방식은 사용하지 않았다.

캡처, 진단 소스, 빌드 로그, 파라미터 스윕 결과, 경로 CSV 및 그림:
`/home/a/autoware-task-backups/request4-left-sweep-20260911.rvtlswqq`.
주요 로그는 `sweep.log`, `verify-installed.log`, `live-baseline.log`이며,
그림은 `left-path-request4.png`다.
