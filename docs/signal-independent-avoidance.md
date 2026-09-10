# 신호색과 회피 후보 계산 분리 (2026-09-10)

`AvoidanceByLaneChange::isRouteDepartureRequired()`에서 다음 두 차단 조건을 제거했다.

- 현재 차선열에 대한 `isTrafficSignalStop()` 판정.
- 현재 차선열의 후속 차선들에 대한 `isTrafficSignalStop()` 판정.

신호 정지 판정만으로 목적 차선 밖 회피 후보의 계산을 건너뛰지 않는다.
실제 신호등 정지 제어, 거리에 따른 규제구간 제한, 정체 차량열 판정, 지속 관측된
정적 장애물 조건, 충돌·곡률·복귀 검사와 RTC 승인은 변경하지 않았다.
앞서 확인된 5도 경로 연결각 제한 및 곡률 초과 문제도 이번 변경에 포함하지 않는다.

## 패키징

기준 이미지: `selfcar-2026-vtd:occupied-lane-lookthrough-20260910`
(`4032785905c9618fb7139333fda832f89f56dfb712167c73e54fae7f9af81bf8`).
기존 바깥 차선 탐색·후보별 복귀 검사 대기시간·35 m 승인 거리·최소 길이 배율 0.5를 보존한다.
변경된 avoidance-by-lane-change 모듈만 빌드한다.

```bash
docker build -f docker/vtd/signal-independent-avoidance.Dockerfile \
  -t selfcar-2026-vtd:signal-independent-avoidance-20260910 .
```

검증 범위는 컴파일, 동적 라이브러리 로드와 이미지 전후 비교다.
회귀시험·장면 재생·시나리오 주행은 하지 않는다. 실행 중인 Autoware/브릿지도 변경하지 않는다.
백업 및 확인 자료: `/home/a/autoware-task-backups/signal-independent-avoidance-20260910.Jc1D0N`.

## 완료 결과

- 빌드 성공. 새 이미지 ID:
  `db0d9a154ae0220bccadfe9c647e682730a6b4b4c1524997567f8d80cd432779`.
- `selfcar-2026-vtd:local`을 새 이미지에 연결했다. `./autoware_run`으로 다음 재실행 시 적용된다.
- 수정 소스의 이미지/호스트 해시 일치. 관련 라이브러리 4개 즉시 심볼 해석 로드 성공.
- overlay 라이브러리 33개 중 avoidance-by-lane-change 1개만 변경, 나머지 32개 동일.
  35 m 설정 3곳, 최소 길이 배율 0.5와 정적 회피/브릿지 소스 해시도 동일하다.
- 실행 중 Autoware는 기존 `4032785905c9...`, 브릿지는 `f2b1866d7835...`로 유지된다.
  재시작, 실시간 파라미터 변경, 회귀시험, 시나리오 주행은 하지 않았다.
