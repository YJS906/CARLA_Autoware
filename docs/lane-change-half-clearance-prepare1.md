# 차선변경 RSS 여유 50% 및 다중 차선 준비시간 1초

2026-09-11 요청: `overlap_extended_polygon`의 좌우 및 앞뒤 추가 여유를 절반으로 줄이고,
3차선에서 1차선으로 직접 이동하는 경로의 준비시간을 1초로 설정한다.

## 설정과 적용 범위

- `lane_change.safety_check.polygon_expansion_scale: 0.5`
  - 차선변경의 `is_colliding()`에서 최종 종방향 여유와 횡방향 여유에 공통으로 적용한다.
  - 종방향은 `max(RSS 거리, 최소 종방향 거리)` 계산 후 0.5배가 된다.
    따라서 RSS 거리가 최소거리보다 커져도 추가 여유는 정확히 절반이다.
  - prepare/execution/parked/cancel/stuck 프로필을 선택한 뒤 적용한다.
  - 실제 차량/장애물 footprint의 크기와 최초 실제 footprint 교차 검사는 그대로다.
  - 정차 물체의 rectangle 정책에서는 앞/뒤 추가분이 각각 기존의 절반이 된다.
  - 기존 관측 장면의 좌우 추가분은 한쪽 1.0 m에서 0.5 m로,
    검사 전체 폭은 3.896 m에서 2.896 m로 바뀐다.
  - 별도 정적 궤적 검사와 obstacle_stop 모듈의 설정은 변경하지 않는다.
  - 기본 코드값 1.0은 기존 동작이며 VTD 설정과 패키지 설정은 0.5이다.

- `lane_change.trajectory.multi_lane_prepare_duration: 1.0`
  - 현재 차선에서 목표 차선까지 routing graph의 합법적인 left/right 연결을
    두 번 이상 거치는 직접 차선변경에 적용한다. 지도 ID나 위치를 하드코딩하지 않는다.
  - 이 경우 준비시간 샘플을 1초로 고정한다. 신호 활성 시간, 장애물 거리,
    차선 끝 근접에 따른 다른 준비시간 샘플링으로 덮어쓰지 않는다.
  - 감속 준비는 실제 감속 및 기존 안정화 거리를 1초 안에 완료할 수 있을 때만
    사용하고, 남는 시간은 목표속도로 진행한다. 속도 재설정으로 준비시간이 달라지는
    후보도 승인하지 않는다.
  - 가속도/감속도/저크/유효거리 검사를 계속 적용한다. 1초 내 최소 속도에 도달하지
    못하는 정지 상태 등에서는 해당 후보가 나오지 않을 수 있다.
  - 인접 한 차선 변경은 기존 준비시간 탐색을 사용한다. 코드 기본값 0.0은 이
    특별 고정 시간을 비활성화한다.

## 빌드와 확인

`docker/vtd/translated-curve.Dockerfile`로 일반/회피/외부 요청 차선변경 플러그인을
함께 재빌드하며 VTD YAML을 이미지의 실제 launch 설정 경로에도 설치한다.

```bash
docker build -f docker/vtd/translated-curve.Dockerfile \
  -t selfcar-2026-vtd:half-clearance-prepare1-20260911 .
```

변경 전 소스/설정/이미지 정보와 빌드/검증 로그:
`/home/a/autoware-task-backups/half-clearance-prepare1-20260911.1txp8v2s`.

## 실행 이미지 연결

일반/회피/외부 요청 차선변경 패키지 빌드를 완료하고, 사용자 요청에 따라
`selfcar-2026-vtd:local`을 `selfcar-2026-vtd:half-clearance-prepare1-20260911`에 연결했다.
실행 중인 Autoware와 VTD 브릿지는 유지했으며, 다음 `./autoware_run` 실행부터 적용된다.

이미지: `sha256:a7814e3266d2f761395f13c2fa90f0cc2e9fb00c3929bd56d810cb8f063972a1`.
