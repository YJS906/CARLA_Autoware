# 7초 재판정을 이전 동적 객체에만 적용

## 변경 범위

직전 `stop-hold-stationary7-20260910`의 모든 객체 공통 7초 조건을 분리한다.

- 이동 이력이 없는 객체: 기존 `route_priority.blockage_min_duration=3.0` 복원.
- 이동 이력이 있는 객체: 새 `route_priority.stopped_dynamic_min_duration=7.0` 사용.
  첫 정지 관측부터 연속 7초가 지나면 ABLC 지속 정차 장애물 자격을 얻는다.
- 재출발은 같은 관측 주기에 즉시 자격을 취소한다. 다음 정지는 다시 7초를 센다.
- 기존 속도 임계값(공유 stationary velocity)과 0.5 m 위치 변화 기준으로 이동 이력을
  판별한다. CAR/UNKNOWN 같은 인식 클래스는 동적/정적 구분이 아니며 변경하지 않는다.
- 관측 이력은 ABLC 매니저가 소유하여 회피 모듈 교체에도 같은 UUID의 연속 관측과
  이동 이력이 유지된다. 승인 상태나 경로를 공유/예약하는 것이 아니다.
- 기존 미검출·0.5초 초과 관측 공백·시간 역행에 대한 연속 관측 무효화는 유지한다.
  추적이 끊긴 객체의 재식별이나 프로세스 재시작을 넘는 영구 이력은 추가하지 않는다.

회피 필요 여부는 기존 `is_avoidance_target && avoid_required` 그대로다. 3/7초를 만족해도
모든 객체를 회피 대상으로 만들지 않는다. 별도 정적 회피 모듈의 주차 판정도 바꾸지 않는다.
경로 우선순위, 대기열/규제거리 조건, 35 m 승인 범위 및 실제 경로의 정적/동적 충돌검사,
조향·제동 한계는 그대로다. 충돌검사를 회피 대상 객체만으로 축소하지 않는다.

PID 정지 유지 설정과 `obstacle_stop.stop_margin=4.5`도 직전 수정값을 유지한다.
`return_plan`을 복원하지 않는다.

## 배포

- 기준 이미지: `selfcar-2026-vtd:stop-hold-stationary7-20260910`
  (`24ff1e47b8c07026db4068758485226ea802ce82d6fab2580890cbcebb0a4592`).
- 빌드 파일: `docker/vtd/dynamic-only-stationary7.Dockerfile`.
- 수정 전 백업: `/home/a/autoware-task-backups/dynamic-only-stationary7-20260910.TPfEmT/before.tar.gz`.
- ABLC만 빌드한다. 실행 중인 Autoware/브릿지와 ROS 파라미터는 변경하지 않는다.
- 회귀시험 및 시나리오 주행은 하지 않는다. 빌드 및 배포 파일 일치 여부만 확인한다.

## 완료 확인

- ABLC Release 빌드 성공, 공유 라이브러리 로드 성공.
- 이미지: `selfcar-2026-vtd:dynamic-only-stationary7-20260910`.
  ID: `385675e9de64650cf34027ad9a9c48c26c2d2ec956f2f5e594da873dce2fe69e`.
  `selfcar-2026-vtd:local`에 연결하여 다음 재실행부터 사용한다.
- 확인한 배포 파일 중 허용된 14개 파일만 변경, 나머지 708개 동일.
  PID 정지 유지와 obstacle_stop 4.5 m 관련 파일은 직전 이미지와 동일하다.
- 실행 중인 `vtd-autoware-run-eea2650ffe0b`와 `selfcar-vtd-bridge`는 기존 이미지
  `24ff1e47...` 및 시작 시각을 유지한다. 실행 중 파라미터 변경·재시작 없음.
- 검증/빌드 기록은 위 백업 디렉터리의 `verification.json`, `verify_image.py`, `build.log`.
