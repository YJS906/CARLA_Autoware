# Return plan 제거 (2026-09-10)

이전 이미지로 롤백하지 않고 현재 `87cb33010537` 이미지의 차선변경 회피 모듈만 수정한다.

## 제거 범위

- 가상 미래 자세에서 복귀 경로를 미리 생성하는 `RouteReturnProbe`와 다단계 예약.
- `return_plan` 확보 여부를 회피 후보 승인 및 승인 유지에 추가하던 조건.
- 복귀 지점을 예약하기 위해 추가하던 속도 상한과 정지점.
- 복귀 후보가 나올 때까지 정지 상태로 회피 모듈을 유지하던 인계 및 완료 조건.
- 복귀 검색 간격·계산 예산·시간 여유·최대 단계 파라미터 4개.

`generateOutput`, `updateApprovedPath`, `resetParameters`, `hasFinishedLaneChange`는
기본 `NormalLaneChange` 구현을 사용한다. 실행 경로의 정적·예측 객체 충돌 검사는 유지한다.

## 유지 범위

- 목적지 차선 우선순위, 지속적인 물리적 막힘 및 대기 행렬 판정.
- 35m 승인 거리, 조기 감속, 현재 자세에서의 승인 경로 재계획.
- 일반 차선변경과 다차선 후보 검색, 조향·기하 유효성 검사.
- 기존 신호등·횡단보도·obstacle_stop·intersection 등 다른 모듈과 설정.
- 지도, 브릿지, CSV 마커, 실행 중인 세션.

회피 후 목적지 차선 이동은 일반 차선변경 모듈이 현재 자세와 객체로 독립적으로 판단한다.
미래 복귀 가능성을 사전에 보장하지 않으며, 이 제거가 충돌 문제를 해결했다는 뜻은 아니다.

## 배포

- 빌드 파일: `docker/vtd/remove-return-plan.Dockerfile`
- 수정 이미지: `selfcar-2026-vtd:remove-return-plan-20260910`
- 원본 보관: `/home/a/autoware-task-backups/remove-return-plan-20260910.W3e3bV/before.tar.gz`
- 빌드 및 패키징 확인 후 `local` 태그에 연결한다. 적용은 사용자의 다음 재실행 때 이뤄진다.
- 회귀시험, 시나리오 주행 및 라이브 파라미터 변경은 수행하지 않는다.

## 완료 확인

- Release 빌드 완료. 이미지 ID: `59328dfa33e728af535a9435ec66e90f9c0e918ccd275cf1470dd96e7b6118e5`.
- `selfcar-2026-vtd:local`을 수정 이미지에 연결했다.
- 변경된 실행 라이브러리는 `libautoware_behavior_path_avoidance_by_lane_change_module.so` 하나뿐이다.
- 관련 라이브러리 4개 로드 성공, 설치 소스/설정 7개 일치, 복귀 예약 심볼 및
  예약 전용 lifecycle override 제거를 확인했다. 기존 충돌 검사 함수는 유지된다.
- 다른 설정 파일 246개와 일반 차선변경/정적 회피 소스 파일 170개의 해시는 이전 이미지와 같다.
- 실행 중인 Autoware `87cb33010537`과 bridge `db0d9a154ae0`의 이미지·시작 시각은 그대로다.
- 확인 기록: 백업 폴더의 `build.log`, `verify_image.py`, `image-before.json`, `image-after.json`.
