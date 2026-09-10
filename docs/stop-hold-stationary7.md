# 정지 유지 제동·7초 정차 재판정·obstacle_stop 4.5 m

## 0.1 km/h 표시 원인

분석 bag: 컨테이너의
`/tmp/autoware_docker_bags/20260910_165725_2368572/rosbag2_2026_09_10-07_57_33`.
bag 시작 후 236~239초의 longitudinal diagnostic에서:

- 실제 속도 약 0.0183 m/s = 0.066 km/h. RViz의 소수점 한 자리 표시에서 0.1 km/h.
- 목표 속도와 목표 가속도는 이미 0. 제어 상태는 DRIVE(0).
- PID P 성분 약 -0.0186 m/s², I 성분 +0.018527 m/s²가 상쇄되어 제동이 거의 0.
- 저속 적분 갱신이 꺼져 있으므로 남은 양의 I 성분이 유지된다.
- 실제 위치도 약 3초 동안 5.46 cm 전진했다. 단순 표시 오차나 가짜 수신 속도가 아니다.
- 기존 STOPPED 진입 속도는 0.01 m/s라 이 미세 주행 상태에서 정지 유지로 넘어가지 못했다.
- HLVTD 브릿지는 ego 위치 차분으로 속도를 계산하며, 제어 API에는 속도가 아닌 가속도를 보낸다.
  따라서 목표 속도 0만으로는 실제 정지를 보장할 수 없다.

기존 PID 기능으로 정지 직전 제동 유지(`enable_brake_keeping_before_stop`)를 켜고,
STOPPED 진입 속도를 0.05 m/s로 변경한다. 정지점 부근에서 0.1초의 기존 진입 조건을
충족하면 목표 속도 0 및 기존 `stopped_acc=-3.4`의 유지 제동을 사용한다.
정지점이 멀리 있는 저속 주행에 이 속도 기준만으로 정지를 걸지는 않는다.
`enable_smooth_stop=false`, 출발 조건, 일반 주행 PID 이득, 브릿지 수신 속도는 유지한다.
측정 속도를 강제로 0으로 덮거나 crosswalk 정지 확인 조건을 무력화하지 않는다.

## 7초 정차 재판정

후속 변경: [이전 동적 객체에만 7초 적용](dynamic-only-stationary7.md).
아래는 최초 공통 7초 배포의 이력이며, 현재 코드는 정적 객체의 기존 3초 확인과
동적 객체의 정지 후 7초 재판정을 분리한다.

ABLC의 평생 이동 이력 플래그를 현재 정지 관측 상태로 대체한다.

- 처음 정지한 유효 관측부터 연속 7초가 지나면 지속 정차 객체로 인정한다.
- 이전에 움직였던 CAR/UNKNOWN 등에도 같은 규칙을 적용한다. 인식 class는 바꾸지 않는다.
- 다시 움직이는 관측을 받으면 같은 주기에서 즉시 정차 자격을 취소한다.
  재출발 후 7초를 기다리는 것이 아니다. 다시 정지하면 첫 정지 관측부터 새로 7초를 센다.
- 기존 공유 정지 속도 임계값 및 0.5 m 위치 변화 한도를 사용한다.
- 중복 timestamp로 시간을 적립하지 않는다. 객체 미검출·0.5초 초과 수신 공백·시간 역행은
  연속 관측을 끊고, 오래된 관측으로 승인하지 않는다.
- 정적 회피 모듈의 별도 주차 분류는 이번 수정 대상이 아니다.
- 기존 경로 우선순위, 대기열/규제거리 판단, 차선 합법성, 실제 경로의 정적·동적 충돌
  검사는 유지한다. 7초는 정차 자격 조건이지 무조건 회피 승인 조건이 아니다.
- 제거한 `return_plan`은 복원하지 않는다.

## 정지 거리

`obstacle_stop.stop_planning.stop_margin`: 4.0 → 4.5 m.
이는 일반적인 종방향 정지 여유이며, 전체 장애물 탐색 범위가 아니다.
전단 trajectory safety와 후단 obstacle_stop이 읽는 동일 YAML의 배포 사본을 함께 갱신한다.
`terminal_stop_margin`, `min_behavior_stop_margin`, 대향 차량 전용 여유, 횡방향 여유는 유지한다.

## 배포·검증 범위

기준 이미지: `selfcar-2026-vtd:remove-return-plan-20260910`
(`59328dfa33e728af535a9435ec66e90f9c0e918ccd275cf1470dd96e7b6118e5`).
ABLC만 Release 빌드하고 PID/obstacle_stop/ABLC 설정 3종을 함께 설치한다.
실행 중인 Autoware·브릿지와 ROS 파라미터는 변경하지 않는다. 다음 재실행에 적용한다.
회귀시험 및 시나리오 주행은 하지 않는다. 실제 잔여 속도 해소 여부의 주행 확인은 미수행이다.

### 완료 확인

- ABLC Release 빌드 성공. PID는 기존 제어기 바이너리에 설정만 변경했다.
- 이미지 `selfcar-2026-vtd:stop-hold-stationary7-20260910`을 `selfcar-2026-vtd:local`에 연결했다.
  이미지 ID: `24ff1e47b8c07026db4068758485226ea802ce82d6fab2580890cbcebb0a4592`.
- 변경 허용 목록의 14개 이미지 파일만 달라졌으며, 검사한 다른 파일 708개는 동일하다.
  설정 3종의 의미상 변경 키도 확인했다. ABLC와 PID 라이브러리 로드 성공.
- Autoware 컨테이너 `vtd-autoware-run-929c179b6606`과 `selfcar-vtd-bridge`의
  이미지·시작 시각은 변경하지 않았다.
- 기록 및 수정 전 백업:
  `/home/a/autoware-task-backups/stop-hold-stationary7-20260910.Jou7UX/`
  (`before.tar.gz`, `build.log`, `verify_image.py`, `verification.json`).
