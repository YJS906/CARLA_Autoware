# 경계 정지 후 재계획

`path_optimizer`가 `outside_drivable_area`로 정지한 경우에만 작동한다. 기존 경계
검사와 신호등·장애물·AEB 정지는 유지하며, 시간 경과로 속도를 복원하거나 RTC를
강제 승인하거나 차선변경을 완료 처리하지 않는다.

1. 정지 속도(절댓값 0.1 m/s 이하)에서 경계 이탈 지점이 2 m 이내이면 이전 최적화
   결과를 버리고 한 번 다시 최적화한다. 성공하지 않으면 기존 경계 정지를 발행한다.
2. 새 `/planning/planning_factors/path_optimizer` 피드백이 10초 동안 연속해서
   가까운 경계 정지를 보고하면, 실행 중인 차선변경이 현재 차량 위치·방향에서
   승인된 목표 차선으로 연결하는 새 곡선을 찾는다. 진행 중 목표 차선과 RTC는 유지한다.
3. 연결 길이 6–40 m와 기존 정지 후 재계획 속도 상한(현재 1.5 m/s) 이하를 탐색한다.
   차체의 실제 사용할 차선 영역, 완료 지점 폭, 조향 한계, 정지 공간과 기존 충돌
   검사를 통과한 후보만 교체한다. 멀리 있는 꼬리 구간은 차선변경 후 정지 공간이
   확보되는 경우에만 잘라낼 수 있다.
4. 새 경로에 대응하는 최적화 초기화 요청을 보낸다. 최종 출력에는 기존 경계 검사를
   다시 적용한다. 교체 실패 시 원래 경로와 정지를 유지하고, 다음 탐색은 1초 이후에
   이어서 한다. 승인한 후보도 하류에서 다시 막히면 새 피드백으로 10초를 다시 센다.

일반 최적화/차선변경 주기에 전체 후보 경계 검사를 추가하지 않는다. 복구 탐색만
기존 `lane_change.time_limit` 예산을 사용하며, 후보 시작 인덱스를 보존한다. 개별
기하/충돌 검사 한 번의 실행 시간은 이 예산보다 길 수 있다.

피드백은 ROS 시간으로 검사한다. 수신/발행 시각이 0.5초 이상 오래되거나, 피드백이
끊기거나, 정지 원인이 사라지거나, 주행/수동 상태가 되면 대기를 다시 시작한다.
다른 프레임이나 현재 경로에서 떨어진 정지는 사용하지 않는다. 차선변경 종료 시
복구 상태를 지운다. 경계 정지 중에는 기존 강제승인 3초 완료 처리도 적용하지 않는다.

VTD 설정:

- `option.enable_drivable_area_recovery: true` (path optimizer)
- `lane_change.drivable_area_recovery.enabled: true`
- `duration: 10.0`, `retry_interval: 1.0`, `message_timeout: 0.5`, `stop_distance: 2.0`

코드 기본값은 비활성이다. `enable_outside_drivable_area_stop`을 끄지 않는다.
지도/주행가능영역 자체가 잘못됐거나 현재 자세에서 연결 가능한 후보가 없으면 정지를
유지한다. 이 기능이 모든 위치에서 복구 경로의 존재를 보장하지는 않는다.

증분 이미지 빌드:

```bash
docker buildx build --load -f docker/vtd/drivable-area-recovery.Dockerfile \
  -t selfcar-2026-vtd:drivable-area-recovery-20260912 .
```

기준 이미지는 AEB 점군 좌표 수정이 들어간 `aeb-cloud-frame-20260912`다. 차선변경
기본 클래스의 레이아웃이 바뀌므로 avoidance/external-request 파생 모듈도 함께
빌드한다. 전체 Dockerfile도 새 path optimizer overlay를 자동으로 빌드한다.
