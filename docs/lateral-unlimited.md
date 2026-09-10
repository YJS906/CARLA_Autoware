# VTD 차선변경 횡가속도·횡저크 제한 해제 (2026-09-10)

## 적용 범위

일반 차선변경, 차선변경 회피(avoidance_by_lane_change), 외부 요청 차선변경이 공유하는
lane_change 계산에 적용한다. 차선 내 정적 회피 모듈과 후단 속도계획기·제어기의 설정은
이번 작업에서 변경하지 않는다. 실행 중인 Autoware/bridge는 변경하거나 재시작하지 않는다.

VTD lane_change 설정:

```yaml
trajectory:
  enable_lateral_acceleration_limit: false
  enable_lateral_jerk_limit: false
```

숫자를 크게 만드는 방식이 아니라 제한 검사 자체를 끈다. 기존 lateral_jerk 및
lateral_acceleration 테이블은 다시 제한을 켤 때 사용할 유한한 값으로 남겨 두되,
두 스위치가 false이면 횡이동 최소 시간·후보 승인·곡률 기반 속도 상한에 적용하지 않는다.
일반 패키지의 기본값과 미지정 기본값은 true이며, VTD 실행 설정에서만 false이다.

## 계산 오류 수정

이전 후보 검사에서는 prepare와 lane_changing 종가속도의 절댓값 중 큰 값을
횡이동 전 구간에 적용했다. 준비 때 가속한 뒤 정속으로 횡이동하는 경로에도
준비 가속도가 들어가 횡저크가 과대 계산됐다.

현재 경로 지점이 준비 구간인지 횡이동 구간인지 확인하여 해당 구간의 가속도를
사용한다. 감속 프로파일이 있으면 해당 지점의 프로파일 가속도를 사용한다.
횡가속도의 시간 미분은 아래와 같이 계산하고, 마지막에 절댓값을 비교한다.

```text
a_lateral = v² κ
j_lateral = v³ (dκ/ds) + 2 v a_longitudinal κ
```

속도 상한의 이분 탐색에서는 단조성을 위해 두 항의 절댓값 합을 사용하지만,
여기에도 준비 가속도를 횡이동으로 가져오지 않는다.

## 제한 해제 시 후보 생성

무한대 값을 시간 계산식에 넣지 않는다. 두 제한이 꺼지면 횡가속도·횡저크로
정해지는 최소 시간은 없으며, 차량 기하에서 얻은 유한 길이와 계획 속도로 시간을
계산한다. 기존 기하 최소 길이의 0.5 배 설정은 유지한다.
이 길이의 1.0, 1.5, 2.0, 2.5, 3.0 배 후보를 검색하여 지나치게 짧은 후보만
생성되지 않게 한다. 이는 후보 표본이지 최대 조향각 완화가 아니다.
차선 말단/복귀 거리 추정에도 같은 제한 스위치와 기하 길이를 사용한다.

후보 검사, 곡률 속도 상한, 저속 재계획, 감속 예측, abort 경로에 같은 스위치를
적용한다. 최대 조향각으로 정해지는 실제 곡률, 경로 연결 각도, 충돌 검사, RSS,
경계·중앙선 및 종방향 가감속·저크 제한은 유지한다. 따라서 다른 사유의 unsafe가
사라진다는 뜻은 아니다. 타이어 접지·횡동역학의 실차 안전성을 보장하는 설정이 아니며
시뮬레이터 전용이다.

## 이미지와 복원

- 이전 이미지: selfcar-2026-vtd:signal-independent-avoidance-20260910
- 새 이미지: selfcar-2026-vtd:lateral-unlimited-20260910
- 빌드 정의: docker/vtd/lateral-unlimited.Dockerfile
- 수정 전 소스: /home/a/autoware-task-backups/lateral-unlimited-20260910.JtHFdI/before.tar.gz

기존 실행 이미지 위에 이번에 수정한 lane_change 파일만 복사한다.
파라미터 구조체 변경에 따른 ABI 불일치를 막기 위해 lane_change,
avoidance_by_lane_change, external_request_lane_change 세 패키지를 함께 빌드한다.
차선 내 정적 회피의 이전 롤백 상태와 bridge 소스는 그대로 보존한다.
검증 후 local 태그를 새 이미지로 연결하며, 적용 시점은 사용자의 다음 재실행이다.
회귀시험·시나리오 주행은 수행하지 않는다.

완료 확인: 세 패키지 Release 빌드와 네 모듈의 동적 라이브러리 로드에 성공했다.
변경된 라이브러리는 위 세 패키지뿐이며, 나머지 overlay 라이브러리와 기존 정적 회피·
bridge 소스의 해시는 이전 이미지와 일치한다. 실제 실행 설정 두 경로에서 두 스위치가
false임을 확인했다. local 태그를 새 이미지
`b3c074f48927d30f11433736ec97abc77d6de83fee9d77c2f86fe9f3162396a2`로 연결했다.
실행 중인 두 컨테이너는 기존 이미지와 시작 시각을 유지했다.
검증 기록은 백업 폴더의 image-before.json, image-after.json에 있다.
