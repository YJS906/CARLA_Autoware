# 승인 거리 35 m 및 S자 최소 전진 길이 절반 (2026-09-10)

## 변경 범위

- `avoidance_by_lane_change.max_execution_distance`: 20.0 → 35.0 m.
  VTD 시작 YAML, 모듈 기본 YAML, 구조체 기본값과 파라미터 미지정 시 기본값을 일치시킨다.
- `minimumGeometricShiftLength`: 기존 기하학적 최소 길이에 0.5를 곱한다.
  관측된 횡이동 3.1997 m와 6.3997 m에서 각각 9.45885 → 4.72942 m,
  13.37714 → 6.68857 m다. 준비 구간 길이는 별도다.
- 이 함수는 차선변경과 정적 회피가 공유한다. 실제 생성 길이는 저크·횡가속도 등
  다른 하한 때문에 더 길어질 수 있다. 실제 곡률·충돌 검사는 완화하지 않는다.
- 복귀경로 사전 확보, 도로 경계, 객체 충돌, RTC 승인 조건은 그대로 유지한다.
  35 m 안이라고 승인되는 것은 아니다.
- 실행 중인 Autoware/브릿지에는 파라미터나 코드를 주입하지 않고 재시작 시 적용한다.

## 이미지 재현

기준은 기존 실행 이미지 `selfcar-2026-vtd:lane-speed-preparation-20260909`,
ID `f2b1866d7835a39b4a9cef344a3c0a489b8291142d10fc6d06425c177ee4819b`다.
기준 이미지의 기존 정적 회피 소스(롤백 반영본)를 보존하여 사용한다.
호스트의 전체 정적 회피 디렉터리를 덮어쓰지 않는다.

공통 inline 함수의 사용처인 lane-change, static-obstacle-avoidance와
관련 avoidance-by-lane-change, external-request-lane-change 4개 모듈을 다시 빌드한다.
그 밖의 모듈과 브릿지/제어 설정은 기준 이미지에서 유지한다.

```bash
docker build -f docker/vtd/approval35-halfshift.Dockerfile \
  -t selfcar-2026-vtd:approval35-halfshift-20260910 .
```

패키징 확인 후에만 `selfcar-2026-vtd:local` 태그를 새 이미지에 연결한다.
`/home/a/autoware_run`은 이 local 태그를 조회하여 새 컨테이너를 실행한다.
기존 실행 컨테이너는 태그 변경만으로 바뀌지 않는다.

변경 전 파일 백업 및 패키징 확인 자료:
`/home/a/autoware-task-backups/approval35-halfshift-20260910.SppD2z`.
전체 회귀시험, VTD 시나리오 주행과 실제 추종 검증은 이 작업에 포함하지 않는다.

## 완료 결과

- 빌드: 4개 패키지 성공, 컴파일 오류 없음. 기존 의존 패키지 경고만 발생했다.
- 새 이미지 ID: `83e78aeeb9ffb2bad7b9f3784250d94dcf008c196a29256c53543efb6edfeec4`.
- `selfcar-2026-vtd:approval35-halfshift-20260910`과 `selfcar-2026-vtd:local`을
  새 이미지에 연결했다. 기존 실행 컨테이너는 재시작하지 않았다.
- 이미지 내부의 VTD 설정/실제 launch 설정/모듈 기본 설정 3곳 모두 35.0 m다.
- 설치된 헤더로 계산한 최소 길이는 4.72942 m, 6.68857 m다.
- 4개 모듈의 동적 라이브러리를 즉시 심볼 해석 방식으로 로드하는 확인을 통과했다.
- overlay 라이브러리 33개 중 위 4개만 변경됐다. 나머지 29개는 동일하다.
  기존 정적 회피 helper/scene/속도 프로파일/충돌 소스와 브릿지 소스도 동일하다.
- 패키징 확인은 네트워크를 차단한 별도 컨테이너에서 수행했으며 ROS 노드나
  시나리오는 실행하지 않았다. 복귀경로 확보 조건은 여전히 승인을 막을 수 있다.
