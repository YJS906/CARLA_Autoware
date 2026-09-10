# 0911 아주잘되는

2026-09-11 사용자가 지정한 현재 버전 백업명이다.
Git 태그: `backup-20260911-very-good`.

## 포함된 주요 설정과 변경

- 회피 요청 최소거리 4m.
- 차선변경 RSS 추가 여유 배율 0.5.
- 두 차선 이상 직접 변경 시 준비시간 1초.
- obstacle_stop 정차 후 승인된 원래 곡선을 앞쪽으로 옮기는 재계획.
- 교차로 통과 판단 이후 충돌 재검사 및 차선변경의 신호 근접 제한 조정.
- 브릿지에서 높이 0.1m 이하 객체를 Autoware 인식 객체와 합성 장애물 점에서 제외.
- 이 시점의 소스, 설정, 테스트 코드, 빌드 파일 및 변경 설명 문서 전체.

차선 왕복 문제에 대해 논의한 추가 선택·복귀 정책은 아직 구현되지 않았다.
각 변경의 검증 범위는 해당 문서에 기록되어 있다.

## 실행 이미지

현재 `selfcar-2026-vtd:local`과 브릿지의 이미지:
`sha256:1063c7bb11c22ed3582307bc54cea5fb8293410e6d68c3c37f6e259148f0eee2`.
동일 이미지에 `selfcar-2026-vtd:backup-20260911-very-good` 로컬 태그를 추가한다.

실행 중인 Autoware는 브릿지 재빌드 전 이미지
`sha256:921dc10975c9e9de204cccde86f23739d220d9cfc4aa8c8faaaf628c1ab1b947`이며,
새 이미지의 변경 범위는 브릿지 패키지다. 백업 작업에서는 실행 세션을 재시작하지 않는다.

GitHub에는 저장소 소스와 빌드 정의를 백업한다. Docker 이미지 자체와 외부 VTD 설치,
지도, rosbag, 로컬 `.env`는 포함하지 않는다.

## 버전 확인 및 복원

```bash
git show backup-20260911-very-good
# 별도 작업 디렉터리에서 백업 소스를 확인한다.
git worktree add ../autoware-0911-very-good backup-20260911-very-good
# 보존된 로컬 이미지를 다음 실행 이미지로 선택한다.
docker tag selfcar-2026-vtd:backup-20260911-very-good selfcar-2026-vtd:local
```
