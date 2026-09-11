# 최종

2026-09-12 사용자가 지정한 현재 버전의 GitHub 백업. 커밋 메시지와 Git 태그는 `최종`이다.

`backup-20260912-almost-final` 이후 다음 변경을 포함한다.

- 장애물 정지 10초, 연속 정지 20초 및 짧은 신호 전이에 따른 객체 UUID 제외·재계획.
- 신호등 정지 유지, 새 UUID 보호, 수동 모드·새 경로·비활성화 시 제외 상태 해제.
- 계획기의 제외 목록을 AEB 전용 점군 필터에도 공유. 대응이 명확한 제외 객체의 점만 제거.
- 관련 소스, 테스트, ROS launch 연결, Docker 빌드 정의와 사용 문서.

직전 작업에서 정책·기하·ROS 연결 테스트 44개가 통과했다. 격리 환경의 설치된 AEB에서
제외 전 비상제동, 제외 후 해제, 새 UUID 등장 시 비상제동을 확인했다. 기존 overlay
라이브러리 40개, VTD 설정 37개 및 AEB 바이너리 3개의 해시는 변경 전과 같았다.
이번 백업을 위해 추가 주행 테스트나 Autoware 재시작은 하지 않는다.

빌드된 이미지: `selfcar-2026-vtd:aeb-uuid-exclusion-20260912`.
이미지 ID: `sha256:e98f9778925d56ee322f09db5062875281b83c4a73445d6e3bd635dbac53df44`.
기본 `selfcar-2026-vtd:local` 태그는 이 이미지를 가리킨다.

GitHub 백업에는 저장소의 소스·설정·빌드 정의·문서가 포함된다. 외부 지도, VTD 설치,
모델, rosbag, `.env`, Docker 이미지 바이너리는 포함하지 않는다.

```bash
git worktree add ../autoware-final 최종
```

동작과 해제 방법은 [obstacle-timeout-replan.md](obstacle-timeout-replan.md)를 참조한다.
