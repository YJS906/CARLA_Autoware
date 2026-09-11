# 0912 진짜거의마지막

2026-09-12 사용자가 지정한 수정 전 버전 백업.
Git 태그: `backup-20260912-almost-final`.

`0911 진짜더잘되는버전` 이후, 멀어지는 회피 대상의 후보·RTC 요청·저장된 감속 준비 상태를 함께 해제하는 수정까지 포함한다. 기존 7초 강제 차선변경 흐름과 교통신호 처리는 유지된 상태이다.

이번에 요청한 장애물 무시 및 10초/20초 재계획 타임아웃은 아직 포함하지 않는다.

기본 Autoware 이미지: `sha256:46c5d416acc06bbf206c874d787e864e0b1924bc6f85788cf5910981ac4065f1`.
로컬 이미지 백업 태그: `selfcar-2026-vtd:backup-20260912-almost-final`.

GitHub 백업 범위는 저장소의 소스·설정·빌드 정의·문서이다. 외부 지도, VTD 설치, 모델, rosbag, `.env`, Docker 이미지 바이너리는 포함하지 않는다.

```bash
git worktree add ../autoware-0912-almost-final backup-20260912-almost-final
# 다음 실행에서 보존된 이미지를 사용할 경우:
docker tag selfcar-2026-vtd:backup-20260912-almost-final selfcar-2026-vtd:local
```
