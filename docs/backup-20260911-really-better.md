# 0911 진짜더잘되는버전

2026-09-11 사용자가 지정한 현재 버전 백업이다.
Git 태그: `backup-20260911-really-better`.

멀어지는 객체에 대한 회피 후보·RTC 요청·감속 준비 상태 해제 수정을 적용하기 전의
저장소 전체 소스와 설정을 보존한다. 이전 `0911 아주잘되는` 백업 이후의 신규 회피
완료 지점 폭 검사, 신호 UNKNOWN 통과 및 점멸 신호 1회 정지 처리, 정지선 복원에
따른 신호 매핑, Goal Pose 도로 제한속도 저장 스크립트가 포함되어 있다.

이 시점에는 회피 대상이 멀어져도 감속 준비 상태 때문에 승인 대기 후보가 유지될 수
있다. 해당 문제의 해제 로직은 이 백업 이후에 적용한다.

실행 중인 Autoware와 브릿지 및 기본 `selfcar-2026-vtd:local` 이미지:
`sha256:852930c5b97daf91245f8f1ba7a34ac1a426f7b8f434ed52d0b0b0e360976f1d`.
동일 이미지에 로컬 태그 `selfcar-2026-vtd:backup-20260911-really-better`를 보존한다.

GitHub 백업은 저장소 소스·설정·빌드 정의·문서를 포함한다. 외부 지도, VTD 설치,
ML 모델, rosbag, `.env`, Docker 이미지 바이너리는 저장소 백업에 포함하지 않는다.
이 백업 작업 자체는 실행 중인 Autoware나 브릿지를 재시작하지 않는다.

```bash
git show backup-20260911-really-better
git worktree add ../autoware-0911-really-better backup-20260911-really-better
# 다음 실행에서 보존한 로컬 이미지를 사용하려는 경우:
docker tag selfcar-2026-vtd:backup-20260911-really-better selfcar-2026-vtd:local
```
