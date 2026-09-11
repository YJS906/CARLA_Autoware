# 구버전 PC 업데이트 — 2026-09-12, 브릿지·지도 포함

고정 백업 태그: **`backup-20260912-intersection-exit`**
저장소: <https://github.com/DCAM1/selfcar_2026_>

아래 절차는 이미 구버전 Autoware/VTD와 Docker가 설치된 Linux PC용이다.
기존 저장소·이미지·지도를 보존하고 별도 폴더에서 최신 소스를 빌드한다.
`main`, 예전 `최종` 태그, `chore/reproducible-vtd-source` 대신 위 태그를 사용한다.
기존 `최종` 태그는 2026-09-12의 이전 버전이며 이동하지 않았다.

## 다른 PC의 Codex에 전달할 요청

```text
https://github.com/DCAM1/selfcar_2026_ 의 backup-20260912-intersection-exit 태그를
별도 폴더에 clone하고 docs/update-existing-pc-20260912.md대로 업데이트해줘.
이 PC에는 구버전과 Docker가 설치되어 있으니 기존 설치·.env·실행 래퍼·이미지를 먼저
확인해서 보존하고 재사용해. 저장소의 지도 백업도 새 디렉터리에 복원해서 사용해.
브릿지까지 정식 전체 Docker 빌드하고 새 이미지와 새 실행 경로로 연결해줘.
구버전 로컬 중간 이미지에 의존하는 incremental Dockerfile은 사용하지 마.
현재 PC의 VTD 경로, Host IP, DISPLAY, CSV 경로를 유지하고 검증은 최소한으로 해.
기존 프로세스는 빌드 중 유지하고, 실행 중인 Autoware/브릿지 중지나 새 실행은 내가
별도로 요청할 때 해. 자동주행 engage나 테스트 주행은 하지 마.
마지막에 commit/tag, 이미지 ID, 지도 hash, 실행 명령, 롤백 방법을 보고해줘.
```

## 포함 범위와 준비물

| 항목 | 이 백업의 처리 |
| --- | --- |
| Autoware 변경, launch, 파라미터 | `vtd_overlay/src/`, `config/vtd/`, `docker/`에 포함 |
| 브릿지 C++/Python/메시지/신호 ID 매핑 | `vtd_overlay/src/vtd_ros2_bridge/`에 포함, 같은 이미지로 빌드 |
| CSV 표시 갱신, 실행 래퍼 | `scripts/`, `vtd_bridge`, `autoware`, `set_route`에 포함 |
| 현재 지도 | `maps/backup-20260912/`의 압축 파일과 SHA-256 manifest에 포함 |
| 지도 구성 | 실제 사용하는 OSM, projector YAML, PCD, 원본 OpenDRIVE 4개 파일 |
| `.env`, 사용자 CSV, RViz 저장 상태 | 대상 PC의 기존 것을 보존·재사용. Git clone에 개인 설정은 없음 |
| VTD 설치, ML 모델, Docker 이미지 바이너리, rosbag | Git에 없음. 기존 설치 재사용, 이미지는 소스로 빌드 |

지도에는 현재 파일에 저장된 차선 연결·정지선·속도 제한도 포함된다. 과거 지도 백업이나
분석 이미지/보고서는 실행 지도 대신 사용하지 않는다. 파일명만 같은 구버전 지도를
그대로 쓰면 같은 동작을 재현할 수 없으므로 **이번 압축본을 새 경로에 복원**한다.

필요한 호스트 도구는 Git, Python 3, Docker Compose/Buildx이다. 기존 GUI 실행 환경의
NVIDIA Container Toolkit 및 X11도 유지한다. 호스트에 ROS 전체를 새로 설치할 필요는 없다.
Docker만 있고 VTD 개발 헤더가 없으면 브릿지 소스 빌드는 불가능하므로 기존
`VTD_INSTALL_DIR`의 아래 두 파일을 확인한다.

```text
Develop/Communication/VtdApi/lib_cxx11/include/VtdToolkit/viRDBIcd.h
Develop/Communication/Common/viRDBTypes.h
```

## 1. 기존 설치 보존 후 고정 태그 받기

기존 홈 실행 파일 `~/autoware_run`, `~/vtd_bridge`, `~/set_route`를 읽어서 실제
저장소 위치를 먼저 확인한다. 아래 `old_repo`는 **대상 PC의 실제 경로**로 바꾼다.
기존 폴더를 `reset --hard`, `clean`, 삭제하거나 전체 Docker prune을 하지 않는다.

```bash
old_repo="$HOME/autoware"
new_repo="$HOME/autoware-0912"
backup_dir="$HOME/autoware-update-backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup_dir"
git -C "$old_repo" status --short > "$backup_dir/old-status.txt"
git -C "$old_repo" rev-parse HEAD > "$backup_dir/old-commit.txt"
git -C "$old_repo" diff --binary HEAD > "$backup_dir/old-changes.patch"
cp -p "$old_repo/.env" "$backup_dir/old.env"
docker image ls --no-trunc > "$backup_dir/images.txt"
docker ps --no-trunc > "$backup_dir/containers.txt"
git clone --branch backup-20260912-intersection-exit --single-branch \
  https://github.com/DCAM1/selfcar_2026_.git "$new_repo"
cd "$new_repo"
git rev-parse HEAD
git rev-parse 'backup-20260912-intersection-exit^{commit}'
cp -p "$old_repo/.env" .env
```

위 두 SHA가 일치해야 한다. 새 폴더가 이미 있으면 다른 이름을 사용한다.
기존 폴더 전체를 남기므로 미추적 파일도 보존된다.

## 2. 지도 복원과 새 `.env` 설정

```bash
python3 scripts/restore-map-backup.py --destination "$HOME/autoware-maps-0912"
```

압축본과 복원된 각 파일의 SHA-256을 검사한다. 기존 대상이 동일하면 재사용하고,
다르면 덮어쓰지 않고 실패한다. 이 경우 새 destination 이름을 선택한다.
복원은 현재 실행 중인 지도나 ROS 토픽을 변경하지 않는다.

새 `.env`에서는 아래 항목만 우선 바꾼다. `AUTOWARE_MAP_DIR`, 모델 경로,
`VTD_INSTALL_DIR`, `HLVTD_HOST`, `DISPLAY`는 **대상 PC의 값**을 유지한다.
CSV를 지정하려면 `AUTOWARE_CSV_PREVIEW_CSV`에 대상 PC 파일의 절대 경로를 사용한다.
동일 이미지 태그를 덮어쓰지 않도록 새 이미지에 별도 이름을 준다.

```bash
python3 - <<'PY'
from pathlib import Path
import shlex
p = Path('.env')
values = {
    'AUTOWARE_BASE_IMAGE': 'ghcr.io/autowarefoundation/autoware@sha256:86d12e0f2504b058b54faead3a6113b5777c778899fc7a7c362e787808fdac24',
    'VTD_IMAGE': 'selfcar-2026-vtd:backup-20260912-intersection-exit',
    'VTD_MAP_DIR': str(Path.home() / 'autoware-maps-0912'),
    'VTD_MAP_RELATIVE_PATH': 'HL_FMA_VTD_LivingLab_topology_fixed',
}
lines = [line for line in p.read_text().splitlines()
         if line.split('=', 1)[0].strip() not in values]
lines += [k + '=' + shlex.quote(v) for k, v in values.items()]
p.write_text('\n'.join(lines) + '\n')
PY
docker compose version
docker buildx version
./scripts/verify-reproducibility.sh
docker compose --env-file .env -f docker/vtd/compose.yaml config --quiet
```

복원 destination을 바꿨다면 Python 예제의 `VTD_MAP_DIR`도 그 값으로 바꾼다.
`HOST_UID/HOST_GID`, X11 `DISPLAY`, NVIDIA runtime은 대상 PC에서 확인한다.
HLVTD Host IP를 원본 PC의 주소로 무조건 덮어쓰지 않는다.

## 3. 브릿지 포함 전체 이미지 빌드

```bash
set -o pipefail
./scripts/build-vtd-image.sh 2>&1 | tee "$backup_dir/build.log"
docker image inspect selfcar-2026-vtd:backup-20260912-intersection-exit \
  --format '{{.Id}}'
```

빌드가 실패하면 다음 단계로 진행하지 않고 로그의 오류를 해결한다.

**정식 `docker/vtd/Dockerfile` 한 번으로 19개 overlay 패키지를 빌드한다.**
신규 path optimizer, lane change와 파생 모듈, 브릿지, AEB 제외 필터가 포함된다.
`src/` 재다운로드, `vcs import`, 호스트 colcon 설치는 이 경로에 필요하지 않다.
메모리 사용을 줄이기 위해 단일 컴파일 작업으로 빌드하므로 구버전 실행보다 오래 걸릴 수 있다.

`intersection-exit.Dockerfile`, `drivable-area-recovery.Dockerfile` 등은 원본 PC에서
사용한 증분 작업 기록이다. 다른 PC에 없는 중간 이미지 태그에 의존하므로 이번 설치에
사용하지 않는다. 기존 이미지를 새 이름으로 tag만 바꾸는 것도 업데이트가 아니다.

네트워크 없이 패키지 경로만 확인한다. 차량과 연결하거나 노드를 띄우지 않는다.

```bash
docker run --rm --network none --entrypoint bash \
  selfcar-2026-vtd:backup-20260912-intersection-exit -lc '
  source /opt/ros/jazzy/setup.bash
  source /opt/autoware/setup.bash
  source /opt/selfcar_overlay/setup.bash
  for p in vtd_ros2_bridge autoware_path_optimizer \
      autoware_behavior_path_lane_change_module selfcar_obstacle_timeout_replan; do
    test "$(ros2 pkg prefix "$p")" = /opt/selfcar_overlay || exit 1
  done
  ros2 pkg executables vtd_ros2_bridge
  '
```

여기까지 완료하면 **설치 완료, 기존 프로세스는 그대로**이다. 소스만 바뀌어도 실행 중인
컨테이너의 C++는 바뀌지 않는다. 아래 재실행은 사용자가 요청한 뒤 진행한다.

## 4. 실행 경로 연결과 사용자가 시작할 명령

홈의 기존 `autoware_run`, `vtd_bridge`, `set_route` 래퍼는 먼저 `backup_dir`에 복사한다.
그 후 각각 새 저장소의 `autoware`, `vtd_bridge`, `set_route`를 `exec`하도록 절대 경로를
수정한다. 예를 들어 새 `~/vtd_bridge` 내용은 다음과 같다(사용자 홈에 맞춰 경로 변경).

```bash
#!/usr/bin/env bash
set -euo pipefail
exec /home/USER/autoware-0912/vtd_bridge "$@"
```

실행 권한도 부여한다. 저장소 launcher를 홈에 단순 심볼릭 링크하면 `.env` 검색 위치가
달라질 수 있으므로 위처럼 래퍼를 쓴다. 별도 래퍼 없이 아래 명령을 직접 실행해도 된다.

기존 관련 컨테이너 이름을 `docker ps`로 확인하고, 사용자 요청 시 해당 컨테이너만
정지한다. 실행 중인 구버전 브릿지가 있으면 새 브릿지가 중복 실행을 거부한다.
이미지 빌드만으로 기존 브릿지 컨테이너가 업데이트되지는 않는다.

```bash
# 터미널 1: .env의 HLVTD_HOST 사용. 필요하면 실제 Host IP를 인자로 전달.
cd "$HOME/autoware-0912"
./vtd_bridge

# 터미널 2: Autoware 시작. CSV 없이 실행하면 주행 경로를 자동 제출하지 않음.
cd "$HOME/autoware-0912"
./autoware

# 사용자가 CSV 주행 경로 제출을 원하는 경우에만 사용
# ./autoware "$HOME/route_example.csv"
```

두 launcher의 시작 로그에 동일한 새 이미지 태그/ID가 나오는지 확인한다.
`CSV preview only`, `no route request`는 브릿지 실패가 아니라 표시 전용 동작이다.
브릿지는 별도 실행이고 Autoware GUI는 두 번째 명령으로 시작한다. 새 도로지도는
새 Autoware 컨테이너가 로드할 때 적용된다. 자동 모드 활성화는 사용자가 수행한다.

HLVTD 연결은 TCP 9910, LiDAR는 UDP 9912이고 RTSP 8554는 해당 기능 사용 시의 포트다.
TCP 클라이언트에는 OS가 출발 포트를 배정한다. 이를 없애려고 설정을 바꾸지 않는다.

## 5. 롤백과 완료 보고

새 프로세스 정지 후 보존해 둔 **기존 저장소의 launcher와 `.env`**로 실행한다.
새 설치는 다른 이미지 태그와 다른 지도 경로를 쓰므로 기존 이미지·지도는 그대로다.
홈 래퍼를 바꿨다면 `backup_dir`의 원본도 복원한다. 새 지도를 옛 지도 위에 복사하거나
원격 `최종` 태그를 이동시키지 않는다.

Codex는 완료 시 고정 태그/commit, 새 이미지 ID, 복원 지도 hash, 실제 `.env` 경로,
홈 래퍼 연결, 시작 명령 및 재실행 여부를 짧게 보고한다.

원본 PC에서는 최신 C++ 증분 빌드 및 현재 상황 재생이 통과했다. 준비 구간 5.23 m 후
변경을 시작했고 최종 optimizer 경계 정지 factor 0, 출력 속도 1.5 m/s를 확인했다.
이번 이관 문서 작성 중에는 전체 clean build와 다른 PC 주행을 재실행하지 않았다.
새 PC의 빌드 성공과 최소 패키지 확인 결과는 새 PC에서 별도로 확인해야 한다.
