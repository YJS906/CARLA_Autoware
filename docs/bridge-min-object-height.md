# 브릿지의 0.1m 이하 객체 제외

2026-09-11: `perception.object_min_height_m: 0.1`을 추가했다.
`publish_api_arrays()`에서 높이가 설정값 이하인 객체를 Autoware용 목록에서 제외한다.
차량/사람/UNKNOWN 구분 없이 적용하며, 거리 필터를 비활성화해도 높이 필터는 적용된다.

영향을 받는 출력:

- `/perception/object_recognition/detection/objects`
- `/perception/obstacle_segmentation/pointcloud`의 합성 장애물 점

디버깅용 원본 `/vtd/objects`는 기존대로 유지한다.
물체 전체 높이를 기준으로 거르며 물체 중심의 z 좌표나 개별 LiDAR 점의 높이로 판단하지 않는다.
VTD 높이가 float32이므로 설정값도 같은 정밀도로 비교해 정확히 0.1m인 입력까지 제외한다.
설정은 시작 시 읽으며 변경 적용에는 브릿지 재시작이 필요하다.

## 검증

네트워크를 분리한 컨테이너에서 실제 브릿지 실행 파일에 1109바이트 HLVTD 패킷을
전송하고 발행되는 원본 객체, 인식 객체, 합성 포인트클라우드를 확인했다.

- 0.012 / 0.0121 / 0.0999 / 0.1m: 제외.
- 0.1000001 / 0.32 / 1.09 / 2.5m: 유지.
- 1.352m 차량과 1.625m 사람: 크기와 분류 유지.
- 모든 합성 포인트가 유지된 객체에 속하며, 유지된 객체마다 포인트가 있음을 확인.
- 낮은 객체만 있는 다음 프레임에서는 인식 객체와 합성 포인트클라우드가 모두 비워짐.
- 거리 제한 200m와 거리 필터 비활성화(0m) 두 설정에서 통과.

## 배포

`docker/vtd/bridge-height-filter.Dockerfile`로 이전 요청 4m 이미지 위에 브릿지만 재빌드했다.
빌드에 필요한 VTD 헤더 두 개는 `scripts/build-vtd-image.sh`와 같은 방식으로
별도 디렉터리에 준비해 `--build-context vtd=<헤더 디렉터리>`로 전달한다.

이미지 태그: `selfcar-2026-vtd:bridge-height-filter-20260911` 및 `selfcar-2026-vtd:local`.
이미지 ID: `sha256:1063c7bb11c22ed3582307bc54cea5fb8293410e6d68c3c37f6e259148f0eee2`.
브릿지를 재시작하고 `192.168.50.11:9910` 연결 및 실행 중 파라미터 0.1을 확인했다.
Autoware 컨테이너는 재시작하지 않았다.

확인 시점에는 VTD DATA 수신이 없어 실제 장면에서 필터링된 객체 목록은 확인하지 못했다.
적용 전에도 객체/자차 메시지가 수신되지 않았고, 새 브릿지 진단은
`control_connected=1`, `control_rx_bytes=0`, `participant_data_packets_decoded=0`이었다.
이는 위의 별도 패킷 입력 검증과 구분한다.

변경 전 파일, 이미지 정보, 빌드 로그, 통합 검증 스크립트와 결과:
`/home/a/autoware-task-backups/bridge-height-filter-20260911.j5neymdo`.
기존 브릿지 이미지는 `selfcar-2026-vtd:bridge-before-height-20260911`로 보존했다.
