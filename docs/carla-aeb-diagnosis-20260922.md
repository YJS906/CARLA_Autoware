# CARLA 보행자 충돌 후 AEB 조사

기존 보행자 시험에서 계획용 예측 객체가 누락됐고, LiDAR 기반 AEB도 제동하지 않았다. **당시 AEB 내부 후보·RSS·진단을 수집하지 않아 과거 충돌의 AEB 미개입 원인을 하나로 확정할 수는 없다.** 아래는 실행 설정·소스·바이너리에서 확인한 조건과 이번 수정이다. 실제 수정 후 주행 결과는 별도 최종 보고서를 따른다.

## 확인한 입력 및 활성 조건

수정 전 `/control/autonomous_emergency_braking`은 `use_pointcloud_data=true`, `use_predicted_object_data=false`였다. 객체 토픽을 구독하는 것으로 보이더라도 실제 판정에는 객체를 사용하지 않았다. 입력 점군은 `/perception/obstacle_segmentation/pointcloud`이며 VTD의 합성 장애물 점군과 다르다.

동작에는 차량 속도, 점군, MPC 예측 궤적이 필요하다. `check_autoware_state=true`이므로 `/autoware/state`가 DRIVING이어야 하며, 속도 절댓값이 0.1m/s 이상이어야 한다. `use_imu_path=false`이므로 MPC 궤적이 없거나 TF 변환이 실패하면 IMU 경로로 대신 검사하지 않는다. STOP 상태에서 수행한 이전 점군 진단은 AEB 주행 활성 여부를 검증하는 시험이 아니다.

## 실제 LiDAR 후보 생성 조건

`onPointCloud()` → 경로 footprint crop → 군집 생성 → 경로 위 객체 → 속도 추정 → RSS 판정 순서다.

| 단계 | 이전 실행 값 |
|---|---|
| 프레임 | `base_link`, 필요시 TF 변환 |
| 높이 | 0~1.524833m, Prius 차체 높이까지 |
| voxel | x/y/z = 0.1/0.1/0.5m |
| 경로 주변 crop | 차량 반폭 + `expand_width=-0.2` + 추가 1.0m |
| 군집 연결 거리 | 3D Euclidean 0.15m |
| 군집 크기 | 10~10,000점 |
| 군집 높이 | 최소 한 점이 z>0.1m |
| 최종 경로 폭 | 차량 반폭 −0.2m |
| 객체 속도 추정 | 활성, 최초 관측만 있으면 아직 RSS 판정 대상이 되지 않을 수 있음 |
| 과거 객체 유지 | 1.0초 |

이전 정지 진단의 AEB 입력 ROI에는 보행자 점이 5~11개, 중앙값 6개였다. 40개 조사 프레임 중 38개가 10점 미만이었다. AEB가 그 후 높이·voxel 필터와 0.15m 군집 연결을 적용하므로, 이 점군의 상당수는 최소 군집 크기 조건을 충족할 수 없다. 이는 **관측된 입력과 구현상 제약**이며 충돌 순간 모든 프레임이 같은 조건이었다는 증명은 아니다.

정지 물체를 속도 0으로 정상 인지했을 때 RSS 기준은 `v×1.0 + v²/(2×6.0) + 1.0`m이다. 기존 충돌 직전 속도 5.309m/s라면 약 8.66m다. 이 계산은 실제 당시 AEB가 RSS를 산출했다는 의미가 아니다. 후보 생성과 속도 추정이 선행되어야 한다.

## 확인된 MPC 경로 생성 오류와 수정

이전 `generateEgoPath(Trajectory)`는 첫 궤적 점을 추가하기 전에 비어 있는 `std::vector`의 `back()`을 읽었다. 이는 정의되지 않은 동작이다. 설치된 AEB 헤더 SHA256도 로컬 소스와 같았으며, 실행 라이브러리를 역어셈블한 결과 첫 push 전에 vector end 앞 56바이트를 읽는 코드가 확인됐다.

- 이전 실행 라이브러리: `/opt/autoware/autoware_autonomous_emergency_braking/lib/libautoware_autonomous_emergency_braking_node.so`
- 이전 설치 헤더 SHA256: `380b2f93e0c1acacdee9a15889ee001de8a6dc7a4f24be9914630607328b433a`
- 관련 함수 시작 주소: `0x193280`, 문제 읽기: `0x193871` 이후

추적 가능한 overlay 패키지 [autoware_autonomous_emergency_braking](../carla_overlay/src/autoware_autonomous_emergency_braking)에 `!path.empty()` 조건을 추가했다. 첫 점을 무조건 보존하고, 두 번째부터 기존 중복 거리 필터를 적용한다. 빈 궤적·원점의 단일 점·중복/근접 점이 포함된 궤적의 회귀 시험 3개를 추가했다. 기존 시험 11개와 합쳐 총 14개다.

이 오류가 존재한다는 사실과 **그 오류가 과거 보행자 충돌의 직접 원인이라는 주장**은 구분해야 한다. 당시 후보·경로 내부 기록이 없으므로 후자는 입증하지 않았다.

수정 전 설치 라이브러리에 회귀 시험을 링크하여 실행한 결과 14개 중 12개가 통과하고, `predictedPathRetainsFirstPoint`와 `predictedPathFiltersDuplicatesAfterFirstPoint`가 실패했다. 실제로 첫 점 보존이 깨지는 것도 재현했다. [수정 전 gtest 결과](validation/carla-aeb-repair-20260922/before-tests.xml), [로그](validation/carla-aeb-repair-20260922/before-tests.log), [바이너리 검사](validation/carla-aeb-repair-20260922/before-generate-path-assembly.txt)를 보존했다.

수정 후 최종 이미지의 설치 라이브러리에 동일한 시험을 링크하여 **14개 모두 통과**했다. [수정 후 gtest 결과](validation/carla-aeb-repair-20260922/after-tests.xml)와 [로그](validation/carla-aeb-repair-20260922/after-tests.log)를 보존했다. 실제 GT 실행에서도 `use_predicted_object_data=true`, `use_pointcloud_data=true`를 확인했다. 단위 시험 통과와 실제 AEB 주행 개입 성공 여부는 구분한다.

## CARLA ground-truth 모드의 AEB 설정

GT 모드에서만 [추가 설정](../config/carla/control/autonomous_emergency_braking/ground_truth.param.yaml)을 기존 AEB 설정 뒤에 적용하여 `use_predicted_object_data=true`로 한다. LiDAR 경로와 기존 임계값은 보존하며, 객체의 형상·위치와 MPC 차량 경로의 교차 검사도 함께 사용한다. 센서 모드의 기본 설정은 바꾸지 않는다.

이는 센서 검출 모델을 고친 것이 아니라 **시뮬레이터 정답 객체 입력을 활용하는 추가 AEB 경로**다. 주행 계획 단계에서 미리 정지하여 AEB가 필요하지 않았던 시험을 AEB 개입 성공으로 세면 안 된다. 실제 AEB 개입은 `aeb_emergency_stop` ERROR, RSS/거리, `metrics decision=brake`와 제동 반응으로 구분한다.

## 수집 도구 및 시험 방법

[carla_aeb_observer.py](../tools/carla_aeb_observer.py)는 파라미터·진단·MPC 궤적·속도·AEB marker/RSS/metrics·예측 객체·점군 군집 요약을 읽기 전용으로 수집한다. CARLA tick, 차량 제어, 파라미터 변경을 하지 않는다. 점군 계산은 고정 전방 영역에 대해 높이/voxel/연결 군집 조건을 재현한 참고값이며, MPC footprint crop 전체를 재현한 것은 아니다. 실제 판정 근거는 AEB 자체 메시지다.

초기 수정 후 회피 1회 관측 파일은 ROS `byte` 타입의 진단 레벨을 JSON으로 쓰다가 중단되어 사용할 수 없었다. 시험 도구의 주행 결과 파일과는 별개다. 관측기를 수정하여 레벨을 정수로 변환하고 파일 저장을 원자적으로 처리했다. 수정된 관측기의 4초 스모크에서 실제 AEB 진단 13개가 정상 JSON으로 보존됐으며 콜백 오류는 없었다. 이 이전의 불완전한 파일로 AEB 개입 여부를 주장하지 않는다.

최종 고정 이미지로 수행한 8개 시험은 모두 통과했고, 그중 기본 주행을 제외한 7개의 정상 AEB 관측 파일을 분석했다. 주행 시험 구간의 AEB 진단 800개 모두 `No Collision`이었으며 ERROR와 `decision=brake`는 0개였다. 따라서 보행자 정지 3회는 **GT 객체 입력을 이용한 계획·제어 정지 성공**이며 독립적인 AEB 긴급 개입 성공으로 보고하지 않는다. 보행자 시험의 점군 전방 영역에서는 높이·voxel 적용 후 최대 연결 군집이 각각 3·3·2점이었다. 센서 인식 및 희박한 점군 기반 긴급 제동의 성능은 여전히 별도 검증 대상이다.

회피는 세 번 모두 장애물 통과 및 후속 주행 조건을 만족했지만, 도중 일시 정지 후 재출발했고 실제 감속이 ROS 가속도 명령보다 크게 나타났다. AEB 개입 기록은 없었다. 실행 제동 보정값과 실제 동역학의 불일치가 원인 후보지만, 이번에는 원인 분리 교정 시험이나 추가 튜닝을 하지 않았다. 최종 주행 보고서에 제동 응답 불일치와 남은 일시 정지를 명시한다.

[설치 라이브러리 시험 CMake](../tools/testing/aeb/CMakeLists.txt)는 최종 이미지의 설치된 AEB 라이브러리에 14개 gtest를 링크한다. ROS가 실제 주행에 연결되지 않도록 `--network none`과 별도 `ROS_DOMAIN_ID`를 사용하는 임시 컨테이너에서 실행한다. 컴파일 시 `CCACHE_DISABLE=1`을 지정하면 기본 이미지의 읽기 불가능한 ccache 디렉터리를 사용하지 않는다.
