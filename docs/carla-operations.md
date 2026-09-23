# CARLA·Autoware 실행 명령

이 머신의 저장소는 `/home/a/carla_pp`다. 다음은 Town05에서 CARLA 객체 정보를
직접 입력하는 `ground_truth` 모드 명령이다. 기본 `sensor` 모드와 구분한다.
수정 이미지는 `selfcar-2026-carla:local`이며, 시작 명령 자체는 이미지를 빌드하지 않는다.

## 시나리오 편집기

CARLA가 실행된 상태에서 별도 터미널에 입력한다.

```bash
/home/a/carla_pp/scripts/carla/carla_scenario_gui
```

배치·저장·불러오기는 [GUI 사용법](carla-scenario-gui.md)을 참고한다.

## CARLA와 Autoware 모두 다시 시작

재시작하면 현재 차량·보행자·센서와 Autoware 경로가 초기화된다.
필요한 편집기 시나리오는 **저장**으로 JSON 파일에 보관한 뒤 편집기를 닫는다.

1. 교통 컨트롤러, Autoware, CARLA 순서로 종료한다.

```bash
cd /home/a/carla_pp
./scripts/carla/carla_city_traffic stop
./scripts/carla/carla_autoware stop
pkill -TERM -f '[C]arlaUE4-Linux-Shipping'
```

각 종료 명령이 끝난 뒤 다음 단계로 진행한다. `pkill`은 해당 CARLA 서버 프로세스를
종료하며 이미 꺼져 있으면 대상이 없다. 교통 정리 오류가 나오면 상태/로그를 확인한다.

2. CARLA 창이 완전히 닫힌 뒤 다시 실행한다. 이 터미널은 실행 중인 채로 둔다.

```bash
/home/a/carla_pp/scripts/carla/carla_run -quality-level=Low
```

3. CARLA 창과 월드가 로드되면 **다른 터미널**에서 Autoware를 시작한다.

```bash
cd /home/a/carla_pp
export CARLA_PERCEPTION_MODE=ground_truth
./scripts/carla/carla_autoware start-town05
./scripts/carla/carla_autoware status
```

`start-town05`는 Town05 지도와 직접 위치 입력, 시뮬레이터 신호 연동으로 시작한다.
Autoware는 자동으로 AUTO에 진입하지 않는다. RViz에서 목적지를 지정하고 주행 모드를 선택한다.
편집기를 다시 열어 저장한 시나리오를 불러온 후 현재 ego와의 위치를 확인하고 실행한다.

## Autoware만 다시 시작

CARLA 서버가 켜져 있는 상태에서 다음을 실행한다. 이 경우에도 현재 객체 배치와 경로는 초기화된다.

```bash
cd /home/a/carla_pp
./scripts/carla/carla_city_traffic stop
./scripts/carla/carla_autoware stop
export CARLA_PERCEPTION_MODE=ground_truth
./scripts/carla/carla_autoware start-town05
```

## 줄바꿈 오류

`carla_autoware` 다음 줄에 `start-town05`만 입력하면 셸이 이를 별도 프로그램으로
해석하여 `start-town05: command not found`를 출력한다. 아래 전체를 같은 줄에 입력한다.

```bash
./scripts/carla/carla_autoware start-town05
```

`Autoware is already running.`은 기존 컨테이너가 실행 중이라는 뜻이다.
시작 명령은 실행 중인 Autoware를 재시작하지 않으므로 위 종료·시작 순서를 사용한다.

## 배경 교통과 검증 범위

도시 교통 컨트롤러는 별도 시작한다. 파일의 현재 목표는 배경 차량 0대·보행자 250명이다.
기본 Autoware 브리지가 시작 시 만드는 NPC와 기존 객체는 별개이므로 전체 차량 수가 0대라는
뜻은 아니다. 컨트롤러는 자신이 만든 객체만 삭제한다.

```bash
cd /home/a/carla_pp
./scripts/carla/carla_city_traffic start
./scripts/carla/carla_city_traffic status
```

[교통량 설정](carla-city-traffic.md) 및 [회피 수정·VTD 비교](carla-lane-start-fix-20260923.md).
회피·복귀는 정상 배속, 이어지는 우회전은 0.5배속에서 검증했다.
정상 배속의 경로 발행 지연/MRM 문제는 남아 있으며 재시작으로 해결됐다고 검증하지 않았다.
