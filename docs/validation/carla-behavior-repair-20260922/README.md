# 2026-09-22 CARLA 행동 수정 검증 기록

최종 결과는 [종합 보고서](../../carla-behavior-repair-20260922.md)를 따른다.

- `final/CASE_runN.json`: 최종 고정 이미지의 실제 AUTO 주행 샘플, 계획 요인·경로·제어·충돌·정리 결과.
- `final/environment.json`: 실제 실행 이미지, 지도, 시험 도구 SHA256과 8개 시험 종료 상태.
- `final/analysis.json`, `summary.png`: 원시 주행 샘플을 독립적으로 분석한 지표와 그림.
- `final/*_aeb.json.gz`: 준비·주행·종료 중 읽기 전용 AEB 관측 기록. `aeb-analysis.json`은 실제 주행 시간 구간만 분석한다.
- `attempts/`: 초기 GT 이미지 시험과 최종 이미지 첫 suite를 그대로 보존한 기록. 실패·시작 오류·이전 판정 도구 결과를 최종 성공으로 덮어쓰지 않았다.
- `attempts/final-suite-attempt1/`: native 차로 조회의 내부 경계 오판으로 중단된 첫 suite. 해당 좌표의 기하 증명은 `final/query-seam-proof.json`에 있다.
- `scene_restore.json`, `before_scene.json`: 기존 장면과 시험 종료 후 STOP·경로·제한 해제·배경 차량 복원 확인.
- `runtime-*.json`: 모드별 ROS 입력 발행자·시각·프레임·설정 확인. `runtime-sensor.json`과 `attempts/runtime-final-gt-startup.json.gz`는 초기화 완료 전 수행한 실패 기록이며, 준비 완료 후 검사는 별도 파일에 보존했다.
- `compressed-records.json`: 압축 전 내용의 SHA256 및 크기. `SHA256SUMS.json`은 저장된 파일 자체의 SHA256이다.

`.gz`는 원본 바이트를 보존한 gzip이다. 초기 `attempts/pedestrian_stop_attempt1_aeb_partial.json.gz`와 `attempts/vehicle_avoidance_run1_aeb.json.gz`는 관측기의 JSON 직렬화 오류로 불완전한 원본이며, AEB 개입 여부 판단에 사용하지 않았다. 원시 파일은 수정하지 않았다.

그래프와 주행 분석을 다시 생성하려면 저장소 루트에서 다음을 실행한다.

```bash
python3 tools/plot_carla_behavior_repair.py \
  --input-dir docs/validation/carla-behavior-repair-20260922/final \
  --before-dir docs/validation/carla-closed-loop-20260922
python3 tools/analyze_carla_aeb_validation.py \
  docs/validation/carla-behavior-repair-20260922/final
```

분석 도구는 시뮬레이터를 제어하지 않는다. 반면 `run_final_suite.py`, `run_observed.py`, `restore_scene.py`는 이 세션의 실제 실행 절차를 보존한 자료로, 경로와 기존 장면을 가정하므로 실행 전 내용을 확인해야 한다. 주행 도구는 STOP 상태에서 장면을 준비한 뒤 Autoware AUTO를 사용하며 기존 bridge가 tick을 소유한다.
