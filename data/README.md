# 결과와 보관 자료

[시각 보고서](visual_report_2026-10-28/report.html)에서 파일·주행·영상을 먼저 확인한다.
원본은 [logs](../logs/README.md), 형식·단위·사건 해석은 [차량 데이터 안내](../docs/13_scout_integration.md)에 있다.

| 필요한 자료 | 보존 위치 |
|---|---|
| 최종 소스·과거 설치 식별·설정 출처 | [인계 기록](final_vehicle_2026-10-28/README.md) |
| BMI088 연결 원본과 최종 배선 그림 | [배선 참고](imu_wiring_reference_2026-10-15/README.md) |
| 구형 차체의 원래 주행 라벨·치수·단위 | [라벨](vehicle_runs/history/original_log_labels_excerpt.md), [출처](vehicle_runs/history/provenance.json) |
| 대회 영상과 연결된 주행 및 관련 짧은 주행의 수치 | [기존 결과 JSON](vehicle_runs/practice_2026-10-23/run_metrics.json) |
| 운영자가 확인한 복도 주행 | [기존 결과 JSON](vehicle_runs/hallway_7ca72d5e/hallway_7ca72d5e_analysis.json) |
| 실제 두 Scout 점프의 구간·수치·품질 | [구간 결과](vehicle_runs/scout_two_jumps_2026-10-24/summary.json), [전체 원본 ZIP](../logs/scout_jumps/2026-10-24_two_jumps.zip) |
| 원본 영상의 경과시간·수신 조각·당시 재생 결과 | [재생 기록](vehicle_runs/practice_video/playback_receipt.json) |

`practice_2026-10-23`은 과거 경로명이다. 사용자가 확인한 대회 식별은
[후속 확인](visual_report_2026-10-28/operator_clarification.json)에 별도로 보존한다.
원본의 날짜·source/configuration을 소급해서 고치지 않았다.

## 실제 기록을 다시 분석할 때

| 방법 | 입력/용도 |
|---|---|
| [analyze_runs.py](vehicle_runs/practice_2026-10-23/analyze_runs.py) | 두 Rover CSV 세션의 수치 |
| [plot_runs.py](vehicle_runs/practice_2026-10-23/plot_runs.py) | 같은 기록의 그래프 |
| [plot_latest_drive.py](vehicle_runs/hallway_7ca72d5e/plot_latest_drive.py) | 복도 기록의 그래프 |
| [plot.py](vehicle_runs/scout_two_jumps_2026-10-24/plot.py) | 보존 ZIP에서 두 점프를 해독·그리기 |
| [make_playback.py](vehicle_runs/practice_video/make_playback.py) | 원본 TS 조각을 이어 재생물 만들기 |

각 방법은 **새 출력 폴더**를 인수로 받는다. 기존 결과를 덮어쓰지 않는다.
그래프는 `numpy`·`matplotlib`, 영상 재생물 생성은 `av`·`imageio-ffmpeg`가 필요하다.
기존 PNG 그래프는 시각 보고서에, 선택한 점프 CSV는 원본 ZIP 안에 있으므로 별도 중복 파일은 두지 않는다.
