# 운영·복구 도구

일상 운전은 [README](../README.md), 설치·환경은 [Jetson 안내](../docs/11_jetson_bringup.md),
장치 설정은 [하드웨어](../docs/06_하드웨어.md)를 따른다.

| 용도 | 파일 |
|---|---|
| Jetson 배포·백업·설치 hash·기존 모드 재시작 | `deploy_jetson.py` |
| Clegg 수신·파형 창 시작 | `start_clegg_station.ps1`, `start_clegg_plot.ps1` |
| Clegg 무선망·파형·읽기 전용 위치 context | `clegg_hotspot.ps1`, `plot_clegg.py`, `serve_clegg_context.py` |
| Nano 펌웨어 빌드 | `build_clegg_wireless_sdk.py` — [빌드·복구](../firmware/clegg_wireless_sdk/README.md) |
| Pico·browser 설정 생성 | `build_pico_config.py`, `build_scout_operator_profiles.py`, `build_link_policy.py` |
| 명시적 거리 보정 profile 생성 | `calibrate_wheel_distance.py` |
| 실린더 읽기 점검 | `inspect_lift.py` — 구동 명령 아님 |
| 문서 링크·구조 확인 | `check_docs.py` |

설정 생성기는 `rover/config.py`를 원본으로 사용한다. 생성·빌드는 보드 flash와 별개다.

## Laptop environment

설치·복원은 [노트북 환경](../docs/11_jetson_bringup.md#laptop-environment)을 따른다.
