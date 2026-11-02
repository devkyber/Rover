# Rover 차량 핸드북

이 문서는 처음 합류한 사람이 차량을 이해하고, 기존 구성을 운용하고, 이어서 개발할 수 있도록 구성했다.
현재 소스·설정을 기준으로 **무엇이 연결되어 있고, 왜 이렇게 동작하며, 어느 파일을 읽으면 되는지** 설명한다.
운전할 때 바로 필요한 버튼과 명령은 [루트 README](../README.md)에 모았다.
기존 SSH 별칭과 service·직접 실행·ping·종료 명령은 [자주 쓰는 명령](11_jetson_bringup.md#common-commands)에서 찾는다.

## 어디부터 읽을까

| 알고 싶은 것 | 읽을 문서 | 들어 있는 내용 |
|---|---|---|
| 차량이 어떻게 구성되는가 | [하드웨어·전장](06_하드웨어.md) | 전체 구성, 전원·버스, 장치 역할, 모터·센서·Pico·카메라 선정, 배선과 PCB 진입점 |
| 바퀴와 IMU로 어떻게 자세·경로를 만드는가 | [이론·계산](01_이론_오도메트리와_ESKF.md) | 좌표·단위, 오도메트리, ESKF, 정렬·갱신 순서, 계산 예제, 카메라 시야 계산 |
| 화면과 기록의 수치를 어디까지 해석하는가 | [현재 동작 범위](05_한계와_로드맵.md) | 명령·측정·추정의 차이, 장치별 현재 기능과 표시의 의미 |
| 새 PC나 Jetson에서 어떻게 실행하는가 | [설치·네트워크·복구](11_jetson_bringup.md) | 실행 주체, 환경 설치, service, 펌웨어 빌드, 배포, LAN·Wi-Fi 복구, [자격 파일·복원 자료 위치](11_jetson_bringup.md#credentials-and-recovery) |
| 무엇을 기록하고 어떻게 다시 사용하는가 | [데이터·임무 계약](13_scout_integration.md) | Rover/Scout/Clegg 형식, 시각·단위, 기록 수명, 사건·지도 좌표와 예제 |

처음에는 **하드웨어 → 루트 README → 설치 안내** 순서로 읽으면 장치와 운용 방법이 연결된다.
추정기를 수정할 때는 이론 문서와 해당 소스를 함께 읽고, 기록을 분석할 때는 데이터 계약에서 run·frame·clock을 먼저 구분한다.
문서 번호는 기존 링크를 유지한 것이다. 빠진 번호의 문서를 따로 찾을 필요는 없다.

## 차량을 구성하는 프로그램

| 실행 위치 | 프로그램 | 맡은 일 |
|---|---|---|
| Jetson | [rover_main.py](../rover/rover_main.py) | 구동·임무 명령, 장치 연결, telemetry와 worker의 수명 관리 |
| Jetson | [imu_process.py](../rover/imu_process.py), [estimation_process.py](../rover/estimation_process.py) | BMI088 취득과 자세·경로 추정을 구동 loop에서 분리 |
| Jetson | [camera.py](../rover/camera.py), [video_recording.py](../rover/video_recording.py) | USB H.264 영상의 전송·기록 |
| 노트북 | [station.py](../station/station.py), [station/web](../station/web/) | dashboard 제공, 브라우저 조종·표시, 선택적 Clegg 수신 |
| Pico 2 | [main.py](../firmware/pico/main.py) | PAN·STRING PWM 출력과 INA226 배터리 계측 |
| Scout 두 대 | [Scout firmware](../firmware/scout/README.md) | 각 장치의 모터·서보 제어와 센서 원본 전송 |
| Nano RP2040 Connect | [Clegg firmware](../firmware/clegg_wireless_sdk/README.md) | ADXL375 취득과 Wi-Fi 전송 |

Station의 조종 명령은 차량으로 향하고, 차량의 상태인 **telemetry**는 화면으로 돌아온다.
화면의 버튼, 원격 프로그램, 실제 구동기는 각각 다른 단계다. 전체 연결은 하드웨어 문서,
프로세스·포트는 설치 문서, 메시지·원본 형식은 데이터 계약에서 설명한다.

## 코드를 이어서 읽는 순서

| 바꾸거나 이해할 부분 | 진입점 | 설명 문서 |
|---|---|---|
| 포트·치수·주기·noise·출력 설정 | [config.py](../rover/config.py) | 각 주제 문서에서 설정 이름으로 연결 |
| 주행 명령에서 모터 출력까지 | [station_link.py](../rover/station_link.py) → [rover_main.py](../rover/rover_main.py) → [motors.py](../rover/motors.py), [dxl_control.py](../rover/dxl_control.py) | 하드웨어·데이터 계약 |
| 센서에서 자세·위치까지 | [imu_reader.py](../rover/imu_reader.py) → [estimation_process.py](../rover/estimation_process.py) → [odometry.py](../rover/odometry.py) → [eskf.py](../rover/eskf.py) | 이론·계산 |
| 실린더·STRING·PAN | [hammer_lift.py](../rover/hammer_lift.py), [hammer.py](../rover/hammer.py), [pico_link.py](../rover/pico_link.py) | 하드웨어 |
| 세션·IMU·영상 기록 | [telemetry_log.py](../rover/telemetry_log.py), [logger.py](../rover/logger.py), [video_recording.py](../rover/video_recording.py) | 데이터 계약 |
| Scout·Clegg·지도 | [jumper_service.py](../rover/jumper_service.py), [clegg_service.py](../rover/clegg_service.py), [mission_map.py](../rover/mission_map.py) | 데이터 계약 |
| 화면 조작·표시 | [main.js](../station/web/main.js), [index.html](../station/web/index.html) | 루트 README·데이터 계약 |

하드웨어 없이 회전각 계산을 따라가는 예제는 [learn_turn_angle.py](../rover/learn_turn_angle.py)다.
별도 교육용 필터를 만들지 않고 실제 ESKF predictor를 호출한다. 실행과 수식 해설은 이론 문서에 있다.

## 설정과 자료의 기준

숫자를 바꾸는 곳은 `rover/config.py`, 실제 식과 순서는 `eskf.py`·`odometry.py`다.
문서의 표와 예제는 해당 구현을 설명한다. Pico·browser 생성 설정의 원본과 생성 명령은 [운영 도구](../tools/README.md)에 있다.
회로·BOM·pinout은 형제 PCB 프로젝트가 관리하며 [하드웨어 문서](06_하드웨어.md#rover-pdb)를 통해 들어간다.

| 자료 | 용도 |
|---|---|
| [logs](../logs/README.md) | 선택한 실차 주행·점프·영상 원본과 당시 label·metadata |
| [data](../data/README.md) | 원본 분석 방법, 필요한 결과, 배선 참고 |
| [시각 보고서](../data/visual_report_2026-10-28/report.html) | 보존된 주행·영상·파일을 한눈에 보기 |
| [2026-10-28 인계 기록](../data/final_vehicle_2026-10-28/README.md) | 당시 source hash·설치·백업 식별 |

7월 전장 아키텍처와 카메라 선정 메모, archive의 설명은 현재 문서를 복원하는 참고로 사용했다.
현재와 다른 옛 부품 구성·제어 방식·설계 예산은 현재 실물 사양으로 옮기지 않는다.
과거 원본과 설치 receipt는 당시 사실을 그대로 남기며, 현재 동작 설명과 구분한다.

문서를 수정한 뒤 저장소 루트에서 `python tools/check_docs.py`를 실행한다.
