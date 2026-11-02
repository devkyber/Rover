# 차량 데이터·Scout·임무 좌표 계약

Rover에는 서로 다른 세 종류의 기록이 있다. **Rover의 주행·자세**, **Scout의 점프 센서**, **Clegg의 충격 파형**이다.
이 문서는 각 기록이 어디에서 만들어지고, 어떤 단위·시각·좌표를 갖고, 같은 임무에 어떻게 연결되는지 설명한다.
현재 프로그램을 기준으로 하며 보존된 옛 원본에는 기록 당시 metadata와 형식이 우선한다.

Scout는 장치 이름이고 `Jumper`는 ESP 통신 코드에 남아 있는 이름이다.
운전은 [README](../README.md), 배선은 [하드웨어](06_하드웨어.md), 실행 환경은 [설치·복구](11_jetson_bringup.md),
실제 보드 설치 identity는 [최종 인계](../data/final_vehicle_2026-10-28/README.md)에 있다.

## 데이터가 만들어지는 곳

```text
Rover: wheel/BMI088/Pico → Jetson 제어·추정 → session + IMU CSV
USB camera             → Jetson camera 취득 → TS 영상 + frame journal
Scout 센서             → ESP → Jetson       → record binary + CSV + events
Clegg ADXL375          → Nano → 노트북      → TCP byte + CSV + impacts
운영자 관측            → MISSION MAP → Jetson → map journal
```

화면은 이 데이터를 보여 주는 창이다. 화면을 닫거나 `STOP LOG`를 누르는 것과 모든 원본 기록을 끝내는 것은 다르다.
각 기록은 자신만의 시작·종료와 저장 상태를 가진다.

| 기록 | 저장 컴퓨터·기본 위치 | 시작·종료 |
|---|---|---|
| Rover 전체 상태·명령 | Jetson `logs/sessions/<session>/` | runtime/`NEW RUN`으로 시작, `STOP LOG` 또는 종료로 닫음 |
| 추정 입력·IMU CSV | Jetson `logs/estimation/<run_id>/` | 별도 추정 프로세스가 소유. Rover 기록 요청과 연결 |
| Scout 수신 원본 | Jetson `logs/jumper/scout_1/`·`scout_2/` | 각 연결·재연결 단위. `STOP LOG` 뒤에도 독립 수신 유지 |
| 카메라 영상 | Jetson `logs/video/<capture>/` | runtime 시작 시 자동 시작, `STOP & SAVE`로 종료, `RECORD`로 새 기록 |
| Nano 충격 파형 | 노트북 `logs/clegg/<capture>/` | Clegg 수신기의 TCP 연결 단위. 파형 창 수명과 별개 |
| 공통 지도 | Jetson `logs/mission_maps/<map_id>/` | 명시적인 map 생성·등록·관측 저장 |

`--log`로 Rover 기록 위치를 바꾸면 그 옆에 session·estimation 기록이 생긴다.
Scout·영상은 별도 기본 저장 경로를 사용한다. 저장 위치는 화면의 recording 상태와 metadata에서 읽을 수 있다.
노트북에 페이지가 보인다고 Jetson 원본이 노트북에 자동 복사되는 것은 아니다.

## Rover 기록을 읽는 순서

### 전체 session과 IMU CSV

현재 전체 session의 대표 구조는 다음과 같다. `0001` 이후 파일은 크기에 따라 이어지는 조각이다.

```text
logs/sessions/<session>/
  metadata.json
  rover/
    telemetry_0001.jsonl   화면에 보내는 전체 Rover 상태
    system_0001.csv        자주 쓰는 숫자만 뽑은 표
    sensors_0001.jsonl     제어 루프의 입력·출력·추정 상태
    commands.jsonl        받은 명령·요청 기록
    events.jsonl          상태 전환·오류 등 사건
  jumper/
    telemetry_0001.jsonl   같은 순간의 Scout 집계 상태
    commands.jsonl
    events.jsonl
    scout_1/              슬롯별 같은 sequence의 기록
    scout_2/
  summary.json            종료 시 기록 수·유실·오류·저장 상태
```

JSONL은 **한 줄이 JSON 객체 하나**인 형식이다. 첫 줄만 전체 문서처럼 파싱하거나 파일을 하나의 JSON 배열로 읽지 않는다.
`system_*.csv`는 편하게 그래프를 그리기 위한 부분 추출이다. 전체 필드와 `null`은 JSONL에 남는다.
Rover/Scout의 같은 snapshot은 `run_id`와 `seq`로 연결한다. 파일의 줄 번호를 같은 시각으로 가정하지 않는다.

별도 추정 프로세스는 `logs/estimation/<run_id>/` 아래에 IMU CSV와 자신의 `sessions/`를 만든다.
추정 session에는 전달된 원래 IMU batch, 그 앞에 대응시킨 wheel 표본, 추정 상태가 남는다.
제어 session의 최근 추정값은 이 원시 IMU 기록을 대신하지 않는다.

| 파일 | 내용 |
|---|---|
| `*.csv` | 처리 구간별 wheel·IMU 입력 |
| `*.csv.meta.json` | sensor 설정·축·run/frame·IMU generation·source/config 식별 |
| `*.csv.capture.json` | 비동기 CSV writer의 수락/기록/유실/실패·종료 결과 |
| session `metadata.json` | 실행 인수·기록 역할·source·설정·열 의미 |
| session `summary.json` | 최종 저장 상태와 누락/오류 집계 |

IMU가 복구되어 새 취득 구간이 열리면 같은 run 안에서도 CSV segment와 generation이 달라질 수 있다.
기록 파일은 새 이름으로 만들며 기존 원본을 덮어쓰지 않는다. `summary.json`이 없으면 종료 완료를 나타내는 파일이 없는 부분 기록이다.
남아 있는 행을 사용할 수는 있지만 정상 종료했다고 표시하지 않는다.

### 열·단위 사전

| 필드 | 의미·단위 |
|---|---|
| IMU CSV `t` | run 시작 이후 host 경과시간, s |
| `imu_t` | IMU의 nominal device clock. host와 원점·속도가 같다는 뜻은 아님 |
| `w1_pos` … | config의 모터 순서에 따른 축 위치, rad |
| `w1_vel`, `w1_cur` … | 축 각속도 rad/s, 전류 A |
| `ax`, `ay`, `az` | `imu_frame`에 표시한 좌표계의 가속도 입력, m/s² |
| `gx`, `gy`, `gz` | 각속도 입력, rad/s |
| `imu_dt`, `gyro_samples` | 적분 구간과 구간 내 gyro 표본. 단순 행 간격으로 대체하지 않음 |
| `imu_host_t`, `accel_host_t` | host 취득 시각. device 시각과 분리 |
| `accel_raw`, `gyro_raw` | 센서 원시 count 보존 |
| telemetry `pos`, `vel`, `rpy` | 위치 m, 속도 m/s, roll/pitch/yaw deg |
| `pose_valid`, `pose_quality` | 현재 pose를 유효 위치로 사용할 수 있는지와 품질 |
| `pose_estimate_available`, `pose_estimate` | 보관·참고 추정의 존재. 현재 유효 pose와 구분 |
| `pose_frame_id`, `imu.generation` | 좌표 원점과 센서 취득 구간의 식별 |
| `pico.battery.cell_v` | pack 전압에서 얻은 평균 셀 전압. 개별 셀 측정값은 아님 |

wheel 축의 rad를 차체 m로 바꾸는 반지름·부호·거리 보정과 추정 수식은
[설계와 계산](01_이론_오도메트리와_ESKF.md), 필드 원본은 [logger.py](../rover/logger.py)·[telemetry_log.py](../rover/telemetry_log.py)다.

### 시각을 연결하는 예

`captured_monotonic_s`는 같은 host에서 경과시간을 계산하는 값이고 `captured_host_utc`는 당시 시스템 시계다.
예를 들어 같은 Jetson에서 두 record의 monotonic이 100.20 s, 100.25 s이면 저장 요청 사이 간격은 0.05 s다.
ESP의 `time_us=100200000`과 숫자가 비슷하다고 같은 사건 시각인 것은 아니다. 장치마다 부팅 원점과 clock이 다르다.

자료를 읽을 때는 `metadata → summary/capture 상태 → 원시 행과 seq → 값·사건` 순으로 연결한다.
1970년 UTC가 남은 과거 파일은 당시 잘못된 시스템 시계를 기록한 원본이다. 날짜를 추측해 원본에 덮어쓰지 않고
후속 식별·운영자 라벨을 별도로 연결한다. 알려지지 않은 수치의 `null`이나 CSV 빈 칸은 0이 아니다.

## 영상 기록과 telemetry의 관계

카메라가 출력한 H.264를 Jetson이 받아 browser 미리보기와 로컬 녹화에 나눈다.
[video_recording.py](../rover/video_recording.py)는 재인코딩 없이 MPEG-TS에 담으며 audio나 dashboard overlay를 넣지 않는다.
현재 저장 폴더에는 `metadata.json`, `video_0001.ts` 등의 조각, `frames.jsonl`, 종료 시 `summary.json`이 있다.

새 조각은 독립 해독이 가능한 IDR frame부터 시작한다. 설정된 segment 시간에 도달하면 다음 IDR에서 나눈다.
기록 queue 유실·명시적 불연속·camera generation 변경은 journal과 누락 상태로 남긴다.
미리보기 전송이 늦어 재동기화되는 것과 Jetson 원본 저장의 유실은 서로 다른 상태다.

`frames.jsonl`에는 원래 PTS/DTS, host 수신 시각, 출력 시각, 조각 경계·건너뛴 frame 등이 남는다.
PTS/DTS가 없거나 진행하지 않거나 크게 튀면 출력 파일의 시각에는 nominal cadence가 사용될 수 있다.
따라서 조각을 이어 재생한 영상의 길이는 실제 경과시간과 같지 않을 수 있다. 사건 시각은 journal과 함께 읽는다.

| 화면 조작/상태 | 기록 의미 |
|---|---|
| `NEW RUN` | Rover run을 바꿈. 진행 중 영상은 계속됨 |
| `STOP LOG` | Rover session·IMU 기록을 닫음. 영상·Scout 수신과 별개 |
| `STOP & SAVE` → `SAVED` | 해당 영상 writer가 종료·저장 정리를 마침 |
| `GAPS`, `REC ERROR`, `VIDEO SAVE ERROR` | 누락 또는 저장 오류가 있음 |
| browser 단절 | Jetson 자동 녹화가 같은 이유로 종료되는 것은 아님 |

영상 metadata의 시작 run context만 보고 긴 영상 전체를 한 run으로 단정하지 않는다.
관련 run의 시각·frame journal·촬영 내용을 함께 연결한다.

## 장치와 통신 경로

노트북 dashboard → Jetson [scouts.py](../rover/scouts.py) → 같은 LAN의 두 ESP TCP 경로다.
`scout_1`·`scout_2`는 IP/MAC/실물과 대조하는 슬롯이며 장치 고유번호가 아니다.
두 슬롯에서 요청의 `scout_id`는 필수다. 전역 `E-STOP`은 둘 모두에 전달한다.
포트·timeout·기본 주소는 [config.py](../rover/config.py), 보드 source/build는 [firmware](../firmware/scout/README.md)가 원본이다.

각 Scout는 control과 data 두 TCP 연결을 사용한다. control hello가 알려 준 session으로 data 연결을 묶는다.
control은 JSON+LF 요청을 받고 JSF1 hello/ACK/status를 보낸다. data는 센서/Flash frame을 보낸다.
USB는 현재 Wi-Fi 앱의 상태·설정용이며 일반 센서 수신은 TCP 경로다.

현재 v12.4는 센서 data 채널의 준비와 수동 출력을 분리한다.
`status.healthy`는 센서/data 품질이며 수동 출력 승인값으로 쓰지 않는다.
제어 heartbeat·실제 actuator 오류·network maintenance 정지는 별도의 제어 조건이다.

## 코드 경계와 요청 처리

| 책임 | 구현 |
|---|---|
| 버튼·설정·상태 | [jumper.js](../station/web/jumper.js) |
| 슬롯·WebSocket 요청/결과 | [scouts.py](../rover/scouts.py), [station_link.py](../rover/station_link.py) |
| worker·재연결·우선 정지·context | [jumper_service.py](../rover/jumper_service.py) |
| 명령 취소·선택 설정 전달 | [manager.py](../rover/vendor/jumper/manager.py) |
| TCP·session/ACK·CRC·센서 원본 | [scout_client.py](../rover/vendor/jumper/scout_client.py) |

버튼을 한 번 누르면 `browser 요청 → host queue → ESP 명령 → ACK/timeout → 화면·event` 순으로 처리한다.
`queued`는 host가 접수했다는 뜻이다. ESP ACK와 실제 기구 운동은 그 뒤의 서로 다른 사실이다.
ACK timeout은 결과를 모르는 상태이며 같은 운동 명령을 자동 재시도하지 않는다.

| 화면 | 명령 | 출력 의미 |
|---|---|---|
| `LOCK` | `latch` | J5의 firmware 고정 위치 선택 |
| `LAUNCH` | `launch` | J5의 launch 위치 선택. 현재 hello의 `launch_supported=true` 필요 |
| `PULL` | `prepare` | J5의 pull 위치 선택 |
| `COMPRESS` | `compress` | DC 압축 방향을 유지 |
| `DECOMPRESS` | `decompress` | DC 역방향을 유지 |
| `STOP ROTATION` | `compression_stop` | 대기 명령 취소 우선 경로, DC만 끔 |
| `E-STOP` | 전체 stop/estop 경로 | DC와 서보 PWM 모두 끔 |

호스트 API는 `scout-process-v12`다. 별도 CLAIM·RESUME·단계 승인·WIND/count 절차는 없다.
configure·구동은 연결/복구 때 자동 실행하지 않는다. 재접속은 새 connection ID로 상태·센서만 복구하고 이전 명령을 재생하지 않는다.

브라우저 focus·페이지 종료는 Scout 출력 정지 조건이 아니다. 서버가 heartbeat를 유지한다.
명시적 `DISCONNECT`, 실제 ESP 단절, 서버 종료, Rover 루프 만료, 전역 `E-STOP`은 정지 경로다.
따라서 `COMPRESS`를 누른 뒤 버튼이나 페이지에서 손을 떼는 것은 `STOP ROTATION`과 다르다.

## 지원하는 장치 계약

J5 LOCK/LAUNCH/PULL의 pulse는 firmware가 소유하며 DC 상태와 독립이다.
명령 각도는 실측 축 각도가 아니다. `COMPRESS`/`DECOMPRESS`에는 counts·시간·stall 자동 정지가 없다.
`STOPPED`, `E-STOP`, 현재 `DRIVER ALERT`, 과거 `STATUS STALE`을 구분하며 정상 버튼의 새 입력으로 동작한다.
`compression_result_valid`일 때의 결과는 encoder 변화량이며 mm·힘·압축 완료를 뜻하지 않는다.

호스트의 mechanism 설정은 모델·position 모드·duty·방향·encoder 부호와 고정된 정수 0 필드를 사용한다.
물리 pulse·목표 counts를 호스트 설정으로 전달하지 않는다. 정확한 필드는 현재 firmware와 생성기가 소유한다.

### JSF1 frame과 sensor record

TCP는 byte stream이므로 `recv()` 한 번과 packet 하나는 일치하지 않는다.
[scout_client.py](../rover/vendor/jumper/scout_client.py)는 buffer에서 frame을 조립하고 CRC를 검사한다.

| 구조 | 내용 |
|---|---|
| JSF1 header | little-endian 12 byte: magic, payload 길이, kind, version, frame sequence |
| JSF1 payload·CRC | payload 뒤 4-byte CRC32. hello/status·센서·Flash data를 frame 종류로 구분 |
| sensor record | **CRC를 포함한 40 byte**: `time_us`, sequence, sensor ID, flags, payload 길이, 20-byte payload, CRC32 |
| sensor ID | 1=`ICM-45605`, 2=`ADXL375`, 3=`MMC5983MA` |

socket frame sequence, sensor record sequence, 명령 sequence, cycle ID는 각각의 순서다.
Flash prepare/start/export는 별도 수동 경로이며 발사나 재접속에 맞춰 자동 회수하지 않는다.
LAN session 확인은 인터넷 계정 인증이나 TLS 암호화를 뜻하지 않는다.

## 시각·기록·품질 해석

각 슬롯의 수신 폴더에는 `metadata.json`, `events.jsonl`과 연결 시도별
`scout_<번호>_<식별>.records.bin`, `.csv`, `.meta.json`이 생긴다.
상위 metadata와 events는 host 동작·연결·명령·Rover context를 설명하고, 접두사가 같은 세 파일은 그 연결의 센서 기록이다.

| 파일/필드 | 해석 |
|---|---|
| `.records.bin` | 수신 sensor record byte. 전체 TCP packet capture는 아님 |
| `.csv` | 해독 가능한 record의 열. 원본 binary를 대신하지 않음 |
| `.meta.json` | record 수·sequence gap·CRC·시각 오류·종료 상태 |
| `time_us` | ESP가 붙인 시각. 다른 ESP·Jetson의 clock과 직접 비교하지 않음 |
| `sensor_timestamp_raw` | ICM 내부 timestamp 보존 |
| `estimated_timestamp` | FIFO 표본 시각이 추정된 값인지 표시 |
| `fifo_full`, `saturated`, `sensor_error` | FIFO·범위·센서 오류 표시 |
| `timing_valid`, `sequence_order_valid` | 해독된 표본의 시각/순서 판정 |
| `units_verified` | 알려진 통신 계약의 환산 사용 여부. 실물 calibration 완료를 뜻하지 않음 |

CRC 불량 byte가 raw에 남을 수 있으며 CSV는 그 모든 byte를 표현하지 않는다.
유실·역행을 정렬이나 보간으로 숨기지 않는다. `INIT OK`는 초기 설정 결과이고 새 표본 수신은 센서별 sequence/age로 읽는다.
ICM만 갱신이 멎을 수도 있으므로 다른 센서의 증가는 그 ICM의 최신성을 대신하지 않는다.

현재 decoder의 nominal 환산은 ICM accel `raw/2048` g, gyro `raw/16.4` deg/s, ADXL `raw×0.049` g다.
MAG는 signed raw를 남기고 `mag_qualified=false`로 표시한다. 이 환산은 축 장착·bias·온도·자력 보정을 포함하지 않는다.
배터리 ADC 환산값과 `current_adc_mv`도 구분한다. 후자는 이름 그대로 mV이며 전류 A가 아니다.

Rover context에는 pose 유효성·age·run/frame/IMU generation을 남긴다.
미측정 offset·clock sync·불확도는 `null`이다. `stop_sent`와 파일 finalization은 물리 정지 ACK가 아니다.

## 오프라인 자료와 확장 경계

현재 보존 자료의 입구는 [logs](../logs/README.md)와 [data](../data/README.md)다.
실제 두 Scout 점프는 [원본 범위·품질](../data/vehicle_runs/scout_two_jumps_2026-10-24/summary.json)과
[plot.py](../data/vehicle_runs/scout_two_jumps_2026-10-24/plot.py)에 nominal 파형 해독 방법이 남아 있다.
재생성할 때는 새 출력 폴더를 사용하며 원본 ZIP·기존 결과를 덮어쓰지 않는다.

Station은 발사 후 Matplotlib worker를 자동 시작하지 않는다. `Plot viewer`는 별도로 저장한 결과를 수동으로 읽는다.
보존된 점프 파형은 범용 실측 궤적이나 검증된 landing JSON이 아니다.

관성 궤적의 의미는 원본 hash, 단일 boot/순서 구간, 축·scale·bias, 센서 장착 위치, clock 대응,
world frame, 초기 pose/속도, 이륙·접촉 label에 달려 있다. 이 정보가 없는 기존 자료는 없는 상태로 남긴다.
자유낙하의 낮은 specific force를 중력 tilt로 바꾸거나 조용한 IMU를 무조건 정지로 해석하지 않는다.
센서 추정 사건은 `sensor_estimate`이며 독립 영상·운영자·측량 사건 label과 구분한다.
포화·누락·역행 구간의 충격 원본은 보존하되 근거 없는 위치·불확도를 만들어 넣지 않는다.

## 사건·위치·품질

| 사건/대상 | 보존할 의미 |
|---|---|
| 발사 요청 | 운영자가 발사를 요청한 시각 |
| ACK | ESP가 명령 결과를 응답한 시각 |
| 실제 이륙 | 기구가 지면에서 떠난 사건. 요청/ACK와 별도 |
| Scout 첫 지면 접촉 | 처음 닿은 시각·위치와 해당 충격 |
| Scout 최종 정지 | 튕김·구름 이후 회수 위치 |
| Rover local pose | run/frame/IMU generation과 유효성·age를 갖는 추정 |
| Clegg 실제 타격 | 실제 접촉점·기구 조건·순서에 연결한 사건 |

예를 들어 한 Scout가 `(2.0, 1.0)` m에 처음 닿고 `(2.4, 1.1)` m에서 멈췄다면 두 관측을 같은 `shot_id`로 묶되
`scout_first_contact`와 `scout_rest`로 나눈다. 하나의 “착지 좌표”로 합치면 충격 지점과 회수 지점이 섞인다.
이 숫자는 형식 설명용 예이며 보존 점프의 측량값이 아니다.

원본 파일·장치/session·source/config hash·사건 clock·불확도를 함께 연결한다.
local yaw=0은 북쪽이 아니며 새 원점과 다른 frame은 자동으로 이어지지 않는다.
Rover 위치를 Scout 착지점으로 복사하지 않는다. Rover 기준점과 Clegg 해머 접촉점도 서로 다른 위치다.
Scout 착지 충격과 Clegg 타격은 기구 조건이 다르므로 같은 CIV/CBR로 합치지 않는다.

## 공통 지도 기록과 좌표 등록

[mission_map.py](../rover/mission_map.py)는 운영자가 정의한 평면 XY 지도에 관측을 연결한다.
단위는 m이며 자동 SLAM·위경도 지도가 아니다. dashboard의 `MISSION MAP`이 이 기록의 입력 화면이다.

### local frame을 map frame에 맞추기

Rover의 `pose_frame_id`는 추정 원점을 식별한다. 지도에서 연결할 odom frame은 이 값과 같아야 한다.
새 원점이면 새 frame이다. 같은 물리 기준점의 `local_xy`/`map_xy` 쌍 두 개 이상으로 회전·평행 이동을 구한다.
scale이나 반사는 fitting하지 않는다. 기준점 간 최소 거리와 입력 한도는 config의 `MAPPING_*`가 소유한다.

형식 설명용 예:

| 기준점 | local XY [m] | map XY [m] |
|---|---|---|
| A | `(0, 0)` | `(10, 5)` |
| B | `(1, 0)` | `(10, 6)` |

이 등록은 local +X가 map +Y를 향하는 회전과 `(10, 5)` m 평행 이동이다.
같은 frame의 local `(0, 1)`은 map `(9, 5)`로 옮겨진다. 바퀴 거리 scale을 새로 보정하는 연산은 아니다.
수식과 좌표계는 [설계와 계산](01_이론_오도메트리와_ESKF.md)을 함께 읽는다.

`fit_rmse_m`·`fit_max_residual_m`은 입력 기준점이 변환에 얼마나 맞는지를 나타낸다.
등록에 사용하지 않은 위치의 실제 정확도나 통계 신뢰구간이 아니다.
translation/heading bound는 입력한 범위이며 미상 항목이 있으면 합성 범위도 `null`이다.
Rover 마커는 유효·최신 pose와 해당 frame 등록이 있을 때 표시한다. 보관된 추정값만으로 유효 위치를 만들지 않는다.

### 관측 한 건의 내용

관측 종류는 `scout_first_contact`, `scout_rest`, `clegg_impact`, `landmark`다.

| 항목 | 뜻 |
|---|---|
| `kind`, `label` | 어떤 사건인지와 사람이 읽는 이름 |
| `shot_id` | 관련 접촉·정지·타격을 묶는 식별. landmark 외에는 필수 |
| `event_time` | 사건 clock·값·시각 불확도. 저장한 시각과 별개 |
| `evidence` | 영상·관측·측량 등 사건의 근거 |
| `location` | map 측량 좌표, 지정 local frame 좌표, 또는 unknown 사유 |
| `source_ref` | 원본·sidecar·분석 구간 참조 |
| `measurement` | quantity·value·unit·calibration ID·근거를 갖는 선택 측정값 |
| `supersedes` | 앞의 같은 kind/shot 관측을 정정할 때 연결하는 event ID |

위치가 없는 경우의 **location 조각 예시**는 아래와 같다. 전체 저장 요청이나 실제 관측 기록은 아니다.

```json
{"kind":"unknown","reason":"첫 접촉 장면은 있지만 접촉점 좌표는 측정하지 않음"}
```

unknown은 `(0, 0)`이 아니다. local 좌표가 있어도 frame이 미등록이면 `unregistered_frame`으로 남고,
정확히 같은 frame을 등록한 뒤에만 map으로 투영된다. `shot_id`가 같다고 서로 다른 사건의 시각/위치가 같아지지는 않는다.
측정값을 기록해도 CIV/CBR·지반 강도로 자동 환산하지 않는다.

### 저장·정정·export

`logs/mission_maps/<map_id>/journal.jsonl`은 추가 전용 원본이다.
각 요청은 request ID·revision·이전 hash·저장 시각·source/config hash와 연결되고 fsync 뒤 성공 응답한다.
같은 요청의 재수신은 중복을 만들지 않으며 오래된 revision 요청은 거부한다.

관측 정정은 원문 삭제가 아니라 `supersedes`를 갖는 새 관측이다.
frame 등록 자체는 불변이므로 이미 등록한 frame을 같은 map에서 덮어쓰지 않는다.
손상된 journal은 정상 앞부분만 읽기 전용으로 보여 준다. export는 journal·좌표 기준·provenance를 포함하고
손상 시 `valid_prefix_only`를 표시하므로 원본 폴더도 함께 보존한다.

지도 I/O는 별도 worker와 `/mapping` WebSocket을 쓰며 주행 `/control`과 분리되어 있다.
재연결은 snapshot만 요청하고 저장 명령을 자동 재실행하지 않는다.

## Nano IoT Clegg 충격과 Rover 위치 연결

현재 실제 보드는 [Nano RP2040 Connect](06_하드웨어.md#clegg-가속도-무선-계측)다.
`Nano IoT`는 기존 임무 경로 이름이며 Nano 33 IoT 보드를 뜻하지 않는다.
ADXL375 → Nano 취득/표본 순서 → Wi-Fi TCP → 노트북 원본/별도 파형 창으로 흐른다.
한 센서 TCP에는 수신기 하나만 연결한다. 파형 창은 기존 수신기의 HTTP 결과를 읽는다.

### Clegg 파일과 packet

| 파일 | 내용 |
|---|---|
| `tcp.bin` | 이 연결에서 수신한 TCP byte stream |
| `samples.csv` | boot·seq·시각·raw·nominal g·flags·FIFO 깊이 |
| `events.jsonl` | 연결·명령 의도·수동 marker·impact candidate·위치 context |
| `metadata.json`, `source/` | 장치·endpoint·설정·수신기 source snapshot/identity |
| `summary.json`, `sha256.json` | 연결 종료 집계와 파일 hash |

wire format은 `CG01` magic의 version 1 frame이며 header의 길이·CRC32로 payload를 해독한다.
표본 packet은 `boot_id`와 표본 수, 각 표본의 `seq`, `estimated_us`, `fifo_poll_us`, raw XYZ, flags, FIFO 깊이를 담는다.
정확한 byte layout은 [clegg.py](../rover/clegg.py), 수신·저장은 [clegg_service.py](../rover/clegg_service.py)가 소유한다.

`estimated_us`는 FIFO·설정 표본률을 사용한 추정 시각이고 `fifo_poll_us`는 polling 시각이다.
nominal g는 raw scale 환산이며 영점·축·장착 보정값과 다르다.
sequence 유실/역행, FIFO/ring 손실, CRC 오류, raw limit를 파형 값과 함께 읽는다.
수신이 끝난 뒤 보드 Flash에서 이 파형을 자동 복구하는 경로는 없다.

### 명령 의도·후보·실제 타격

화면 `ARM`은 내부 `drop` 동작이고 Clegg 수신기에 release 요청 의도를 전달한다.
이 이름은 저장된 기록과 연결되므로 임의로 바꾸지 않는다.
명령 요청, 임계값을 넘은 `impact_candidate`, 수동 marker, 실제 지면 접촉, 파형 peak는 각각 다른 항목이다.

| 기록 | 연결하는 근거 |
|---|---|
| 원본 | 장치/boot/sequence·firmware/config/source·축/scale/단위·손실 |
| 타격 | impact ID·지점/반복 순서·관련 shot ID·원본 구간·사건 근거 |
| 시각 | Nano/노트북/Jetson clock 대응·오차·유효 구간 |
| Rover context | 같은 run/frame/IMU generation·pose_valid·age·근거 표본 |
| 실제 타격점 | Rover 기준점에서 해머 접촉점까지의 실측 변환 또는 독립 측량 |
| 해석 조건 | 질량·낙하 높이·장착·대역폭·calibration·방법 |

읽기 전용 context 서비스는 Jetson의 기존 telemetry 이력에서 위치를 가져온다.
후보 시각의 범위를 같은 유효 run/frame/generation의 표본이 둘러쌀 때만 연결한다.
없거나 오래된 위치·다른 boot·손실 구간은 unknown 사유로 남긴다.
context 위치의 시각 기준은 session recorder capture이며 센서 취득 시각 그 자체가 아니다.

Rover 중심 위치와 실제 `hammer_contact_position`은 별도 값이다. 변환이 없으면 복사하지 않는다.
peak는 nominal/보정 여부를 표시하고 CIV/CBR로 자동 변환하지 않는다.
고속 파형을 지도 journal에 복제하지 않고 `source_ref`로 원본과 sidecar를 연결한다.

## 한 번의 활동을 다시 읽을 수 있게 남기기

파일명 하나보다 **원본·설정·사건의 연결**이 중요하다. 예를 들어 Rover를 이동한 뒤 Scout를 발사하고 Clegg를 사용한 활동은
Rover session/추정 기록, 해당 Scout 연결의 binary/CSV/metadata/events, 별도 영상,
노트북 Clegg capture, 지도 journal을 각각 가진다. `run_id`, `shot_id`, boot/frame과 실제 관측 라벨로 연결한다.

원본은 `logs/`, 필요한 분석 결과·방법은 `data/`에 둔다. 계산 예제·가상 입력은 실제 운동 원본으로 분류하지 않는다.
과거 날짜·라벨의 정정은 원본을 바꾸지 않고 후속 설명으로 남긴다.
원본을 다른 컴퓨터로 옮길 때는 해당 폴더의 metadata·summary·journal·source/config identity까지 함께 옮긴다.
분석 결과에는 입력 원본의 hash와 사용 방법을 남기면 어느 설정으로 만든 값인지 다시 찾을 수 있다.
