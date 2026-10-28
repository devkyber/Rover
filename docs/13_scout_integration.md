# 차량 데이터·Scout·임무 좌표 계약

2026-10-28 KST 보관 기준. Scout는 임무 장치, Jumper는 ESP 통신 코드 이름이다.
주행 dashboard에 두 Scout의 독립 수동 제어·센서 수신·기록을 유지한다.
운전은 [README](../README.md#scout지도), 연결은 [하드웨어](06_하드웨어.md#jumper-esp-연결과-설정),
실제 설치 identity는 [최종 인계](../data/final_vehicle_2026-10-28/README.md)를 따른다.

## 장치와 통신 경로

노트북 dashboard → Jetson [scouts.py](../rover/scouts.py) → 같은 LAN의 두 ESP TCP 경로다.
`scout_1`·`scout_2`는 IP/MAC/실물과 대조하는 슬롯이며 장치 고유번호가 아니다.
두 슬롯에서 요청의 scout_id는 필수다. 전역 E-STOP은 둘 모두에 전달한다.
포트·timeout·기본 주소는 [config.py](../rover/config.py), 보드 source/build는 [firmware](../firmware/scout/README.md)가 원본이다.

현재 v12.4는 센서 data TCP의 준비 여부와 수동 출력을 분리한다. 제어 heartbeat·실제 actuator 오류·network maintenance 정지는 유지한다.
`status.healthy`는 센서/data 품질이며 수동 출력 승인값으로 쓰지 않는다.
control 채널은 JSON+LF 요청과 JSF1 hello/ACK/status를 사용한다.
data 채널은 control hello의 session으로 결합하고 센서/Flash frame을 수신한다. USB는 Wi-Fi 상태·설정용이다.

## 코드 경계와 요청 처리

| 책임 | 구현 |
|---|---|
| 버튼·설정·상태 | [jumper.js](../station/web/jumper.js) |
| 슬롯·WebSocket 요청/결과 | [scouts.py](../rover/scouts.py), [station_link.py](../rover/station_link.py) |
| worker·재연결·우선 정지·context | [jumper_service.py](../rover/jumper_service.py) |
| 명령 취소·선택 설정 전달 | [manager.py](../rover/vendor/jumper/manager.py) |
| TCP·세션/ACK·CRC·센서 원본 | [scout_client.py](../rover/vendor/jumper/scout_client.py) |

호스트는 scout-process-v12 API를 조작한다. 버튼은 latch·launch·prepare·compress·decompress·compression_stop을 한 번 보낸다.
launch는 보드의 launch_supported=true가 필요하다. configure·구동은 연결/복구 때 자동 실행하지 않는다.
별도 CLAIM·RESUME·단계 승인·WIND/count 절차는 없다. 접수 queued와 ESP ACK는 별개다.
ACK timeout은 결과 불명이며 자동 재시도하지 않는다. 정상 ACK는 실제 위치·하중·점프 성공을 측정한 값이 아니다.

브라우저 focus·페이지 종료는 Scout 출력 정지 조건이 아니다. 서버가 heartbeat를 유지한다.
명시적 DISCONNECT·실제 ESP 단절·서버 종료·Rover 루프 만료·전역 E-STOP은 정지 경로다.
재접속은 새 connection ID로 상태/센서만 복구하며 이전 명령을 재생하지 않는다.
compression_stop은 대기 명령을 취소하는 우선 경로이며 DC만 끈다. 전체 stop/estop은 서보 PWM도 끈다.

## 지원하는 장치 계약

J5 LOCK/LAUNCH/PULL의 pulse는 firmware가 소유하며 DC와 독립이다. 명령 각도는 실측 축 각도가 아니다.
COMPRESS/DECOMPRESS는 버튼을 놓아도 회전한다. counts·시간·stall 자동 정지는 없다.
STOPPED·E-STOP·현재 DRIVER ALERT·과거 STATUS STALE을 구분한다. 다음 정상 버튼으로 재개한다.
compression_result_valid일 때의 결과는 encoder 변화량이며 mm·힘·압축 완료의 측정이 아니다.
선택 mechanism 설정은 모델·position 모드·duty·방향·encoder 부호와 정수 0 필드를 사용한다.
물리 pulse·목표 counts는 호스트 설정에 포함하지 않는다. 정확한 필드는 현재 firmware/생성기를 따른다.

JSF1은 little-endian 12-byte header + payload + CRC32, sensor record는 40-byte와 별도 CRC32다.
TCP recv 경계·socket frame seq·sensor seq·명령 seq·cycle ID는 서로 다르다.
Flash prepare/start/export는 별도 수동 경로이며 발사/재접속에 맞춰 자동 회수하지 않는다.
LAN 세션 확인은 인터넷 인증/TLS가 아니다.

## 시각·기록·품질 해석

ESP time_us·센서 내부 timestamp·host monotonic/UTC·두 보드 clock은 서로 다르다.
센서별 sequence·age·flags·CRC·queue drop·timestamp 오류를 구분하고 손실/역행을 재정렬·보간으로 숨기지 않는다.
INIT OK는 초기 설정 결과다. ICM만 멎을 수 있어 센서별 frame 증가를 확인한다.
배터리 ADC 환산값·자력계 raw는 독립 보정 전 물리 정확도를 보장하지 않는다.

각 슬롯의 raw .records.bin·해독 CSV·metadata·events를 함께 저장한다. raw는 전체 TCP packet capture가 아니라 수신 sensor record byte다.
CRC 불량 byte도 raw에 남을 수 있다. CSV는 해독 가능한 행이며 원본을 대신하지 않는다.
Rover context에는 pose 유효성·age·run/frame/IMU generation을 남긴다. 미측정 offset·clock sync·불확도는 null이다.
stop_sent·파일 finalization은 물리 정지 ACK가 아니다. 실제 원본 선택/위치는 [logs](../logs/README.md)에 있다.

## 오프라인 자료와 확장 경계

실제 두 점프의 [원본 범위·품질](../data/vehicle_runs/scout_two_jumps_2026-10-24/summary.json)과
[plot.py](../data/vehicle_runs/scout_two_jumps_2026-10-24/plot.py)는 보존 binary를 해독하는 nominal 파형 방법이다.
자료 위치와 재생성 안내는 [data](../data/README.md)를 따른다.
범용 실측 궤적·위치 추정기가 아니며 기존 generic Scout 분석·자동 감시 도구는 보관 core에서 제거했다.
Station은 발사 후 Matplotlib worker를 자동 시작하지 않는다. Plot viewer의 기존 입력 형식을 사용하려면
독립 사건/좌표 근거와 맞는 결과를 별도로 제공해야 한다. 두 점프 파형을 검증된 landing JSON으로 바꾸지 않는다.

새 관성 분석을 구현할 때는 원본 SHA256·단일 boot·순서 구간·축/scale/bias·lever arm·clock 오차·world frame·초기 pose/속도·사건 근거가 필요하다.
ICM 가속도/gyro, 고중력 ADXL, MAG의 축·시각·장착 보정을 각각 검증한다.
nominal 결과와 실측 calibration을 구분하며 fit residual·gate 통과를 독립 정확도로 표시하지 않는다.

초기 속도·축·장착·시각 근거가 없으면 임의로 채우지 않는다. 자유낙하 가속도를 중력 tilt로 강제 보정하지 않는다.
포화·누락·역행·기간 한계는 궤적 중단 사유다. 원본/충격 파형은 남기되 미검증 위치·불확도는 null이다.
낮은 가속도·충격·조용한 IMU의 사건은 sensor_estimate이며 독립 이륙/첫 접촉/정지 label과 구분한다.
조용한 IMU만으로 일정 병진 운동과 정지를 구별할 수 없다. ACK를 이륙 사건으로 쓰지 않는다.

공통 지도에는 독립 첫 접촉·최종 정지 label과 frame/shot/source를 연결한다. Rover 위치를 Scout 착지점으로 복사하지 않는다.
단절·재부팅으로 시작점을 잃으면 새 상대 frame으로 나누고 이전 좌표를 자동 연결하지 않는다.
두 실제 점프도 측량 trajectory/CIV/CBR 검증이 아니다. 임무 좌표 규칙은 [아래 공통 지도](#공통-지도-기록과-좌표-등록), 미완료 검증은 [JUMPER-01](05_한계와_로드맵.md#jumper-01)을 따른다.

## 사건·위치·품질

| 사건/대상 | 보존할 의미 |
|---|---|
| 발사 요청·ACK·출력·실제 이륙 | 서로 다른 사건. ACK는 실제 이륙 증거가 아님 |
| Scout 첫 지면 접촉 | 접촉 시각/위치·충격 파형·품질 |
| Scout 최종 정지 | 튕김·구름 이후의 회수 위치. 첫 접촉점과 구분 |
| Rover local pose | run/frame/IMU generation·유효성·표본 age를 갖는 추정 |
| Clegg 실제 타격 | 실제 접촉점·기구 조건·순서에 연결한 독립 측정 |

원본 파일·장치/session·source/configuration hash·사건 clock·불확도를 연결한다.
host 수신 시각을 센서 취득 시각으로 대체하거나 두 장치 clock 값을 바로 빼지 않는다.
local yaw=0은 북쪽이 아니며 새 원점/generation과 다른 frame을 자동 연결하지 않는다.
공개 wheel path의 오차가 내부 ESKF covariance로 검증됐다고 표시하지 않는다.
장착축·lever arm·카메라/PAN 변환은 실측한다. 미상 위치·시각·오차는 null과 사유로 남긴다.
raw·해독 가능한 행·유효 표본·보정 물리량·임무 결과를 구분하고 포화·유실·역행을 숨기지 않는다.
Scout 착지 충격과 Clegg 타격은 기구 조건이 다르므로 같은 CIV/CBR로 합치지 않는다.

## 공통 지도 기록과 좌표 등록

[mission_map.py](../rover/mission_map.py)는 명시한 평면 XY 기준에 관측을 연결한다. 단위는 m이며 자동 SLAM/위경도 지도가 아니다.
odom_frame_id는 runtime pose_frame_id와 대응하고 새 원점마다 달라진다.
무효/지연 pose·미등록 frame에서는 현재 Rover 마커를 표시하지 않는다. 화면 보관값을 지도 유효 위치로 승격하지 않는다.

같은 물리 기준점의 local_xy/map_xy 쌍 2개 이상으로 회전·평행 이동을 맞춘다.
서로 떨어진 3개 이상과 독립 검증점을 권장한다. scale·반사를 fitting으로 숨기지 않는다.
기준점 baseline·입력/기록 한도는 [config.py](../rover/config.py)의 MAPPING_*가 소유한다.
fit residual은 입력 허용치와의 비교이며 실제 정확도·검증점 오차가 아니다.
translation/heading bound는 사용자가 제공한 범위다. 미상 범위가 있으면 전체 범위도 null이며 독립 검증 신뢰구간으로 표시하지 않는다.
거리·slip으로 늘어난 odometry drift 범위도 별도다.

관측 종류는 scout_first_contact·scout_rest·clegg_impact·landmark다.
map 측량 좌표, 지정 frame의 local 좌표 또는 이유가 있는 unknown을 기록한다.
미등록 local 관측은 unresolved이며 정확히 같은 frame의 등록 뒤에만 투영한다.
shot_id는 관련 사건을 묶는 값이지 사건 시각/위치를 같게 만드는 값이 아니다.
event_time의 clock·값·불확도를 기록하고 저장 시각과 구분한다.
측정값은 quantity·unit·calibration ID·근거가 필요하며 CIV/CBR를 자동 환산하지 않는다.

logs/mission_maps/의 map별 journal.jsonl은 추가 전용 원본이다. 소스/config hash·request ID·revision·이전 hash·저장 시각을 갖는다.
fsync 뒤 성공 응답하며 재수신은 중복을 만들지 않고 오래된 revision은 거부한다.
정정은 supersedes로 추가한다. 손상 journal은 정상 앞부분만 읽기 전용으로 보여 주며 원본도 함께 보관한다.
export는 journal·좌표 기준·provenance를 포함한다. 손상 시 valid prefix 표시와 원본 폴더를 함께 남긴다.
지도 I/O는 별도 worker와 /mapping WebSocket을 사용하고 주행 /control과 분리한다.
재연결은 snapshot만 요청하며 저장 명령을 자동 재실행하지 않는다.

## Nano IoT Clegg 충격과 Rover 위치 연결

현재 실제 보드는 [Nano RP2040 Connect](06_하드웨어.md#clegg-가속도-무선-계측)다. 여기서 Nano IoT는 기존 임무 경로 이름이다.
ADXL375 → Nano 취득/표본 순서 → Wi-Fi TCP → 노트북 원본/별도 파형 창을 사용한다.
한 센서에는 한 수신기만 연결한다. Jetson 직접 수신은 별도 실물 검증이 필요하다.
임계값 후보 impact_candidate·수동 marker·실제 접촉·peak를 구분한다.
현재 ARM의 내부 release 요청은 명령 의도이며 실제 낙하/접촉이나 저장 완료를 증명하지 않는다.

| 기록 | 필요한 연결 근거 |
|---|---|
| 원본 | 장치/boot/sequence·firmware/config/source·축/scale/단위·포화/FIFO/전송 손실 |
| 타격 | impact_id·지점/반복 순서·관련 shot_id·원본 표본/구간·사건 판정 근거 |
| 시각 | 표본 clock·host 수신 시각·Nano/호스트/Jetson 대응과 오차·유효 구간 |
| Rover context | 같은 run/frame/IMU generation·pose_valid·age·근거 표본 |
| 실제 타격점 | Rover 기준점과 접촉점 사이 실측 변환/기구 상태 또는 독립 측량 |
| 해석 조건 | 질량·낙하 높이·장착·대역폭·보정·원본 hash와 방법 |

현재 context는 읽기 전용 위치 이력과 후보 시각 범위를 같은 유효 run/frame/generation에서 연결한다.
없거나 오래된 위치·다른 boot·손실 구간은 unknown으로 남긴다. 명령 시각을 접촉 시각으로 쓰지 않는다.
Rover 중심 위치와 실제 hammer_contact_position은 별도 값이며 실측 변환이 없으면 복사하지 않는다.
peak는 nominal/보정 여부를 표시하며 CIV/CBR·지반 강도로 자동 변환하지 않는다.
고속 파형을 지도 journal에 복제하지 않고 source_ref로 원본과 sidecar를 참조한다.
수신·disk 작업은 주행을 막지 않으며 누락/중복/overflow와 재부팅 clock 구간을 드러낸다.
물리 검증에는 독립 접촉 시각·측량 기준의 반복 타격과 Rover 진동/slip·pose 변화 확인이 필요하다.
