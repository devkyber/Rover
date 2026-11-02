# postechME Rover

2026 대한민국 우주로버 챌린지용 차량의 제어·계측·기록 프로젝트다. Jetson에서 바퀴와 임무 장치를 제어하고,
노트북의 **Rover Station**에서 영상·자세·경로를 보며 조종한다. 두 Scout의 제어·센서 수신과 Clegg 충격 계측도 함께 다룬다.

이 저장소는 계속 개발하는 차량의 기준이다. 아래는 기존 차량을 켜고 사용하는 안내이며,
구성 원리·부품 선택·배선·계산·설치·데이터 형식은 [차량 핸드북](docs/README.md)에 있다.
2026-10-28 [인계 기록](data/final_vehicle_2026-10-28/README.md)과 [release 식별](CONTEST_RELEASE.json)은 당시 파일·설치 이력이다.
현재 문서와 과거 설치 기록의 날짜를 구분한다.

## 연결·시작

필요한 것은 전원이 연결된 차량, 기존 공유기, 노트북이다. 노트북의 USB Ethernet을 공유기 **LAN**에 연결한다.
Jetson과 Scout는 같은 LAN을 사용한다. 차량 내부 연결에는 인터넷이 필요 없다.
새 PC·새 Jetson의 패키지 설치는 [설치 안내](docs/11_jetson_bringup.md)를 먼저 따른다.

| 어디에 접속하는가 | 주소 | 하는 일 |
|---|---|---|
| 공유기 | http://192.168.0.1 | 차량 LAN 관리 |
| Jetson | `192.168.0.200` | 차량 프로그램 실행·원본 저장 |
| 노트북 브라우저 | http://localhost:8766/?rover=192.168.0.200 | Rover Station 열기 |

`localhost`는 **노트북 자신**이다. 화면은 노트북이 제공하고, 화면의 `ROVER` 주소로 Jetson에 연결한다.

1. 노트북 PowerShell에서 Jetson에 접속한다.

   ```powershell
   ssh taeholee@192.168.0.200
   ```

2. Jetson에서 기존 프로그램을 확인한다. 출력에 `rover_main.py`가 있으면 중복 시작하지 않는다.

   ```bash
   pgrep -af '[r]over_main.py'
   ```

   실행 중인 프로그램이 없으면 기존 service를 시작한다.

   ```bash
   sudo systemctl start rover-field.service
   ```

3. 노트북의 **새 PowerShell 창**에서 Station을 실행한다. 이 창은 운용 중 열어 둔다.

   ```powershell
   cd C:\Users\tae06\CODE\Rover
   python station/station.py --rover 192.168.0.200 --port 8766
   ```

4. Chrome/Edge에서 위 dashboard 주소를 연다. `ROVER` 주소를 확인하고 `CONNECT`를 누른다.
   시작 직후 약 5초간 차량을 정지시켜 IMU를 정렬한다. `NEW RUN` 뒤에도 같은 정지 구간이 필요하다.

SSH 창이 끊겨도 service는 계속 실행된다. 화면이 열렸지만 연결되지 않는 경우는
[네트워크와 복구](docs/11_jetson_bringup.md#네트워크와-복구)를 따른다. 소스 업데이트 뒤에는 브라우저를 새로고침한다.

## 주행·화면

기존 단일 dashboard에서 운전한다. 처음에는 `SPEED LIMIT`을 낮게 두고 입력을 준다.

| 조작 | 동작 |
|---|---|
| `DRIVE` 패드 | 누르고 드래그해 주행·회전, 놓으면 정지 |
| `W / S`, `A / D` | 전진·후진, 좌·우 회전; 놓으면 정지 |
| `↑ / ↓`, `SPEED LIMIT` | 명령 속도 제한 조절 |
| `Space`, `E-STOP` | Rover·두 Scout 중단 요청 |
| E-STOP 상태의 `RELEASE` | Rover 입력 잠금 해제; 새 입력으로 재개 |
| `PAN ◀ / ▶`, `← / →` 또는 `Q / E` | 누르는 동안 카메라 회전 |
| `EXPAND`·`RESTORE`·`FULL SCREEN` | 영상 표시 크기 변경; `Esc`로 닫거나 복원 |
| `RESET` | 이 페이지의 그래프·경로·통계 초기화; 차량·로그는 유지 |

Gamepad는 왼쪽 stick 전후, 오른쪽 stick 좌우, D-pad PAN, B 중단을 사용한다.
재접속·E-STOP 해제 뒤에는 stick을 중립으로 돌린 뒤 새로 움직인다.
입력 칸에 글을 쓰는 동안에는 주행 키 입력을 사용하지 않는다.

| 화면에서 보는 것 | 읽는 법 |
|---|---|
| `READY` | 바퀴 정지 표시. 모든 장치의 준비 완료를 뜻하지 않는다 |
| `ATTITUDE` | IMU로 추정한 roll/pitch/yaw. yaw=0은 시작 방향 |
| `PATH · 2D` | 시작 방향 X, 왼쪽 Y인 local 경로. 단위는 m |
| `FROM START` | 시작점과 현재 표시 위치 사이의 거리. 누적 주행거리와 다르다 |
| `FILTER CHECK` | 추정 상태·입력 품질·표본 나이 확인 |
| `TELEMETRY / SYSTEM LOG` | 수신 상태·장치 오류·사건 확인 |
| `LOW BATTERY` | 배터리 교체·충전 안내. 셀 보호나 전원 차단 동작은 없다 |

IMU 취득·복구는 수동 주행과 별도로 동작한다. 자세·경로가 화면에 남아 있어도 표본이 오래되면 현재 위치로 읽지 않는다.
계산 방식은 [오도메트리·ESKF](docs/01_이론_오도메트리와_ESKF.md)에 설명한다.

PAN의 `SET 0`은 누적 명령 시간의 수동 기준이다. 실제 각도 feedback과 자동 끝단 제한이 없으므로 케이블 장력을 보며 조작한다.

## 기록을 시작하고 끝내기

**세션 기록과 영상 기록은 별개다.** `STOP LOG`로 영상을 끝내거나 차량을 멈추지는 않는다.

| 버튼·표시 | 의미 |
|---|---|
| `RUN` 이름 → `NEW RUN` | 기존 세션·IMU 로그를 닫고 새 기록·IMU 정렬 시작 |
| `STOP LOG` | 세션·IMU 기록 종료; runtime과 주행 제어는 계속 동작 |
| 영상 `RECORD` | 영상 기록 시작. 기존 runtime은 시작 시 자동 녹화를 요청한다 |
| 영상 `STOP & SAVE` | 영상 기록을 닫고 저장 |
| 영상 `SAVED` | 영상 종료·저장 결과 표시 |
| `REC GAPS`·`REC ERROR`·`VIDEO SAVE ERROR` | 기록 누락 또는 저장 실패 표시 |

예를 들어 새 주행을 `hallway_01`로 구분하려면 `RUN`에 이름을 쓰고 `NEW RUN`을 누른 뒤 약 5초 정지한다.
영상은 이 버튼과 독립적으로 이어진다. 종료할 때 영상도 함께 끝내려면 `STOP & SAVE`를 별도로 누른다.
Scout raw capture와 노트북 Clegg 수신도 각각의 기록 수명을 가진다.

Rover 원본은 **Jetson 저장소의 `logs/`**에 저장된다. 노트북으로 자동 복사되지 않는다.
파일 종류·저장 위치·metadata와 함께 가져오는 방법은 [데이터 계약](docs/13_scout_integration.md)에 있다.
이미 남아 있는 주행·점프·영상은 [실차 원본 목록](logs/README.md)에서 찾는다.

## Clegg·실린더

실린더를 움직일 때는 하중을 지지한다. `UP`/`DOWN`을 누르면 움직이고 놓으면 torque ON `HOLD`한다.
`TORQUE OFF`·E-STOP·입력 만료·종료는 토크를 해제할 수 있다. 화면의 기준 그림과 encoder는 자동 끝단 제한이 아니다.

| STRING 조작 | 동작 |
|---|---|
| `WIND` | 누르는 동안 동작 |
| `ARM` | 설정된 시간 동안 동작 |
| `RELEASE` | 설정된 시간 동안 동작 |
| `STRING SETTINGS` | PWM·Duration 변경. 다음 입력부터 적용 |

STRING은 정해진 선행 순서를 강제하지 않는다. 여기의 `RELEASE`는 E-STOP 해제 버튼과 다른 임무 동작이다.
시간이 끝났다는 표시는 실제 위치 도달을 측정한 결과가 아니다. 축·출력·내부 이름은 [하드웨어](docs/06_하드웨어.md)에 설명한다.

Clegg 파형은 별도의 Nano RP2040 Connect에서 노트북으로 수신한다.
설치된 Nano 이미지, 2.4 GHz 전용 핫스팟, ignored `clegg_wifi.local.md`, 노트북 `.venv`가 준비된 상태에서 실행한다.
이때는 일반 Station 대신 아래 실행기를 사용한다. 일반 Station이 이미 실행 중이면 차량을 정지시키고
그 실행 창에서 `Ctrl+C`로 종료한 뒤 실행한다.

```powershell
powershell.exe -NoProfile -File tools/start_clegg_station.ps1
```

파형 창을 닫아도 수신은 계속된다. 다시 열려면 아래 명령을 사용한다.

```powershell
powershell.exe -NoProfile -File tools/start_clegg_plot.ps1
```

실행기는 수신기를 백그라운드에 띄우므로 실행기 창을 닫아도 수신기가 종료되지 않는다.
종료 방법과 콘솔에서 직접 실행하는 방법은 [Clegg 수신기](docs/11_jetson_bringup.md#clegg-수신기와-파형-창)를 따른다.

설치·무선 연결은 [설치 안내](docs/11_jetson_bringup.md), 센서·배선은 [하드웨어](docs/06_하드웨어.md),
raw count·nominal g·타격 사건의 의미는 [데이터 계약](docs/13_scout_integration.md)을 따른다.

## Scout·지도

Scout 패널의 IP·MAC을 실물과 대조한 뒤 해당 패널을 조작한다. 두 패널은 서로 다른 장치 슬롯이다.

| 버튼 | 동작·정지 방법 |
|---|---|
| `LOCK`·`LAUNCH`·`PULL` | 서보 위치와 DC 상태를 독립적으로 유지하는 명령 |
| `COMPRESS`·`DECOMPRESS` | 버튼을 놓아도 DC 회전 유지 |
| `STOP ROTATION` | DC 회전 중단 |
| `E-STOP` | DC·서보 출력 모두 중단 |
| `Plot viewer` | 저장 결과를 수동으로 열기; 발사 뒤 자동으로 열리지 않음 |

counts·시간·stall에 따른 자동 회전 정지는 없다. 창 전환·페이지 닫기는 Scout 정지 조건이 아니다.
명시적 `DISCONNECT`, 실제 제어 연결 단절, 서버 종료는 정지 경로다.
`LAUNCH`는 장치의 지원 응답이 있어야 가능하며 ACK는 명령 수락을 나타낸다.

`MISSION MAP`에는 기준점을 등록하고 `First contact`·`Final rest`·Clegg 타격을 각각 저장한다.
위치를 모르면 `Unknown`을 선택한다. local 경로를 지도 좌표에 놓는 방법과 사건 입력 예제는
[Scout·지도 계약](docs/13_scout_integration.md#공통-지도-기록과-좌표-등록)에 있다.

## 단절·종료·기록

통신이 끊기면 입력을 놓고 `E-STOP`을 요청한 뒤 실제 정지를 확인한다. 불통이면 중단 요청도 전달되지 않을 수 있다.
계속 움직이면 하중을 지지하고 구동 전원을 차단한다. 소프트웨어 E-STOP은 전원 스위치가 아니다.
정지 상태에서 자동 재접속을 기다린다. 연결이 돌아오지 않으면 [네트워크와 복구](docs/11_jetson_bringup.md#네트워크와-복구)를 따른다.

정상 종료 순서는 다음과 같다.

1. 주행·Scout 회전을 멈추고 하중을 지지한다.
2. 영상 `STOP & SAVE`를 누르고 `SAVED`를 확인한다.
3. `STOP LOG`로 세션·IMU 기록을 끝낸다.
4. Jetson에서 service를 멈추고 종료한다.

   ```bash
   sudo systemctl stop rover-field.service
   sudo shutdown -h now
   ```

5. 노트북에서 직접 실행한 Station은 실행 창의 `Ctrl+C`로 종료한다.
   실행기가 백그라운드로 띄운 Clegg 수신기는 [별도 종료 순서](docs/11_jetson_bringup.md#clegg-수신기와-파형-창)를 따른다.

원본의 관련 폴더·metadata·당시 source/config 식별을 함께 보존한다.
보존 자료의 해석과 분석 도구는 [결과·방법](data/README.md)에 있다.
