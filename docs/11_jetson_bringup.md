# 설치·네트워크·복구

이 문서는 **노트북에서 화면을 열고, Jetson에서 차량 프로그램을 실행하는 환경**을 설명한다.
이미 설치된 차량의 평소 시작·운전·종료는 [README](../README.md)를 따른다.
여기서는 새 작업 PC를 준비하거나 Jetson 환경·통신·설치를 복구할 때 필요한 경로와 명령을 다룬다.
기준은 현재 소스와 2026-10-28 설치 기록이며, 과거 설치의 파일 식별은 [인계 기록](../data/final_vehicle_2026-10-28/README.md)에 있다.

## 어느 컴퓨터가 무엇을 하는가

```text
Windows 노트북
  station/station.py ── HTTP ── Chrome/Edge의 기존 dashboard
                                │
                                ├─ WebSocket /control ─┐
                                ├─ WebSocket /video ───┤
                                └─ WebSocket /mapping ─┤
                                                      ▼
공유기 LAN ── 5 GHz Wi-Fi ── Jetson: rover_main.py
                               ├─ U2D2 · Pico · USB camera
                               ├─ 별도 추정 프로세스 → 별도 취득 프로세스 → BMI088
                               ├─ 같은 LAN → Scout 1 · Scout 2
                               └─ Jetson의 logs/에 기록

노트북의 Clegg 수신기 ── 별도 2.4 GHz 핫스팟 ── Nano + ADXL375
  └─ 노트북의 logs/clegg/에 기록
```

Station은 화면 파일을 제공하는 작은 Python 서버다. 일반 Rover 통신은 **브라우저가 Jetson에 직접 연결**한다.
따라서 노트북의 페이지가 열려도 Jetson 연결은 따로 필요하다. Clegg를 켠 Station은 Nano 수신기도 함께 실행한다.
이 경우 Jetson이 꺼져 있어도 Nano 원본 수신은 가능하다.

페이지를 `http://localhost:8766/`에서 여는 이유는 브라우저 H.264 디코더인 WebCodecs 때문이다.
현재 Station은 Chrome/Edge의 localhost 환경을 사용한다. 공유기나 Jetson 주소에서 평문 HTTP로 페이지를 열면
화면 요소가 보여도 영상 디코더를 사용할 수 없는 환경이 있다.

| 경로 | 누가 기다리는가 | 용도·설정 원본 |
|---|---|---|
| 노트북 HTTP `8766` | Station | 운용 명령에서 지정하는 페이지 포트. CLI 자체 기본값은 `8765` |
| Jetson TCP `8000` | StationLink | `/control`, `/video`, `/mapping`; `STATION_PORT` |
| Scout TCP `9000` / `9001` | 각 ESP | 제어 / 센서 data; `JUMPER_CONTROL_PORT`, `JUMPER_DATA_PORT` |
| Nano TCP `9060`, UDP `9061` | Nano / 노트북 발견 수신기 | Clegg 표본 / 장치 발견; `CLEGG_TCP_PORT`, `CLEGG_DISCOVERY_PORT` |
| Jetson HTTP `8011` | 선택 실행하는 context 서비스 | Clegg용 읽기 전용 위치 이력; `CLEGG_CONTEXT_PORT` |
| Jetson TCP `8012` | Scout raw feed | 저장된 Scout record의 읽기 전용 전달; `SCOUT_PLOT_FEED_PORT` |

위 숫자는 현재 기본값을 읽기 쉽게 표시한 것이다. 변경할 때는 [config.py](../rover/config.py)와 해당 CLI 인수가 원본이다.
비밀번호는 문서나 명령 예제에 넣지 않고 ignored `*.local.md`·`private/`·credential store로 관리한다.

<a id="credentials-and-recovery"></a>

## 비밀번호·설치 복원 자료의 위치

아래는 **현재 작업 PC에서 2026-11-02 확인한 보관 위치**다. 경로는 별도 표기가 없으면 Rover 저장소 루트 기준이다.
비밀번호 값은 이 문서에 복제하지 않는다. 로컬 파일 존재 확인과 실제 장치 로그인 성공은 서로 다른 상태다.

| 자료 | 로컬 위치 | 보관 내용·범위 |
|---|---|---|
| Jetson 접속 정보 | `jetson_credentials.local.md` | 주소·계정·비밀번호. 이전 로컬 archive의 같은 파일과 byte 일치 |
| 현장 Wi-Fi 자격 | `rover_wifi_credentials.local.md` | Rover 무선망 접속 정보와 비밀번호 |
| Clegg 핫스팟 자격 | `clegg_wifi.local.md` | 실행기·builder가 읽는 `SSID:`·`Password:` |
| Jetson SSH key | 노트북 `~/.ssh/id_ed25519_jetson`와 `.pub` | 배포기가 기본으로 사용하는 개인키·공개키 |
| 설치된 Nano 복구 이미지 | `private/clegg_clegg_wireless_sdk.uf2` | 자격이 포함될 수 있는 UF2. 보관된 설치 SHA256과 일치 |
| 과거 설정 참고 문서 | `private/references_2026-10-08.zip` | 당시 Jetson·통신·전장 설명 등 29개 항목. 현재 설정은 본 핸드북을 따름 |
| 설치·이미지 식별 | [installation_identity.json](../data/final_vehicle_2026-10-28/installation_identity.json) | 과거 설치 receipt·설정 출처·private 이미지 hash |

세 자격 파일은 모두 내용과 비밀번호 필드가 남아 있다. `*.local.md`와 `private/`는 Git에서 제외되므로
**GitHub에 push하거나 새 PC에서 clone해도 위 비밀 파일·UF2·참고 ZIP은 따라가지 않는다.**
SSH key도 저장소 밖에 있다. 기존 PC의 이 로컬 자료와 공개 소스·설치 기록을 함께 구분해 보관한다.

Jetson 기본 설치·Wi-Fi 설정을 만드는 스크립트와 service는 공개 저장소의 `jetson/`에 남아 있다.
다만 현재 Rover 폴더와 `Rover-archive-20261008-111026`의 파일명 조회에서는
**AX210 `.ko` 파일이나 AX210·커널 복구용으로 식별되는 바이너리 묶음을 찾지 못했다.**
private 참고 ZIP도 `.ko`나 Jetson 복원 이미지가 아니라 문서 묶음이다.
AX210의 기존 모듈 설치 경로·Jetson 빌드 작업 경로·커널 식별은 [AX210 복구 조건](#ax210-복구-조건)에 있으며,
그 원격 파일의 현재 존재 여부는 이번 로컬 문서 확인에서 조회하지 않았다.

## 기존 Jetson 환경

| 항목 | 기록된 환경 | 의미 |
|---|---|---|
| OS | JetPack 6.2.1 / L4T R36.4.4 | 시스템 GI/GStreamer와 커널 드라이버의 기준 |
| 커널 | `5.15.148-tegra` | AX210 모듈이 맞춰진 커널 |
| 작업 폴더 | `/home/taeholee/Rover` | service와 배포기가 사용하는 경로 |
| Python | `/home/taeholee/rover-venv` | `--system-site-packages`로 만든 Jetson 전용 venv |
| serial | `/dev/u2d2`, `/dev/pico` | USB 연결 순서와 분리한 [udev 이름](../jetson/99-rover-serial.rules) |
| camera | USB H.264 → GStreamer | 카메라가 압축한 영상을 받아 표시·저장 |
| BMI088 | `spidev==3.8` | accel `/dev/spidev0.0`, gyro `/dev/spidev0.1` |

`~/Rover`와 `~/rover-venv`는 서로 다른 폴더다. 소스 복사만으로 venv나 OS 드라이버가 설치되지는 않는다.
노트북 venv를 Jetson으로 복사하지 않는다. CPU 아키텍처와 시스템 패키지가 다르다.

## 환경 복원

### 소스와 Python 준비

차량이 정지한 설치 작업에서 Jetson 저장소 루트로 이동해 실행한다.

```bash
cd ~/Rover
bash jetson/setup_jetson.sh
```

[setup_jetson.sh](../jetson/setup_jetson.sh)가 수행하는 일은 다음과 같다.

| 작업 | 설치하는 이유 |
|---|---|
| Python·V4L2·GI/GStreamer 패키지 설치 | Python 실행, USB 카메라 조회, H.264 취득·TS 저장 |
| `brltty`가 있으면 제거 | 사용하지 않는 점자 서비스의 USB serial 점유 방지 |
| `dialout`, `video` 그룹과 udev 규칙 설정 | 일반 사용자로 serial·카메라 접근 |
| Wi-Fi powersave 해제 | 절전 상태로 인한 수신 묶임을 줄이는 기존 운용 설정 |
| `~/rover-venv` 생성 | NumPy·pyserial·spidev·mpremote 설치 |

그룹 변경은 재로그인 또는 재부팅 뒤 반영된다. venv는 JetPack의 `python3-gi`를 읽어야 하므로
`--system-site-packages`를 사용한다. `numpy<2` 제약도 JetPack의 기존 바이너리 패키지와 맞추기 위한 것이다.
카메라 저장에는 GStreamer의 `appsrc`, `appsink`, `h264parse`, `mpegtsmux`가 필요하다.

이 스크립트의 마지막 출력에는 가상 IMU를 사용하는 예제 실행도 포함되어 있다. 그것은 실제 차량 service 시작 명령이 아니다.
그 예제는 모든 외부 연결을 끄는 인수 조합도 아니다.
실제 운용은 [Field 실행 service](#field-실행-service)를 사용한다.

### serial·카메라 경로

장치를 연결한 뒤 이름과 접근 권한을 읽는다.

```bash
ls -l /dev/u2d2 /dev/pico
ls -l /dev/v4l/by-id/
groups
```

`/dev/ttyUSB0`처럼 연결 순서에 따라 바뀌는 이름을 임의로 config에 대입하지 않는다.
고정 별칭이 없으면 [udev 규칙](../jetson/99-rover-serial.rules)의 USB 식별과 실제 장치를 대조한다.
카메라 경로는 `CAMERA_DEVICE`가 소유하며, 녹화와 미리보기는 같은 camera 취득기를 사용한다.
U2D2·Pico·카메라를 두 runtime이 동시에 열지 않도록 기존 프로세스부터 확인한다.

연결된 카메라의 실제 UVC 포맷은 `v4l2-ctl --list-devices`로 node를 찾은 뒤
`v4l2-ctl -d /dev/videoN --list-formats-ext`로 읽는다. 여기의 `videoN`은 조회한 실제 node로 바꾼다.
USB 제품 이름만으로 H.264·해상도·fps 지원을 대신 판단하지 않는다.

### BMI088 헤더 기능

기존 설치는 Jetson-IO에서 SPI1 헤더 기능을 활성화하고 재부팅한 구성이다.
이름이 SPI1이어도 Linux 장치 경로는 현재 설정의 `/dev/spidev0.0`·`/dev/spidev0.1`을 사용한다.
OS 복원 시 실제 헤더 설정을 확인한 뒤 기존 활성화 명령을 적용한다.

```bash
sudo python3 /opt/nvidia/jetson-io/config-by-function.py -o dt spi1
sudo reboot
```

`/dev/spidev*`는 Linux 장치 등록을 뜻한다. 헤더 pinmux와 센서 결선은 [BMI088 연결](06_하드웨어.md#bmi088-연결)을 따른다.
현재 runtime의 별도 추정 프로세스가 BMI088 reader 하나를 소유하므로, 운용 중 별도 SPI reader를 함께 띄우지 않는다.

## Field 실행 service

[rover-field.service](../jetson/rover-field.service)는 SSH 창이 닫혀도 runtime을 유지하는 systemd unit이다.
실행 내용은 `/home/taeholee/rover-venv/bin/python .../rover_main.py --tag field`이고 작업 폴더는 `~/Rover/rover`다.
`Restart=no`이며 기존 운영에서는 부팅 자동 시작을 켜지 않았다. 전원이 켜졌다는 사실과 runtime 실행은 별개다.

service 파일을 복원하는 경우 사용자와 경로가 위 환경과 일치할 때 다음처럼 등록한다.

```bash
sudo cp ~/Rover/jetson/rover-field.service /etc/systemd/system/rover-field.service
sudo systemctl daemon-reload
```

현재 상태와 최근 로그는 다음 명령으로 읽는다.

```bash
pgrep -af '[r]over_main.py'
systemctl status rover-field.service --no-pager
journalctl -u rover-field.service -n 50 --no-pager
```

직접 실행한 runtime이 있다면 service가 inactive여도 장치는 사용 중일 수 있다. 기존 실행을 중복 시작하지 않는다.
시작·종료는 `sudo systemctl start rover-field.service`, `sudo systemctl stop rover-field.service`다.
정상 종료는 SIGINT를 보내 actuator·worker·기록의 정리 경로를 실행한다. 강제 프로세스 종료와 전원 차단은 이 경로와 다르다.

### 실행 인수의 역할

일상 운전은 service 인수를 유지한다. 아래는 기존 실행 모드를 읽거나 유지보수 시 이해할 항목이다.

| 인수 | 의미 |
|---|---|
| `--tag NAME` | 자동 로그 이름에 쓰는 태그 |
| `--log PATH` | 지정한 로그 경로 사용 |
| `--no-log` | Rover session·추정 기록 비활성화. 별도 영상·Scout 원본의 수명과 구분 |
| `--imu bmi088` | 실제 BMI088 입력; 지원 선택지는 현재 CLI를 따름 |
| `--no-camera` | 카메라 취득 비활성화 |
| `--no-motors` | wheel 구동 비활성화 |
| `--pico off` | Pico 연결 비활성화 |
| `--jumper ESP_IP` | Scout 주소 지정. 반복 순서가 두 UI 슬롯 순서 |
| `--odometry-profile PATH` | 명시적인 거리 보정 profile 사용 |

`--hammer-test`와 `--cylinder-test`는 실제 출력이 가능한 별도 작업 모드다.
환경 복원용 기본 시작 명령으로 사용하지 않는다. 세부 동작은 [하드웨어](06_하드웨어.md#clegg-설정과-시험)에 있다.

## 소프트웨어 배포

노트북의 [deploy_jetson.py](../tools/deploy_jetson.py)는 소스를 묶고 기존 원격 파일과 실행 모드를 보존하며 설치한다.
일반 소스 배포는 이 도구를 사용한다. Pico·Scout·Nano 펌웨어를 flash하는 도구는 아니다.

노트북 저장소 루트에서 먼저 bundle을 만든다. 이 단계는 Jetson에 접속하지 않는다.

```powershell
python tools/deploy_jetson.py
```

출력의 `Bundle:` 경로에는 `release.tar`, `manifest.json`, `bundle.json`이 만들어진다.
실제 배포 작업에서는 출력된 경로를 그대로 사용한다.

```powershell
python tools/deploy_jetson.py --apply --bundle "data/deployments/releases/출력된_release_폴더"
```

기본 SSH 대상은 `taeholee@192.168.0.200`, key는 `~/.ssh/id_ed25519_jetson`이다.
`--host`, `--key`로 SSH 대상을 지정할 수 있지만 원격 작업 경로와 Python 경로는 현재 차량 설치에 맞춰져 있다.
다른 사용자에게 배포할 때 주소 인수만 바꾸면 설치 경로까지 바뀌는 구조는 아니다.

| 배포 단계 | 도구의 처리 |
|---|---|
| bundle 식별 | tar SHA256과 현재 로컬 파일 대조. bundle 이후 파일이 바뀌면 새 bundle 요구 |
| 원격 비교 | 설치 파일 hash와 알려진 baseline/이전 manifest 대조 |
| 충돌 처리 | 알 수 없는 원격 변경은 보존하고 `remote_drift.json`을 남겨 중단 |
| 기존 실행 처리 | 실제 인수·Python·환경·실행 상태를 읽고 정지 상태를 요구 |
| 설치 | 바뀌는 파일 백업, 파일 교체, 설치 hash 확인 |
| 재시작 | 원래 실행 중인 runtime만 같은 모드로 재시작. 꺼져 있던 runtime은 새로 켜지 않음 |
| 결과 보관 | 원격 백업과 receipt를 노트북의 해당 attempt 폴더로 회수 |

구동·실린더 HOLD·진행 중 영상 저장은 배포 전 정리한다. 도구는 원격 변경을 무시하는 강제 덮어쓰기 절차를 제공하지 않는다.
별도 주행을 실행하지 않고 설치 파일과 기존 모드의 재시작 결과를 기록한다.

`data/deployments/releases/<release>/attempt_<id>/receipt.json`에서 설치·재시작 결과를 읽는다.
실패하면 같은 폴더의 `failure.json`, `inspection.json`, 단계별 stderr와 원격 receipt를 함께 본다.
백업 회수 실패는 설치 자체의 성공/실패와 별개일 수 있으므로 명령을 반복하기 전에 receipt의 설치 상태를 읽는다.
Jetson 쪽 원본 백업·receipt는 `~/Rover/logs/deployments/`에 남는다.

## 보드 firmware와 생성 설정

설정 숫자의 원본은 [config.py](../rover/config.py)다. 같은 값을 여러 소스에서 각각 손으로 바꾸지 않는다.

| 대상 | 준비 경로 | 설치와의 관계 |
|---|---|---|
| Pico 2 | `python tools/build_pico_config.py` → `firmware/pico/main.py` 설정 블록 | 생성 후 MicroPython 보드 복사는 별도 작업 |
| Scout UI profile | `python tools/build_scout_operator_profiles.py` | browser가 읽는 profile 생성; ESP flash와 별개 |
| browser 링크 정책 | `python tools/build_link_policy.py` | host와 dashboard의 공유 정책 생성 |
| Scout ESP | [v12.4 source·빌드 기록](../firmware/scout/README.md) | ESP 앱과 보드별 readback identity를 따로 보관 |
| Nano RP2040 Connect | [SDK 빌드·복원](../firmware/clegg_wireless_sdk/README.md) | SDK·ARM compiler·자격 파일로 이미지 생성 후 별도 flash |

빌드는 장치 설치가 아니다. Nano 이미지와 Flash 백업에는 Wi-Fi 자격이 포함될 수 있으므로 ignored 로컬 저장소에 둔다.
공개 인계에는 source/config identity와 이미지 hash를 남긴다.

<a id="laptop-environment"></a>

## 노트북 환경

기본 Station은 Python 표준 라이브러리만 사용한다. Clegg 파형과 오프라인 그래프에는 NumPy·Matplotlib·pyserial이 필요하다.
기록된 노트북 환경은 **Tk 포함 Windows CPython 3.13.14**이며 Clegg launcher는 저장소의 `.venv`를 사용한다.
저장소 루트에서 환경을 복원한다.

```powershell
py -3.13 -m venv .venv
.venv/Scripts/python.exe -m pip install -r tools/requirements-laptop.txt
.venv/Scripts/python.exe -c "import tkinter, numpy, matplotlib, serial"
```

`requirements.txt`는 Jetson용, `tools/requirements-laptop.txt`는 노트북용이다.
노트북 목록의 CMake·Ninja는 Nano 빌드에 쓰며 Pico SDK와 ARM compiler까지 설치해 주지는 않는다.
일반 dashboard는 다음과 같이 실행한다.

```powershell
python station/station.py --rover 192.168.0.200 --port 8766
```

`--no-browser`는 자동 창 열기만 끈다. `--bind`의 기본값은 `127.0.0.1`이다.
LAN의 다른 화면에 제공하려면 Station이 실행되는 **노트북의 LAN IP**로 bind한다.
`?view=control`은 조작 화면, `?view=monitor`는 수동 입력 없는 감시 화면이다.
localhost가 아닌 HTTP 화면의 영상 지원은 앞의 WebCodecs 조건과 구분한다.

### Clegg 수신기와 파형 창

Nano의 자격·고정 주소와 맞는 **2.4 GHz WPA2 핫스팟**을 사용하는 기존 실행기는 다음과 같다.

```powershell
powershell.exe -NoProfile -File tools/start_clegg_station.ps1
```

실행기는 저장소 venv·자격 파일·해당 포트의 기존 Station을 식별하고 수신기와 별도 파형 창을 준비한다.
기존 같은 Station을 재사용할 수 있지만 다른 프로그램의 포트를 임의로 빼앗지 않는다.
`-NoPlot`은 파형 창을 생략하고 닫은 파형 창은 `tools/start_clegg_plot.ps1`로 다시 연다.
파형 창을 닫아도 수신·저장은 계속된다. 수신기 로그는 `logs/clegg_station/`, 실제 Nano 원본은 `logs/clegg/`다.

이 실행기는 Station을 **숨겨진 별도 프로세스**로 띄운 뒤 반환한다. 실행기를 호출한 창의 `Ctrl+C`는
이미 시작한 수신기를 종료하지 않는다. 기존 일반 Station을 바꿔야 할 때는 그 실행 창에서 먼저 `Ctrl+C`로 닫는다.
현재 포트의 소유자는 `powershell.exe -NoProfile -File tools/start_clegg_station.ps1 -CheckOwner`로 읽는다.

터미널에서 수신기까지 함께 관리하려면 일치하는 핫스팟이 켜져 있고 기존 수신기가 없는 상태에서 아래처럼 직접 실행한다.
센서 주소는 현재 `CLEGG_WIFI_STATIC_IP` 기본값이며, 이미 다른 주소로 구성했다면 그 값을 사용한다.

```powershell
.venv/Scripts/python.exe station/station.py --rover 192.168.0.200 --port 8766 --clegg-host 192.168.137.2
```

직접 실행한 경우 이 창의 `Ctrl+C`가 수신기 종료·파일 닫기 경로를 호출한다.
launcher가 만든 백그라운드 수신은 Nano 전원을 끈 뒤 마지막 capture의 `summary.json`·`sha256.json` 생성으로 연결 종료를 확인하고,
`http://localhost:8766/clegg/status`의 `connected=false`를 읽는다.
그 뒤 `-CheckOwner` 결과와 `logs/clegg_station/<실행>/launch.json`의 PID를 대조해 작업 관리자에서 해당 Station 프로세스만 종료한다.
수신 중 프로세스를 강제로 끄면 종료 집계가 남지 않을 수 있다.

Rover 위치 이력이 필요한 구성에서는 Jetson 저장소 루트에서 별도 읽기 전용 서비스를 실행한다.

```bash
~/rover-venv/bin/python tools/serve_clegg_context.py --logs logs/sessions --bind 192.168.0.200
```

노트북 실행기에 `-ContextHost 192.168.0.200`을 추가한다. 같은 포트의 서비스를 중복 실행하지 않는다.
Station 직접 실행에는 `--clegg-context-host 192.168.0.200`을 추가한다.
이 서비스는 기존 telemetry 기록을 읽으며 모터·IMU를 새로 열지 않는다.
context의 시간·좌표 의미는 [Clegg 데이터 계약](13_scout_integration.md#nano-iot-clegg-충격과-rover-위치-연결)에 있다.

## 네트워크와 복구

### LAN 구성

| 장치 | 기존 설정 | 역할 |
|---|---|---|
| 공유기 | ipTIME AX3000Q, `192.168.0.1/24` | Rover·Scout·노트북이 만나는 LAN |
| 노트북 | USB Ethernet → **LAN** 포트, DHCP `.100–.199` | 인터넷 없이 dashboard 사용 |
| Jetson | `rover-5G`, `rover-field`, 고정 `.200/24` | 5 GHz 차량 링크 |
| Scout | `rover-2G`, 같은 `192.168.0.x` LAN | 각 ESP와 Jetson 사이 TCP |
| 기존 5 GHz 설정 | channel 36 / 80 MHz, WPA2PSK + AES | 현재 기록된 AP 기준 |

공유기의 WAN은 인터넷용이다. 노트북을 LAN에 연결하면 인터넷이 없어도 차량과 통신한다.
2.4 GHz와 5 GHz가 서로 다른 SSID여도 같은 LAN이어야 Scout에 연결된다.
guest/client isolation은 이 장치 간 통신을 막으므로 사용하지 않는다.
Nano용 노트북 핫스팟은 위 현장 LAN과 별도 경로다.

### 저장된 field profile 복원

[wifi_field.sh](../jetson/wifi_field.sh)는 암호를 숨겨 입력받고 NetworkManager의 `rover-field` profile을 저장한다.
이름과 IP가 기존 환경에 맞는 설치 작업에서 사용한다.

```bash
cd ~/Rover
bash jetson/wifi_field.sh "rover-5G" 192.168.0.200
```

저장만 수행하며 활성 연결을 즉시 바꾸지 않는다. profile은 5 GHz 전용, 고정 `/24` 주소,
gateway `.1`, 자동 연결 우선순위 100, Wi-Fi 절전 해제로 설정된다.
집 Wi-Fi가 이미 연결되어 있으면 우선순위만으로 즉시 field로 갈아타지 않는다.
로컬 모니터/키보드 또는 유지되는 관리 경로에서 명시적으로 전환한다.

```bash
nmcli connection show --active
sudo nmcli connection up rover-field
```

전환 전 Wi-Fi를 사용하던 SSH는 끊길 수 있다. 노트북을 현장 LAN에 연결한 뒤 `.200`으로 다시 접속한다.
스크립트의 오래된 USB SSH 안내는 현재의 보장된 복구 경로가 아니다.

### 채널 폭과 현장 복구

단절 때는 입력을 놓고 실제 정지 상태부터 확인한다. 연결된 구간을 순서대로 좁힌다.

| 관찰 | 먼저 볼 곳 | 읽기/복구 방법 |
|---|---|---|
| 페이지 자체가 안 열림 | 노트북 Station | 실행 창·포트 확인, `http://localhost:8766/` 접속 |
| 공유기 `.1`도 안 됨 | 노트북 Ethernet·공유기 전원 | LAN 포트와 `ipconfig`의 주소 확인 |
| `.1`은 되지만 `.200`은 안 됨 | Jetson 전원·field Wi-Fi | 로컬 `nmcli`에서 radio/profile 확인 |
| `.200` ping/SSH는 되지만 dashboard 단절 | runtime·TCP 포트 | `pgrep`, service journal, `Test-NetConnection` |
| telemetry는 오는데 영상만 없음 | 카메라·브라우저 | camera/녹화 상태, camera 경로, localhost Chrome/Edge 확인 |
| 한 Scout만 단절 | 그 ESP·IP·해당 슬롯 | MAC/IP와 보드 전원·control 상태 확인 |

노트북 PowerShell:

```powershell
ipconfig
ping 192.168.0.1
ping 192.168.0.200
Test-NetConnection 192.168.0.200 -Port 8000
```

Jetson 로컬 터미널:

```bash
nmcli radio wifi
nmcli connection show --active
sudo nmcli radio wifi on
sudo nmcli connection up rover-field
```

`radio on`·`connection up`은 해당 상태가 꺼졌거나 연결이 필요한 경우 사용한다.
같은 `192.168.0.x` 대역을 쓰는 다른 노트북 Wi-Fi가 있으면 의도하지 않은 경로로 갈 수 있으므로 접속 경로를 구분한다.
ping은 IP 통신 확인이고 새 telemetry·영상·제어 연결은 각 서비스의 상태다.

현재 AP 폭은 80 MHz다. 폭을 좁히면 사용하는 주파수 범위와 최대 전송 여유가 함께 줄어든다.
관리 화면에서 값을 변경한 경우 되돌릴 기준은 변경 직전의 채널/폭이다.
브라우저 영상 창의 크기는 표시 크기이며 카메라가 보내는 압축 데이터율을 바꾸지 않는다.

### 자동 복구

[wifi_recover.py](../jetson/wifi_recover.py)는 `rover-wifi-recover.timer`가 주기적으로 호출한다.
저장된 field profile의 인증 대기/실패를 재시도하고 연결 상태에서는 해당 무선 interface를 통한
LAN gateway의 ICMP/TCP 응답을 읽는다. 인터넷 서버를 판단 기준으로 사용하지 않는다.

반복 손실·유예·재연결 간격 조건을 충족하면 field profile을 재연결한다.
단발 손실, 검사 도구 오류, 다른 profile 사용, 수동 radio off/device disconnect에는 강제 복구하지 않는다.
전역 NetworkManager 재시작·driver 재결합·재부팅도 자동 수행하지 않는다.

| 파일/명령 | 용도 |
|---|---|
| `WIFI_RECOVERY` in config.py | 유예·timeout·실패 횟수·timer 정책의 원본 |
| `/etc/rover/wifi-recovery.json` | 설치된 helper 설정 |
| `logs/wifi_recovery/decisions.jsonl` | 판단·시도 기록. PSK는 기록하지 않음 |
| `systemctl status rover-wifi-recover.timer --no-pager` | timer 상태 |
| `journalctl -u rover-wifi-recover.service -n 50 --no-pager` | 최근 helper 실행 |

설치 도구는 [install_wifi_recovery.py](../jetson/install_wifi_recovery.py)다.
interface·profile·기존 설치 manifest·receipt 경로를 받아 설정과 unit을 설치하며 기존 keyfile은 수정하지 않고 root 전용으로 백업한다.
단순 source 배포와 timer 설치는 별개다. 복구 시 해당 설치 receipt의 manifest·rollback을 함께 사용한다.

dashboard는 응답 없는 socket을 교체한다. 재연결 후 이전 조작을 재전송하지 않으므로 운전은 새 입력으로 이어간다.
Rover 프로세스 자체의 수명은 [field service](#field-실행-service)가 담당한다.

## AX210 복구 조건

AX210은 기존 JetPack 커널에 맞춘 `iwlwifi.ko`·`iwlmvm.ko` 두 모듈을 사용한다.
NVIDIA의 해당 커널 source에서 빌드했으며 `cfg80211`·`mac80211`·Realtek 모듈은 유지한 구성이다.

| 항목 | 기록된 위치/식별 |
|---|---|
| 커널 | `5.15.148-tegra` / L4T R36.4.4 |
| 설치 모듈 | `/lib/modules/5.15.148-tegra/extra/rover-ax210-r36.4.4/` |
| 기존 Jetson 빌드·설치·제거 작업 | `~/ax210-native-r36.4.4-1791135235640/` |
| PCI | `8086:2725` |
| firmware | `66.f1c864e0.0 ty-a0-gf-a0-66.ucode` |

`setup_jetson.sh`는 이 모듈을 설치하지 않는다. OS·커널 복원에서는 기존 모듈·firmware·빌드 자료와 커널 버전을 함께 다룬다.
다른 커널용 `.ko`를 파일명만 맞춰 복사하거나 일반 Ubuntu backport/DKMS 절차를 기존 설치와 같은 것으로 취급하지 않는다.
보관된 복구 자료의 위치·범위는 [최종 인계](../data/final_vehicle_2026-10-28/README.md)에 있다.

현재 로드 경로를 읽는 명령:

```bash
uname -r
lspci -nnk | grep -A3 -i network
modinfo -n iwlwifi
modinfo -n iwlmvm
nmcli connection show --active
iw dev
```

## 복구 범위

복구할 상태는 소스·Python 환경·OS 드라이버·무선 profile·service·보드 firmware로 나뉜다.
어느 층을 바꾸었는지 기록하면 다음 작업자가 같은 설치를 다시 만들 수 있다.
source/config hash, 설치 이미지 hash, 바뀐 파일의 백업, service 인수와 재시작 receipt를 해당 작업과 함께 남긴다.
모든 과거 로그를 모을 필요는 없으며 현재 설치를 설명하는 최종 기록을 보존한다.

Jetson이 꺼졌거나 runtime이 종료됐으면 중복 실행 여부를 읽고 [README](../README.md)의 수동 시작을 따른다.
USB SSH에는 과거 NCM 오류 기록이 있어 로컬 모니터/키보드가 별도 복구 경로다.
실제 운동 원본을 옮길 때 필요한 파일 묶음과 시각·좌표 의미는 [데이터 계약](13_scout_integration.md)에 있다.
