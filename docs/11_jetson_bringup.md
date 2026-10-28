# 설치·네트워크·복구

2026-10-28 KST 인계 기준. 일상 실행은 [README](../README.md), 파일·설치·백업 식별은
[최종 인계](../data/final_vehicle_2026-10-28/README.md), 미완료 검증은 [로드맵](05_한계와_로드맵.md)을 따른다.

| 항목 | 기존 차량 환경 |
|---|---|
| OS | JetPack 6.2.1 / L4T R36.4.4, `5.15.148-tegra` |
| 코드·Python | `~/Rover`, `~/rover-venv` (`--system-site-packages`) |
| serial 장치 | `/dev/u2d2`, `/dev/pico`: [udev](../jetson/99-rover-serial.rules) |
| 카메라 | USB H.264 → GStreamer, 재인코딩 없이 표시/저장 |
| BMI088 | `spidev==3.8`, accel `/dev/spidev0.0`, gyro `/dev/spidev0.1` |

## 환경 복원

장치가 정지된 상태에서 `bash ~/Rover/jetson/setup_jetson.sh`로 기본 패키지·udev·venv를 설치한다.
이 스크립트는 **AX210 모듈·SPI pinmux를 설치하지 않는다.** 커널/driver는 [AX210 복구 조건](#ax210-복구-조건)을 따른다.
venv는 JetPack의 GI/GStreamer를 읽도록 system-site-packages를 사용한다. NumPy는 JetPack 호환 범위를 유지한다.
카메라 저장에는 `appsrc`·`appsink`·`h264parse`·`mpegtsmux`와 plugins-good/bad가 필요하다.
serial/video 그룹 변경은 재로그인·재부팅 뒤 반영된다.

기존 SPI1 설정은 Jetson-IO의 `config-by-function.py -o dt spi1`을 적용하고 재부팅한 구성이다.
OS 복원 때 Jetson-IO에서 실제 헤더 기능을 확인한다. `/dev/spidev*` 존재만으로 배선·pinmux 완료를 판단하지 않는다.
확인 없이 다른 SPI bus 번호로 바꾸지 않는다. 현재 runtime은 한 BMI088 reader만 사용한다. 배선은 [하드웨어](06_하드웨어.md#bmi088-연결)를 따른다.

## Field 실행 service

[rover-field.service](../jetson/rover-field.service)는 기존 사용자·venv·작업 폴더에서 `--tag field`로 실행한다.
SSH 단절과 프로세스 수명을 분리하며 `Restart=no`다. 부팅 자동 시작도 기존 운영에서 켜지 않았다.
시작/종료는 README, 로그 확인은 `journalctl -u rover-field.service -n 50 --no-pager`다.
unit을 복원할 때 사용자·절대 경로를 실제 설치와 대조하고 `daemon-reload`한다. 기존 실행이 있으면 중복 시작하지 않는다.
정상 종료는 SIGINT로 actuator/worker/기록을 정리한다. kill·bus 단절의 실제 정지 보장은 별도 검증이다.

## 소프트웨어 배포

노트북 저장소 루트에서 `python tools/deploy_jetson.py`로 현재 운영 source와 SHA256 manifest의 bundle을 만든다.
출력된 경로로 설치한다.

```powershell
python tools/deploy_jetson.py --apply --bundle PATH
```

기존 SSH key를 사용한다. 모르는 원격 변경은 덮어쓰지 않으며 설치 전 백업·hash·기존 인수/venv·재시작 결과를 보존한다.
구동·HOLD·영상 녹화가 있으면 배포 중단 조건을 먼저 해소한다. 꺼진 runtime을 임의로 새로 켜지 않는다.
최종 보관 시 배포 결과와 재시작 성공 여부는 인계 receipt를 따른다. 소스 파일 복사는 Pico·Scout·Nano flash가 아니다.
보드별 source·이미지·기존 앱 백업·readback을 따로 식별한다. 자격 포함 Flash/image는 공개 파일에 넣지 않는다.

<a id="laptop-environment"></a>
## 노트북 환경

마지막 환경은 Windows CPython 3.13.14의 Tk 포함 설치다. Station 기본 서버와 Clegg/오프라인 분석 의존성을 구분한다.
Clegg launcher는 저장소 .venv를 사용한다. 환경을 지웠다면 저장소 루트에서 복원한다.

```powershell
py -3.13 -m venv .venv
.venv/Scripts/python.exe -m pip install -r tools/requirements-laptop.txt
.venv/Scripts/python.exe -c "import tkinter, numpy, matplotlib, serial"
```

Jetson requirements.txt와 노트북 requirements-laptop.txt는 별도다. Jetson은 JetPack GI/GStreamer와 numpy<2를 사용한다.
비밀 설정·credential-bearing Nano image는 ignored *.local.md/private에 별도로 인계한다. clone만으로 암호·설치 이미지를 복원했다고 주장하지 않는다.

## 네트워크와 복구

| 항목 | 저장된 기준 |
|---|---|
| 공유기 | ipTIME AX3000Q, `192.168.0.1/24` |
| 노트북 | USB Ethernet → LAN, DHCP `.100–.199` |
| Jetson | `rover-5G`, `rover-field`, 고정 `.200/24` |
| 무선 | channel 36 / 80 MHz, WPA2PSK + AES |
| 자동 연결 | field 우선순위 100, Wi-Fi 절전 꺼짐 |
| 복구 | NetworkManager 자동 연결 + `rover-wifi-recover.timer` |

재개 시 실제 설정을 대조한다. 집 Wi-Fi가 이미 연결되어 있으면 field로 자동 전환하지 않을 수 있다.
두 대역 Scout/Rover 간 LAN 통신을 허용하고 guest/client isolation을 사용하지 않는다.
암호는 ignored 파일/credential store에만 둔다.

### 채널 폭과 현장 복구

입력을 놓고 실제 정지를 확인한 뒤 노트북 Ethernet/LAN을 확인한다. 같은 주소 대역의 다른 Wi-Fi는 잠시 끈다.
PowerShell에서 ipconfig, ping 192.168.0.1, ping 192.168.0.200을 확인한다. 공유기부터 불통이면 전원/유선, Jetson만 불통이면 해당 전원/Wi-Fi를 확인한다.
Jetson 로컬 터미널에서 nmcli radio wifi와 nmcli connection show --active를 확인하고 필요할 때 sudo nmcli connection up rover-field로 연결한다.
radio가 꺼졌으면 sudo nmcli radio wifi on으로 켠다. 전환 시 SSH가 끊길 수 있으므로 field 주소로 다시 접속한다.

정상 연결이면 현재 값을 유지한다. 폭을 좁히면 겹치는 주파수 범위와 최대 전송 여유가 함께 줄어든다.
거리·손실·지연 개선량은 측정해야 한다. 같은 채널 혼잡·가림·AP/driver 고착을 해결한다고 보장하지 않는다.
영상 크기를 줄여도 카메라 압축 전송량은 바뀌지 않는다. 정지·하중 지지 후 유선 공유기 관리 화면에서 실제 채널/폭을 메모한다. Channel Width만 80 → 40 MHz로 바꾸고 새 영상/telemetry를 확인한다. 필요하면 20 MHz를 별도로 비교하고 개선이 없으면 변경 전 값으로 돌린다. 채널과 폭을 한 번에 함께 바꾸지 않는다.
실제 거리·방향·가림·채널/폭·시간·영상/telemetry 누락을 함께 기록한다. ping 응답만으로 주행 정지나 영상 복구를 판정하지 않는다.

### 자동 복구

[helper](../jetson/wifi_recover.py)는 인증 대기/실패를 재시도하고, 연결 상태에서는 해당 무선 interface의
LAN gateway ICMP/TCP 응답을 검사한다. 인터넷은 필요 없다. 반복 손실과 유예를 통과하면 저장된 field 프로필을 재연결한다.
단발 손실·검사 도구 오류·다른 프로필·수동 radio off/device disconnect는 강제 복구하지 않는다.
동작 직전·해제 뒤 수동 상태를 다시 확인하며 중복 실행은 잠금으로 막는다.

대기/검사 값은 [config.py](../rover/config.py)의 `WIFI_RECOVERY`, 설치는
[install_wifi_recovery.py](../jetson/install_wifi_recovery.py)가 소유한다.
설정은 `/etc/rover/wifi-recovery.json`, unit은 저장소의 service/timer를 사용한다.
Jetson의 `logs/wifi_recovery/decisions.jsonl`·journal에 판단을 기록하며 PSK는 기록하지 않는다.
Wi-Fi keyfile 백업은 root 전용 위치에 둔다. 실제 복구와 연결 명령 접수를 구분한다.

Rover는 [field service](#field-실행-service)로 SSH 수명과 분리한다.
dashboard는 응답 없는 socket을 교체하며 복구 뒤 이전 조작을 재전송하지 않는다.
전역 NetworkManager 재시작·driver 재결합·재부팅은 자동화하지 않는다.
AP/client/RF 최초 원인과 모든 packet 고착 복구는 미검증이다.

## AX210 복구 조건

기존 환경은 JetPack 6.2.1 / L4T R36.4.4 / `5.15.148-tegra`다.
NVIDIA 해당 커널 source에서 만든 `iwlwifi.ko`·`iwlmvm.ko`를 사용하며 cfg80211/mac80211/Realtek 모듈은 유지했다.

| 항목 | 기록된 위치/식별 |
|---|---|
| 설치 모듈 | `/lib/modules/5.15.148-tegra/extra/rover-ax210-r36.4.4/` |
| 기존 Jetson 빌드/설치/제거 작업 | `~/ax210-native-r36.4.4-1791135235640/` |
| PCI | `8086:2725` |
| firmware | `66.f1c864e0.0 ty-a0-gf-a0-66.ucode` |

setup_jetson.sh는 이 모듈을 설치하지 않는다. OS 재설치·커널 업데이트 전 기존 모듈/firmware와 빌드 방법을 백업하고
커널 일치를 확인한다. 보관한 복구 파일의 범위는 최종 인계를 따른다. Ubuntu backport/DKMS가 현재 설치 절차는 아니다.
6 GHz·Bluetooth·stock 대비 우위·10 m/장시간 신뢰성은 검증하지 않았다.

읽기 전용 재개 확인:

```bash
uname -r
lspci -nnk | grep -A3 -i network
modinfo -n iwlwifi
modinfo -n iwlmvm
nmcli connection show --active
iw dev
```

## 복구 범위

Jetson이 꺼졌거나 프로세스가 끝났다면 중복 실행 여부를 확인한 뒤 루트 README의 수동 시작을 따른다.
USB SSH는 과거 NCM 오류가 있어 보장된 복구 수단이 아니다. 로컬 모니터/키보드 경로를 남긴다.
SPI·무선 profile·커널 모듈·보드 flash는 각각 별도 상태다. 파일 복사·장치 파일 존재·ping만으로 전체 복구를 판정하지 않는다.
