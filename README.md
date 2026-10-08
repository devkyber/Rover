# Rover — 현장 사용법

Jetson과 공유기의 전원이 켜져 있고, 기존 장치 설정이 완료된 상태에서 사용한다.

## 1. 연결

```text
노트북 ── USB Ethernet ── 공유기의 LAN 포트 (WAN 아님)
Jetson / AX210 ── Wi-Fi: rover-5G ── 같은 공유기
카메라 · U2D2 · Pico(사용 시) ── Jetson USB
```

| 대상 | 주소 |
|---|---|
| 공유기 설정 | http://192.168.0.1 |
| Jetson | `192.168.0.200` |
| 조종 화면 | http://localhost:8766/?rover=192.168.0.200 |

공유기 LAN은 `192.168.0.1/24`, DHCP 범위는 `.100–.199`로 둔다.
Jetson의 고정 주소 `.200`은 DHCP 범위 밖이다. 인터넷 연결은 필요 없다.
노트북의 집 Wi-Fi는 인터넷용으로 함께 연결해도 된다.

## 2. Jetson 프로그램 시작

노트북 PowerShell에서 접속한다.

```powershell
ssh taeholee@192.168.0.200
```

Jetson 터미널에서 실행한다. `pgrep`에 기존 프로그램이 나오면 중복 실행하지 않는다.

```bash
pgrep -af '[r]over_main.py'
source ~/rover-venv/bin/activate
cd ~/Rover/rover
```

**카메라·통신만 확인** — 모터와 Pico를 사용하지 않는다. 표시되는 IMU는 가상 값이다.

```bash
python rover_main.py --no-motors --imu fake --pico off --no-log
```

**주행** — U2D2와 BMI088 연결을 확인한 뒤 실행한다. 모터 제어가 활성화된다.

```bash
python rover_main.py --imu bmi088 --pico off --tag field
```

Pico 서보·배터리 기능까지 사용하려면 `--pico off`를 `--pico usb`로 바꾼다.
시작 후 약 5초간 정지해 IMU 정렬을 기다린다. IMU 오류가 나면 주행하지 않는다.
AX210의 근거리 연결은 확인했지만, 10 m·가림 조건의 신뢰성과 BMI088를 포함한
전체 주행은 별도로 검증해야 한다.

## 3. 노트북 조종 화면

새 PowerShell 창에서 실행한다. Python이 설치되어 있으면 추가 패키지는 필요 없다.

```powershell
cd C:\Users\tae06\CODE\Rover
python station/station.py --rover 192.168.0.200 --port 8766
```

Chrome 또는 Edge에서 위 조종 화면 주소를 연다. `ROVER`가 `192.168.0.200`인지
확인하고 `CONNECT`를 누른다. 조종 탭은 하나만 사용한다.

| 조작 | 기능 |
|---|---|
| `DRIVE` 원형 패드 드래그 | 화면 조이스틱. 놓으면 정지 |
| `W / S`, `A / D` 또는 방향키 | 전진·후진, 좌·우 회전. 키를 놓으면 정지 |
| `SPEED LIMIT` 슬라이더 | 속도 제한. 낮게 시작 |
| `Space` / `E-STOP` | 소프트웨어 비상 정지. 재개는 `RELEASE` |
| `R / F`, `Q / E` | Pico 사용 시 리프트, 카메라 팬 |
| `RUN` 이름 → `NEW RUN` | 새 기록 시작·IMU 재정렬. 약 5초 정지 |
| `STOP LOG` | 기록만 종료. 주행 정지는 아님 |

영상·연결이 끊기면 조작을 멈추고 아래 연결 확인부터 한다.
소프트웨어 `E-STOP`은 물리적 전원 차단을 대신하지 않는다.

## 4. 연결이 안 될 때

노트북에서 LAN 케이블과 주소를 확인한다.

```powershell
ipconfig
ping 192.168.0.1
ping 192.168.0.200
```

공유기는 응답하지만 Jetson이 응답하지 않으면 Jetson 로컬 터미널이나
다른 연결의 SSH에서 Wi-Fi를 확인한다.

```bash
nmcli connection show --active
sudo nmcli connection up rover-field
```

`rover-field`는 부팅·재연결 때 자동 연결된다. 이미 집 Wi-Fi에 연결된 경우에는
위 명령으로 전환한다. 전환하면 기존 Wi-Fi SSH가 끊기므로 `.200`으로 다시 접속한다.
프로필이 없을 때만 아래 명령으로 만들고, 위 명령으로 활성화한다.

```bash
bash ~/Rover/jetson/wifi_field.sh "rover-5G"
```

`.200`은 응답하는데 화면이 끊겨 있으면 Jetson의 `rover_main.py` 실행 여부와
화면의 `ROVER` 주소를 확인한다. `Address already in use`는 기존 프로그램이나
서버가 실행 중이라는 뜻이다. 기존 창을 사용하거나 그 창에서 `Ctrl+C`로 종료한다.

## 5. 종료·기록

정지 → `STOP LOG` → Jetson 실행 창에서 `Ctrl+C` 순서로 종료한다.
기록은 Jetson의 `~/Rover/logs/`에 저장된다. CSV와 같은 이름의 `.meta.json`을 함께 보관한다.
필요하면 노트북 PowerShell에서 복사한다.

```powershell
scp -r taeholee@192.168.0.200:~/Rover/logs ./field-logs
```

Jetson 전원을 끄려면 Jetson 터미널에서 `sudo shutdown -h now`를 실행한다.
노트북 스테이션 서버는 실행 창에서 `Ctrl+C`로 종료한다.

## 설치 파일

- [rover/rover_main.py](rover/rover_main.py): Jetson 실행. 설정은 [rover/config.py](rover/config.py).
- [station/station.py](station/station.py): 노트북 조종 화면 서버.
- [jetson/setup_jetson.sh](jetson/setup_jetson.sh): 새 Jetson의 기본 패키지·USB 장치 이름 설정.
  `bash ~/Rover/jetson/setup_jetson.sh`로 실행한다. AX210 드라이버와 BMI088 SPI 설정은 포함하지 않는다.
- [firmware/pico/main.py](firmware/pico/main.py): MicroPython이 설치된 Pico용 펌웨어.
  Jetson의 저장소 루트에서 `mpremote connect /dev/pico cp firmware/pico/main.py :main.py + reset`으로 복사한다.

참고 설정·문제는 [docs](docs/README.md), 원본은 [logs](logs/README.md), 실측·보관 분석은 [data](data/README.md)에 있다.
오래된 자료는 저장소 안 ZIP으로 보관하고 필수 내용만 문서에 남겼다.
코드를 배우려면 [학습 순서](docs/08_learning_guide.md)와 [회전각 예제](docs/09_turn_angle_lesson.md)부터 읽는다.
현재 AX210 드라이버는 Jetson에 설치되어 있다. 커널 업데이트 시 재빌드가 필요할 수 있다.
암호는 Git에 기록하지 않고 개인 `*.local.md` 또는 자격 증명 저장소에서 관리한다.
