# Jetson 설정·복구 핵심

기록 기준: 2026-10-08. 일상 실행은 [README](../README.md), 열린 문제는
[IMU-01](05_한계와_로드맵.md#imu-01)과 [OPS-01](05_한계와_로드맵.md#ops-01).

| 항목 | 확인된 구성 |
|---|---|
| OS | JetPack 6.2.1 / L4T R36.4.4, `5.15.148-tegra` |
| 코드 / Python | `~/Rover`, `~/rover-venv` (`--system-site-packages`) |
| 장치 이름 | `/dev/u2d2`, `/dev/pico`: [udev 규칙](../jetson/99-rover-serial.rules) |
| 카메라 | GStreamer / USB H.264, 1080p 약 30 fps·10 Mbit/s 벤치 확인 |
| BMI088 | `spidev==3.8`, accel `/dev/spidev0.0`, gyro `/dev/spidev0.1` |

기본 패키지는 `bash ~/Rover/jetson/setup_jetson.sh`로 설치한다.
이 스크립트는 AX210 모듈이나 SPI pinmux를 설치하지 않는다.
SPI1은 당시 `sudo python3 /opt/nvidia/jetson-io/config-by-function.py -o dt spi1`
후 재부팅해 켰다. 장치 파일 존재만으로 헤더 설정 완료를 판단하지 않는다.

## BMI088 연결

Shuttle Board 3.0의 **P1은 7핀 전원/인터럽트, P2는 9핀 SPI**다.
아래는 기록된 배선이며 Jetson 번호는 40핀 헤더의 물리 번호다.
전원을 끄고 실제 보드의 pin 1·커넥터 라벨을 확인한다.

| BMI088 | Jetson |
|---|---|
| P1-1 VDD / P1-2 VDDIO | 1 / 17 (각 3.3 V) |
| P1-3 GND | 6 |
| P2-2 SCK / P2-4 SDI / P2-3 SDO | 23 / 19 / 21 |
| P2-1 accel CS / P2-5 GPIO4 gyro CS | 24 / 26 |
| P2-6 GPIO5 | 9 (GND, gyro SPI 선택) |

loopback용 19–21 점퍼는 센서 연결 전에 제거한다. 과거 P1/P2를 뒤집은 안내는 오류였다.
당시 회로도 그림은 현재 스냅샷에 없으므로 실제 보드 확인을 이 표로 대신하지 않는다.

## 드라이버와 검증 범위

구현·설정의 원본은 [imu_reader.py](../rover/imu_reader.py)와 [config.py](../rover/config.py).
SPI mode 3, 요청 1 MHz, accel ±6 g / gyro ±500 deg/s, 각각 400 Hz.
장착 가정은 `[body x,y,z]=[sensor Y,-sensor X,sensor Z]`이며 실차 축/회전 부호는 미검증이다.

gyro FIFO 프레임을 모두 처리하되 프레임별 시각은 host 간격에서 추정한다.
accel sensor time을 gyro 적분 시계로 쓰지 않는다. 오류는 fault로 중단하며 자동 복귀하지 않는다.
CSV와 `.meta.json`에 원시 counts·FIFO·시각·설정·소스 snapshot을 함께 남긴다.

01:42 벤치에서 약 100 Hz 제어 / 400 Hz gyro, 1,432 cycles·5,745 frames를 기록했다.
[CSV](../logs/2026-10-08_0142_bmi088_main.csv),
[metadata](../logs/2026-10-08_0142_bmi088_main.csv.meta.json),
[배포 기록](../logs/2026-10-08_bmi088_driver_deployment.json)이 근거다.
모터 없이 느슨한 보드로 측정했으므로 장착·주행 정확도 검증이 아니다.

**이후 ID 읽기 실패가 다시 발생했고 원인은 미해결이다.**
[마지막 검증](../logs/2026-10-08_bmi088_driver_final_verification.json)의 14개 소프트웨어 테스트
통과와 [SPI 실패](../logs/2026-10-08_bmi088_spi_property_probe.json)를 구분한다.
통신 복구·반복 부팅·장착축 확인 후 실차 시험을 진행한다.

## 복구할 때 기억할 것

- AX210 커널 모듈 경로는 [통신 문서](12_field_link.md). 커널 변경 전에 확인한다.
- USB SSH는 NCM `Wrong NTH SIGN` 오류가 재발했다. 보장된 복구 수단으로 간주하지 않는다.
- BMI088 배포 전 백업은 Jetson의 `~/bmi088-driver-20261008-013419/target-before.zip`에 기록되어 있다.
- 최초 NVMe 부팅·실패 분석의 자세한 과정은 [문서 아카이브](archive/README.md)에 있다.
