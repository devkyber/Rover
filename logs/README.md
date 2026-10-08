# 원본 기록

CSV·JSON·metadata·실측 라벨은 수정 없이 보관한다.
[experiments.json](experiments.json)은 수동 정답/구간의 원본이며 출력 형식은
[logger.py](../rover/logger.py)가 정의한다. 과거 도구는 [소스 아카이브](../data/archive/README.md).

| 기록 | 의미 / 한계 |
|---|---|
| `2026-09-06_*still.csv` | 정지 baseline |
| `2026-09-06_*airborne*.csv` | 바퀴 공중 회전: 버스·부호 확인용, 주행 보정에 사용 금지 |
| `2026-09-06_2323_drive.csv` | 시작 정지가 3.8초뿐이므로 5초 alignment 미완료 |
| `2026-09-06_2329/2351_drive.csv` | 주행 진단; 외부 위치/각도 정답 없음 |
| `2026-09-06_2334/2347_drive.csv` | 줄자 종방향 정지점만 실측. 연속 XY·heading 정답 아님 |
| `2026-10-02_*` | 10개 Arduino 기록. `0204_drive` 정지, `0205` 코너 튜닝, `0223/0224` 코너 예약 구간. 약 90°라는 운영자 라벨이며 측량 각도 아님 |
| `2026-10-07/08_*probe.json`, loopback | SPI 진단. raw TX/RX·실행 소스/해시를 포함하는 파일 유지 |
| [BMI088 60초 정지](2026-10-08_011054_bmi088_stationary.json) | 6,001 samples. 같은 기록의 시간 분할 평가, 독립 회전 정확도 아님 |
| `2026-10-08_0135/0139/0142_*.csv` + sidecar | motor-free BMI088 통합; loose board, 실차 축·각도 정답 없음 |

파일명의 `/` 표기는 여러 suffix의 약칭이다. 자세한 각 기록의 관찰은
[옛 로그 설명 원문](../docs/archive/README.md)에 보존했다.

## 당시 치수·축

| 기록 날짜 | 바퀴 반지름 | 좌우 간격 |
|---|---:|---:|
| 2026-09-06 | 0.0625 m | 0.485 m |
| 2026-10-02 기록 | 0.055 m | 0.1585 m |

현재 config로 과거 결과를 덮어쓰지 않는다. 구형 Arduino sensor-frame CSV는
`logger.replay()`가 body로 변환한다. `imu_frame=body`에는 중복 적용하지 않는다.
BMI088는 별도 map·gyro FIFO/host 간격을 쓴다. accel time은 gyro 적분 시간이 아니다.

## 잘못된 해석을 피할 것

- `2026-10-07_bmi088_corrected_wiring_probe.json`과 `2026-10-08_bmi088_clock_parent_probe.json`은
  TX 버퍼 재사용 오류가 있어 반복 ID 증거로 쓰지 않는다. fresh-buffer 재시험을 따른다.
- `2026-10-08_005530_bmi088_live_probe.json`에는 움직임이 있어 정지 bias 보정에 쓰지 않는다.
- driver lock·SPI property 후속 기록의 ID 실패를 성공으로 집계하지 않는다.
- CSV 옆 `.meta.json`을 함께 보관한다. 새 로그에는 센서/펌웨어·축·치수·보정·시각·코드 식별과
  독립 정답을 기록한다. tuning에 쓴 데이터를 unseen validation으로 다시 이름 붙이지 않는다.
