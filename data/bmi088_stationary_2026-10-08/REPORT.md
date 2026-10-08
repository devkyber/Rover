# BMI088 정지 측정 — 2026-10-08

소유자가 정지를 확인한 60초 / 6,001 samples. 모터 off.
**센서 축 baseline이며 장착 보정·회전 정확도 검증이 아니다.**

| 관측 | 결과 |
|---|---|
| 설정 | SPI mode 3, 요청 1 MHz, accel ±6 g / 400 Hz normal BW, gyro ±500 deg/s / 400 Hz / 47 Hz BW |
| host polling | 약 100 Hz; 중간 sensor update는 생략됨 |
| gyro 평균 X/Y/Z | +0.1592625 / +0.0532163 / +0.0314914 deg/s |
| gyro 표준편차 X/Y/Z | 0.089705 / 0.137625 / 0.096774 deg/s |
| 온도 | 22.875–23.250 °C |
| accel 평균 크기 | 9.63539 m/s²; 한 자세로 scale/bias 분리 불가 |
| host 간격 중간 / p99 / 최대 | 9.9997 / 10.6933 / 11.7713 ms |
| accel clock 대 host | nominal +1.05465%; gyro scale 보정값 아님 |

앞 3,000개로 bias를 구하고 뒤 3,001개(30초)에 적용했다.
추정 up축으로 투영한 잔여 적분은 끝 +0.00875°, 최대 절댓값 0.06290°.
이는 **같은 정지 기록을 시간으로 나눈 진단**이며 독립 0.009° 회전 정확도를 뜻하지 않는다.
평균 bias/noise를 runtime에 설치하지 않았다. Arduino scale 보정도 적용하지 않는다.

[원본·실행 소스](../../logs/2026-10-08_011054_bmi088_stationary.json),
[분석](analyze.py), [정확한 수치·해시·환경](summary.json), [그림](stationary.png).
재분석은 별도 복사본에서 `python data/bmi088_stationary_2026-10-08/analyze.py`로 실행한다
(NumPy/Matplotlib 필요; runtime 의존성과 별도). 원본 결과를 덮어쓰지 않는다.
이후 runtime의 accel BW·FIFO 방식은 [Jetson 기록](../../docs/11_jetson_bringup.md)과 다르다.
전체 원문은 [문서 보관](../../docs/archive/README.md).
