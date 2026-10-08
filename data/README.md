# 측정·분석 자료

원본은 [logs](../logs/README.md), 현재 작업은 [로드맵](../docs/05_한계와_로드맵.md).
실측은 삭제 가능한 캐시가 아니다. 원본·라벨·설정/소스·방법·결과를 함께 보관한다.

| 자료 | 핵심 정보 |
|---|---|
| [link_tests.csv](link_tests.csv) | 원본 통신 측정. 새 측정은 `station/link_test.py`로 추가 |
| [AX210](ax210_bench_2026-10-08/REPORT.md) | 실제 driver·홈 Wi-Fi 60초·AX3000Q 화면 확인. 10 m 미검증 |
| [BMI088](bmi088_stationary_2026-10-08/REPORT.md) | 60초 정지 baseline. gyro bias·timing·한계 |
| [2026-10-01 평가](archive/odometry_evaluation_2026-10-01.zip) | 실제 줄자 정지점 중 ±5 cm 밖 결과 존재. synthetic pass는 실차 신뢰성 아님 |
| [2026-10-02 방법 비교](archive/odometry_reference_review_2026-10-02.zip) | 16 logs / 222,897 samples. 외부 정답은 일부 종방향 정지점뿐 |
| 2026-10-02 Arduino 튜닝 | [분할 보관](archive/README.md). 1.14688 단위 환산으로 예약 코너의 nominal 90° 차이가 평균 9.49° → 3.86°. 측량 각도 오차 아님; BMI088 적용 금지 |

옛 분석은 출력·manifest·방법을 바꾸지 않고 ZIP으로 묶었다.
현재 소스로 과거 결과가 재현된다고 가정하지 않는다. 펼치는 방법과 소스는
[보관 안내](archive/README.md), 원래 긴 해석은 [문서 보관](../docs/archive/README.md).
