# 보정·로그 해석 핵심

설정은 [config.py](../rover/config.py), 계산 순서는 [odometry.py](../rover/odometry.py)와
[eskf.py](../rover/eskf.py)가 원본이다. 과거 비교는 [data](../data/README.md)에 보관한다.

- ESKF 자세로 바퀴 속도를 회전해 공개 위치를 적분한다. 내부 ESKF 위치와 같은 출력이 아니다.
- 중력은 절대 yaw를 관측하지 못한다. 정지 bias 보정으로 모든 heading drift가 없어지는 것은 아니다.
- 바퀴가 공중에 있거나 미끄러진 기록을 정상 wheel 속도 관측의 튜닝 기준으로 쓰지 않는다.
- NIS가 좋아져도 실제 위치·각도 정확도를 증명하지 않는다. 독립 줄자/각도 기준을 기록한다.
- 9월과 10월의 바퀴 치수가 다르다. [로그 목록](../logs/README.md)의 당시 치수를 사용한다.
- 과거 Arduino 환산 실험의 `1.14688`은 해당 encoding에 관한 값이다. BMI088에 적용하지 않는다.
- BMI088 stationary bias는 센서 축의 벤치 관측이다. 장착축 확인 없이 yaw bias로 쓰지 않는다.
- BMI088 gyro 적분은 host batch 간격과 모든 FIFO frame을 사용한다. accel clock을 대신 쓰지 않는다.
- 기존 replay는 metadata의 모든 설정을 자동 복원하지 않는다. 현재 config로 과거 결과를 덮어쓰지 않는다.

새 시험은 설치 축/펌웨어/코드·설정/치수/온도/하중과 실제 정답을 함께 남긴다.
튜닝에 쓴 기록과 최종 독립 검증 기록을 분리한다. 단순 폐곡선·명령 90°를 정답으로 만들지 않는다.
보정·검증 완료 조건은 [ODOM-02/03](05_한계와_로드맵.md#odom-02).
