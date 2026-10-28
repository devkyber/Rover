# 실차 원본 기록

운영자가 확인한 주행·점프와 별도로 식별한 최근 발췌를 보존한다. 원본 byte와 기록 당시 metadata는 바꾸지 않았다.

| 원본 | 내용 |
|---|---|
| [drives](drives/) | 9월 바닥 주행, 10월 2일 주행, 대회 영상과 연결된 주기록·관련 짧은 기록, 실내 복도 주행과 최근 발췌. CSV·세션 metadata·운영자 라벨 함께 보존 |
| [drives/experiments.json](drives/experiments.json) | 9월 실측 체크포인트·운영자 라벨 원문 |
| [연습 라벨 정정](drives/operator_labels/operator_label_correction_20261023T115527Z_01e23b30.json) | 추가 주행은 약 3 m라는 회상이다. 앞선 라벨의 180초 해석은 에이전트 오류이며 이전 라벨도 정정 이력으로 보존 |
| [scout_jumps](scout_jumps/) | 두 실제 점프의 원본 SDK CSV·record binary·metadata·당시 소스가 든 기존 ZIP, 명령·ACK 발췌와 운영자 관측 |
| [video](video/) | 대회 영상과 연결된 주기록 `e2c0c0d3…`의 원본 TS 조각·frame journal·metadata |

사용자는 `e2c0c0d3`와 연결된 arena 영상을 실제 10월 24일 대회 영상으로 확인했다.
과거 `practice` 파일명·날짜 라벨은 원문 그대로 두고 [후속 식별](../data/visual_report_2026-10-28/operator_clarification.json)에 정정을 기록한다.
약 15분은 운영자 회상이다. 9:20·7:28의 별도 기록을 연속된 한 주행으로 단정하지 않는다.
[최근 발췌](drives/recent_684775d5/) `684775d5`는 별도 세션이며 대회 주행으로 자동 연결하지 않는다.
원본 일치와 복구 범위는 [복구 기록](../data/visual_report_2026-10-28/recovery_receipt.json)에 있다.

세션의 시작 정지·정렬 구간도 그대로 포함한다. 대회 영상과 연결된 주기록·관련 짧은 기록과 복도 세션은 부분 기록이며 정상 종료 summary가 없다.
1970년 host 시각은 당시 잘못된 시계를 그대로 보존했다. run ID·monotonic·장치 시각을 구분한다.
영상 조각을 이어 재생하면 빠진 시간이 생략되며 실제 경과시간을 재현하지 않는다.

거리·코너 라벨은 측량 정답과 구분한다. encoder 이동량·화면 경로·명령 ACK는 실제 위치 정확도나 이륙 시각의 독립 검증이 아니다.
이전 차체의 CSV에는 당시 source/config sidecar가 없으므로 현재 설정을 소급 적용하지 않는다.
보존된 소스·설정 식별과 분석 방법은 [결과·방법](../data/README.md)을 따른다.
