# 문서

KSRC 2026 달 탐사 로버 — 엔코더 + IMU 오도메트리.

## English learning path

1. [08 — Learn the code through the mechanics](08_learning_guide.md): study method, Python essentials, and every project source file.
2. [09 — How a gyro becomes a turn angle](09_turn_angle_lesson.md): executable example and line-by-line explanation.
3. [10 — Code review and cleanup record](10_code_review.md): changes, remaining issues, and verification evidence.

## 읽는 순서

| | 문서 | 무엇 |
|---|---|---|
| **01** | [이론 — 오도메트리와 ESKF](01_이론_오도메트리와_ESKF.md) | 왜 이렇게 하는가. 좌표계·휠 기구학·관성항법·칼만 필터·ESKF 유도·관측 5종·관측가능성 |
| **02** | [코드 해설](02_구현_코드해설.md) | 01의 식이 어느 파일 어느 줄인가 |
| **03** | [튜닝과 NIS](03_튜닝과_NIS.md) | `Q`와 `R`을 어떻게 정하는가. NIS 이론과 실측 |
| **04** | [실행 가이드](04_실행_가이드.md) | 연결부터 로그 남기기까지 절차 |
| **05** | [한계와 로드맵](05_한계와_로드맵.md) ★ | 시스템 한계와 다음 실험. 구현 점검은 [10](10_code_review.md) |
| **06** | [하드웨어 — DYNAMIXEL](06_하드웨어.md) | 레지스터, SDK, 통신 문제 해결 |
| **07** | [개발 기록](07_실패기록.md) | 실제로 깨졌던 것들. 증상이 원인과 안 닮았던 사례 모음 |

### 아카이브

| | 문서 |
|---|---|
| **90** | [설계 노트](90_설계노트_아카이브.md) — 초기 브레인스토밍. 낡음 |
| **91** | [KSRC 기술제안서](91_KSRC_기술제안서.md) — 확정된 제약 |
| **99** | [카메라 / Jetson](99_카메라_노트.md) — 별개 서브시스템 |

---

## 빠른 참조

| 궁금한 것 | 위치 |
|---|---|
| `F` 행렬이 왜 저런가 | [01 §7.3](01_이론_오도메트리와_ESKF.md) |
| `H` 행렬 유도 | [01 §8](01_이론_오도메트리와_ESKF.md) |
| ZUPT가 왜 안 걸렸나 | [01 §8.6](01_이론_오도메트리와_ESKF.md) |
| **yaw가 왜 표류하나** | [01 §9](01_이론_오도메트리와_ESKF.md) · [05 §1-1](05_한계와_로드맵.md) |
| NIS가 뭔가 | [03 §3](03_튜닝과_NIS.md) |
| `R`을 어느 쪽으로 옮기나 | [03 §5.3](03_튜닝과_NIS.md) |
| **주행 로그를 움직이며 재생/실측 비교** | [04 §4](04_실행_가이드.md) |
| **다음에 뭘 해야 하나** | [05 §3](05_한계와_로드맵.md) |
| 4.5 Mbps가 안 될 때 | [06 §9.5](06_하드웨어.md) · [F-15](07_실패기록.md#f-15) |
| 바퀴 방향 뒤집기 | [04 §3](04_실행_가이드.md) |
| **왜 이렇게 안 했나** | [07 실패 기록](07_실패기록.md) |
| 상수 전체 | [`rover/config.py`](../rover/config.py) |
