# Rover — KSRC 2026 달 탐사 로버

엔코더 + IMU를 융합해 **위성항법 없이 자기 위치를 추정하는** 오도메트리
시스템. 4륜 스키드스티어 모함, DYNAMIXEL XH430-V350-R 구동, 15상태
Error-State Kalman Filter.

**Learning in English:** Start with the [learning guide and complete source map](docs/08_learning_guide.md),
then run the [turn-angle lesson](docs/09_turn_angle_lesson.md).
The [code review](docs/10_code_review.md) separates this cleanup from unresolved behavior issues.

---

## 처음 왔다면 여기서부터

| 목적 | 문서 |
|---|---|
| **원리를 처음부터 알고 싶다** | [docs/01 — 오도메트리와 ESKF 이론](docs/01_이론_오도메트리와_ESKF.md) |
| **코드가 어디서 무엇을 하나** | [docs/02 — 코드 해설](docs/02_구현_코드해설.md) |
| **필터를 맞추고 싶다 / NIS가 뭔가** | [docs/03 — 튜닝과 NIS](docs/03_튜닝과_NIS.md) |
| **당장 돌려보고 싶다** | [docs/04 — 실행 가이드](docs/04_실행_가이드.md) |

전체 목록은 [`docs/README.md`](docs/README.md).

---

## 30초 요약

```bash
cd rover
```

| 명령 | 하는 일 | 모터 |
|---|---|---|
| `python test_filter.py` | 야코비안 5 + 시나리오 10 (하드웨어 불필요) | — |
| `python dxl_tool.py check` | 모터 레지스터 점검 | 안 움직임 |
| `python dxl_tool.py scan` | 포트·보드레이트·ID 탐색 | 안 움직임 |
| `python test_pipeline.py --seconds 5` | 센서→로깅→재생 자동 감사 | 안 움직임 |
| `python record.py --seconds 20` | 정지 로그 녹화 | 안 움직임 |
| `python teleop.py --tag drive` | **폰으로 조종** + 로깅 | **움직임** |
| `python replay_gui.py` | **차량 재생 + 실측점 + 필터 단계별 원인 분석** | — |
| `python replay.py ../logs/<파일>.csv --plot ../data/<파일>.png` | 로그 → 필터 → NIS 판정 | — |

조종은 폰(`http://<IP>:5000`), 실시간 상태는 브라우저 패널(`/panel`).

**로그 이름은 자동이다** — `logs/2026-09-06_1432_drive.csv`처럼 날짜·시각·태그로
붙는다. 고정 이름은 다음 실험이 이전 실험을 조용히 덮어쓴다.
`--tag`로 이름 조각을, `--log`로 경로 전체를 지정할 수 있고 `--no-log`면 안 남긴다.

---

## 지금 상태

| 항목 | 결과 |
|---|---|
| 파이프라인 감사 | **10/10** fake audit 통과; 실기 기록은 기존 100.0 Hz, 지터 0.28 ms, 드롭 0 |
| 필터 시나리오 | **10/10** (10시드 몬테카를로, 최악값 판정) |
| 수학·구조 검사 | **11/11** (야코비안 5개 + trace + yaw/좌표계/출력모델 검사) |
| 정지 20 s 실측 드리프트 | **5.3 mm**, ZUPT 95.8 % |
| Sync Read 왕복 (4륜) | 1.14–1.48 ms (≈700–875 Hz 한계) |
| `23:47` 재생 | yaw **+0.06°**, 복귀 **1.2 cm**, 최대 횡이탈 **4 mm** |

### 알아야 할 두 가지

> **1. Relative heading can still drift.**
> Stationary startup alignment estimates the initial gyro bias; later stationary
> ZARU updates can refine it. Residual bias and sensor noise still accumulate,
> and neither update supplies an absolute heading reference. Periodic stops can
> help bias estimation, but real turn accuracy still needs independent measurement.

> **2. 직진 거리용 휠 `R`은 바닥 주행으로 맞췄지만 동적 `R`과 회전 정확도는 아직 모른다.**
> 0.50 m 체크포인트 주행에서 종방향 오차는 4 cm 이내였지만, 회전이 많은
> 로그에는 외부 위치 정답이 없다. 다음 검증은 크기를 잰 정사각형 코스다.
> `python replay_gui.py`에서 추정 궤적과 등록된 줄자 체크포인트를 함께 본다.

> 실행 직후에는 **5초간 로버를 건드리지 않는다.** 이 동안 모터 명령은 강제로
> 0이고 패널에 `ALIGNING`이 표시된다. 완료 시 시작 방향이 yaw 0°가 된다.

> 임시 Arduino IMU는 로버 축과 90° 돌아가 장착되어 있다.
> `+X_body=-Y_sensor`, `+Y_body=+X_sensor`, `+Z_body=+Z_sensor` 변환을
> 드라이버가 적용한다. 2026-09-06 구형 로그는 replay가 같은 변환을 한 번 적용한다.

> 화면에 보이는 위치는 `p += R(q_ESKF)[v_wheel,0,0]dt`로 계산한다.
> 엔코더가 이동량, ESKF 자세가 진행방향을 담당한다. 내부 strapdown `p`는
> 위치 관측이 없어 교차공분산 보정으로 점프할 수 있으므로 진단선으로만 표시한다.

> 물리 치수(`WHEEL_RADIUS_M` 62.5 mm, `WHEELBASE_M` 485 mm)는 **실측 완료.**
> 공식 31 rpm과 기본 command limit의 환산은 약 **0.203 m/s**지만 24 V 무부하
> 기준이다. 적재된 실차의 지속 최고속도는 아직 별도로 측정하지 않았다.

한계와 다음 할 일은 전부 → **[docs/05 한계와 로드맵](docs/05_한계와_로드맵.md)**

---

## 하드웨어

| 항목 | 사양 |
|---|---|
| 구동 | DYNAMIXEL XH430-V350-R ×4 (모델 1040, FW v50) |
| 통신 | U2D2 (FTDI FT232H), COM22, **4.5 Mbps**, Protocol 2.0 |
| IMU (임시) | Arduino Nano 33 IoT — LSM6DS3, COM25, 104 Hz |
| IMU (목표) | BMI088 |
| 배치 | ID1 좌전 · ID2 우전 · ID3 좌후 · ID4 우후 |

> 4.5 Mbps는 상용 SDK가 거부한다. 로컬 패치가 있다 —
> [`rover/vendor/PATCHES.md`](rover/vendor/PATCHES.md). SDK를 올릴 때
> 반드시 다시 적용할 것.

---

## 폴더

```
rover/      코드. vendor/ 는 DYNAMIXEL SDK 사본 + 로컬 패치
arduino/    IMU 스트리밍 스케치 (Nano 33 IoT)
jetson/     카메라 서브시스템 (별개) -- docs/99
logs/       기록한 CSV. 입력이고, 다시 만들 수 없다
data/       로그에서 만들어낸 것 (그래프 등). 지워도 다시 생성된다
docs/       문서
```

`logs/`와 `data/`를 나눈 이유: **로그는 실험을 다시 해야만 얻을 수 있고,
`data/`는 언제든 `replay.py`로 다시 만들 수 있다.** 지워도 되는 것과 절대
지우면 안 되는 것이 한눈에 갈린다.

## 설정

바꿀 값은 전부 [`rover/config.py`](rover/config.py) 한 곳에 있다.
포트, 보드레이트, 바퀴 배치, 물리 치수, Q, R, 임계값.
그 외 파일에 상수를 새로 만들지 말 것.
