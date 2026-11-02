# 차량 설계와 계산 — 바퀴, IMU, ESKF, 카메라

이 문서는 Rover가 바퀴·IMU로 자세와 이동을 계산하는 모델을 설명한다.
수치 설정은 [config.py](../rover/config.py), 식·갱신 순서는 [eskf.py](../rover/eskf.py)와
[odometry.py](../rover/odometry.py)가 원본이다. 이 문서의 계산 예제는 센서 보정값이 아니다.
운전은 [README](../README.md), 실제 부품·배선은 [하드웨어](06_하드웨어.md),
기록 형식은 [차량 데이터 계약](13_scout_integration.md)을 따른다.

## 먼저 이해할 전체 그림

Rover는 **바퀴가 얼마나 돌았는지로 이동량을 얻고, IMU로 차체가 어느 방향을 향하는지 구한다.**
이 둘을 합쳐 시작점에서 현재 위치까지의 경로를 그린다. 이처럼 이전 위치에 작은 이동을 계속 더하는 방법이
오도메트리(odometry), 또는 추측항법(dead reckoning)이다.

```text
이번 위치 = 이전 위치 + 이번 구간의 이동
이번 이동 = 이번 속도 × 경과 시간
세계 좌표의 속도 = 차체가 향한 방향으로 회전한 차체 전진속도
```

예를 들어 차체가 시작 방향으로 1 m 전진한 뒤 왼쪽으로 90° 돌아 1 m 전진하면 local 위치는 `(1, 1)`이다.
첫 1 m와 두 번째 1 m의 길이는 같지만 방향이 다르다. 바퀴만 읽으면 방향을 알기 어렵고,
자이로만 읽으면 이동 거리를 알기 어렵기 때문에 두 입력이 필요하다.

| 입력·계산 | 직접 얻는 것 | Rover에서 맡는 일 |
|---|---|---|
| 모터 Present Velocity | 모터 축의 회전속도 | 반지름·좌우 부호를 적용해 전진속도 계산 |
| BMI088 gyro | 센서 축의 각속도 | 장착축 변환·bias 제거 후 자세 적분 |
| BMI088 accel | 센서가 느끼는 비력 | 정지 정렬과 조건부 중력 방향 관측 |
| ESKF | 자세·bias·내부 속도와 공분산 | IMU 예측을 바퀴·정지·기구 제약으로 보정 |
| 공개 odometry | 바퀴 속도와 최종 자세의 적분 | 화면·telemetry에 내보내는 연속 경로 |

센서 융합은 각 입력을 그대로 평균 내는 일이 아니다. 서로 다른 단위의 입력을 같은 운동 모델에 넣고,
그 모델이 예측한 센서값과 실제 센서값의 차이로 상태를 수정한다.
현재 알고리즘은 15차원 ESKF다. 초기 전장 구상에 있던 Mahony 상보필터나 바퀴만의 차동 yaw 계산을 실행하지 않는다.

### 용어를 코드와 연결하기

| 용어 | 쉬운 뜻 | 여기서의 표기 |
|---|---|---|
| State | 지금 위치·속도·방향·센서 오프셋을 묶은 값 | `p, v, q, bg, ba` |
| Bias | 가만히 있어도 남는 센서의 치우침 | `bg`, `ba` |
| Prediction | IMU와 지난 상태로 다음 상태 계산 | `eskf.predict()` |
| Observation | 상태를 보정하는 센서값 또는 조건 | wheel, gravity, ZUPT 등 |
| Residual / Innovation | 관측값 − 모델이 예상한 관측값 | `y = z - h` |
| Covariance | 각 오차의 크기와 함께 변하는 정도 | `P`, `Q`, `Rnoise` |
| Jacobian | 작은 상태 오차가 관측에 얼마나 영향을 주는지 | `F`, `H` |
| Injection | 계산한 작은 오차를 실제 상태에 반영 | `p += δp`, `q ⊗ Exp(δθ)` |
| Gate | 해당 관측을 지금 써도 되는지 판단하는 조건 | gravity norm·turn-rate gate |

### 이 문서 읽는 순서

처음에는 좌표계와 데이터 흐름을 읽고 [회전각 계산 예제](#12-회전각-계산-예제)를 실행하면 된다.
필터를 이해하려면 그다음에 자세 오차 → 예측 → 관측 → 갱신 순서로 읽는다.
카메라·전력 숫자를 계산할 때는 뒤의 [카메라 계산](#카메라-시야와-영상량-계산)과
[전력 계산](#전력질량-예산-계산)을 바로 사용할 수 있다.

## 1. 좌표계·상태·단위

Body는 +X 전진, +Y 좌측, +Z 위쪽이다. World는 실행 시작점의 local 좌표이며 +Z가 위쪽이다.
초기 yaw=0은 시작 방향일 뿐 북쪽이 아니다. `R(q)`는 body 벡터를 world로 옮기므로 역변환은 Rᵀ이다.
센서 축은 body 축과 다르다. raw→SI 환산과 장착 회전은 입력 경계에서 한 번만 수행한다.
구형 Arduino 기록과 BMI088는 서로 다른 장착 map을 사용하므로 현재 config를 과거 기록에 덮어씌우지 않는다.

| 기호 | 크기 | 단위·의미 |
|---|---|---|
| p, v | 각 3 | 내부 world 위치 m, 속도 m/s |
| q | 4, wxyz | body→world 단위 쿼터니언 |
| bg, ba | 각 3 | gyro bias rad/s, accel bias m/s² |
| δx | 15 | `[δp,δv,δθ,δbg,δba]` |
| P | 15×15 | 오차 공분산; 대각은 분산, 교차항은 해당 단위의 곱 |
| gyro, accel | 각 3 | body rad/s, specific force m/s² |
| dt | scalar | 실제 적분 구간 s |

명목상태는 3+3+4+3+3=16개 숫자지만 자세 오차는 3차원이므로 오차상태는 15차원이다.
가속도계는 중력 제거 후 world 가속도를 주지 않는다. 수평·지지된 정지에서 body accel은 `[0,0,+g]`다.

### 축 변환을 한 번만 하는 이유

센서에 인쇄된 X와 차량의 전진 방향 X가 같은 뜻은 아니다. 현재 BMI088 설정은 sensor +Y가 차체 전방,
sensor +X가 차체 오른쪽인 장착을 표현한다. `BMI088_IMU_TO_BODY`를 벡터에 곱하면 다음과 같다.

```text
[x_body]   [ 0  1  0 ] [x_sensor]
[y_body] = [-1  0  0 ] [y_sensor]
[z_body]   [ 0  0  1 ] [z_sensor]
```

따라서 sensor `[1, 2, 3]`은 body `[2, -1, 3]`이 된다. gyro와 accel 모두 같은 회전을 쓴다.
단위 환산과 축 변환을 마친 값을 ESKF에 넣으며, 이미 body인 CSV에 이 행렬을 다시 곱하지 않는다.
장착 행렬은 현재 소프트웨어의 가정이다. 과거 Arduino 축 변환이나 뒤집어 놓은 단품 기록과 구분한다.

World에서 yaw +90°이면 body 전방 `[1,0,0]`이 world 좌측 `[0,1,0]`이 된다.
반대로 world 전방 벡터를 차체에서 보려면 `Rᵀ`를 곱한다. 이것이 wheel/NHC 모델에 `Rᵀv`가 나오는 이유다.

### 각도·시간·거리의 단위

```text
degree → radian:  rad = deg × π/180
rpm → rad/s:      ω = rpm × 2π/60
millisecond → s:  dt_s = dt_ms / 1000
```

화면의 °/s와 predictor의 rad/s는 다르다. 30°/s는 약 0.523599 rad/s이며,
10 ms는 0.01 s다. 이 둘을 곱한 0.005236 rad가 한 구간의 회전량이다.
가속도 로그의 `g`와 SI `m/s²`도 구분하며, 계산에 사용하는 중력값은 `GRAVITY`가 소유한다.

반대칭 행렬은 `[u]×v=u×v`로 정의한다.

```text
          [  0   -uz   uy ]
skew(u) = [  uz   0   -ux ]
          [ -uy   ux    0 ]

skew(u)ᵀ = -skew(u)
skew(u)v = -skew(v)u
```

## 2. 차량 데이터 흐름과 공개 위치

```text
Station 입력 → station_link → rover_main → motors/dxl_control → 바퀴
바퀴 관측 → timestamp 보존 → 별도 estimation_process → Odometry.step
IMU → imu_process/imu_reader → 축·단위·FIFO 시각 → ESKF 자세·bias
                                                    └→ wheel 속도 회전 → 공개 위치
카메라 H.264 → 독립 영상 기록 / preview → Station
Pico → 서보·배터리                    Scout/Clegg → 독립 센서 원본
```

바퀴 i의 각속도 ωi를 좌우 부호 si, 반지름 r, 선택한 거리 배율 k로 환산한다.

```text
vi = si · r · k · ωi                 rad/s × m = m/s
vforward = mean(vi)
vpublic = R(qpost) · [vforward,0,0]ᵀ
```

### 모터 raw 값이 1 cm 이동이 되기까지

모터 통신 계층이 Present Velocity raw를 [config.py](../rover/config.py)의 `VELOCITY_TO_RADS`로 환산한다.
현재 설정의 1 raw는 0.229 rpm이다. 부호는 signed raw 해석과 좌우 장착 보정을 나누어 처리한다.

```text
ωi = rawi × VELOCITY_TO_RADS
vi = ωi × WHEEL_SIGN[i] × WHEEL_RADIUS_M × distance_scale
```

계산 예제로 네 바퀴의 raw가 `[100, -100, 100, -100]`, 반지름이 0.055 m,
거리 배율이 1이라고 하자. 오른쪽 두 바퀴의 부호를 보정하면 네 바퀴 모두 전진이다.

```text
100 × 0.229 = 22.9 rpm
22.9 × 2π/60 ≈ 2.398 rad/s
2.398 × 0.055 ≈ 0.1319 m/s
0.1319 × 0.1 s ≈ 0.01319 m = 13.19 mm
```

이때 yaw=0°면 이동은 약 `[13.19, 0, 0] mm`, yaw=90°면 `[0, 13.19, 0] mm`다.
반지름이나 distance scale을 10% 크게 쓰면 이 계산의 거리도 10% 커진다.
위 raw는 계산용 입력이며 저장된 실차 주행 표본이 아니다.

이는 접지·유효 반지름·낮은 slip을 가정한 전진속도 모델이다. encoder 회전은 실제 지면 이동 정답이 아니다.
Rover는 skid-steer이므로 `(vright-vleft)/track`을 기본 yaw-rate 관측으로 사용하지 않는다.
회전 중 scrub이 생기며 유효 회전 track과 물리 track이 같다고 보장할 수 없다.

이상적인 차동구동에서는 좌우 속도차가 `yaw_rate × track`이다. 그러나 4륜 스키드스티어는
조향축 없이 바퀴를 옆으로 끌면서 회전한다. 현재 추정기는 이 차동 식을 yaw 관측으로 쓰지 않고
gyro 회전량을 사용한다. `WHEELBASE_M`라는 이름의 설정은 코드에서 좌우 track 폭을 뜻하며,
앞뒤 축간 거리와 혼동하지 않는다.

공개 `Odometry.position`은 바퀴 속도와 ESKF 자세의 적분이다. 내부 strapdown `state['p']`와 다르다.
기존 endpoint 방식은 관측 갱신 후 자세로 전체 구간의 속도를 회전한다.

```text
Δppublic = R(qpost) · [vforward,0,0]ᵀ · dt
```

현재 gyro_midpoint 방식은 FIFO의 각 회전 구간을 반영한다. 각 rate가 동일한 Δti를 대표할 때:

```text
qmid,i = qi ⊗ Exp((ωi-bg)·Δti/2)
d = Σ R(qmid,i) · ex · Δti
Δppublic = vforward · R(qpost) · R(qpred)ᵀ · d
ex = [1,0,0]ᵀ
```

`qpred`는 해당 구간의 IMU 예측 끝, `qpost`는 관측 갱신 끝이다.
따라서 회전 중 방향을 중간값으로 적분하고 마지막 관측 보정을 구간에 적용한다.
회전이 없으면 endpoint 결과와 일치한다. smoothing이나 slip 복원은 아니다.
ZUPT가 유효하거나 바퀴 관측이 없으면 공개 전진속도/이동을 0으로 두며 실제 위치 정답으로 해석하지 않는다.

### 회전 구간에서 midpoint가 필요한 이유

속도 1 m/s로 1초 동안 heading 0°에서 90°까지 일정하게 회전한다고 가정하자.
endpoint는 전체 1초를 마지막 방향 90°로 보아 이동을 `[0, 1] m`로 계산한다.
한 구간 midpoint는 중간 방향 45°를 사용해 `[0.707, 0.707] m`가 된다.
실제 원호 적분값은 `[2/π, 2/π] ≈ [0.637, 0.637] m`다.

현재 코드는 하나의 큰 midpoint만 사용하지 않고 FIFO gyro를 작은 구간들로 나누어 더한다.
구간을 작게 나눌수록 위 원호 적분에 가까워진다. 마지막에 `R(qpost)R(qpred)ᵀ`를 곱하는 것은
그 구간 이후 관측으로 보정한 자세 방향에 이동벡터를 맞추기 위해서다.
이 예제는 적분 방식 비교이며 실제 차량 속도·회전 성능을 뜻하지 않는다.

## 3. 국소 자세 오차와 쿼터니언

### 왜 자세는 세 각도를 계속 더하지 않는가

평평한 바닥에서 수직축만 돌 때는 `yaw += gyro_z × dt`로 설명할 수 있다.
하지만 기울어진 차체에서는 sensor z축과 world 수직축이 다르며, X축 회전 뒤 Y축 회전과
Y축 회전 뒤 X축 회전의 결과도 다르다. 현재 predictor는 이 순서를 보존하는 쿼터니언을 쓴다.
roll/pitch/yaw는 사람이 읽기 위한 출력이고, 내부 자세는 `q=[w,x,y,z]`다.

Error-State Kalman Filter라는 이름은 큰 자세 자체 대신 **현재 자세에서의 작은 차이**를
칼만 필터가 구한다는 뜻이다. 실제 상태는 quaternion으로 보관하고, 작은 자세 오차는 3개 숫자 `δθ`로 다룬다.
관측 갱신 때마다 이 차이를 현재 자세에 반영하므로 자세의 3차원 자유도와 4개 저장값을 구분할 수 있다.

이 구현은 body에 정의한 국소 오차를 오른쪽에 주입한다.

```text
qtrue = q ⊗ δq
Rtrue ≈ R · (I + skew(δθ))
```

전역 오차 `δq⊗q` 관례와 섞으면 F/H의 부호가 달라진다.
회전벡터 θ의 norm을 α라 하면:

```text
Exp(θ) = [cos(α/2), (θ/α)·sin(α/2)]
α→0: Exp(θ) ≈ [1,θ/2]
qnew = normalize(q ⊗ Exp((gyro-bg)·dt))
```

쿼터니언 곱은 순서에 따라 결과가 달라진다. 코드의 small_angle_to_quat는 일반 구간에 sin/cos를 사용한다.
표시 yaw는 `atan2(R[1,0],R[0,0])`다. body gz는 기울어진 3차원 운동의 Euler yaw-rate와 같지 않다.

## 4. 예측과 오차 공분산

### 명목상태

`a=accel_raw-ba`, `ω=gyro_raw-bg`, `gworld=[0,0,-g]`로 둔다.

```text
aworld = R · a + gworld
pnew = p + v·dt + ½ aworld·dt²
vnew = v + aworld·dt
qnew = normalize(q ⊗ Exp(ω·dt))
bg,new = bg ; ba,new = ba
```

수평 정지에서 R=I, accel_raw=[0,0,g], ba=0이면 aworld=0이 된다.
Bias random walk는 명목 평균을 직접 바꾸지 않고 Q에 불확실성을 더한다.

### 가속도를 두 번 적분할 때 생기는 오차

가속도계 bias가 수평 방향으로 0.01 m/s² 남아 있다고 가정하면, 보정 없이 10초 적분한
속도 오차는 0.1 m/s, 위치 오차는 `½ × 0.01 × 10² = 0.5 m`다.
센서가 잘못 읽는 양은 작아도 같은 방향으로 누적되는 오차는 계속 더해진다.

또한 roll 또는 pitch가 1° 틀리면 중력의 수평 투영 약 `9.81 sin(1°) = 0.171 m/s²`를
실제 가속도로 오인할 수 있다. 그 오차를 6초 동안 그대로 두 번 적분하면 약 3.08 m다.
이것은 **관측 보정이 없는 내부 관성 적분**의 예제다. 실제 Rover의 공개 위치는 바퀴 속도로 계산하므로
이 숫자를 화면 위치 오차 예측으로 그대로 사용하지 않는다.

Bias와 noise도 구분한다. 일정 gyro bias의 각도 오차는 `bias × 시간`이고,
독립적인 백색 gyro noise만을 한 번 적분한 각도 표준편차는 이상적인 연속 모델에서 시간의 제곱근에 비례한다.
고정 오프셋, 시간에 따라 변하는 bias, 샘플 잡음은 같은 파라미터가 아니다.

### 오차 동역학과 F

자세 오차가 가속도에 미치는 1차 영향은 다음과 같다.

```text
R(I+skew(δθ))a - Ra = R·skew(δθ)a = -R·skew(a)δθ
```

따라서 위치/속도/국소 자세 오차의 선형화는:

```text
δp_dot  = δv
δv_dot  = -R·skew(a)δθ - R·δba + 가속도 잡음
δθ_dot  = -skew(ω)δθ - δbg + gyro 잡음
δbg_dot = bias random walk
δba_dot = bias random walk
```

F=I+A·dt의 1차 이산화로 코드의 다음 블록을 얻는다.

```text
          δp     δv       δθ             δbg     δba
δp     [   I    I·dt      0               0       0    ]
δv     [   0     I    -R·skew(a)dt         0      -R·dt ]
δθ     [   0     0    I-skew(ω)dt        -I·dt     0    ]
δbg    [   0     0        0               I       0    ]
δba    [   0     0        0               0       I    ]
```

자세 오차가 속도 오차, 속도 오차가 위치 오차로 연결된다.
작은 고정 gyro bias가 tilt를 만들면 중력의 잘못된 투영이 가속도에 들어가므로 내부 INS 위치 오차가 대략 시간의 3제곱으로 커질 수 있다.
이 관계는 slip과 공개 wheel 위치의 전체 오차 모델을 대신하지 않는다.

### Q의 단위와 전파

연속 잡음밀도 σ를 적분한 분산은 `σ²·dt`다. `(σ·dt)²`가 아니다.
gyro noise의 단위 rad/s/√Hz는 rad/√s이므로 σ²·dt의 단위는 rad²가 된다.

```text
Qvv   = I · ACCEL_NOISE_DENSITY² · dt
Qθθ   = I · GYRO_NOISE_DENSITY² · dt
Qbgbg = I · GYRO_BIAS_RANDOM_WALK² · dt
Qbaba = I · ACCEL_BIAS_RANDOM_WALK² · dt
Ppred = F · P · Fᵀ + Q
```

이는 현재 코드의 1차 공분산 모델이다. 위치에는 직접 Q를 넣지 않고 F를 통해 속도 불확실성이 전달된다.
현재 config의 noise density는 데이터시트 기반이며 bias random walk는 가정값이다.
이 값의 출처는 설치된 센서의 온도·진동을 포함한 실측 noise 모델과 구분한다.

## 5. 관측 모델과 Jacobian

관측에는 물리 센서값뿐 아니라 모델 조건도 포함된다. 예를 들어 정지라고 판단한 순간에는
속도가 0이라는 조건을 사용할 수 있다. 각 관측이 무엇을 알려 주는지 먼저 구분하면 수식을 읽기 쉽다.

| 관측 | `z`에 넣는 값 | `h`가 예상하는 값 | 전제 |
|---|---|---|---|
| Wheel | 바퀴 평균 전진속도 | 내부 world 속도를 body로 바꾼 x성분 | 바퀴 회전이 전진을 대표함 |
| NHC | body y/z 속도 0 | 내부 속도의 body y/z성분 | 횡·수직 이동이 작음 |
| ZUPT | world 속도 `[0,0,0]` | 내부 속도 `v` | 차체 정지 |
| ZARU | raw gyro 3축 | gyro bias `bg` | 실제 각속도 0 |
| Gravity | bias를 뺀 accel | 현재 자세에서 보이는 중력 지지 방향 | 동적 가속이 충분히 작음 |

NHC는 Non-Holonomic Constraint, ZUPT는 Zero Velocity Update,
ZARU는 Zero Angular Rate Update의 약자다. 센서가 없는 방향에 임의의 정답을 만드는 기능이 아니라
현재 운동 조건에서 성립한다고 둔 제한을 사용하는 것이다.

모든 관측은 `y=z-h≈Hδx`를 사용한다. 아래 H의 블록 순서는 `[δp|δv|δθ|δbg|δba]`다.
먼저 body 속도 w=Rᵀv를 국소 오차로 전개하면:

```text
δw = Rᵀδv - skew(δθ)Rᵀv
   = Rᵀδv + skew(w)δθ
```

body 중력 `gb=Rᵀgup`도 같은 이유로 자세 Jacobian이 skew(gb)가 된다.

### Wheel·NHC

```text
Wheel: z=vforward ; h=wx
Hwheel = [0 | Rᵀ[0,:] | skew(w)[0,:] | 0 | 0]          1×15

NHC: z=[0,0]ᵀ ; h=[wy,wz]ᵀ
Hnhc   = [0 | Rᵀ[1:3,:] | skew(w)[1:3,:] | 0 | 0]    2×15
```

NHC는 측면·수직 속도가 작다는 모델 가정이다. 접지 상실·횡미끄럼에서는 깨진다.
Wheel 표준편차는 `R_WHEEL_VELOCITY+R_WHEEL_TURN_GAIN·|yaw_rate|`,
NHC 측면 분산은 `R_NHC_VY_BASE²+(R_NHC_VY_TURN_GAIN·yaw_rate)²`로 회전 중 완화한다.
여기서 yaw_rate는 bias-corrected body z rate를 쓰는 proxy다.

### ZUPT·ZARU

```text
ZUPT: z=[0,0,0]ᵀ ; h=v
Hzupt = [0 | I | 0 | 0 | 0]                          3×15

ZARU: 정지에서 gyro_raw=bg+noise ; z=gyro_raw ; h=bg
Hzaru = [0 | 0 | 0 | I | 0]                          3×15
```

ZUPT는 속도, ZARU는 gyro bias를 직접 관측한다. 둘 다 실제 정지라는 전제가 필요하다.
다른 관측도 교차 공분산을 통해 bias를 바꿀 수 있지만 그 자체가 독립 bias/heading 정답은 아니다.

### 중력 방향

```text
accel_raw = Rtrueᵀgup + ba,true + noise
y = accel_raw - ba - Rᵀgup
  ≈ skew(gb)δθ + I·δba
Hgravity = [0 | 0 | skew(gb) | 0 | I]                 3×15
```

Accel-bias 블록의 부호는 +I다. 정지에서는 tilt와 accel bias가 축퇴해 잘못된 모델이 작은 drift로도 숨을 수 있다.
크기 `|norm(accel-ba)-g|`와 회전율 gate를 함께 사용한다. 측면 구심가속도는 norm을 조금만 바꿔도 방향을 기울일 수 있다.
통과한 관측의 표준편차도 norm deviation에 따라 늘린다. 문턱과 계수는 config가 소유한다.

## 6. 갱신·주입·NIS

### 칼만 이득을 숫자로 이해하기

한 방향 속도만 추정하는 단순 예제를 생각하자. 예측값이 0.20 m/s, 관측값이 0.10 m/s이고
예측 표준편차가 0.05 m/s, 관측 표준편차가 0.10 m/s라면:

```text
P = 0.05² = 0.0025 (m/s)²
Rnoise = 0.10² = 0.0100 (m/s)²
K = P/(P+Rnoise) = 0.2
y = 0.10 - 0.20 = -0.10 m/s
보정값 = K y = -0.02 m/s
갱신값 = 0.20 - 0.02 = 0.18 m/s
```

이 예제에서는 관측이 예측보다 불확실하므로 관측 쪽으로 20%만 움직인다.
`P`가 크면 새 관측의 영향이 커지고, `Rnoise`가 크면 줄어든다. 실제 ESKF는 같은 관계를
15차원 오차와 여러 관측에 적용하며, 교차 공분산 때문에 속도 관측이 bias 같은 다른 상태도 수정할 수 있다.

`config.py`의 `R_WHEEL_VELOCITY`, `R_ZUPT` 등은 이름에 R이 들어가지만 **표준편차**다.
`model_*()`가 이를 제곱해 관측 공분산을 만든다. 회전행렬 `R(q)`와 관측 공분산 `Rnoise`도 별개다.

예측 오차 공분산과 관측 noise covariance를 각각 P, Rnoise라 하면:

```text
y = z-h
S = H·P·Hᵀ + Rnoise
K = P·Hᵀ·S⁻¹
δx = K·y
Pnew = (I-KH)P(I-KH)ᵀ + K·Rnoise·Kᵀ                 Joseph form
```

코드는 S를 직접 역행렬로 만들지 않고 Cholesky 분해와 선형 풀이를 사용한다.
Joseph form과 대칭화는 수치 안정성을 돕지만 잘못된 입력/모델을 정당화하지 않는다.
오차를 명목상태에 주입한다.

```text
p←p+δp ; v←v+δv ; bg←bg+δbg ; ba←ba+δba
q←normalize(q⊗Exp(δθ))
```

Wheel/NHC/gravity/ZUPT에는 절대 heading 정보가 없다.
현재 구현은 K의 자세 보정에서 body로 표현한 world 수직축 방향을 제거하고 유한 회전의 잔여 yaw 변화도 보정한다.
이 yaw 보존은 외부 heading 관측을 생성하지 않는다. 내부 P는 공개 wheel 경로의 검증된 오차 범위가 아니다.

```text
NIS = yᵀS⁻¹y
```

일관된 모델에서 NIS 평균의 기준 자유도는 residual 차원이다: wheel 1, NHC 2, 중력/ZUPT/ZARU 각 3.
높은 값은 과신·부정확한 모델·동기 문제의 신호가 될 수 있고 낮은 값도 과도한 noise 등의 점검 대상이다.
NIS 일치만으로 위치·각도 정확도나 올바른 보정을 증명하지 않는다.

앞의 1차원 예제에서 `S=0.0125 (m/s)²`, `y=-0.10 m/s`이므로 `NIS=0.01/0.0125=0.8`이다.
단위를 가진 오차를 그 오차에 대해 예상한 분산으로 나눈 값이라 NIS는 무차원이다.
한 표본의 NIS가 자유도와 같아야 하는 것은 아니다. 자유도는 모델이 맞는 여러 관측을 모았을 때의 평균 기준이다.

### 어떤 방향이 관측되는가

수평 정지에서 중력은 위아래를 알려 주지만 북쪽은 알려 주지 않는다.
차체를 수직축 주위로 돌려도 중력벡터는 같으므로 accel만으로 yaw를 결정할 수 없다.
ZARU는 앞으로 쌓일 gyro bias 오차를 줄이는 관측이며 이미 틀어진 heading의 절대 정답을 주지 않는다.
현재 wheel/NHC/gravity/ZUPT의 자세 갱신이 yaw를 보존하는 이유가 여기에 있다.

## 7. 정렬·갱신 순서·유실 구간

정지 판정은 wheel 속도, gyro/accel 산포, accel norm과 정렬 때 얻은 gyro 기준의 차이를 함께 본다.
작은 분산만으로 실제 정지를 증명할 수 없다. 일정한 회전이나 운반도 낮은 산포를 만들 수 있다.
연속 정지 구간의 평균 gyro로 초기 bg를, 평균 accel 방향으로 roll/pitch를 정하고 yaw는 0으로 둔다.

정렬은 `STARTUP_ALIGNMENT_SECONDS`만큼 연속해서 정지로 판정된 자료를 모은다.
중간에 정지 조건이 깨지면 모으던 구간을 버리고 다시 시작한다. 이동평균 창이 차는 시간도 있으므로
프로그램 시작 후 정확히 그 초에 완료되는 타이머는 아니다. 정렬 전에는 공개 위치를 적분하지 않는다.

```text
gyro 평균 → 초기 bg
accel 평균을 정규화한 벡터 â →
  roll  = atan2(ây, âz)
  pitch = atan2(-âx, sqrt(ây²+âz²))
  yaw   = 0
```

정렬 때의 raw gyro 평균은 filter의 변화하는 `bg`와 별도로 정지 판정기에 남긴다.
이후 raw gyro가 그 기준에서 크게 벗어나면 산포가 작아도 정지로 보지 않는다.
현재 bias 추정값을 그대로 정지 기준으로 쓰면 지속 회전을 bias로 학습하는 문제가 생길 수 있기 때문이다.

각 accepted interval에서 FIFO gyro 예측을 수행한 뒤 정지 조건에 맞춰 ZUPT/ZARU 또는 wheel/NHC를 적용하고 중력 관측을 gate한다.
실제 순서·문턱은 Odometry.step이 소유한다. 바퀴가 비활성화된 placeholder를 zero-motion 관측으로 쓰지 않는다.
한 interval의 형상·유한성·수치 검사를 실패하면 상태/공개 위치·관련 history를 되돌린다.

| 순서 | 실제 처리 | 읽는 함수 |
|---|---|---|
| 1 | 입력 shape·단위 경계·유한성 확인, wheel 속도 환산 | `Odometry._step()` |
| 2 | 정지 판정 및 필요한 startup alignment | `StillnessDetector.update()`, `_collect_alignment()` |
| 3 | FIFO 각 rate의 midpoint 방향 계산과 IMU 예측 | `eskf.predict()` |
| 4a | 정지 유지 조건을 만족하면 ZUPT → ZARU | `update_zupt()`, `update_zaru()` |
| 4b | 정지 갱신 대신 주행 중이면 Wheel → NHC | `update_wheel_velocity()`, `update_nhc()` |
| 5 | accel norm·회전율 gate를 통과한 중력 관측 | `update_gravity()` |
| 6 | 최종 자세로 공개 속도·위치 갱신 | `Odometry._step()` 마지막 부분 |

4a와 4b는 같은 interval에서 함께 실행되지 않는다. 바퀴 관측이 없는 모드에서는 둘 다 생략한다.
정지 판정 `stopped`와 연속 유지까지 통과한 `zupt_active`도 다르다.

BMI088 gyro frame별 시각은 host FIFO-count bracket에서 추정한다. accel clock을 gyro 적분 시계로 대체하지 않는다.
같은 ODR와 한 loop의 read가 accel/gyro 동기화 증거는 아니다.
소비 지연과 실제 표본 유실을 구분하고 각 IMU interval보다 앞선 wheel 관측을 연결한다.
유실·재연결 뒤 보관 pose를 누락 운동의 복원이나 새 정밀 위치로 취급하지 않는다. run/frame/generation과 원본 시각·품질을 함께 기록한다.

### 시계와 표본 나이를 분리하기

IMU의 ODR, 제어 루프 목표 주파수, 화면 전송 주파수는 서로 다른 숫자다.
현재 설정은 BMI088 400 Hz와 필터 목표 100 Hz이며, 이상적인 10 ms에는 gyro 약 4개가 들어간다.
실제 `dt`는 받아들인 표본 구간을 뜻한다. 화면이 10 Hz로 바뀌었다고 IMU 적분 간격을 0.1 s로 바꾸지 않는다.

`gyro_samples`가 있으면 `dt/표본 수`를 각 rate에 배분하고 accel은 그 interval에서 유지한다.
gyro FIFO는 여러 회전 변화가 있던 순서를 보존한다. gyro 평균값 하나만 여러 번 넣는 것과 일반적으로 같지 않다.
현재 BMI088의 accel과 gyro는 별도 취득원이며, 같은 ODR 설정 자체가 두 센서의 동기화를 뜻하지 않는다.

유실 구간을 만난 뒤 `interrupt_interval()`은 마지막 정렬 상태를 보관하고 정지 갱신 이력을 비운다.
보관 상태에서 새 표본으로 재개하는 구간은 ZUPT/ZARU를 끈다. 새 `NEW RUN`은 새 정렬을 만든다.
이 정책은 끊긴 시간을 다음 `dt`에 몰아 넣어 가짜 이동을 만들지 않기 위한 것이다.

`GapErrorBudget`는 별도로 짧은 유실에 대한 조건부 추가 오차를 계산한다.
인접 rate/speed와 config의 margin을 사용해 유실 시간 동안의 각도·거리를 더하고,
이후 거리 L에는 `2 sin(δheading/2) × L`만큼 방향 차이에 의한 추가 위치 차이를 누적한다.
이는 전체 경로의 실제 정확도나 ESKF 공분산이 아니라, 명시한 운동 범위 가정 아래의 유실 영향 지표다.

<a id="12-회전각-계산-예제"></a>

## 8. 회전각 계산 예제

수평에서 실제 각속도 +30°/s, 센서 bias +0.5°/s, 추정 bias +0.5°/s, dt=0.01 s인 300개 구간을 가정한다.

```text
센서값 = 30.5°/s
한 구간의 보정 회전 = (30.5-0.5)·0.01 = 0.3°
3초의 보정 회전 = 300·0.3 = 90°
보정하지 않은 적분 = 30.5·3 = 91.5°
```

단위 변환 후 실제 predictor는 3축 bias를 빼고 회전벡터를 quaternion increment로 합성한다.
3초를 적분하면서 dt를 두 배로 잘못 주면 180°가 된다. 잔여 bias 0.5°/s는 3초에 1.5° 오차를 만든다.
표시 yaw는 ±180°로 감싸므로 한 바퀴 뒤 0°여도 누적 회전은 360°일 수 있다.
평면 누적각의 `wrap180(yawk-yawk-1)` 합산은 각 구간 변화가 180°보다 작고 reset/보정 jump가 없다는 별도 가정이 필요하다.
[learn_turn_angle.py](../rover/learn_turn_angle.py)는 이 이상 표본과 실제 predictor의 일치를 확인하는 오프라인 예제다. 실물 보정/회전 정확도의 증거는 아니다.

프로젝트 루트에서 다음 명령을 실행한다. 이 예제는 센서를 열거나 모터를 구동하지 않으며 새 로그도 만들지 않는다.

```powershell
python -B rover/learn_turn_angle.py
```

```text
Elapsed time:                  3.00 s
Known physical turn:           90.000 deg
Raw gyro integral:             91.500 deg
Bias-corrected integral:        90.000 deg
Production quaternion yaw:     90.000 deg (wrapped)
PASS: scalar integration matches the analytic answer and quaternion heading.
```

### 예제 코드가 실제 predictor에 연결되는 지점

| 예제의 표현 | 하는 일 |
|---|---|
| `true_rate_deg_s`, `sensor_bias_deg_s` | 이상적인 센서 입력을 만들기 위한 참 회전율과 센서 오프셋 |
| `state = eskf.create_state()` | runtime과 같은 명목상태·공분산 생성 |
| `state['bg'][2] = math.radians(...)` | 이미 알고 있다고 가정한 z축 bias를 rad/s로 저장 |
| `gyro_raw = [0, 0, radians(true_rate+bias)]` | bias가 포함된 body gyro 입력 생성 |
| `accel_raw = [0, 0, config.GRAVITY]` | 수평 지지 상태의 비력 입력 생성 |
| `raw_angle_rad += gyro_raw[2] * dt_s` | bias를 제거하지 않은 비교용 적분 |
| `corrected_angle_rad += (gyro_raw[2]-bg[2])*dt_s` | 손계산과 같은 1축 적분 |
| `eskf.predict(state, gyro_raw, accel_raw, dt_s)` | 제품 코드의 3축 회전·내부 상태·공분산 예측 |
| `eskf.get_euler_deg(state)[2]` | quaternion을 사람이 읽는 yaw로 변환 |
| 두 `assert` | 해석적 답과 1축 적분, 1축 적분과 quaternion heading 비교 |

`predict()` 안의 회전 계산은 다음 네 단계다. 실제 파일에서는 사이에 위치·공분산 계산도 있다.

```python
gyro = gyro_raw - state['bg']
dtheta = gyro * dt
dq = small_angle_to_quat(dtheta)
state['q'] = quat_normalize(quat_multiply(state['q'], dq))
```

순수 z축 0.3° 회전이라면 `dq=[cos(0.15°), 0, 0, sin(0.15°)]`다.
반각을 사용하는 이유는 quaternion의 회전 표현이기 때문이며, 최종 회전각을 반으로 줄이는 것이 아니다.
이 예제에는 관측 갱신이 없으므로 초기 입력한 `bg`는 바뀌지 않는다. 실제 runtime의 bias는 정렬과 관측 갱신이 맡는다.

| 가정 변경 | 3초 적분 결과 |
|---|---:|
| 센서 bias와 추정 bias 모두 +0.5°/s | 90° |
| 센서 bias +0.5°/s, 추정 bias 0°/s | 91.5° |
| 센서 bias +0.5°/s, 추정 bias +1°/s | 88.5° |
| 참 회전율 −30°/s, bias 정확 | −90° |

이 표는 코드 입력을 이해하는 계산 예제다. 실제 차량의 회전 명령을 바꾸는 절차가 아니다.

## 9. 보정과 원본 해석

거리 profile은 geometry·wheel 부호·원본/재생 hash가 맞는 실측 정지점으로 단일 배율을 정한다.
[calibrate_wheel_distance.py](../tools/calibrate_wheel_distance.py)는 기존 결과를 덮어쓰지 않으며 모터 명령을 바꾸지 않는다.
선택한 방식·배율·profile hash는 metadata에 남기고 NEW RUN에도 유지한다.
현재 기본 설정은 거리 배율 1이며 자동 선택되는 차체 실측 profile은 없다.

### 거리 배율이 맞추는 것

각 정지점의 추정 종방향 거리 `xi`와 측정 거리 `yi`가 있고 시작점 offset을 0으로 고정할 때,
[odometry_calibration.py](../rover/odometry_calibration.py)의 단일 배율은 다음 제곱오차를 최소화한다.

```text
min Σ(k xi - yi)²
미분: 2 Σ xi(k xi - yi) = 0
k = Σ(xi yi) / Σ(xi²)
```

추정 `[1,2] m`, 측정 `[0.9,1.8] m`인 계산 예제라면 `k=(0.9+3.6)/(1+4)=0.9`다.
이 배율은 추정 거리 환산에 적용되고 모터 명령에는 적용되지 않는다.
출발 offset, 시간창, yaw, 좌우 slip까지 동시에 맞추는 모델도 아니다.

Profile에는 숫자 하나뿐 아니라 wheel radius·track·wheel sign, 사용한 원본·재생 결과의 hash와 자료 역할이 들어간다.
이 정보가 현재 geometry와 맞아야 로드할 수 있으며, 생성 도구는 기존 파일을 덮어쓰거나 만든 profile을 자동 활성화하지 않는다.
같은 표본에 맞춘 오차는 training error다. 이미 tuning에 사용한 기록은 새 독립 검증 자료가 되지 않는다.

9월/10월 차체 치수와 구형 Arduino/BMI088의 축·encoding을 구분한다.
구형 sensor-frame CSV의 body 변환은 replay 경계에서 한 번만 수행한다. 이미 body인 기록에 중복 적용하지 않는다.
Arduino encoding 실험의 계수를 BMI088에 적용하지 않는다. replay가 모든 과거 설정을 자동 복원한다고 가정하지 않는다.
원본·metadata·실제 label·당시 source/config를 함께 보존하며 현재 설정으로 과거 결과를 덮어쓰지 않는다.

줄자로 측정한 종방향 정지점은 lateral/heading 정답이 아니다. 약 90° operator label도 측량 각도가 아니다.
Tuning 자료와 독립 검증 자료를 구분한다. Noise 수치는 입력 clock/단위·장착·온도·하중·slip 조건과 함께 해석한다.
길이 L의 직선에서 일정 heading 오차 δ가 만들 횡오차는 대략 `L·sin(δ)`지만 wheel scale·slip·장착 offset도 별도 오차를 만든다.
IMU/wheel 적분만으로 임의의 긴 경로에서 heading/위치 오차가 제한된다고 보장할 수 없다.

예를 들어 heading이 1° 다른 채 5 m 직진하면 횡방향 차이는 `5 sin(1°) ≈ 0.087 m`다.
속도 scale이 2% 크면 같은 5 m 구간의 종방향 차이는 0.10 m다. 두 원인은 방향과 거리로 다르게 나타나며,
NIS의 숫자 하나만으로 둘을 구별할 수 없다.

## 10. 코드 경계와 수학 참고

| 책임 | 원본 |
|---|---|
| 축·geometry·noise·gate 설정 | [config.py](../rover/config.py) |
| quaternion·F/Q·H·Joseph·오차 주입 | [eskf.py](../rover/eskf.py) |
| 정렬·정지 판정·갱신 순서·공개 적분 | [odometry.py](../rover/odometry.py) |
| 취득/FIFO·process 경계·wheel pairing | [imu_reader.py](../rover/imu_reader.py), [estimation_process.py](../rover/estimation_process.py) |
| CSV·metadata·불완전 capture 해석 | [logger.py](../rover/logger.py), [데이터 계약](13_scout_integration.md) |

ESKF의 국소 오차·quaternion 관례를 더 읽을 때는 Solà의
[Quaternion kinematics for the error-state Kalman filter (2017)](https://arxiv.org/abs/1711.02508)를 참고한다.
문헌의 일반 ESKF와 현재 코드의 yaw 보존·공개 wheel 위치·복구 정책은 구분한다.

<a id="카메라-시야와-영상량-계산"></a>

## 11. 카메라 시야와 영상량 계산

이 절의 목적은 카메라 표에 적힌 화각·화소·fps·bitrate가 차량에서 무엇을 뜻하는지 계산하는 것이다.
현재 부품과 선정 근거는 [하드웨어](06_하드웨어.md)에 있다. 아래 기하 계산은 왜곡 없는 pinhole 투영과
같은 평면에 있는 물체를 가정한다. 실제 광각 렌즈의 왜곡·crop·초점·노출·압축은 별도 영향을 준다.
[OpenCV의 카메라 모델](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)은 이 투영과 렌즈 왜곡을 구분한다.

### 화각: 얼마나 넓게 보이는가

렌즈 중심에서 거리 D인, 광축에 수직인 평면의 가로 시야폭 W는 직각삼각형으로 구한다.

```text
tan(HFOV/2) = (W/2)/D
W = 2 D tan(HFOV/2)
```

대각 화각 dFOV만 있고 같은 영상 영역의 가로·세로 비가 a:b이면:

```text
tan(HFOV/2) = tan(dFOV/2) × a/sqrt(a²+b²)
HFOV = 2 atan(tan(dFOV/2) × a/sqrt(a²+b²))
16:9의 비율 계수 = 16/sqrt(16²+9²) ≈ 0.8716
```

dFOV 120°, 16:9인 계산 예제는 HFOV 약 113.0°를 준다. 이것은 렌즈의 실제 가로 화각 측정값이 아니다.
Fisheye 투영이나 출력 crop이 있으면 위 환산을 그대로 쓰지 않는다.

### GSD: 물체가 몇 픽셀을 차지하는가

같은 평면에 투영한 폭을 가로 화소 수 Nx로 나누면 평균 공간 간격을 얻는다.
여기서는 지면을 내려다보는 경우에만 한정하지 않고 물체 평면의 `mm/px`라는 뜻으로 사용한다.

```text
GSD = W/Nx = 2 D tan(HFOV/2)/Nx
물체의 가로 화소 수 ≈ 물체 폭/GSD
```

HFOV 113°, 출력 가로 1920 px를 가정한 계산표다.

| 거리 D | 시야폭 W | GSD | 폭 30 mm 물체 |
|---:|---:|---:|---:|
| 0.5 m | 약 1.51 m | 약 0.79 mm/px | 약 38 px |
| 2 m | 약 6.04 m | 약 3.15 mm/px | 약 9.5 px |
| 5 m | 약 15.11 m | 약 7.87 mm/px | 약 3.8 px |

이는 물체가 차지할 크기 계산이다. 특정 px 수만으로 인지·식별 성공을 보장하지 않는다.
명암, 움직임 흐림, 화면 가장자리 왜곡, 렌즈 해상력과 압축 때문에 같은 px 수도 다르게 보인다.
전방 바닥은 광축에 수직인 평면이 아니므로 화면 전체에 하나의 GSD를 적용할 수도 없다.

### 픽셀 피치와 디스플레이 PPI

센서 픽셀 피치는 이웃한 픽셀 중심 사이의 거리다. 실제 유효 센서폭 S와 그 폭의 화소 수 Nx를 알고 있을 때:

```text
센서 pixel pitch [µm] = S[mm] × 1000 / Nx
디스플레이 PPI = sqrt(Wpx² + Hpx²) / 대각 길이[inch]
```

피치 2.9 µm와 1.12 µm의 정사각형 픽셀 면적비는 `(2.9/1.12)² ≈ 6.70`이다.
이것은 같은 조도·노출에서의 픽셀 면적 비교다. 양자효율·read noise·렌즈·출력 resize까지 포함한
완성 영상의 SNR이나 품질이 정확히 6.70배라는 뜻은 아니다.
센서의 `1/2.8 inch` 같은 광학 포맷명도 물리 대각 길이 그 자체가 아니므로 피치는 unit-cell 사양을 기준으로 읽는다.

### f-number: 같은 노출에서 얼마나 빛을 받는가

f-number N은 `초점거리/입사동공 지름`이다. 같은 장면·투과율·촬영 배율에서 센서면 조도는
대략 `1/N²`에 비례한다. 같은 노출 시간이라면 받은 빛의 양도 같은 비율로 비교할 수 있다.
[Edmund Optics의 f-number 설명](https://www.edmundoptics.com/knowledge-center/application-notes/imaging/lens-iris-aperture-setting/)
에서 조리개 면적과 이 관계를 설명한다.

```text
E ∝ 1/N²
E(f/1.8) / E(f/2.5) ≈ (2.5/1.8)² ≈ 1.93
```

따라서 다른 조건이 같으면 f/1.8은 f/2.5보다 약 1.93배의 빛을 받아 같은 수광량에 필요한 노출 시간을 줄일 수 있다.
렌즈 투과율·비네팅·초점·수차·피사계 심도도 달라지므로 이 비율이 완성 영상의 선명도나 SNR 배율은 아니다.

### 다이나믹 레인지의 dB 읽기

Dynamic range는 포화 전 최대 신호와 구별 가능한 최소 신호 사이의 범위다.
최소 신호를 noise 진폭 기준으로 정의하고 같은 단위의 신호 진폭비를 쓰는 관례에서는 다음과 같다.

```text
DR[dB] = 20 log10(Smax / σnoise)
진폭비 = 10^(DR[dB]/20)
80 dB  → 10^4 = 10,000:1
100 dB → 10^5 = 100,000:1
```

[EMVA 1288 Linear 4.0](https://www.emva.org/wp-content/uploads/EMVA1288Linear_4.0Release.pdf)는
포화 입력과 SNR=1인 절대 감도 문턱의 비를 DR로 정의하고 `20 log10(DR)`로 dB를 표시한다.
따라서 비교할 때에는 최소 신호의 정의와 gain·노출·온도·HDR mode 등 측정 조건이 같아야 한다.
80 dB라는 숫자만으로 실제 장면의 10,000:1 명암을 화면에서 모두 식별한다고 보장할 수는 없다.

### fps, 노출 시간, 지연은 서로 다르다

```text
프레임 간격 = 1/fps
30 fps → 약 33.3 ms
15 fps → 약 66.7 ms
```

프레임 간격은 새 화면이 생기는 주기다. 전체 조종 지연에는 노출·센서 readout·인코딩·큐·무선·디코딩·표시가 더해진다.
30 fps라도 긴 큐가 있으면 오래된 장면을 보게 되고, 노출이 길면 각각의 프레임에 motion blur가 생긴다.
따라서 fps 하나를 조종 가능·불가능의 절대 경계로 사용하지 않는다.

### 비트레이트를 저장 용량으로 바꾸기

십진 단위로 1 Mbit/s는 1,000,000 bit/s이고, 8 bit가 1 byte다.

```text
저장량[GB] ≈ bitrate[Mbit/s] × 시간[s] / 8 / 1000
```

| 일정 bitrate 가정 | 1분 payload | 30분 payload |
|---:|---:|---:|
| 8 Mbit/s | 60 MB | 1.80 GB |
| 10 Mbit/s | 75 MB | 2.25 GB |
| 12 Mbit/s | 90 MB | 2.70 GB |
| 16 Mbit/s | 120 MB | 3.60 GB |

파일 container overhead·metadata·다른 로그는 이 표에 더해진다. 가변 bitrate라면 실제 평균 bitrate로 계산한다.
같은 bitrate라도 장면의 움직임·질감·codec 설정에 따라 화질은 달라진다.
현재 Rover는 카메라가 만든 같은 H.264 원본을 preview와 독립 녹화 경로로 전달한다.
이 표에 12–16 Mbit/s가 있다고 해서 로컬 녹화만 별도로 화질을 높이는 옵션이 있다는 뜻은 아니다.

<a id="전력질량-예산-계산"></a>

## 12. 전력·질량 예산 계산

부품이나 전원 레일을 바꿀 때 필요한 계산법이다. 실제 레일·전압·부품 구성의 원본은
[하드웨어](06_하드웨어.md)에서 연결한 PCB 프로젝트다. 여기의 예제는 7월 설계 자료의 계산 방식을 설명하며
현재 조립 차량의 소비전력·질량·지속시간 측정값으로 사용하지 않는다.

### 레일마다 전류를 따로 계산하기

전력은 `P=VI`다. 12 V 레일에서 2 A를 쓰는 부하는 24 W이며, 22.2 V 배터리에서 항상 2 A를 쓰는 것이 아니다.
변환 효율 η가 0.9라면 배터리 입력전력은 `24/0.9 ≈ 26.67 W`, 입력전류는 `26.67/22.2 ≈ 1.20 A`다.

```text
Pload = Vout Iout
Pinput = Pload/η
Iinput = Pinput/Vinput
Ptotal = 직결 부하 전력 + Σ(배터리에서 갈라진 각 분기의 입력전력)
```

한 분기에서 변환기가 직렬로 두 번 연결되면 최종 부하를 배터리 입력으로 환산할 때
`Pinput = Pload/(η1 η2)`를 쓴다. 중간 레일에 다른 부하가 있으면 그 부하는 자신이 거치는 변환 단계만 적용한다.
변환기 출력 부하와 그 변환기의 입력 합계를 한 표에서 다시 더하면 이중 계산이다.
예를 들어 Jetson 입력 합계에 이미 USB 카메라가 포함되어 있다면 카메라 전력을 전체 배터리 합계에 다시 추가하지 않는다.
평균 소비전력과 시작·stall·무선 burst의 순간 최대전력도 다른 예산이다.

### 배터리 용량에서 시간 구하기

```text
공칭 에너지[Wh] = 공칭 전압[V] × 용량[Ah]
가용 에너지[Wh] = 공칭 에너지 × 가용 비율
예상 시간[h] = 가용 에너지/평균 배터리 입력전력[W]
```

7월 자료의 가정인 22.2 V, 2.8 Ah, 가용 비율 0.8, 평균 66.5 W를 넣으면:

```text
22.2 × 2.8 = 62.16 Wh
62.16 × 0.8 = 49.728 Wh
49.728/66.5 × 60 ≈ 44.87 min
```

44.9분은 위 가정에서 나온 산술값이다. 현재 구성의 실측 런타임을 의미하지 않는다.
전압 강하·cutoff·온도·배터리 상태·하중에 따라 가용 에너지와 평균 소비전력이 달라진다.

### 질량표의 읽는 법

```text
품목 소계 = 수량 × 단품 질량
전장 합계 = 부품 + PCB + 하네스 + 고정구 + 배터리
기구부 잔여 = 전체 허용 질량 - 이미 집계한 질량
```

`DS`는 제조사 사양, `MEAS`는 실제 측정, `EST`는 추정, `CALC`는 다른 값으로 계산한 항목처럼
근거 종류를 함께 적으면 합계를 해석할 수 있다. PCB와 케이블의 추정치를 포함한 합계는 실측 총질량과 다르다.
과거 자료의 1,330 g 같은 합계는 그 당시 부품 목록에만 대응하며 현재 목록에 자동 승계되지 않는다.
