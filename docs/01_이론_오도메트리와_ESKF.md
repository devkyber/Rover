# 차량 설계와 오도메트리·ESKF 계산

이 문서는 Rover가 바퀴·IMU로 자세와 이동을 계산하는 모델을 설명한다.
수치 설정은 [config.py](../rover/config.py), 식·갱신 순서는 [eskf.py](../rover/eskf.py)와
[odometry.py](../rover/odometry.py)가 원본이다. 이 문서의 계산 예제는 센서 보정값이 아니다.
운전은 [README](../README.md), 실제 배선은 [하드웨어](06_하드웨어.md), 검증 범위는 [한계](05_한계와_로드맵.md)를 따른다.

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

이는 접지·유효 반지름·낮은 slip을 가정한 전진속도 모델이다. encoder 회전은 실제 지면 이동 정답이 아니다.
Rover는 skid-steer이므로 `(vright-vleft)/track`을 기본 yaw-rate 관측으로 사용하지 않는다.
회전 중 scrub이 생기며 유효 회전 track과 물리 track이 같다고 보장할 수 없다.

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

## 3. 국소 자세 오차와 쿼터니언

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
실제 센서의 noise/bias 특성은 설치·온도·진동과 함께 실측해야 한다.

## 5. 관측 모델과 Jacobian

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

## 7. 정렬·갱신 순서·유실 구간

정지 판정은 wheel 속도, gyro/accel 산포, accel norm과 정렬 때 얻은 gyro 기준의 차이를 함께 본다.
작은 분산만으로 실제 정지를 증명할 수 없다. 일정한 회전이나 운반 중 오분류도 독립 확인이 필요하다.
연속 정지 구간의 평균 gyro로 초기 bg를, 평균 accel 방향으로 roll/pitch를 정하고 yaw는 0으로 둔다.

각 accepted interval에서 FIFO gyro 예측을 수행한 뒤 정지 조건에 맞춰 ZUPT/ZARU 또는 wheel/NHC를 적용하고 중력 관측을 gate한다.
실제 순서·문턱은 Odometry.step이 소유한다. 바퀴가 비활성화된 placeholder를 zero-motion 관측으로 쓰지 않는다.
한 interval의 형상·유한성·수치 검사를 실패하면 상태/공개 위치·관련 history를 되돌린다.

BMI088 gyro frame별 시각은 host FIFO-count bracket에서 추정한다. accel clock을 gyro 적분 시계로 대체하지 않는다.
같은 ODR와 한 loop의 read가 accel/gyro 동기화 증거는 아니다.
소비 지연과 실제 표본 유실을 구분하고 각 IMU interval보다 앞선 wheel 관측을 연결한다.
유실·재연결 뒤 보관 pose를 누락 운동의 복원이나 새 정밀 위치로 취급하지 않는다. run/frame/generation과 원본 시각·품질을 함께 기록한다.

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

## 9. 보정과 원본 해석

거리 profile은 geometry·wheel 부호·원본/재생 hash가 맞는 실측 정지점으로 단일 배율을 정한다.
[calibrate_wheel_distance.py](../tools/calibrate_wheel_distance.py)는 기존 결과를 덮어쓰지 않으며 모터 명령을 바꾸지 않는다.
선택한 방식·배율·profile hash는 metadata에 남기고 NEW RUN에도 유지한다. 현재 차체 실측 profile 유무는 한계 문서를 따른다.

9월/10월 차체 치수와 구형 Arduino/BMI088의 축·encoding을 구분한다.
구형 sensor-frame CSV의 body 변환은 replay 경계에서 한 번만 수행한다. 이미 body인 기록에 중복 적용하지 않는다.
Arduino encoding 실험의 계수를 BMI088에 적용하지 않는다. replay가 모든 과거 설정을 자동 복원한다고 가정하지 않는다.
원본·metadata·실제 label·당시 source/config를 함께 보존하며 현재 설정으로 과거 결과를 덮어쓰지 않는다.

줄자로 측정한 종방향 정지점은 lateral/heading 정답이 아니다. 약 90° operator label도 측량 각도가 아니다.
Tuning 자료와 독립 검증 자료를 구분한다. 입력 clock/단위·장착·온도·하중·slip부터 확인한 뒤 noise를 해석한다.
길이 L의 직선에서 일정 heading 오차 δ가 만들 횡오차는 대략 `L·sin(δ)`지만 wheel scale·slip·장착 offset도 별도 오차를 만든다.
IMU/wheel 적분만으로 임의의 긴 경로에서 heading/위치 오차가 제한된다고 보장할 수 없다.

## 10. 코드 경계와 수학 참고

| 책임 | 원본 |
|---|---|
| 축·geometry·noise·gate 설정 | [config.py](../rover/config.py) |
| quaternion·F/Q·H·Joseph·오차 주입 | [eskf.py](../rover/eskf.py) |
| 정렬·정지 판정·갱신 순서·공개 적분 | [odometry.py](../rover/odometry.py) |
| 취득/FIFO·process 경계·wheel pairing | [imu_reader.py](../rover/imu_reader.py), [estimation_process.py](../rover/estimation_process.py) |
| CSV·metadata·불완전 capture 해석 | [logger.py](../rover/logger.py), [데이터 계약](13_scout_integration.md) |

ESKF의 국소 오차 전개는 Solà, Quaternion kinematics for the error-state Kalman filter(2017)를 따른다.
INS 오차/관측가능성은 Groves, Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems(2013),
일관성/NIS는 Bar-Shalom·Li·Kirubarajan, Estimation with Applications to Tracking and Navigation(2001)을 참고한다.
