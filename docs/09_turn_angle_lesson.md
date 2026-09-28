# Lesson 1 — How a gyro reading becomes a turn angle

Goal: explain the calculation in your own words, calculate a simple turn on
paper, and locate the same calculation in the production estimator.

Run from the project root:

```powershell
python -B rover/learn_turn_angle.py
```

This uses ideal generated samples and the actual `eskf.predict()` function.
It does not read sensors, move motors, create logs, or run measurement updates.
It isolates the prediction stage so you can understand it before the full cycle.

## 1. Begin with rotational kinematics

For a level rover rotating about the vertical axis:

```text
gyro measurement = true angular rate + gyro bias + noise
estimated angular rate = gyro measurement - estimated bias
angle increment = estimated angular rate × sample interval
new angle = previous angle + angle increment
```

In symbols, `Δψ ≈ (ω_measured,z − b_estimated,z) Δt`. Units are
`(rad/s) × s = rad`. A gyroscope measures angular rate, not angle.

Use these values:

| Quantity | Value |
|---|---:|
| True angular rate | +30 degrees/s |
| Sensor bias | +0.5 degrees/s |
| Measured angular rate | +30.5 degrees/s |
| Estimated bias | +0.5 degrees/s |
| Time interval | 0.01 s |
| Number of intervals | 300 |
| Elapsed time | 3 s |

One corrected increment is `(30.5 − 0.5) × 0.01 = 0.3 degrees`.
After 300 increments the answer is 90 degrees. Integrating the raw reading
instead gives `30.5 × 3 = 91.5 degrees`.

Positive yaw follows the right-hand rule about +z. With this project's body
axes (+x forward, +y left, +z up), a level positive-yaw turn is counterclockwise
when viewed from above.

## 2. The executable, line by line

Open [learn_turn_angle.py](../rover/learn_turn_angle.py) next to this table.
Line numbers refer to the lesson as introduced in this cleanup; the expressions
also identify the relevant lines if it is edited later.

| Lines | What each statement does |
|---|---|
| 1–6 | The triple-quoted module description. It becomes the module's documentation; it performs no sensor work. |
| 7 | `import math` makes standard mathematical functions available as `math.<name>`. |
| 9 | `import numpy as np` loads NumPy under a short conventional alias. |
| 11 | `import config` reads this project's constants; this lesson uses its gravity value. |
| 12 | `import eskf` loads the production estimator's functions without running a hardware loop. |
| 15 | `def main():` defines a function. Its indented body runs when `main()` is called. |
| 16 | A comment describing how to use the experiment. Comments do not execute. |
| 17 | Set the known physical rate in degrees per second. `30.0` is a floating-point number. |
| 18 | Set the simulated sensor's bias. This is the error the artificial sensor adds. |
| 19 | Set what the estimator believes the bias is. This may differ from the sensor's actual bias. |
| 20 | Set the interval in seconds, not milliseconds. |
| 21 | Set how many equal intervals to simulate. `300` is an integer. |
| 23 | `create_state()` returns a dictionary holding `p`, `v`, `q`, biases, covariance and diagnostics. |
| 24 | `state['bg']` selects the gyro-bias array; `[2]` selects its z component. `math.radians()` converts degrees to radians. |
| 25 | Build a three-component gyro vector: zero x/y rates and the biased z rate, converted to rad/s. |
| 26 | Build the level accelerometer reading `[0,0,+g]`. A stationary supported accelerometer measures upward specific force. In `predict`, adding world gravity `[0,0,−g]` cancels it for this level example. |
| 27 | Initialize the raw integrated angle to zero radians. |
| 28 | Initialize the corrected integrated angle independently. |
| 30 | `range(steps)` yields 300 loop indices. `_` says we do not use their values. Indentation groups lines 31–33 into this loop. |
| 31 | `+=` adds one raw rate-times-time increment to the accumulated raw angle. |
| 32 | Subtract the estimated z bias, multiply by the interval, and accumulate the corrected angle. |
| 33 | Call the production predictor. It changes `state` in place: orientation, internal translation and covariance. Its bias estimate stays constant here because there are no measurement updates. |
| 35 | Compute elapsed time as number of intervals times interval length. This is simulated time, not how long the program takes to run. |
| 36 | Compute the known physical turn independently from the true rate and elapsed time. |
| 37 | Convert the corrected integral from radians to degrees for reporting. |
| 38 | Extract `(roll,pitch,yaw)` from the quaternion, then select index 2, yaw. |
| 39 | Compute the analytic corrected integral, including any difference between actual and estimated bias. |
| 40 | Compare two headings modulo 360 degrees. `%` is the remainder operator; adding/subtracting 180 places the difference in `[-180,180)`. |
| 42 | Check numerical integration against the analytic result, allowing tiny floating-point rounding. `assert` raises an error if the condition is false. |
| 43 | Check that quaternion heading and the scalar integral describe the same direction. This permits a difference of whole revolutions. |
| 44–48 | Print results. An `f"..."` string inserts values inside `{...}`; `:.3f` displays three decimal places and does not change the stored value. |
| 49 | Print the explanation of what the two checks established. |
| 52–53 | Execute `main()` only when running this file directly, not when importing it. |

Blank lines separate ideas and have no runtime effect. The `-B` command option
suppresses creation of Python bytecode cache files; it does not disable assertions.

Expected default output:

```text
Elapsed time:                  3.00 s
Known physical turn:           90.000 deg
Raw gyro integral:             91.500 deg
Bias-corrected integral:        90.000 deg
Production quaternion yaw:     90.000 deg (wrapped)
PASS: scalar integration matches the analytic answer and quaternion heading.
```

Passing proves agreement for this generated level-turn example. It does not
prove that a real sensor is calibrated or that the rover turns accurately.

## 3. Find the same idea in production code

Inside [eskf.predict()](../rover/eskf.py), the relevant calculation is:

```python
gyro = gyro_raw - state['bg']
dtheta = gyro * dt
dq = small_angle_to_quat(dtheta)
state['q'] = quat_normalize(quat_multiply(state['q'], dq))
```

Other prediction statements occur between these lines in the source.

1. Subtract a **three-axis** bias vector. Both arrays have shape `(3,)` and units rad/s.
2. Multiply each component by seconds to obtain a rotation vector in radians.
3. Convert that local rotation vector into a quaternion increment. Despite the
   historical function name, it uses sine/cosine axis-angle conversion, with an
   identity return for a very small norm.
4. Compose the old body-to-world orientation with the local increment, then
   normalize to unit length to control numerical roundoff.

For a pure z rotation through angle `θ`, the increment is
`[cos(θ/2), 0, 0, sin(θ/2)]` in this code's `[w,x,y,z]` ordering.
Quaternion multiplication is order-dependent, just like composing 3D rotations.

The final displayed yaw is computed by `get_euler_deg()` using
`atan2(R[1,0], R[0,0])`. The heading comes from the orientation, not directly from
the commanded joystick turn or a wheel-difference angle.

The scalar integral is an introductory model for a level, pure-yaw turn. During
general 3D motion, body z angular rate is **not** equal to Euler yaw rate. That is
why the production predictor integrates a three-dimensional orientation.

## 4. Where does the real bias estimate come from?

This lesson supplies a known bias estimate by hand. Normal operation uses
[Odometry._collect_alignment()](../rover/odometry.py):

1. The stillness detector checks wheel speed and IMU statistics.
2. Once stillness is detected, accumulate a contiguous stationary interval. The
   configured duration is five seconds; detector warm-up adds some startup time.
3. Average the gyro readings. Under the stationary assumption, the mean estimates bias.
4. Estimate roll and pitch from mean accelerometer direction; define the initial
   heading as zero because gravity does not reveal heading.
5. Begin prediction and measurement updates on subsequent cycles. Later stationary
   ZARU updates can refine the bias. They do not provide a compass heading.

If the sensor rotates steadily while supposedly stationary, a mean gyro reading
can contain real rotation. Stillness classification is therefore a physical
assumption to validate, not just a Boolean that can be trusted automatically.

Residual bias `b_error` gives approximately `angle_error = b_error × time` in
the level constant-bias example. Averaging at startup reduces the initial error;
it cannot guarantee zero future drift as temperature, vibration and bias change.

## 5. Heading versus accumulated turn

Two quantities answer different questions:

| Quantity | Question | After one positive full turn |
|---|---|---:|
| Wrapped yaw | Which direction am I facing relative to the initial frame? | Approximately 0 degrees |
| Accumulated turn | How much signed rotation occurred along this path? | +360 degrees |

An orientation alone does not encode how many revolutions occurred. The lesson's
scalar accumulator retains them. The production Euler display wraps around
±180 degrees. Replay also accumulates stage-wise yaw changes for diagnostics;
those changes include any estimator corrections, not only physical rotation.

To accumulate from a sampled heading, the usual planar calculation is:

```text
increment = wrap180(current_yaw - previous_yaw)
accumulated_turn += increment
```

This assumes less than 180 degrees of heading change between samples. A large
sample gap, reset, or estimate correction needs explicit handling. It is a next
feature to design and validate, not a claim that this cleanup added a live turn counter.

## 6. Predict, then experiment

Edit only the named lesson constants, run it, and restore the defaults afterward.

| Change | Predict these values before running |
|---|---|
| `estimated_bias_deg_s = 0.0` | Physical turn 90; raw integral 91.5; corrected integral and quaternion yaw 91.5 degrees. |
| `steps = 1200` | Physical/corrected turn 360; raw integral 366; quaternion yaw approximately 0 degrees. |
| `true_rate_deg_s = -30.0` | Physical/corrected turn −90; raw integral −88.5 degrees. |
| `dt_s = 0.02`, keeping 300 steps | Simulated duration becomes 6 s, so the corrected angle becomes 180 degrees. It is a different experiment duration. |
| `dt_s = 0.02` and `steps = 150` | Duration returns to 3 s and corrected angle to 90 degrees. |

For a timing-error thought experiment, keep the **physical** duration at 3 s but
tell the integrator each of its 300 intervals lasted 0.02 s. It would report 180
degrees for a true 90-degree turn. This is different from correctly simulating
six seconds in the fourth row above.

## 7. What a real turn experiment must establish next

Use an independently measured initial and final orientation, such as a marked
reference fixture or a calibrated overhead view. A command to turn 90 degrees is
not a measurement that the rover actually turned 90 degrees.

Measure left and right turns, including 90, 180 and full 360 degrees, with a
stationary startup interval. Record timestamped gyro data, estimated bias and
heading. Compare the heading at the physical end of motion and after the stop
updates settle; this separates integration error from later estimator correction.
Use continuous heading/turn tracking for the full-revolution check.

Choose the acceptable angular error from the mission's navigation requirement
before tuning against the measurements. Existing simulation passes do not supply
that requirement or replace this physical validation.

**Self-check:** without looking at the source, explain why a +0.5 degrees/s bias
produces +1.5 degrees in 3 seconds, why +g appears in the generated accelerometer
reading, and why a full turn can end with yaw zero.
