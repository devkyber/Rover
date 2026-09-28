"""
Offline filter tests. No hardware needed.

    python test_filter.py

Two layers, and the order matters.

  1. JACOBIANS -- every H against finite differences. Pure math.
     Catches sign and convention errors that performance testing never
     will: a wrong sign in a weakly observable direction barely moves the
     numbers. The accel-bias block of the gravity update was wrong and it
     cost 0.07 mm of drift -- invisible to layer 2, obvious to layer 1.

  2. SCENARIOS -- the whole cycle over synthetic trajectories, 10 noise
     seeds each, judged on the WORST seed. Catches behaviour: hold a
     line, close a circle, recover a bias, stay put when stopped.

Layer 1 first. If the math is wrong there is no point measuring how
wrong the behaviour is.

Both layers drive odometry.Odometry -- the same object the live loop and
replay.py use -- so these cannot pass while the rover is broken.
"""
import sys

import numpy as np

import config
import eskf
import imu_reader
import odometry
import sim


# ═══════════════════════════════════════════════════════════════════
# Layer 1 -- measurement Jacobians
# ═══════════════════════════════════════════════════════════════════

def perturb(state, i, eps):
    """
    state (+) eps*e_i  -- the same operator _inject uses.

    Returns a fresh state; the original is untouched.
    """
    s = {k: (v.copy() if isinstance(v, np.ndarray) else v)
         for k, v in state.items()}
    dx = np.zeros(15)
    dx[i] = eps
    s['p'] = s['p'] + dx[0:3]
    s['v'] = s['v'] + dx[3:6]
    s['bg'] = s['bg'] + dx[9:12]
    s['ba'] = s['ba'] + dx[12:15]
    s['q'] = eskf.quat_normalize(eskf.quat_multiply(
        s['q'], eskf.small_angle_to_quat(dx[6:9])))
    return s


def check_model(name, model, state, eps=1e-6, tol=1e-4):
    """Compare the analytic H against a central difference, column by column."""
    z, h, H, _ = model(state)
    m = len(np.atleast_1d(z))
    H_num = np.zeros((m, 15))

    for i in range(15):
        zp, hp, _, _ = model(perturb(state, i, +eps))
        zm, hm, _, _ = model(perturb(state, i, -eps))
        # y = z - h, evaluated at the perturbed NOMINAL state. Note z is
        # re-evaluated too: some observations (gravity) subtract a state
        # estimate from the raw reading, so z itself moves with the state.
        y_plus = np.atleast_1d(zp) - np.atleast_1d(hp)
        y_minus = np.atleast_1d(zm) - np.atleast_1d(hm)
        H_num[:, i] = -(y_plus - y_minus) / (2 * eps)

    err = np.abs(H - H_num).max()
    scale = max(1.0, np.abs(H_num).max())
    ok = err / scale < tol

    print(f"  [{'PASS' if ok else 'FAIL':4s}] {name:24s} "
          f"max |H - H_numeric| = {err:.2e}")

    if not ok:
        bad = np.argwhere(np.abs(H - H_num) > tol * scale)
        seen = set()
        for r, c in bad:
            block = ['dp', 'dv', 'dtheta', 'dbg', 'dba'][c // 3]
            if (r, block) in seen:
                continue
            seen.add((r, block))
            print(f"         row {r}, {block} block: "
                  f"analytic {H[r, c]:+.4f} vs numeric {H_num[r, c]:+.4f}")
    return ok


def make_state():
    """
    A state that is NOT at the trivial point.

    Every H has a term that vanishes at identity attitude or zero
    velocity, so testing at the origin would pass a matrix that is wrong
    everywhere else. Tilt it, spin it, and give it speed.
    """
    state = eskf.create_state()
    state['q'] = eskf.quat_normalize(eskf.small_angle_to_quat(
        np.array([0.20, -0.15, 0.55])))
    state['v'] = np.array([0.83, -0.21, 0.06])
    state['p'] = np.array([1.5, -0.4, 0.02])
    state['bg'] = np.array([0.031, -0.065, -0.036])
    state['ba'] = np.array([0.04, -0.02, 0.07])
    return state



def run_jacobian_tests():
    state = make_state()

    # Raw readings stay FIXED while the nominal state is perturbed --
    # they are what the sensor said, and the sensor does not care what
    # the filter currently believes.
    accel_raw = eskf.quat_to_rot(state['q']).T @ np.array(
        [0.0, 0.0, config.GRAVITY]) + state['ba']
    gyro_raw = np.array([0.01, -0.02, 0.13])

    models = [
        ("wheel velocity", lambda s: eskf.model_wheel_velocity(s, 0.80)),
        ("gravity / tilt", lambda s: eskf.model_gravity(s, accel_raw)),
        ("NHC", lambda s: eskf.model_nhc(s, 0.13)),
        ("ZUPT", lambda s: eskf.model_zupt(s)),
        ("ZARU", lambda s: eskf.model_zaru(s, gyro_raw)),
    ]

    print("-" * 62)
    print("LAYER 1  JACOBIANS -- analytic H vs finite differences")
    print("-" * 62)
    results = [check_model(name, m, state) for name, m in models]
    results.append(check_trace_is_read_only())
    results.append(check_non_heading_updates_preserve_yaw())
    results.append(check_startup_alignment_then_turn())
    results.append(check_sim_respects_command_speed_limit())
    results.append(check_arduino_mounting_transform())
    results.append(check_reported_pose_uses_wheels_and_attitude())
    print(f"  {sum(results)}/{len(results)} passed")
    print()
    return all(results)


def check_arduino_mounting_transform():
    """The temporary Arduino IMU is mounted 90 deg from rover forward."""
    sensor = np.array([2.0, -3.0, 4.0])
    expected_body = np.array([3.0, 2.0, 4.0])
    actual_body = imu_reader.arduino_sensor_to_body(sensor)

    C = np.asarray(config.ARDUINO_IMU_TO_BODY, dtype=float)
    rotation_ok = np.allclose(C @ C.T, np.eye(3)) and np.isclose(
        np.linalg.det(C), 1.0)
    vector_ok = np.allclose(actual_body, expected_body)
    ok = rotation_ok and vector_ok
    print(f"  [{'PASS' if ok else 'FAIL':4s}] Arduino IMU frame -> body "
          f"{sensor} -> {actual_body}")
    return ok


def check_reported_pose_uses_wheels_and_attitude():
    """Reported odometry must follow the encoder speed along filtered heading."""
    odo = odometry.Odometry(startup_alignment_s=0.0)
    odo.state['q'] = eskf.quat_from_rpy(0.0, 0.0, np.pi / 2.0)
    forward_mps = 0.10
    wheels = [
        {
            'id': mid,
            'position_rad': 0.0,
            'velocity_rads': forward_mps / (
                config.WHEEL_RADIUS_M * config.WHEEL_SIGN[i]),
            'current_A': 0.0,
        }
        for i, mid in enumerate(config.MOTOR_IDS)
    ]
    for sample in range(100):
        odo.step(wheels, gyro=np.zeros(3),
                 accel=np.array([0.0, 0.0, config.GRAVITY]), dt=0.01)
        if sample == 49:
            # A pseudo-measurement may correct the internal inertial position
            # through covariance. That correction must not teleport the
            # externally reported, continuous wheel-inertial odometry frame.
            odo.state['p'] += np.array([2.0, -3.0, 1.0])

    expected = np.array([0.0, 0.10, 0.0])
    error = np.linalg.norm(odo.position - expected)
    ok = error < 1e-4
    print(f"  [{'PASS' if ok else 'FAIL':4s}] reported pose = wheel speed x "
          f"filtered attitude error={error:.2e} m")
    return ok


def check_trace_is_read_only():
    """Diagnostics must observe the production filter, never alter it."""
    samples = sim.generate("stop and go", seconds=1.0, seed=7,
                           gyro_noise=0.017, accel_noise=0.076)
    plain = odometry.Odometry(trace=False, startup_alignment_s=0.0)
    traced = odometry.Odometry(trace=True, startup_alignment_s=0.0)
    dt = samples[1]['t'] - samples[0]['t']
    for sample in samples:
        kwargs = dict(gyro=sample['gyro'], accel=sample['accel'], dt=dt)
        plain.step(sample['wheels'], **kwargs)
        traced.step(sample['wheels'], **kwargs)

    keys = ('p', 'v', 'q', 'bg', 'ba', 'P')
    error = max(float(np.max(np.abs(plain.state[key] - traced.state[key])))
                for key in keys)
    trace = traced.last_trace
    names = {stage['name'] for stage in trace['stages']}
    complete = ({'predict', 'wheel', 'nhc', 'zupt', 'zaru', 'gravity'}
                <= names and 'final' in trace)
    ok = error == 0.0 and complete
    print(f"  [{'PASS' if ok else 'FAIL':4s}] trace is read-only       "
          f"max state difference = {error:.2e}")
    return ok


def check_non_heading_updates_preserve_yaw():
    """
    Measurements without a heading reference must not rotate global yaw.

    A dense positive-definite P deliberately creates cross-covariance, so
    the unconstrained Kalman gain has a yaw row even when H does not measure
    heading. This is the failure seen in the 23:47 straight-line log.
    """
    base = make_state()
    rng = np.random.default_rng(41)
    A = rng.normal(scale=0.08, size=(15, 15))
    base['P'] = A @ A.T + np.eye(15) * 1e-4

    R_body = eskf.quat_to_rot(base['q'])
    gravity_reading = (
        R_body.T @ np.array([0.0, 0.0, config.GRAVITY])
        + base['ba']
        + np.array([0.08, -0.04, 0.01])
    )

    def copy_state():
        return {k: (v.copy() if isinstance(v, np.ndarray) else v)
                for k, v in base.items()}

    updates = [
        ('wheel', lambda s: eskf.update_wheel_velocity(s, 1.15)),
        ('NHC', lambda s: eskf.update_nhc(s, yaw_rate=0.0)),
        ('ZUPT', lambda s: eskf.update_zupt(s)),
        ('gravity', lambda s: eskf.update_gravity(
            s, gravity_reading, yaw_rate=0.0)),
    ]

    changes = []
    for name, update in updates:
        state = copy_state()
        before = eskf.get_euler_deg(state)[2]
        update(state)
        after = eskf.get_euler_deg(state)[2]
        change = abs(float((after - before + 180.0) % 360.0 - 180.0))
        changes.append(change)
        print(f"         {name:8s} yaw change = {change:.3e} deg")

    worst = max(changes)
    ok = worst < 1e-9
    print(f"  [{'PASS' if ok else 'FAIL':4s}] non-heading updates preserve yaw "
          f"worst = {worst:.3e} deg")
    return ok


def check_startup_alignment_then_turn():
    """Stationary startup sets tilt/bias, then real gyro yaw still moves."""
    dt = 0.01
    gyro_bias = np.array([0.031, -0.065, -0.036])
    roll_true = np.radians(4.0)
    pitch_true = np.radians(-3.0)
    accel = config.GRAVITY * np.array([
        -np.sin(pitch_true),
        np.sin(roll_true) * np.cos(pitch_true),
        np.cos(roll_true) * np.cos(pitch_true),
    ])
    wheels = [
        {'id': mid, 'velocity_rads': 0.0, 'position_rad': 0.0,
         'current_A': 0.0}
        for mid in config.MOTOR_IDS
    ]

    odo = odometry.Odometry(startup_alignment_s=0.5)
    for _ in range(80):
        odo.step(wheels, gyro=gyro_bias, accel=accel, dt=dt)

    aligned_rpy = odo.rpy_deg
    alignment_ok = (
        odo.ready
        and np.linalg.norm(odo.gyro_bias - gyro_bias) < 1e-10
        and abs(aligned_rpy[0] - 4.0) < 1e-6
        and abs(aligned_rpy[1] + 3.0) < 1e-6
        and abs(aligned_rpy[2]) < 1e-9
    )

    # Turn at +0.5 rad/s for one second. The raw gyro includes the bias;
    # startup alignment must remove only that bias, not the real turn.
    turning_wheels = [dict(w, velocity_rads=1.0) for w in wheels]
    for _ in range(100):
        odo.step(turning_wheels,
                 gyro=gyro_bias + np.array([0.0, 0.0, 0.5]),
                 accel=accel, dt=dt)

    yaw = odo.rpy_deg[2]
    turn_ok = abs(yaw - np.degrees(0.5)) < 0.5
    ok = alignment_ok and turn_ok
    print(f"  [{'PASS' if ok else 'FAIL':4s}] startup alignment + real turn "
          f"rpy={np.round(aligned_rpy, 3)}, final yaw={yaw:.3f} deg")
    return ok


def check_sim_respects_command_speed_limit():
    """Synthetic motion must fit the configured Goal Velocity range."""
    worst = 0.0
    for scenario in sim.SCENARIOS:
        for sample in sim.generate(scenario, seconds=6.0, dt=0.02,
                                   gyro_noise=0.0, accel_noise=0.0):
            worst = max(worst, *(abs(v) for v in
                        odometry.wheel_speeds_mps(sample['wheels'])))
    ok = worst <= config.COMMAND_WHEEL_SPEED_LIMIT_MPS + 1e-12
    print(f"  [{'PASS' if ok else 'FAIL':4s}] simulator fits command limit "
          f"max={worst:.4f}, limit={config.COMMAND_WHEEL_SPEED_LIMIT_MPS:.4f} m/s")
    return ok


# ═══════════════════════════════════════════════════════════════════
# Layer 2 -- behaviour over synthetic trajectories
# ═══════════════════════════════════════════════════════════════════

def run(samples):
    """
    Push simulator samples through the real odometry cycle.

    Calls odometry.Odometry, so a change to the update order cannot pass
    here while breaking the rover. This was once a hand-copied duplicate
    with a comment asking the reader to keep it in sync; there were five
    such copies.
    """
    odo = odometry.Odometry(startup_alignment_s=0.0)
    dt = samples[1]['t'] - samples[0]['t']
    for s in samples:
        odo.step(s['wheels'], gyro=s['gyro'], accel=s['accel'], dt=dt)
    return odo


def wrap_deg(rad):
    """Angle difference in degrees, folded into (-180, 180]."""
    return np.degrees((rad + np.pi) % (2 * np.pi) - np.pi)


N_SEEDS = 10


def check(name, scenario, seconds, max_pos_err, max_yaw_err_deg, **sim_kwargs):
    """
    Run one scenario across N_SEEDS different noise draws and judge the
    WORST one.

    Not one seed. A single run tells you how the filter did against one
    particular sequence of random numbers, and the spread is large: on
    "stop and go" a single seed gave 0.58 m while the worst of ten gave
    1.50 m. Tuning against one seed is fitting to noise.
    """
    pos_errs, yaw_errs = [], []

    for seed in range(N_SEEDS):
        samples = sim.generate(scenario, seconds=seconds, seed=seed,
                               **sim_kwargs)
        odo = run(samples)

        pos = odo.position
        yaw = np.radians(odo.rpy_deg[2])
        true_pos = samples[-1]['true_pos']
        true_yaw = samples[-1]['true_yaw']

        pos_errs.append(float(np.linalg.norm(pos[:2] - true_pos[:2])))
        yaw_errs.append(abs(float(wrap_deg(yaw - true_yaw))))

    worst_pos, worst_yaw = max(pos_errs), max(yaw_errs)
    ok_pos = worst_pos <= max_pos_err
    ok_yaw = worst_yaw <= max_yaw_err_deg

    print(f"{name}   ({N_SEEDS} seeds)")
    print(f"  pos error : mean {np.mean(pos_errs):6.3f}  worst {worst_pos:6.3f} m"
          f"   (limit {max_pos_err})  {'PASS' if ok_pos else 'FAIL'}")
    print(f"  yaw error : mean {np.mean(yaw_errs):6.2f}  worst {worst_yaw:6.2f} deg"
          f" (limit {max_yaw_err_deg})  {'PASS' if ok_yaw else 'FAIL'}")
    print()
    return ok_pos and ok_yaw


def test_zaru_finds_gyro_bias():
    """
    Standing still with a biased gyro. ZARU should recover the bias,
    because when nothing is turning, whatever the gyro reads IS the bias.
    """
    print("ZARU recovers a gyro bias")
    true_bias = np.array([0.0, 0.0, 0.01])

    samples = sim.generate("stop and go", seconds=6.0,
                           gyro_bias=true_bias, gyro_noise=0.0, accel_noise=0.0)
    odo = run(samples)

    err = float(np.linalg.norm(odo.gyro_bias - true_bias))
    ok = err < 0.002

    print(f"  true bias      : {true_bias}")
    print(f"  estimated bias : {np.round(odo.gyro_bias, 5)}")
    print(f"  error          : {err:.6f} rad/s  {'PASS' if ok else 'FAIL'}")
    print()
    return ok



def run_scenario_tests():
    print("-" * 62)
    print("LAYER 2  SCENARIOS -- synthetic trajectories from sim.py")
    print("-" * 62)
    print()

    # Clean sensors: the filter should be nearly exact.
    clean = dict(gyro_noise=0.0, accel_noise=0.0)

    # Realistic sensors -- MEASURED, not guessed. These come from a real
    # 20 s stationary log of the Arduino Nano 33 IoT's LSM6DS3 at 100 Hz
    # (white noise = std of the first difference / sqrt(2), which strips
    # out slow drift). The old guesses were gyro 0.005 / accel 0.05,
    # i.e. the sim believed the gyro was 3.4x quieter than it is, which
    # made every test here optimistic.
    #
    # Re-measure when the BMI088 arrives; it will be much quieter.
    noisy = dict(gyro_noise=0.017, accel_noise=0.076)

    # The same board's MEASURED gyro bias: 0.031/-0.065/-0.036 rad/s,
    # i.e. up to 3.7 deg/s. run() disables startup alignment so these
    # scenarios isolate bias recovery during motion/stops. Their large
    # errors are not predictions for a normally aligned live startup.
    real_bias = dict(gyro_bias=(0.031, -0.065, -0.036), **noisy)

    results = [
        check("1. straight, clean sensors", "straight", 20, 0.01, 0.2, **clean),
        check("2. circle, clean sensors", "circle", 20, 0.02, 0.2, **clean),
        check("3. figure 8, clean sensors", "figure 8", 20, 0.01, 0.2, **clean),
        # 0.15 not 0.10: StillnessDetector needs a full ZUPT_WINDOW
        # (0.2 s) of samples before it will call the rover stopped, so
        # ZUPT engages slightly later in each stop phase than the old
        # instantaneous check did. That delay is the price of a detector
        # that actually works on real hardware -- the instantaneous one
        # fired on 0 of 2000 cycles of a real stationary log.
        check("4. stop and go, ZUPT active", "stop and go", 20, 0.05, 2.0, **noisy),
        check("5. turn in place", "turn in place", 20, 0.01, 2.0, **noisy),
        check("6. circle, noisy sensors", "circle", 20, 0.05, 2.0, **noisy),
        # Slip is not modelled, so the filter believes the wheels and
        # over-reports distance by roughly the slip fraction. This test
        # pins that expected behaviour rather than calling it a bug.
        check("7. straight, 10% wheel slip", "straight", 20, 0.35, 2.0,
              wheel_slip=0.10, **noisy),

        # --- what the real LSM6DS3 bias actually costs -------------
        # 8: the rover stops every few seconds, so ZARU can reduce the
        #    initially uncorrected bias; the allowed yaw error remains large.
        check("8. real gyro bias, WITH stops", "stop and go", 20, 0.35, 15.0,
              **real_bias),
        # 9: same bias, but the rover never stops. ZARU never runs and
        #    the -0.036 rad/s bias integrates to about 41 deg in 20 s.
        #    That large error is the CORRECT answer here, not a bug -- it is why the
        #    mission profile needs periodic pauses, and why the BMI088
        #    (or a heading source) matters. The limit records that.
        check("9. real gyro bias, NO stops", "circle", 20, 0.80, 42.0,
              **real_bias),

        test_zaru_finds_gyro_bias(),
    ]


    print(f"  {sum(results)}/{len(results)} passed")
    print()
    return all(results)


if __name__ == "__main__":
    print("=" * 62)
    print("FILTER TESTS -- offline, no hardware")
    print("=" * 62)
    jac_ok = run_jacobian_tests()
    scen_ok = run_scenario_tests()
    print("=" * 62)
    print(f"jacobians  {'PASS' if jac_ok else 'FAIL'}")
    print(f"scenarios  {'PASS' if scen_ok else 'FAIL'}")
    sys.exit(0 if (jac_ok and scen_ok) else 1)
