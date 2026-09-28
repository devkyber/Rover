"""
Synthetic rover + IMU simulator, with ground truth.

We drive a virtual rover along a known 2D path, then work out what the
sensors WOULD have read. That gives us data to tune the filter against
before any hardware exists - and unlike real data, we know the answer.

Everything is 2D (flat ground, no roll/pitch) except the gravity vector,
which the accelerometer always sees.
"""
import numpy as np
import config


# ── Scenarios: each returns (forward_speed, yaw_rate) at time t ────

# The default Goal Velocity limit corresponds to about 0.202 m/s with the
# measured 62.5 mm wheel. This is a COMMAND bound, not a claim about the
# loaded rover's achievable speed. Both sides stay below it in a turn:
# 0.15 + 0.2 * (0.485 / 2) = 0.1985 m/s on the outer wheels.
DRIVE_SPEED_MPS = 0.15
TURN_RATE_RAD_S = 0.20

def _straight(t):
    return DRIVE_SPEED_MPS, 0.0


def _circle(t):
    return DRIVE_SPEED_MPS, TURN_RATE_RAD_S


def _stop_go(t):
    """Drive 3 s, stop 3 s, repeat. Exercises ZUPT/ZARU."""
    phase = t % 6.0
    return (DRIVE_SPEED_MPS, 0.0) if phase < 3.0 else (0.0, 0.0)


def _figure8(t):
    """Two circles in opposite directions. Yaw should come back to ~0."""
    omega = TURN_RATE_RAD_S if (t % 20.0) < 10.0 else -TURN_RATE_RAD_S
    return DRIVE_SPEED_MPS, omega


def _turn_in_place(t):
    """Pure rotation. Wheel odometry says 0 forward, gyro says spinning."""
    return 0.0, 0.5


SCENARIOS = {
    "straight":      _straight,
    "circle":        _circle,
    "stop and go":   _stop_go,
    "figure 8":      _figure8,
    "turn in place": _turn_in_place,
}


# ── The simulator ──────────────────────────────────────────────────

def generate(scenario="circle", seconds=20.0, dt=0.01,
             gyro_bias=(0.0, 0.0, 0.0), accel_bias=(0.0, 0.0, 0.0),
             gyro_noise=0.005, accel_noise=0.05,
             wheel_slip=0.0, seed=42):
    """
    Returns a list of sample dicts, one per timestep:

        {'t', 'accel', 'gyro', 'wheels', 'true_pos', 'true_yaw', 'true_vel'}

    gyro_bias / accel_bias : constant offset added to the sensor (rad/s, m/s^2)
    gyro_noise / accel_noise : white noise std dev (rad/s, m/s^2)
    wheel_slip : 0.0 = wheels perfect, 0.1 = wheels over-report by 10%
    """
    rng = np.random.default_rng(seed)
    profile = SCENARIOS[scenario]

    gyro_bias  = np.asarray(gyro_bias, dtype=float)
    accel_bias = np.asarray(accel_bias, dtype=float)

    n = int(seconds / dt)
    samples = []

    pos = np.zeros(3)       # true world position
    yaw = 0.0               # true heading
    v_prev = np.zeros(3)    # true world velocity, previous step

    for k in range(n):
        t = k * dt
        v_fwd, omega = profile(t)

        # ── true motion in the world frame ─────────────────────────
        c, s = np.cos(yaw), np.sin(yaw)
        v_world = np.array([v_fwd * c, v_fwd * s, 0.0])
        a_world = (v_world - v_prev) / dt
        v_prev = v_world

        # ── what the IMU would read ────────────────────────────────
        # Accelerometer measures SPECIFIC FORCE: true acceleration minus
        # gravity, expressed in the body frame. Sitting flat -> [0,0,+g].
        R_wb = np.array([[c, -s, 0.0],
                         [s,  c, 0.0],
                         [0.0, 0.0, 1.0]])          # body -> world
        f_world = a_world + np.array([0.0, 0.0, config.GRAVITY])
        accel = R_wb.T @ f_world
        gyro  = np.array([0.0, 0.0, omega])

        accel = accel + accel_bias + rng.normal(0, accel_noise, 3)
        gyro  = gyro  + gyro_bias  + rng.normal(0, gyro_noise, 3)

        # ── what the encoders would read ───────────────────────────
        half = config.WHEELBASE_M / 2.0
        v_left  = (v_fwd - omega * half) * (1.0 + wheel_slip)
        v_right = (v_fwd + omega * half) * (1.0 + wheel_slip)

        wheels = []
        for i, mid in enumerate(config.MOTOR_IDS):
            # every wheel is on exactly one side; left/right come from config
            v_side = v_left if i in config.WHEEL_LEFT else v_right
            rads = v_side / config.WHEEL_RADIUS_M * config.WHEEL_SIGN[i]
            wheels.append({'id': mid,
                           'position_rad':  rads * t,
                           'velocity_rads': rads,
                           'current_A':     0.3})

        samples.append({'t': t,
                        'accel': accel,
                        'gyro': gyro,
                        'wheels': wheels,
                        'true_pos': pos.copy(),
                        'true_yaw': yaw,
                        'true_vel': v_world.copy()})

        # ── advance the truth ──────────────────────────────────────
        pos = pos + v_world * dt
        yaw = yaw + omega * dt

    return samples


if __name__ == "__main__":
    for name in SCENARIOS:
        s = generate(name, seconds=5.0)
        end = s[-1]['true_pos']
        print(f"{name:15s} -> ends at "
              f"[{end[0]:+6.2f}, {end[1]:+6.2f}] m, "
              f"yaw {np.degrees(s[-1]['true_yaw']):+7.1f} deg")
