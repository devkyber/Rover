"""
15-State Error-State Kalman Filter (ESKF) for rover odometry.

State vector (nominal):
    p   (3)  position [m] in world frame
    v   (3)  velocity [m/s] in world frame
    q   (4)  orientation quaternion [w, x, y, z] (world ← body rotation)
    bg  (3)  gyroscope bias [rad/s]
    ba  (3)  accelerometer bias [m/s²]

Error state (what the Kalman filter actually estimates):
    δp  (3)  position error
    δv  (3)  velocity error
    δθ  (3)  orientation error (small angle, 3 numbers, NOT a quaternion)
    δbg (3)  gyro bias error
    δba (3)  accel bias error
    ────────
    Total: 15 states → P is 15×15

The filter works in two steps:
    1. PREDICT: integrate IMU forward (updates nominal), grow uncertainty (updates P)
    2. UPDATE:  use an observation to correct the error state, then inject into nominal
"""
import numpy as np
import config


# ═══════════════════════════════════════════════════════════════════
# Quaternion helpers
# Convention: q = [w, x, y, z], w is the scalar part
# ═══════════════════════════════════════════════════════════════════

def quat_multiply(p, q):
    """Multiply two quaternions: result = p ⊗ q."""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])


def quat_to_rot(q):
    """Convert unit q [w, x, y, z] to R_world_from_body (3, 3)."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def quat_normalize(q):
    """Normalize quaternion to unit length."""
    return q / np.linalg.norm(q)


def quat_from_rpy(roll, pitch, yaw):
    """Quaternion for ZYX yaw-pitch-roll angles, in radians."""
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    return quat_normalize(np.array([
        cy*cp*cr + sy*sp*sr,
        cy*cp*sr - sy*sp*cr,
        sy*cp*sr + cy*sp*cr,
        sy*cp*cr - cy*sp*sr,
    ]))


def small_angle_to_quat(dtheta):
    """Convert a rotation vector (3,), in radians, to unit q [w, x, y, z].

    Uses the axis-angle exponential, not a first-order approximation.
    The historical name reflects its use for small per-cycle increments.
    """
    angle = np.linalg.norm(dtheta)
    if angle < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    half = angle / 2
    axis = dtheta / angle
    return np.array([np.cos(half), *(axis * np.sin(half))])


def skew(v):
    """3×3 skew-symmetric matrix from vector v. Used for cross products: skew(a) @ b = a × b."""
    return np.array([
        [    0, -v[2],  v[1]],
        [ v[2],     0, -v[0]],
        [-v[1],  v[0],     0],
    ])


# ═══════════════════════════════════════════════════════════════════
# State creation
# ═══════════════════════════════════════════════════════════════════

def create_state():
    """Create initial ESKF state. Rover starts at origin, stationary, flat on ground."""
    state = {
        # Nominal state
        'p':  np.zeros(3),                          # position [m]
        'v':  np.zeros(3),                          # velocity [m/s]
        'q':  np.array([1.0, 0.0, 0.0, 0.0]),      # quaternion (identity = flat)
        'bg': np.zeros(3),                          # gyro bias [rad/s]
        'ba': np.zeros(3),                          # accel bias [m/s²]

        # Error covariance (15×15)
        'P':  np.diag([
            0.01, 0.01, 0.01,      # position uncertainty [m²]
            0.01, 0.01, 0.01,      # velocity uncertainty [(m/s)²]
            0.01, 0.01, 0.01,      # orientation uncertainty [rad²]
            1e-6, 1e-6, 1e-6,      # gyro bias uncertainty [(rad/s)²]
            1e-4, 1e-4, 1e-4,      # accel bias uncertainty [(m/s²)²]
        ]),

        # NIS samples per observation type. Filled in by every update_*;
        # replay.py reads this to judge whether R and Q are honest.
        'nis': {},

        # Disabled in live driving. Replay diagnostics can enable this to
        # expose the exact calculation performed at each filter stage.
        '_trace_enabled': False,
        '_trace': None,
    }
    return state


def _as_list(values):
    return [float(x) for x in np.asarray(values).reshape(-1)]


def _matrix_list(values):
    return [[float(x) for x in row] for row in np.asarray(values)]


def _angle_delta_deg(before, after):
    """Wrapped Euler difference, used only for human-readable diagnostics."""
    return (np.asarray(after) - np.asarray(before) + 180.0) % 360.0 - 180.0


def trace_state(state):
    """Human-readable nominal state and 1-sigma uncertainty snapshot."""
    std = np.sqrt(np.maximum(np.diag(state['P']), 0.0))
    return {
        'p_m': _as_list(state['p']),
        'v_mps': _as_list(state['v']),
        'q_wxyz': _as_list(state['q']),
        'rpy_deg': _as_list(get_euler_deg(state)),
        'bg_deg_s': _as_list(np.degrees(state['bg'])),
        'ba_mps2': _as_list(state['ba']),
        'sigma': {
            'p_m': _as_list(std[0:3]),
            'v_mps': _as_list(std[3:6]),
            'theta_deg': _as_list(np.degrees(std[6:9])),
            'bg_deg_s': _as_list(np.degrees(std[9:12])),
            'ba_mps2': _as_list(std[12:15]),
        },
    }


def trace_state_delta(before, after):
    """What a stage changed in the nominal state, in display units."""
    return {
        'p_m': _as_list(np.asarray(after['p_m']) - before['p_m']),
        'v_mps': _as_list(np.asarray(after['v_mps']) - before['v_mps']),
        'q_wxyz': _as_list(np.asarray(after['q_wxyz']) - before['q_wxyz']),
        'rpy_deg': _as_list(_angle_delta_deg(before['rpy_deg'], after['rpy_deg'])),
        'bg_deg_s': _as_list(np.asarray(after['bg_deg_s']) - before['bg_deg_s']),
        'ba_mps2': _as_list(np.asarray(after['ba_mps2']) - before['ba_mps2']),
    }


def start_trace(state, *, dt, gyro_raw, accel_raw, wheel_mps):
    """Start one optional per-cycle trace. Has zero effect on filter math."""
    level = state.get('_trace_enabled', False)
    if not level:
        state['_trace'] = None
        return
    if level == 'yaw':
        state['_trace'] = {'stages': []}
        return
    state['_trace'] = {
        'input': {
            'dt_s': float(dt),
            'gyro_raw_deg_s': _as_list(np.degrees(gyro_raw)),
            'accel_raw_mps2': _as_list(accel_raw),
            'wheel_mps': _as_list(wheel_mps),
        },
        'stages': [],
    }


def trace_context(state, **values):
    if state.get('_trace') is not None and state.get('_trace_enabled') != 'yaw':
        state['_trace']['context'] = values


def trace_skip(state, name, reason):
    if state.get('_trace') is not None and state.get('_trace_enabled') != 'yaw':
        state['_trace']['stages'].append({
            'name': name, 'applied': False, 'reason': reason,
        })


def finish_trace(state):
    if state.get('_trace') is not None and state.get('_trace_enabled') != 'yaw':
        state['_trace']['final'] = trace_state(state)
    return state.get('_trace')


# ═══════════════════════════════════════════════════════════════════
# PREDICT — propagate nominal state with IMU, grow uncertainty
# ═══════════════════════════════════════════════════════════════════

def predict(state, gyro_raw, accel_raw, dt):
    """
    IMU strapdown integration.

    Args:
        gyro_raw:  measured angular rate [rad/s] in body frame (3,)
        accel_raw: measured acceleration [m/s²] in body frame (3,)
        dt:        time step [s]
    """
    trace_level = state.get('_trace_enabled', False)
    before = (trace_state(state) if trace_level and trace_level != 'yaw' else
              get_euler_deg(state)[2] if trace_level else None)
    P_before = state['P'].copy() if trace_level and trace_level != 'yaw' else None

    # Strip bias from measurements
    gyro  = gyro_raw - state['bg']
    accel = accel_raw - state['ba']

    # Current rotation matrix (world ← body)
    R = quat_to_rot(state['q'])

    # Gravity in world frame (pointing down)
    g_world = np.array([0, 0, -config.GRAVITY])

    # ── Update nominal state ──
    # Position: p += v*dt + 0.5*(R*accel + g)*dt²
    accel_world = R @ accel + g_world
    state['p'] = state['p'] + state['v'] * dt + 0.5 * accel_world * dt**2

    # Velocity: v += (R*accel + g)*dt
    state['v'] = state['v'] + accel_world * dt

    # Orientation: q = q ⊗ exp(gyro*dt)
    dtheta = gyro * dt
    dq = small_angle_to_quat(dtheta)
    state['q'] = quat_normalize(quat_multiply(state['q'], dq))

    # Biases: unchanged (random walk — the noise is in Q)

    # ── Update error covariance P ──
    # State transition matrix F (15×15) for the error state
    F = np.eye(15)
    F[0:3, 3:6]   = np.eye(3) * dt                    # δp depends on δv
    F[3:6, 6:9]   = -R @ skew(accel) * dt              # δv depends on δθ
    F[3:6, 12:15] = -R * dt                             # δv depends on δba
    F[6:9, 6:9]   = np.eye(3) - skew(gyro) * dt        # δθ propagation
    F[6:9, 9:12]  = -np.eye(3) * dt                     # δθ depends on δbg

    # Process noise Q — how much uncertainty one dt of IMU noise adds.
    #
    # The noise densities are per-sqrt-Hz, so over a step of dt the
    # variance grows by  sigma^2 * dt   (NOT (sigma*dt)^2 — that was a
    # bug: at dt=0.01 it made Q 100x too small, so the filter trusted
    # its own attitude far too much and would not let the gravity
    # update correct tilt).
    Q = np.zeros((15, 15))
    Q[3:6, 3:6]     = np.eye(3) * config.ACCEL_NOISE_DENSITY**2 * dt
    Q[6:9, 6:9]     = np.eye(3) * config.GYRO_NOISE_DENSITY**2 * dt
    Q[9:12, 9:12]   = np.eye(3) * config.GYRO_BIAS_RANDOM_WALK**2 * dt
    Q[12:15, 12:15] = np.eye(3) * config.ACCEL_BIAS_RANDOM_WALK**2 * dt

    # P = F P Fᵀ + Q
    state['P'] = F @ state['P'] @ F.T + Q

    if state.get('_trace') is not None:
        if trace_level == 'yaw':
            dyaw = float(_angle_delta_deg([0, 0, before], [0, 0, get_euler_deg(state)[2]])[2])
            state['_trace']['stages'].append({
                'name': 'predict', 'applied': True,
                'state_delta': {'rpy_deg': [0.0, 0.0, dyaw]},
            })
        else:
            after = trace_state(state)
            state['_trace']['stages'].append({
                'name': 'predict',
                'applied': True,
                'terms': {
                    'gyro_corrected_deg_s': _as_list(np.degrees(gyro)),
                    'accel_corrected_mps2': _as_list(accel),
                    'accel_world_mps2': _as_list(accel_world),
                    'rotation_increment_deg': _as_list(np.degrees(dtheta)),
                    'Q_diagonal': _as_list(np.diag(Q)),
                    'F': _matrix_list(F),
                    'Q': _matrix_list(Q),
                },
                'P_before': _matrix_list(P_before),
                'P_after': _matrix_list(state['P']),
                'before': before,
                'after': after,
                'state_delta': trace_state_delta(before, after),
            })


# ═══════════════════════════════════════════════════════════════════
# Generic Kalman update — all observation types call this
# ═══════════════════════════════════════════════════════════════════

def _ekf_update(state, z, h, H, R, name=None, preserve_yaw=False):
    """
    Standard Kalman filter update step.

    Args:
        z: actual measurement (m,)
        h: predicted measurement (m,)
        H: measurement Jacobian (m × 15)
        R: measurement noise covariance (m × m)
        preserve_yaw: keep global yaw unchanged for measurements that do
                      not contain a heading reference
    """
    trace_level = state.get('_trace_enabled', False)
    before = (trace_state(state) if trace_level and trace_level != 'yaw' else
              get_euler_deg(state)[2] if trace_level else None)
    P_before = state['P'].copy() if trace_level and trace_level != 'yaw' else None
    P = state['P']
    y = z - h                                # innovation (measurement - prediction)
    S = H @ P @ H.T + R                      # innovation covariance
    S_inv = np.linalg.inv(S)
    K_raw = P @ H.T @ S_inv                  # unconstrained Kalman gain

    # Wheel speed, NHC, gravity and ZUPT contain no absolute heading
    # reference. Cross-covariance may still put a correction in the
    # attitude row of K. Remove its component around the WORLD vertical.
    #
    # dtheta is expressed in the body frame, so the world vertical in
    # those coordinates is R(q)^T * [0, 0, 1]. A rotation around this axis
    # is precisely the unobservable global-yaw gauge direction.
    K = K_raw
    yaw_before = None
    if preserve_yaw:
        yaw_before = np.radians(get_euler_deg(state)[2])
        vertical_body = quat_to_rot(state['q']).T @ np.array([0.0, 0.0, 1.0])
        vertical_body /= np.linalg.norm(vertical_body)

        correction_projection = np.eye(15)
        correction_projection[6:9, 6:9] -= np.outer(vertical_body, vertical_body)
        K = correction_projection @ K_raw

    dx = K @ y                               # error state correction

    # Apply correction to error covariance
    I_KH = np.eye(15) - K @ H
    state['P'] = I_KH @ P @ I_KH.T + K @ R @ K.T   # Joseph form (numerically stable)

    # Apply correction to nominal state
    _inject(state, dx)

    if preserve_yaw:
        # The projection above preserves yaw to first order. Remove the
        # tiny second-order change from finite quaternion injection too,
        # so this contract is exact and directly testable.
        yaw_after = np.radians(get_euler_deg(state)[2])
        yaw_correction = (yaw_before - yaw_after + np.pi) % (2*np.pi) - np.pi
        q_world_z = np.array([
            np.cos(yaw_correction / 2.0), 0.0, 0.0,
            np.sin(yaw_correction / 2.0),
        ])
        state['q'] = quat_normalize(quat_multiply(q_world_z, state['q']))

    # NIS = Normalized Innovation Squared.
    #
    # This is how we tell whether R and Q are honest, WITHOUT needing to
    # know the true trajectory -- which means it works on real rover logs,
    # not just simulator runs.
    #
    # If the filter's uncertainty S really describes how wrong the
    # prediction is, then NIS follows a chi-square distribution with
    # len(z) degrees of freedom, so its average should be about len(z).
    #
    #   NIS >> len(z)  ->  S too small: we are OVERCONFIDENT.
    #                      R and/or Q are set too low.
    #   NIS << len(z)  ->  S too big: we are throwing away good data.
    #                      R and/or Q are set too high.
    #
    # Collected by name in state['nis'] so replay.py can report per
    # observation type -- it is usually one specific update that is lying.
    nis = float(y @ S_inv @ y)
    if state.get('_trace') is not None:
        if trace_level == 'yaw':
            dyaw = float(_angle_delta_deg([0, 0, before], [0, 0, get_euler_deg(state)[2]])[2])
            state['_trace']['stages'].append({
                'name': name or 'update', 'applied': True,
                'state_delta': {'rpy_deg': [0.0, 0.0, dyaw]},
            })
        else:
            after = trace_state(state)
            state['_trace']['stages'].append({
                'name': name or 'update',
                'applied': True,
                'measurement': {
                    'z': _as_list(z),
                    'h': _as_list(h),
                    'innovation': _as_list(y),
                    'R_sigma': _as_list(np.sqrt(np.maximum(np.diag(R), 0.0))),
                    'S_sigma': _as_list(np.sqrt(np.maximum(np.diag(S), 0.0))),
                    'nis': nis,
                    'H': _matrix_list(H),
                    'R': _matrix_list(R),
                    'S': _matrix_list(S),
                    'K': _matrix_list(K),
                    'K_unconstrained': _matrix_list(K_raw),
                    'preserve_global_yaw': bool(preserve_yaw),
                },
                'correction': {
                    'dp_m': _as_list(dx[0:3]),
                    'dv_mps': _as_list(dx[3:6]),
                    'dtheta_deg': _as_list(np.degrees(dx[6:9])),
                    'dbg_deg_s': _as_list(np.degrees(dx[9:12])),
                    'dba_mps2': _as_list(dx[12:15]),
                },
                'P_before': _matrix_list(P_before),
                'P_after': _matrix_list(state['P']),
                'before': before,
                'after': after,
                'state_delta': trace_state_delta(before, after),
            })
    return nis


def _record(state, name, nis):
    """Stash one NIS sample under its observation name."""
    state['nis'].setdefault(name, []).append(nis)
    return nis


def _inject(state, dx):
    """Inject error-state correction into nominal state."""
    state['p']  = state['p'] + dx[0:3]
    state['v']  = state['v'] + dx[3:6]
    state['bg'] = state['bg'] + dx[9:12]
    state['ba'] = state['ba'] + dx[12:15]

    # Orientation: q = q ⊗ small_angle_to_quat(δθ)
    dq = small_angle_to_quat(dx[6:9])
    state['q'] = quat_normalize(quat_multiply(state['q'], dq))


# ═══════════════════════════════════════════════════════════════════
# OBSERVATIONS — each one calls _ekf_update with the right H and R
# ═══════════════════════════════════════════════════════════════════

def model_wheel_velocity(state, v_wheel_body_x, yaw_rate=0.0):
    """
    Wheel encoder gives forward velocity in body frame.
    Observation: v_x^body = measured wheel speed.

    R grows with |yaw_rate|. On a skid-steer rover the four wheels must
    scrub sideways to turn at all, so "average wheel speed = forward
    speed" is a straight-line model that degrades exactly when the rover
    turns. Measured on the 2026-09-06 logs: mean NIS 1.7 on the two
    straight-line runs, 19.6 on the two runs with a lot of rotation --
    the same filter, the same wheels, a model that stopped applying.
    Same treatment as model_nhc, for the same physical reason.

    Returns (z, h, H, R) and touches nothing -- see the note above
    model_* / update_* at the top of this section.
    """
    R_body = quat_to_rot(state['q'])

    # Predicted body-frame velocity: R^T * v_world
    v_body_predicted = R_body.T @ state['v']

    # We only observe v_x (forward direction)
    z = np.array([v_wheel_body_x])
    h = np.array([v_body_predicted[0]])

    # H: how does v_x^body change with each error state?
    # v_body = Rᵀ v → derivative w.r.t. δv is Rᵀ, w.r.t. δθ involves skew
    H = np.zeros((1, 15))
    H[0, 3:6] = R_body.T[0, :]                        # ∂v_x^body / ∂δv
    H[0, 6:9] = (skew(R_body.T @ state['v']))[0, :]   # ∂v_x^body / ∂δθ

    sigma = (config.R_WHEEL_VELOCITY
             + config.R_WHEEL_TURN_GAIN * abs(yaw_rate))
    R_noise = np.array([[sigma**2]])
    return z, h, H, R_noise


def update_wheel_velocity(state, v_wheel_body_x, yaw_rate=0.0):
    return _record(state, 'wheel', _ekf_update(
        state, *model_wheel_velocity(state, v_wheel_body_x, yaw_rate),
        name='wheel', preserve_yaw=True))


def model_gravity(state, accel_raw):
    """
    When the rover is nearly still, the accelerometer measures gravity.
    This gives us absolute roll and pitch (but NOT yaw).

    Two gates, and BOTH matter:

    1. |accel| must be close to g. Otherwise we are accelerating and the
       reading is not pure gravity.

    2. |yaw_rate| must be small. While turning, centripetal acceleration
       is a sideways specific force of v*omega. It barely changes |accel|
       (it is perpendicular to gravity) so gate 1 does NOT catch it, but
       the filter reads it as roll. That fake roll then leaks into yaw
       through the covariance and wrecks the heading: on the circle test
       it cost 50 degrees of yaw and 0.5 m of position. Gating on yaw
       rate brings that down to 4 degrees / 0.1 m.
    """
    accel = accel_raw - state['ba']
    R_body = quat_to_rot(state['q'])

    # Predicted accelerometer reading when stationary: specific force = -gravity in body frame
    # When flat: accel reads [0, 0, +g] (sensor pushes up against gravity)
    g_up_world = np.array([0, 0, config.GRAVITY])
    g_body_predicted = R_body.T @ g_up_world

    # Measurement: the accelerometer reading IS gravity (when stationary)
    z = accel
    h = g_body_predicted

    # H: how does the innovation change with each error state?
    #
    # Work it out from the sensor model  a_raw = Rᵀ g_up + ba, which is
    # the same model predict() uses (it forms  R(a_raw - ba) + g_world
    # and gets zero when flat and still). With the local error convention
    # q_true = q ⊗ δq:
    #
    #   y = a_raw - ba_nom - Rᵀ_nom g_up
    #     = (Rᵀ_true - Rᵀ_nom) g_up + (ba_true - ba_nom)
    #     = skew(g_body) δθ + I δba
    #
    # so the accel-bias block is +I. It read -I here until 2026-09-06.
    # A stationary log barely notices -- when the rover never accelerates,
    # tilt and accel bias are nearly the same thing to this measurement,
    # so drift moved only 5.28 -> 5.35 mm -- but the sign drove ba the
    # wrong way and would have shown up as soon as the rover moved.
    # test_filter.py now checks every H by finite differences.
    H = np.zeros((3, 15))
    H[:, 6:9] = skew(g_body_predicted)                # ∂y / ∂δθ
    H[:, 12:15] = np.eye(3)                            # ∂y / ∂δba

    # ADAPTIVE R. The gate below only asks "is this close enough to
    # gravity to use at all?"; it does not say how close. R_GRAVITY was
    # measured on a STATIONARY log, where the answer is "3 cm/s^2". While
    # driving, acceleration and chassis vibration put far more specific
    # force on the sensor than that, and a fixed tight R makes the filter
    # trust a reading it should not: measured mean NIS 20 on a gentle
    # drive log, 210 on a lively one, against a target of 3.
    #
    # | |a| - g | is a disturbance heuristic, not the magnitude of linear
    # acceleration. In particular, sideways acceleration can tilt the
    # measured vector with little norm change. Inflate R with this signal;
    # update_gravity also gates on turn rate. Near rest, base tuning applies.
    dev = abs(np.linalg.norm(accel) - config.GRAVITY)
    sigma = config.R_GRAVITY + config.R_GRAVITY_ACCEL_GAIN * dev
    R_noise = np.eye(3) * sigma**2
    return z, h, H, R_noise


def update_gravity(state, accel_raw, yaw_rate=0.0):
    """
    Gated gravity/tilt update. Both gates live HERE, not in the model,
    so that a caller cannot forget one -- see the docstring above.
    """
    accel = accel_raw - state['ba']

    if abs(np.linalg.norm(accel) - config.GRAVITY) > config.GRAVITY_GATE_MAX_DEV:
        trace_skip(state, 'gravity', 'acceleration magnitude outside gravity gate')
        return  # accelerating -- the reading is not just gravity

    if abs(yaw_rate) > config.GRAVITY_GATE_MAX_YAW_RATE:
        trace_skip(state, 'gravity', 'yaw rate outside gravity gate')
        return  # turning -- centripetal force would look like roll

    return _record(state, 'gravity', _ekf_update(
        state, *model_gravity(state, accel_raw),
        name='gravity', preserve_yaw=True))


def model_nhc(state, yaw_rate):
    """
    Non-Holonomic Constraint: v_y^body = 0, v_z^body = 0.
    Assumes negligible body-frame lateral and vertical velocity. Sideways
    slip and loss of ground contact violate this model; it is not a law.

    yaw_rate: current yaw rate [rad/s] — used to soften v_y constraint
              during skid-steer turns.
    """
    R_body = quat_to_rot(state['q'])
    v_body = R_body.T @ state['v']

    # Measurement: we "observe" that v_y and v_z are zero
    z = np.array([0.0, 0.0])
    h = np.array([v_body[1], v_body[2]])

    # H matrix (2×15): derivatives of v_y^body and v_z^body
    H = np.zeros((2, 15))
    H[0, 3:6] = R_body.T[1, :]                        # ∂v_y^body / ∂δv
    H[0, 6:9] = (skew(R_body.T @ state['v']))[1, :]   # ∂v_y^body / ∂δθ
    H[1, 3:6] = R_body.T[2, :]                        # ∂v_z^body / ∂δv
    H[1, 6:9] = (skew(R_body.T @ state['v']))[2, :]   # ∂v_z^body / ∂δθ

    # Adaptive R: soften v_y constraint when turning (skid-steer slides sideways)
    r_vy = config.R_NHC_VY_BASE**2 + (config.R_NHC_VY_TURN_GAIN * yaw_rate)**2
    r_vz = config.R_NHC_VZ**2
    R_noise = np.diag([r_vy, r_vz])
    return z, h, H, R_noise


def update_nhc(state, yaw_rate):
    return _record(state, 'nhc', _ekf_update(
        state, *model_nhc(state, yaw_rate),
        name='nhc', preserve_yaw=True))


def model_zupt(state):
    """
    Zero Velocity Update: when the rover is stopped, velocity = 0.
    Call this only when you're confident the rover is stationary.
    """
    z = np.zeros(3)           # measured velocity = [0, 0, 0]
    h = state['v']            # predicted velocity

    H = np.zeros((3, 15))
    H[:, 3:6] = np.eye(3)    # velocity is directly in the state

    R_noise = np.eye(3) * config.R_ZUPT**2
    return z, h, H, R_noise


def update_zupt(state):
    return _record(state, 'zupt', _ekf_update(
        state, *model_zupt(state), name='zupt', preserve_yaw=True))


def model_zaru(state, gyro_raw):
    """
    Zero Angular Rate Update: when stopped, true angular rate = 0.
    So: gyro_raw = 0 + bias → gyro_raw IS the bias.
    This directly observes gyro bias — critical for yaw accuracy.
    """
    z = gyro_raw                              # the gyro reading
    h = state['bg']                            # predicted: reading = bias

    H = np.zeros((3, 15))
    H[:, 9:12] = np.eye(3)                    # measurement depends on gyro bias

    R_noise = np.eye(3) * config.R_ZARU**2
    return z, h, H, R_noise


def update_zaru(state, gyro_raw):
    return _record(state, 'zaru', _ekf_update(
        state, *model_zaru(state, gyro_raw), name='zaru'))


# ═══════════════════════════════════════════════════════════════════
# Helper: get useful outputs from state
# ═══════════════════════════════════════════════════════════════════

def get_euler_deg(state):
    """Extract roll, pitch, yaw in degrees from quaternion."""
    R = quat_to_rot(state['q'])
    # Roll (rotation around x)
    roll = np.arctan2(R[2, 1], R[2, 2])
    # Pitch (rotation around y)
    pitch = np.arcsin(-np.clip(R[2, 0], -1, 1))
    # Yaw (rotation around z)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def get_position(state):
    """Get position [x, y, z] in meters."""
    return state['p'].copy()


def get_velocity(state):
    """Get velocity [vx, vy, vz] in m/s."""
    return state['v'].copy()
