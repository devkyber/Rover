"""
The odometry cycle: one function that turns one set of sensor readings
into an updated estimate.

    odo = Odometry()
    odo.step(wheels, gyro=gyro, accel=accel, dt=dt)
    odo.position, odo.rpy_deg, odo.stopped

WHY THIS FILE EXISTS
--------------------
This sequence used to be copy-pasted in FIVE places -- the recording
loop, the teleop loop, the replay tool, the simulator tests and the GUI
tuner. One of them even carried a comment asking the reader to keep them
in sync. They agreed only because nobody had edited one of them yet.

The day they diverge, replay stops telling you what the rover actually
did, and every tuning decision made from a log becomes wrong in a way
nothing would flag.

So there is exactly one copy, and everything calls it. If you change the
update order or add an observation, you change it HERE and live, replay
and simulation all move together.

WHY THE ORDER IS WHAT IT IS
---------------------------
    predict            IMU always moves the estimate forward first.

    stopped?           Wheels + IMU steadiness, with a gyro-rate veto after
                       stationary alignment supplies a fixed bias reference.

    ZUPT + ZARU        Only when stopped. ZARU is the only thing that
      or               removes gyro bias, and bias is what dominates
    wheel + NHC        long-run error, so standing still is genuinely
                       productive for this filter.

    gravity            Always attempted; it gates itself on acceleration
                       and yaw rate internally so no caller can forget.

The two branches are exclusive on purpose: while stopped, wheel speed is
zero anyway and NHC would add nothing, and running both would double-count
the same "not moving" evidence.
"""
from collections import deque

import numpy as np

import config
import eskf


# ── wheels -> body motion ──────────────────────────────────────────

def _checked_distance_scale(value):
    """Validate an explicit wheel-distance calibration without coercing bools."""
    try:
        scale = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError('Wheel distance scale must be a finite positive scalar') from exc
    if (np.asarray(value).dtype.kind == 'b' or scale.shape != ()
            or not np.isfinite(scale) or scale <= 0):
        raise ValueError('Wheel distance scale must be a finite positive scalar')
    return float(scale)


def wheel_speeds_mps(wheels, *, distance_scale=1.0):
    """Per-wheel estimation speed, sign-corrected so positive = forward."""
    distance_scale = _checked_distance_scale(distance_scale)
    return [w['velocity_rads'] * config.WHEEL_SIGN[i] * config.WHEEL_RADIUS_M * distance_scale
            for i, w in enumerate(wheels)]


def body_velocity(wheels, *, distance_scale=1.0):
    """Forward speed of the body = average of all wheels."""
    v = wheel_speeds_mps(wheels, distance_scale=distance_scale)
    return sum(v) / len(v)


# There is deliberately NO wheel-derived yaw rate here.
#
# The differential-drive formula (v_right - v_left) / track is optimistic
# on a skid-steer rover -- the wheels scrub sideways in a turn, so the
# real yaw rate is smaller than the wheels claim. Every yaw rate in this
# file comes from the gyro, bias-corrected. See docs/01 3.3.


# ── "is it standing still?" ────────────────────────────────────────

class StillnessDetector:
    """
    Decides when ZUPT and ZARU may run.

    NOT |gyro| < threshold. That was the first version and on real
    hardware it fired on 0 of 2000 stationary cycles: the LSM6DS3 sits
    at a 0.08 rad/s bias, four times the threshold, while the rover was
    provably still (per-axis std 0.004). Yaw then drifted 31 deg in 20 s.

    It is a deadlock -- removing the bias needs ZARU, ZARU needs the gyro
    to look small, and it cannot look small until the bias is removed.

    The way out is to stop asking "is the rate near zero?" and ask "is
    the rate CHANGING?". A stationary gyro reads a steady wrong number.
    Standard deviation sees that and does not care about bias at all.

    A mutable filter bg is not a trustworthy rate veto: wheel/NHC/gravity
    can change it before zero-rate updates converge. Instead, once startup
    alignment finishes, retain its raw gyro mean as a separate fixed
    reference. A steady external rotation then cannot be learned as bias
    merely because wheel speed and gyro variance are small. Rotation below
    the configured envelope remains indistinguishable from bias drift.

    Before that reference exists, steady rotation and unknown constant bias
    cannot be distinguished with these sensors. Startup still requires the
    operator to keep the rover stationary; low variance is not proof of rest.

    Thresholds come from a real stationary log, not a datasheet.
    """

    def __init__(self, window=None):
        n = window or config.ZUPT_WINDOW
        self.gyro_window = deque(maxlen=n)
        self.accel_window = deque(maxlen=n)
        self.reference_gyro = None
        self.last_metrics = {'window': 0, 'ready': False, 'result': False}

    def set_reference(self, gyro):
        """An explicitly stationary alignment mean, independent of filter bg."""
        reference = np.array(gyro, dtype=float, copy=True)
        if reference.shape != (3,) or not np.isfinite(reference).all():
            raise ValueError('Invalid stationary gyro reference')
        self.reference_gyro = reference

    def update(self, wheels, gyro, accel):
        self.gyro_window.append(np.array(gyro, dtype=float, copy=True))
        self.accel_window.append(np.array(accel, dtype=float, copy=True))
        # Preserved replay bundles predate the named reference envelope.
        reference_limit = getattr(config, 'ZUPT_GYRO_MEAN_DELTA_MAX', config.ZUPT_GYRO_STD_MAX)

        wheel_max = max(abs(w['velocity_rads']) for w in wheels)
        metrics = {
            'window': len(self.gyro_window),
            'window_required': self.gyro_window.maxlen,
            'wheel_max_rads': float(wheel_max),
            'wheel_limit_rads': float(config.ZUPT_WHEEL_SPEED_MAX),
            'accel_norm_error_mps2': float(abs(np.linalg.norm(accel) - config.GRAVITY)),
            'accel_norm_limit_mps2': float(config.ZUPT_ACCEL_DEV_MAX),
            'ready': len(self.gyro_window) == self.gyro_window.maxlen,
            'gyro_reference_available': self.reference_gyro is not None,
            'gyro_reference_delta_rad_s': None,
            'gyro_reference_limit_rad_s': reference_limit,
        }

        if len(self.gyro_window) < self.gyro_window.maxlen:
            metrics['result'] = False
            self.last_metrics = metrics
            return False        # std means nothing until the window fills

        gyro_std = np.std(np.array(self.gyro_window), axis=0).max()
        accel_std = np.std(np.array(self.accel_window), axis=0).max()
        reference_delta = (None if self.reference_gyro is None else
                           float(max(np.max(np.abs(np.mean(self.gyro_window, axis=0) - self.reference_gyro)),
                                     np.max(np.abs(gyro - self.reference_gyro)))))
        metrics.update({
            'gyro_std_max_rad_s': float(gyro_std),
            'gyro_std_limit_rad_s': float(config.ZUPT_GYRO_STD_MAX),
            'accel_std_max_mps2': float(accel_std),
            'accel_std_limit_mps2': float(config.ZUPT_ACCEL_STD_MAX),
            'gyro_reference_delta_rad_s': reference_delta,
        })
        result = bool(
            wheel_max <= config.ZUPT_WHEEL_SPEED_MAX
            and gyro_std <= config.ZUPT_GYRO_STD_MAX
            and accel_std <= config.ZUPT_ACCEL_STD_MAX
            and metrics['accel_norm_error_mps2'] <= config.ZUPT_ACCEL_DEV_MAX
            and (reference_delta is None or reference_delta <= reference_limit)
        )
        metrics['result'] = result
        self.last_metrics = metrics
        return result


# ── the cycle ──────────────────────────────────────────────────────

class GapErrorBudget:
    """Additional manual-driving error from skipped motion, not total accuracy.

    No missing samples are invented. For a short timing loss, assume the
    missing angular speed is at most the larger adjacent corrected rate plus
    the configured margin; similarly for wheel-derived forward speed. Charge
    angle = rate * gap and distance = speed * gap, then charge subsequent
    travel by 2*sin(angle_error/2)*distance. Sum losses, never average them or
    clear them merely because fresh samples arrive. The envelope is a model
    assumption, not a measured bound; precision/scientific validity is separate.
    """

    def __init__(self, position_available=True):
        self.position_available = bool(position_available)
        self.count = 0
        self.duration_s = 0.0
        self.heading_deg = 0.0
        self.position_m = 0.0
        self.unknown = False
        self.last_reason = None

    @staticmethod
    def timing_loss(reason):
        return reason in ('gyro FIFO count inconsistent with host elapsed time',
                          'acquisition gap exceeds limit')

    def charge(self, duration, previous_gyro, next_gyro, bias,
               previous_speed, next_speed, reason, *, timestamp_uncertainties=(0., 0.)):
        self.count += 1
        self.last_reason = reason
        try:
            rates = np.asarray([previous_gyro, next_gyro, bias], dtype=float)
            values = np.asarray([duration, previous_speed, next_speed], dtype=float)
            timing = np.asarray(timestamp_uncertainties, dtype=float)
        except (ValueError, TypeError):
            self.unknown = True
            return
        if (rates.shape != (3, 3) or not np.isfinite(rates).all()
                or values.shape != (3,) or not np.isfinite(values).all()
                or timing.shape != (2,) or not np.isfinite(timing).all() or np.any(timing < 0)
                or duration < 0 or duration + timing.sum() > config.IMU_GAP_MAX_TOLERATED_S
                or not self.timing_loss(reason)):
            self.unknown = True
            return
        # Charge the upper end of the host timestamp brackets as well.
        duration = float(duration + timing.sum())
        self.duration_s += duration
        rate = max(np.linalg.norm(rates[0] - rates[2]), np.linalg.norm(rates[1] - rates[2]))
        self.heading_deg += (float(np.degrees(rate)) + config.IMU_GAP_RATE_MARGIN_DPS) * duration
        if self.position_available:
            self.position_m += (max(abs(previous_speed), abs(next_speed)) + config.IMU_GAP_SPEED_MARGIN_MPS) * duration

    def advance(self, distance_m):
        # Heading uncertainty continues to affect the wheel-integrated path
        # even after the IMU stream becomes perfect again.
        if self.count and not self.unknown and self.position_available:
            if not np.isfinite(distance_m):
                self.unknown = True
                return
            angle = min(np.pi, np.radians(self.heading_deg))
            self.position_m += 2.0 * np.sin(angle / 2.0) * abs(distance_m)

    def snapshot(self):
        known = bool(self.count and not self.unknown)
        return {
            'scope': 'additional error from skipped motion; conditional motion envelope, not total pose accuracy',
            'evaluated_gaps': self.count, 'known': known,
            'within_budget': bool(known and self.heading_deg <= config.IMU_GAP_HEADING_BUDGET_DEG
                                  and (not self.position_available or self.position_m <= config.IMU_GAP_POSITION_BUDGET_M)),
            'heading_error_deg': self.heading_deg if known else None,
            'position_error_m': self.position_m if known and self.position_available else None,
            'position_budget_applicable': self.position_available,
            'evaluated_duration_s': self.duration_s,
            'heading_budget_deg': config.IMU_GAP_HEADING_BUDGET_DEG,
            'position_budget_m': config.IMU_GAP_POSITION_BUDGET_M,
            'rate_margin_dps': config.IMU_GAP_RATE_MARGIN_DPS,
            'speed_margin_mps': config.IMU_GAP_SPEED_MARGIN_MPS,
            'last_reason': self.last_reason,
        }


class Odometry:
    """Filter state + stillness detector + the one update sequence."""

    def __init__(self, use_zupt=True, trace=False, startup_alignment_s=None,
                 wheel_measurements_available=True, *, position_integration=None,
                 wheel_distance_scale=None):
        if position_integration is None:
            position_integration = getattr(config, 'ODOMETRY_POSITION_INTEGRATION', 'endpoint')
        if not isinstance(position_integration, str) or position_integration not in ('endpoint', 'gyro_midpoint'):
            raise ValueError('Position integration must be endpoint or gyro_midpoint')
        if wheel_distance_scale is None:
            wheel_distance_scale = getattr(config, 'WHEEL_DISTANCE_SCALE', 1.0)
        self.position_integration = position_integration
        self.wheel_distance_scale = _checked_distance_scale(wheel_distance_scale)
        self.state = eskf.create_state()
        self.still = StillnessDetector()
        self.use_zupt = use_zupt      # False = measure what ZUPT buys
        self.wheel_measurements_available = bool(wheel_measurements_available)
        self.quiet = 0                # consecutive still cycles
        self.stopped = False          # detector said still THIS cycle
        self.zupt_cycles = 0
        self.cycles = 0
        # Public odometry pose. Translation comes from the wheel encoder;
        # direction comes from the ESKF attitude. This is deliberately
        # separate from state['p']: no sensor on this rover measures absolute
        # position, so velocity/tilt pseudo-measurements can move state['p']
        # through cross-covariance and make the displayed rover teleport.
        self._position = np.zeros(3)
        self._velocity = np.zeros(3)
        # Travel in the last accepted interval, before summing changing
        # directions into a displacement. Recomputed each step; no carried
        # history or checkpoint field is needed for this transient output.
        self.last_wheel_travel_m = 0.0
        self.trace_enabled = trace
        self.last_trace = None
        if startup_alignment_s is None:
            startup_alignment_s = config.STARTUP_ALIGNMENT_SECONDS
        self.alignment_required_s = max(0.0, float(startup_alignment_s))
        self.alignment_elapsed_s = 0.0
        self._alignment_gyro = []
        self._alignment_accel = []
        self.ready = self.alignment_required_s == 0.0

    def _collect_alignment(self, gyro, accel, dt):
        """Collect one sample from a contiguous stationary interval."""
        if not self.stopped:
            self.alignment_elapsed_s = 0.0
            self._alignment_gyro.clear()
            self._alignment_accel.clear()
            return

        self._alignment_gyro.append(gyro.copy())
        self._alignment_accel.append(accel.copy())
        self.alignment_elapsed_s += dt
        if self.alignment_elapsed_s < self.alignment_required_s:
            return

        gyro_mean = np.mean(self._alignment_gyro, axis=0)
        accel_mean = np.mean(self._alignment_accel, axis=0)
        accel_unit = accel_mean / np.linalg.norm(accel_mean)

        # For q = Rz(yaw) Ry(pitch) Rx(roll), stationary specific force is
        # g_body/g = [-sin(pitch), sin(roll)cos(pitch),
        #             cos(roll)cos(pitch)]. Yaw is intentionally absent.
        roll = np.arctan2(accel_unit[1], accel_unit[2])
        pitch = np.arctan2(
            -accel_unit[0],
            np.sqrt(accel_unit[1]**2 + accel_unit[2]**2),
        )
        self.state['q'] = eskf.quat_from_rpy(roll, pitch, 0.0)
        self.state['bg'] = gyro_mean
        self.still.set_reference(gyro_mean)
        self.state['p'].fill(0.0)
        self.state['v'].fill(0.0)
        self._position.fill(0.0)
        self._velocity.fill(0.0)
        self.ready = True

        # The samples are no longer needed after their means are applied.
        self._alignment_gyro.clear()
        self._alignment_accel.clear()

    def step(self, wheels, *, gyro, accel, dt, gyro_samples=None):
        """Apply one complete interval or leave the estimate/history unchanged.

        The bounded rollback copies only numeric state and short detector/
        alignment histories. Existing NIS lists are truncated on failure,
        rather than copied in full on every control-loop cycle.
        """
        previous = dict(self.__dict__)
        previous['_position'] = self._position.copy()
        previous['_velocity'] = self._velocity.copy()
        previous['_alignment_gyro'] = list(self._alignment_gyro)
        previous['_alignment_accel'] = list(self._alignment_accel)
        original_state = self.state
        state_before = dict(original_state)
        for key in ('p', 'v', 'q', 'bg', 'ba', 'P'):
            state_before[key] = original_state[key].copy()
        nis_before = {name: (values, len(values)) for name, values in original_state['nis'].items()}
        original_still = self.still
        still_before = dict(original_still.__dict__)
        still_before['gyro_window'] = deque(original_still.gyro_window, maxlen=original_still.gyro_window.maxlen)
        still_before['accel_window'] = deque(original_still.accel_window, maxlen=original_still.accel_window.maxlen)
        try:
            with np.errstate(over='raise', invalid='raise', divide='raise'), eskf.checked_interval(self.state):
                result = self._step(wheels, gyro=gyro, accel=accel, dt=dt, gyro_samples=gyro_samples)
                if (not np.isfinite(self._position).all() or not np.isfinite(self._velocity).all()
                        or not np.isfinite(self.last_wheel_travel_m)):
                    raise ValueError('Nonfinite public odometry output')
            return result
        except (ValueError, FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
            history = state_before['nis']
            for name in list(history):
                if name not in nis_before:
                    del history[name]
            for name, (values, count) in nis_before.items():
                del values[count:]
                history[name] = values
            original_state.clear()
            original_state.update(state_before)
            original_still.__dict__.clear()
            original_still.__dict__.update(still_before)
            self.__dict__.clear()
            self.__dict__.update(previous)
            raise ValueError(f'Odometry interval rejected: {exc}') from exc

    def _step(self, wheels, *, gyro, accel, dt, gyro_samples=None):
        """
        One cycle: gyro [rad/s] and accel [m/s^2] in the body frame,
        with sensor biases still included. dt is elapsed time in seconds.
        Optional gyro_samples are ordered FIFO rates spanning dt uniformly;
        predict each one, then apply the observations once per wheel cycle.
        Accel is held across this interval (no accel/gyro hardware sync).

        gyro and accel are keyword-only on purpose. They are both plain
        3-vectors, so swapping them is silent -- no exception, no shape
        error, just a filter that quietly free-falls. It happened once
        already (record.py passed accel first and integrated -9.99 m in
        two seconds). Naming them at every call site prevents accidental
        positional swapping and makes the intended inputs visible.
        """
        # Reject a malformed interval before changing the state, stillness
        # history, alignment or counters. Both the ordered FIFO path and
        # legacy single-rate callers have the same input contract. Own the
        # vectors: a caller may reuse its sensor buffers after this step.
        gyro = np.array(gyro, dtype=float, copy=True)
        accel = np.array(accel, dtype=float, copy=True)
        interval = np.asarray(dt, dtype=float)
        if (gyro.shape != (3,) or accel.shape != (3,)
                or not np.isfinite(gyro).all() or not np.isfinite(accel).all()
                or interval.shape != () or not np.isfinite(interval)
                or interval <= 0):
            raise ValueError("Expected finite body-frame IMU vectors and positive scalar dt")
        dt = float(interval)
        if len(wheels) != len(config.WHEEL_SIGN):
            raise ValueError("Expected one velocity sample per configured wheel")
        try:
            wheel_rates = np.asarray([w['velocity_rads'] for w in wheels], dtype=float)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("Expected finite wheel velocity samples") from exc
        if wheel_rates.shape != (len(config.WHEEL_SIGN),) or not np.isfinite(wheel_rates).all():
            raise ValueError("Expected finite wheel velocity samples")
        wheels = [{'velocity_rads': float(rate)} for rate in wheel_rates]
        if gyro_samples is not None:
            gyro_samples = np.array(gyro_samples, dtype=float, copy=True)
            if (gyro_samples.ndim != 2 or gyro_samples.shape[1] != 3
                    or len(gyro_samples) == 0 or not np.isfinite(gyro_samples).all()
                    or not np.allclose(gyro, gyro_samples.mean(axis=0), rtol=0, atol=1e-5)):
                raise ValueError("Invalid ordered gyro batch/mean/interval")
        self.last_wheel_travel_m = 0.0
        s = self.state
        output_position_before = self._position.copy()

        wheel_mps = wheel_speeds_mps(wheels, distance_scale=self.wheel_distance_scale)
        s['_trace_enabled'] = self.trace_enabled
        eskf.start_trace(s, dt=dt, gyro_raw=gyro, accel_raw=accel,
                         wheel_mps=wheel_mps)

        self.stopped = self.still.update(wheels, gyro, accel)

        if not self.ready:
            self._collect_alignment(gyro, accel, dt)
            eskf.trace_context(
                s,
                alignment=True,
                alignment_progress=float(self.alignment_progress),
                stopped=bool(self.stopped),
                stillness=self.still.last_metrics,
            )
            for name in ('predict', 'wheel', 'nhc', 'zupt', 'zaru', 'gravity'):
                eskf.trace_skip(s, name, 'waiting for stationary startup alignment')
            self.last_trace = eskf.finish_trace(s)
            if self.last_trace is not None and self.trace_enabled != 'yaw':
                self.last_trace['odometry_output'] = {
                    'applied': False,
                    'reason': 'waiting for stationary startup alignment',
                }
            self.cycles += 1
            return self

        direction_dt = None
        if self.position_integration == 'gyro_midpoint':
            # Each ordered FIFO rate spans one equal subinterval. Integrate
            # its body-forward direction at the gyro half increment, before
            # observations. Using the actual increment also preserves turn
            # direction for rotations larger than a quaternion shortest arc.
            direction_dt = np.zeros(3)
        rates = (gyro,) if gyro_samples is None else gyro_samples
        sub_dt = dt / len(rates)
        for rate in rates:
            if direction_dt is not None:
                half_increment = eskf.small_angle_to_quat((rate - s['bg']) * (sub_dt / 2.0))
                q_mid = eskf.quat_normalize(eskf.quat_multiply(s['q'], half_increment))
                direction_dt += eskf.quat_to_rot(q_mid)[:, 0] * sub_dt
            eskf.predict(s, rate, accel, sub_dt)
        predicted_rotation = (eskf.quat_to_rot(s['q']) if direction_dt is not None else None)

        self.quiet = self.quiet + 1 if self.stopped else 0

        # Bias-corrected body-z angular rate, used as a yaw-rate proxy by
        # the gates and adaptive NHC noise. It equals Euler yaw rate for
        # a level pure-yaw turn, not for arbitrary tilted motion.
        yaw_rate = float(gyro[2] - s['bg'][2])
        eskf.trace_context(
            s,
            stopped=bool(self.stopped),
            quiet_cycles=int(self.quiet),
            zupt_active=self.zupt_active,
            yaw_rate_corrected_deg_s=float(np.degrees(yaw_rate)),
            body_velocity_mps=float(sum(wheel_mps) / len(wheel_mps)),
            stillness=self.still.last_metrics,
        )

        if not self.wheel_measurements_available:
            # Bench placeholders are not zero-velocity measurements. After
            # operator-held startup alignment, only IMU prediction and gated
            # gravity may update attitude; none of these rest/kinematic
            # constraints has the wheel evidence required by its model.
            for name in ('wheel', 'nhc', 'zupt', 'zaru'):
                eskf.trace_skip(s, name, 'wheel measurements unavailable')
        elif self.zupt_active:
            eskf.trace_skip(s, 'wheel', 'replaced by zero-velocity update')
            eskf.trace_skip(s, 'nhc', 'replaced by zero-velocity update')
            eskf.update_zupt(s)
            eskf.update_zaru(s, gyro)
            self.zupt_cycles += 1
        else:
            eskf.update_wheel_velocity(s, body_velocity(wheels, distance_scale=self.wheel_distance_scale),
                                       yaw_rate=yaw_rate)
            eskf.update_nhc(s, yaw_rate=yaw_rate)
            reason = ('disabled' if not self.use_zupt else
                      'stillness hold not satisfied')
            eskf.trace_skip(s, 'zupt', reason)
            eskf.trace_skip(s, 'zaru', reason)

        eskf.update_gravity(s, accel, yaw_rate=yaw_rate)

        # Continuous wheel-inertial pose output:
        #
        # Endpoint mode retains the historical corrected-end orientation.
        # Midpoint mode integrates gyro-resolved forward directions, then
        # anchors the interval to the final observation-corrected attitude:
        #
        #   delta_p = v_wheel R(q_post) R(q_pred)^T sum(R(q_mid_i)e_x dt_i)
        #
        # The final correction is an explicit endpoint convention, not a
        # smoother. It preserves the corrected-end result for zero rotation.
        #
        # This is the actual rover kinematic model. It does not force yaw to
        # zero: a real gyro turn rotates q, so the integrated path curves.
        # Wheel slip can still make distance wrong; that is an explicit
        # limitation until a second velocity/position source is added.
        # Once stillness is confirmed, ZUPT is stronger evidence than a
        # one-count encoder flicker: the body velocity is exactly zero.
        forward_speed = (0.0 if self.zupt_active or not self.wheel_measurements_available else
                         float(sum(wheel_mps) / len(wheel_mps)))
        velocity_body = np.array([forward_speed, 0.0, 0.0])
        final_rotation = eskf.quat_to_rot(s['q'])
        self._velocity = final_rotation @ velocity_body
        if direction_dt is None:
            output_delta = self._velocity * dt
        else:
            output_delta = forward_speed * (final_rotation @ predicted_rotation.T @ direction_dt)
        self._position += output_delta
        self.last_wheel_travel_m = abs(forward_speed) * dt

        self.last_trace = eskf.finish_trace(s)
        if self.last_trace is not None and self.trace_enabled != 'yaw':
            self.last_trace['odometry_output'] = {
                'applied': self.wheel_measurements_available,
                'reason': (None if self.wheel_measurements_available else
                           'wheel measurements unavailable; public position retained'),
                'formula': ('p_next = p + R(q_ESKF) * [v_wheel, 0, 0] * dt'
                            if direction_dt is None else
                            'p_next = p + v_wheel * R(q_post) * R(q_pred)^T * sum(R(q_mid_i) * e_x * dt_i)'),
                'position_integration': self.position_integration,
                'wheel_distance_scale': self.wheel_distance_scale,
                'wheel_travel_m': self.last_wheel_travel_m,
                'forward_speed_mps': forward_speed,
                'velocity_body_mps': velocity_body.tolist(),
                'velocity_world_mps': self._velocity.tolist(),
                'position_before_m': output_position_before.tolist(),
                'delta_position_m': output_delta.tolist(),
                'position_after_m': self._position.tolist(),
            }
        self.cycles += 1
        return self

    def interrupt_interval(self):
        """Retain an aligned prior without inventing any missing motion.

        The caller must latch degraded quality: this does not recover an
        unmeasured angle/distance or provide its uncertainty. Do not integrate
        the gap with the next sample. Fresh samples may resume from this prior,
        with ZUPT/ZARU disabled for the retained segment until a fresh Run.
        """
        if not self.ready:
            raise ValueError("An unaligned estimator cannot retain a calibrated prior")
        reference = self.still.reference_gyro
        self.still = StillnessDetector()
        if reference is not None:
            self.still.set_reference(reference)
        self.quiet = 0
        self.stopped = False
        # Lost motion removes the evidence needed to relearn zero-rate bias.
        # Keep this off for the retained segment; NEW RUN constructs a fresh
        # estimator. Other observations may still adjust bias indirectly.
        self.use_zupt = False
        self.last_trace = None
        self.state['nis'] = {}
        self.state['_trace'] = None

    def recovery_checkpoint(self):
        """JSON initial conditions for a new segment after interrupt_interval.

        P is the conditional filter covariance only. Unknown motion during
        the missing interval is NOT represented by these finite numbers.
        """
        if not self.ready or self.quiet or self.still.gyro_window:
            raise ValueError("Checkpoint requires an aligned, interrupted interval")
        return {
            'schema': 'odometry_recovery_v1',
            'state': {key: self.state[key].tolist() for key in ('p', 'v', 'q', 'bg', 'ba', 'P')},
            'position_m': self._position.tolist(), 'velocity_mps': self._velocity.tolist(),
            'use_zupt': bool(self.use_zupt), 'ready': True,
            'wheel_measurements_available': self.wheel_measurements_available,
            'position_integration': self.position_integration,
            'wheel_distance_scale': self.wheel_distance_scale,
            'cycles': self.cycles, 'zupt_cycles': self.zupt_cycles,
            'alignment_required_s': self.alignment_required_s,
            'alignment_elapsed_s': self.alignment_elapsed_s,
            'stationary_gyro_reference_rad_s': (self.still.reference_gyro.tolist()
                                               if self.still.reference_gyro is not None else None),
            'history_policy': 'empty stillness/alignment buffers and NIS diagnostics',
            'covariance_scope': 'conditional filter only; missing-motion uncertainty is unknown',
        }

    @classmethod
    def from_recovery_checkpoint(cls, checkpoint):
        """Restore explicitly identified initial conditions for offline replay.

        This is not an accuracy reset. The corresponding segment metadata
        must keep scientific pose validity false after the lost interval.
        """
        if checkpoint.get('schema') != 'odometry_recovery_v1' or checkpoint.get('ready') is not True:
            raise ValueError("Unsupported estimator recovery checkpoint")
        # Missing fields identify older endpoint/scale-1 records. An explicit
        # null is malformed provenance, not permission to use today's default.
        if ('position_integration' in checkpoint and checkpoint['position_integration'] is None
                or 'wheel_distance_scale' in checkpoint and checkpoint['wheel_distance_scale'] is None):
            raise ValueError("Invalid recovery integration options")
        odo = cls(use_zupt=checkpoint['use_zupt'],
                  startup_alignment_s=checkpoint['alignment_required_s'],
                  wheel_measurements_available=checkpoint.get('wheel_measurements_available', True),
                  position_integration=checkpoint.get('position_integration', 'endpoint'),
                  wheel_distance_scale=checkpoint.get('wheel_distance_scale', 1.0))
        shapes = {'p': (3,), 'v': (3,), 'q': (4,), 'bg': (3,), 'ba': (3,), 'P': (15, 15)}
        for key, shape in shapes.items():
            value = np.array(checkpoint['state'][key], dtype=float, copy=True)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f"Invalid recovery state: {key}")
            odo.state[key] = value
        if abs(np.linalg.norm(odo.state['q']) - 1.0) > 1e-6:
            raise ValueError("Invalid recovery quaternion norm")
        for field, name in (('position_m', '_position'), ('velocity_mps', '_velocity')):
            value = np.array(checkpoint[field], dtype=float, copy=True)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError(f"Invalid recovery output: {field}")
            setattr(odo, name, value)
        odo.ready = True
        odo.cycles = int(checkpoint['cycles'])
        odo.zupt_cycles = int(checkpoint['zupt_cycles'])
        odo.alignment_elapsed_s = float(checkpoint['alignment_elapsed_s'])
        reference = checkpoint.get('stationary_gyro_reference_rad_s')
        if reference is not None:
            odo.still.set_reference(reference)
        return odo

    # ── read-out ───────────────────────────────────────────────────

    @property
    def zupt_active(self):
        """True once the hold has been satisfied (what ZUPT actually ran on)."""
        return bool(self.use_zupt and self.wheel_measurements_available and self.ready
                    and self.quiet >= config.ZUPT_HOLD_SAMPLES)

    @property
    def alignment_progress(self):
        """Stationary startup progress from 0.0 to 1.0."""
        if self.ready or self.alignment_required_s == 0.0:
            return 1.0
        return min(1.0, self.alignment_elapsed_s / self.alignment_required_s)

    @property
    def position(self):
        """Continuous reported odometry position (wheel speed + ESKF attitude)."""
        return self._position.copy()

    @property
    def velocity(self):
        """Instantaneous wheel velocity in the final corrected world attitude."""
        return self._velocity.copy()

    @property
    def inertial_position(self):
        """Internal strapdown ESKF position, retained for diagnostics only."""
        return eskf.get_position(self.state)

    @property
    def inertial_velocity(self):
        """Internal ESKF velocity, retained for diagnostics only."""
        return eskf.get_velocity(self.state)

    @property
    def rpy_deg(self):
        return np.array(eskf.get_euler_deg(self.state))

    @property
    def gyro_bias(self):
        return self.state['bg'].copy()

    @property
    def accel_bias(self):
        return self.state['ba'].copy()

    @property
    def nis(self):
        return self.state['nis']

    def diagnostics(self):
        """Detector evidence and scope, never calibrated public-pose confidence."""
        return {
            'stillness': dict(self.still.last_metrics),
            'stationary_gyro_reference_available': self.still.reference_gyro is not None,
            'wheel_measurements_available': self.wheel_measurements_available,
            'position_integration': self.position_integration,
            'wheel_distance_scale': self.wheel_distance_scale,
            'zero_updates_enabled': bool(self.use_zupt and self.wheel_measurements_available),
            'zupt_active': self.zupt_active,
            'startup_stationarity_requires_operator': True,
            'position_uncertainty_m': None,
            'covariance_scope': 'internal ESKF only; not public wheel-integrated position',
        }

    def snapshot(self):
        """One dict per cycle, for plots and logs."""
        return {
            'pos': self.position, 'vel': self.velocity, 'rpy': self.rpy_deg,
            'filter_pos': self.inertial_position,
            'filter_vel': self.inertial_velocity,
            'bg': self.gyro_bias, 'stopped': self.stopped,
            'ready': self.ready, 'alignment_progress': self.alignment_progress,
        }
