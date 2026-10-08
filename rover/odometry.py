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

    stopped?           Decided from wheels + IMU steadiness, not from
                       |gyro| -- see StillnessDetector.

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

def wheel_speeds_mps(wheels):
    """Per-wheel linear speed, sign-corrected so positive = forward."""
    return [w['velocity_rads'] * config.WHEEL_SIGN[i] * config.WHEEL_RADIUS_M
            for i, w in enumerate(wheels)]


def body_velocity(wheels):
    """Forward speed of the body = average of all wheels."""
    v = wheel_speeds_mps(wheels)
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

    Do not add a "bias-corrected magnitude is small" test. It was tried:
    bg picks up non-zero values from the wheel/NHC/gravity updates long
    before ZARU has converged, so the test switches on with a garbage
    bias and blocks the very updates that would fix it. Measured: it
    rejected 1960 of 1981 otherwise-good cycles.

    Thresholds come from a real stationary log, not a datasheet.
    """

    def __init__(self, window=None):
        n = window or config.ZUPT_WINDOW
        self.gyro_window = deque(maxlen=n)
        self.accel_window = deque(maxlen=n)
        self.last_metrics = {'window': 0, 'ready': False, 'result': False}

    def update(self, wheels, gyro, accel):
        self.gyro_window.append(np.asarray(gyro, dtype=float))
        self.accel_window.append(np.asarray(accel, dtype=float))

        wheel_max = max(abs(w['velocity_rads']) for w in wheels)
        metrics = {
            'window': len(self.gyro_window),
            'window_required': self.gyro_window.maxlen,
            'wheel_max_rads': float(wheel_max),
            'wheel_limit_rads': float(config.ZUPT_WHEEL_SPEED_MAX),
            'accel_norm_error_mps2': float(abs(np.linalg.norm(accel) - config.GRAVITY)),
            'accel_norm_limit_mps2': float(config.ZUPT_ACCEL_DEV_MAX),
            'ready': len(self.gyro_window) == self.gyro_window.maxlen,
        }

        if len(self.gyro_window) < self.gyro_window.maxlen:
            metrics['result'] = False
            self.last_metrics = metrics
            return False        # std means nothing until the window fills

        gyro_std = np.std(np.array(self.gyro_window), axis=0).max()
        accel_std = np.std(np.array(self.accel_window), axis=0).max()
        metrics.update({
            'gyro_std_max_rad_s': float(gyro_std),
            'gyro_std_limit_rad_s': float(config.ZUPT_GYRO_STD_MAX),
            'accel_std_max_mps2': float(accel_std),
            'accel_std_limit_mps2': float(config.ZUPT_ACCEL_STD_MAX),
        })
        result = bool(
            wheel_max <= config.ZUPT_WHEEL_SPEED_MAX
            and gyro_std <= config.ZUPT_GYRO_STD_MAX
            and accel_std <= config.ZUPT_ACCEL_STD_MAX
            and metrics['accel_norm_error_mps2'] <= config.ZUPT_ACCEL_DEV_MAX
        )
        metrics['result'] = result
        self.last_metrics = metrics
        return result


# ── the cycle ──────────────────────────────────────────────────────

class Odometry:
    """Filter state + stillness detector + the one update sequence."""

    def __init__(self, use_zupt=True, trace=False, startup_alignment_s=None):
        self.state = eskf.create_state()
        self.still = StillnessDetector()
        self.use_zupt = use_zupt      # False = measure what ZUPT buys
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
        self.state['p'].fill(0.0)
        self.state['v'].fill(0.0)
        self._position.fill(0.0)
        self._velocity.fill(0.0)
        self.ready = True

        # The samples are no longer needed after their means are applied.
        self._alignment_gyro.clear()
        self._alignment_accel.clear()

    def step(self, wheels, *, gyro, accel, dt, gyro_samples=None):
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
        gyro = np.asarray(gyro, dtype=float)
        accel = np.asarray(accel, dtype=float)
        if gyro_samples is not None:
            gyro_samples = np.asarray(gyro_samples, dtype=float)
            if (gyro_samples.ndim != 2 or gyro_samples.shape[1] != 3
                    or len(gyro_samples) == 0 or not np.isfinite(gyro_samples).all()
                    or not np.isfinite(dt) or dt <= 0
                    or not np.allclose(gyro, gyro_samples.mean(axis=0), atol=1e-5)):
                raise ValueError("Invalid ordered gyro batch/mean/interval")
        s = self.state
        output_position_before = self._position.copy()

        wheel_mps = wheel_speeds_mps(wheels)
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

        if gyro_samples is None:
            eskf.predict(s, gyro, accel, dt)
        else:
            for rate in gyro_samples:
                eskf.predict(s, rate, accel, dt / len(gyro_samples))

        self.quiet = self.quiet + 1 if self.stopped else 0

        # Bias-corrected body-z angular rate, used as a yaw-rate proxy by
        # the gates and adaptive NHC noise. It equals Euler yaw rate for
        # a level pure-yaw turn, not for arbitrary tilted motion.
        yaw_rate = float(gyro[2] - s['bg'][2])
        eskf.trace_context(
            s,
            stopped=bool(self.stopped),
            quiet_cycles=int(self.quiet),
            zupt_active=bool(self.use_zupt and self.quiet >= config.ZUPT_HOLD_SAMPLES),
            yaw_rate_corrected_deg_s=float(np.degrees(yaw_rate)),
            body_velocity_mps=float(sum(wheel_mps) / len(wheel_mps)),
            stillness=self.still.last_metrics,
        )

        if self.use_zupt and self.quiet >= config.ZUPT_HOLD_SAMPLES:
            eskf.trace_skip(s, 'wheel', 'replaced by zero-velocity update')
            eskf.trace_skip(s, 'nhc', 'replaced by zero-velocity update')
            eskf.update_zupt(s)
            eskf.update_zaru(s, gyro)
            self.zupt_cycles += 1
        else:
            eskf.update_wheel_velocity(s, body_velocity(wheels),
                                       yaw_rate=yaw_rate)
            eskf.update_nhc(s, yaw_rate=yaw_rate)
            reason = ('disabled' if not self.use_zupt else
                      'stillness hold not satisfied')
            eskf.trace_skip(s, 'zupt', reason)
            eskf.trace_skip(s, 'zaru', reason)

        eskf.update_gravity(s, accel, yaw_rate=yaw_rate)

        # Continuous wheel-inertial pose output:
        #
        #   p[k+1] = p[k] + R(q_ESKF) [v_wheel, 0, 0] dt
        #
        # This is the actual rover kinematic model. It does not force yaw to
        # zero: a real gyro turn rotates q, so the integrated path curves.
        # Wheel slip can still make distance wrong; that is an explicit
        # limitation until a second velocity/position source is added.
        # Once stillness is confirmed, ZUPT is stronger evidence than a
        # one-count encoder flicker: the body velocity is exactly zero.
        forward_speed = (0.0 if self.zupt_active else
                         float(sum(wheel_mps) / len(wheel_mps)))
        velocity_body = np.array([forward_speed, 0.0, 0.0])
        self._velocity = eskf.quat_to_rot(s['q']) @ velocity_body
        output_delta = self._velocity * dt
        self._position += output_delta

        self.last_trace = eskf.finish_trace(s)
        if self.last_trace is not None and self.trace_enabled != 'yaw':
            self.last_trace['odometry_output'] = {
                'applied': True,
                'formula': 'p_next = p + R(q_ESKF) * [v_wheel, 0, 0] * dt',
                'forward_speed_mps': forward_speed,
                'velocity_body_mps': velocity_body.tolist(),
                'velocity_world_mps': self._velocity.tolist(),
                'position_before_m': output_position_before.tolist(),
                'delta_position_m': output_delta.tolist(),
                'position_after_m': self._position.tolist(),
            }
        self.cycles += 1
        return self

    # ── read-out ───────────────────────────────────────────────────

    @property
    def zupt_active(self):
        """True once the hold has been satisfied (what ZUPT actually ran on)."""
        return self.ready and self.quiet >= config.ZUPT_HOLD_SAMPLES

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
        """Velocity consistent with the reported odometry position."""
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

    def snapshot(self):
        """One dict per cycle, for plots and logs."""
        return {
            'pos': self.position, 'vel': self.velocity, 'rpy': self.rpy_deg,
            'filter_pos': self.inertial_position,
            'filter_vel': self.inertial_velocity,
            'bg': self.gyro_bias, 'stopped': self.stopped,
            'ready': self.ready, 'alignment_progress': self.alignment_progress,
        }
