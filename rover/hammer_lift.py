"""Direct ID 5 jog: reference display, release HOLD, lost input OFF."""
import threading
import time
from types import SimpleNamespace

import config


class Lift:
    def __init__(self, wheels, event=print, test_mode=False, allow_motion=True):
        self.event = event
        self.id = config.HAMMER_LIFT_ID
        self.allow_motion = bool(allow_motion)
        self.ready = self.connected = self.enabled = self.touched = False
        self.error = None
        self._sample = {}
        self._at = None
        self._next_poll = 0.0
        self._goal = self._hold_position = self._profile = None
        self._direction = 0
        self._holding = False
        self._up_limit = config.CYLINDER_UP_POSITION_RAW
        self._down_limit = config.CYLINDER_DOWN_POSITION_RAW
        if self.id in config.MOTOR_IDS:
            self.error = 'Cylinder ID overlaps a wheel ID'
        elif not wheels.connected:
            self.error = 'Cylinder motor connection unavailable'
        else:
            self.pkt, self.port = wheels.pkt_h, wheels.port_h
            self.poll()

    def _read(self, size, address):
        value, comm, error = getattr(self.pkt, f'read{size}ByteTxRx')(self.port, self.id, address)
        if comm or error:
            raise RuntimeError(f'ID {self.id} read {address}: comm={comm}, error={error}')
        return value

    def _write(self, size, address, value):
        comm, error = getattr(self.pkt, f'write{size}ByteTxRx')(
            self.port, self.id, address, value & ((1 << (8 * size)) - 1))
        if comm or error:
            raise RuntimeError(f'ID {self.id} write {address}: comm={comm}, error={error}')

    def _position(self):
        raw = self._read(4, 132)
        position = raw - 2**32 if raw >= 2**31 else raw
        self._sample['position_raw'], self._at = position, time.monotonic()
        return position

    def poll(self):
        if not hasattr(self, 'pkt') or time.monotonic() < self._next_poll:
            return
        self._next_poll = time.monotonic() + config.HAMMER_LIFT_READ_INTERVAL_S
        try:
            self._position()
            self._sample.update(mode=self._read(1, 11), torque_enabled=bool(self._read(1, 64)),
                                hardware_error=self._read(1, 70))
            if self.enabled and (not self._sample['torque_enabled'] or
                                 self._sample['mode'] != config.CYLINDER_TEST_POSITION_MODE):
                # A reboot/watchdog or external mode change invalidates setup.
                # The next movement input initializes again; HOLD cannot do so.
                self.enabled = self._holding = False
                self._direction = 0
                self._goal = self._hold_position = self._profile = None
            self.connected = True
            self.ready = self.allow_motion
            self.error = None
        except Exception as error:
            self.connected = self.ready = False
            self.error = str(error)

    def _halt(self):
        needs_off = self.touched or self._sample.get('torque_enabled')
        self.enabled = self._holding = False
        self._direction = 0
        self._goal = self._hold_position = None
        if needs_off:
            self._write(1, 64, 0)
            self._sample['torque_enabled'] = False
            self.touched = False

    def _begin(self, speed):
        self.touched = True
        self._write(1, 64, 0)
        self._write(1, 98, 0)  # Clear a previously expired bus watchdog.
        self._write(1, 11, config.CYLINDER_TEST_POSITION_MODE)
        self._write(1, 10, self._read(1, 10) & ~0x0c)
        self._write(4, 108, config.CYLINDER_TEST_PROFILE_ACCEL)
        self._write(4, 112, speed)
        position = self._position()
        self._write(4, 116, position)
        self._write(1, 98, config.HAMMER_LIFT_WATCHDOG_TICKS)
        self._write(1, 64, 1)
        self.enabled = True
        self._sample.update(mode=config.CYLINDER_TEST_POSITION_MODE, torque_enabled=True)
        self._goal, self._profile = position, speed
        return position

    def drive(self, raw, hold=False):
        if not self.allow_motion or not hasattr(self, 'pkt'):
            return False
        try:
            if type(raw) is not int or abs(raw) > config.CYLINDER_TEST_MAX_RAW or type(hold) is not bool or (hold and raw):
                raise ValueError('Invalid cylinder speed')
            if raw == 0 and not hold:
                self._halt()
                self.poll()
                return True
            if hold:
                if not self.enabled:
                    return False  # HOLD never starts a motor after a stop.
                if not self._holding:
                    self._hold_position = self._position()
                    self._write(4, 116, self._hold_position)
                    self._goal = self._hold_position
                self._holding, self._direction = True, 0
            else:
                position = self._position() if self.enabled else self._begin(abs(raw))
                direction = 1 if raw > 0 else -1
                # Saved reference positions are display metadata, not interlocks.
                # Keep the target close to the measured position while held.
                goal = max(config.CYLINDER_EXTENDED_MIN_RAW,
                           min(config.CYLINDER_EXTENDED_MAX_RAW,
                               position + direction * config.CYLINDER_JOG_LEAD_COUNTS))
                if self._profile != abs(raw):
                    self._write(4, 112, abs(raw))
                    self._profile = abs(raw)
                if goal != self._goal:
                    self._write(4, 116, goal)
                self._goal = goal
                self._holding, self._hold_position, self._direction = False, None, direction
            self.connected = self.ready = True
            self.error = None
            self.poll()
            return True
        except Exception as error:
            self.error = str(error)
            self.connected = self.ready = False
            try:
                self._halt()
            except Exception as stop_error:
                self.error += '; stop: ' + str(stop_error)
            self._next_poll = 0
            self.event('cylinder: ' + self.error)
            return False

    def state(self):
        raw = self._sample.get('position_raw')
        return dict(self._sample, id=self.id, ready=self.ready, connected=self.connected,
                    error=self.error, control_mode='manual' if self.allow_motion else 'read_only',
                    calibrated=False, position_control_ready=self.ready, velocity_control_allowed=False,
                    faulted=False, recovery_required=False, fault_recoverable=False,
                    holding=self.enabled and self._holding, hold_position_raw=self._hold_position,
                    goal_position_raw=self._goal, target_position_mode=config.CYLINDER_TEST_POSITION_MODE,
                    command_min_raw=config.CYLINDER_EXTENDED_MIN_RAW,
                    command_max_raw=config.CYLINDER_EXTENDED_MAX_RAW,
                    reference_limits_enforced=False,
                    max_goal_lead_counts=config.CYLINDER_JOG_LEAD_COUNTS,
                    motor_shaft_deg=None if raw is None else raw * 360.0 / config.CYLINDER_ENCODER_COUNTS_PER_REV,
                    down_position_raw=self._down_limit, up_position_raw=self._up_limit,
                    up_position_source=config.CYLINDER_UP_POSITION_SOURCE,
                    down_position_source=config.CYLINDER_DOWN_POSITION_SOURCE,
                    down_limit_reached=raw is not None and (raw - self._down_limit) * config.CYLINDER_TEST_UP_SIGN <= 0,
                    up_limit_reached=raw is not None and (raw - self._up_limit) * config.CYLINDER_TEST_UP_SIGN >= 0,
                    baseline_position_raw=config.CYLINDER_BASELINE_RAW,
                    relative_to_baseline_raw=None if raw is None else raw - config.CYLINDER_BASELINE_RAW,
                    sample_age_s=None if self._at is None else max(0.0, time.monotonic() - self._at))

    def close(self):
        try:
            self._halt()
        except Exception as error:
            self.event('cylinder stop failed: ' + str(error))


class TestLift:
    """ID-5-only connection with a fresh-command timeout and USB retry."""
    def __init__(self, event=print, port_factory=None, allow_motion=True):
        self.event, self._factory = event, port_factory
        self.allow_motion = bool(allow_motion)
        self._raw, self._hold, self._at = 0, False, -float('inf')
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._state = dict(id=config.HAMMER_LIFT_ID, ready=False, connected=False,
                           error='Waiting for cylinder connection')
        self._state_at = time.monotonic()
        self._thread = threading.Thread(target=self._run, name='cylinder', daemon=True)
        self._thread.start()

    @property
    def ready(self):
        with self._lock:
            return self._state['ready']

    def state(self):
        with self._lock:
            state = dict(self._state)
            if state.get('sample_age_s') is not None:
                state['sample_age_s'] += max(0.0, time.monotonic() - self._state_at)
            return state

    def drive(self, raw, hold=False):
        with self._lock:
            self._raw, self._hold, self._at = raw, hold, time.monotonic()
        return self.allow_motion

    def _publish(self, state):
        with self._lock:
            self._state, self._state_at = dict(state), time.monotonic()

    def _run(self):
        while not self._stop.is_set():
            port, lift = None, None
            try:
                if self._factory is None:
                    import dxl_reader
                    port, packet = dxl_reader.open_port()
                else:
                    port, packet = self._factory()
                # USB reconnection discards input from the old connection.
                with self._lock:
                    self._raw, self._hold, self._at = 0, False, -float('inf')
                lift = Lift(SimpleNamespace(connected=True, port_h=port, pkt_h=packet),
                            self.event, allow_motion=self.allow_motion)
                self._publish(lift.state())
                while not self._stop.wait(config.CYLINDER_TEST_POLL_S):
                    if self.allow_motion:
                        with self._lock:
                            fresh = time.monotonic() - self._at < config.HAMMER_TEST_SETPOINT_AGE_S
                            raw, hold = (self._raw, self._hold) if fresh else (0, False)
                        lift.drive(raw, hold=hold)
                    else:
                        lift.poll()
                    self._publish(lift.state())
                    if not lift.connected:
                        raise RuntimeError(lift.error)
            except Exception as error:
                state = lift.state() if lift else self.state()
                self._publish(dict(state, ready=False, connected=False, error=str(error)))
            finally:
                if lift is not None:
                    lift.close()
                if port is not None:
                    try:
                        port.closePort()
                    except Exception:
                        pass
            if self._stop.wait(config.HAMMER_TEST_RETRY_S):
                break

    def close(self):
        self._stop.set()
        self._thread.join(timeout=3)
