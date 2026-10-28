"""Manual cylinder/string commands with one input timeout and output caps."""
import math
import threading
import time

import config


class Inbox:
    """Latest input wins; sequence identifies a press, not a control owner."""
    def __init__(self):
        self.lock = threading.Lock()
        self.intent, self.at, self.serial = 'idle', -math.inf, 0
        self.stop_generation = 0
        self.lift_hold_authorized = False
        self.test = None
        self.allow_lift = True  # Compatibility for runtime setup; no commissioning gate.
        self._press = self._expired_press = None

    def _expire(self, now):
        if self.intent != 'idle' and now - self.at >= config.HAMMER_COMMAND_TIMEOUT_S:
            self._expired_press = self._press
            self.intent, self.at = 'idle', -math.inf
            self.lift_hold_authorized = False
            self.stop_generation += 1

    def submit(self, client, message, estop=False):
        intent = message.get('intent')
        if intent not in ('idle', 'wind', 'drop', 'rearm', 'lift', 'lift_hold'):
            return False
        settings = validate_test(message.get('test'), intent)
        press = (client, message.get('seq'))
        with self.lock:
            self._expire(time.monotonic())
            if estop or (intent != 'idle' and press == self._expired_press):
                return False
            if press != self._press or intent != self.intent or settings != self.test:
                self.serial += 1
            if intent == 'idle':
                self.stop_generation += 1
                self.lift_hold_authorized = False
            elif intent == 'lift':
                self.lift_hold_authorized = False
            elif intent == 'lift_hold':
                self.lift_hold_authorized = self.intent == 'lift' or self.lift_hold_authorized
            self._press, self.intent, self.test = press, intent, settings
            self.at = time.monotonic()
            return True

    def stop(self, client=None, forget=False):
        with self.lock:
            # A passive dashboard disconnect must not cancel another input.
            # Calls without a client remain global stops (for example E-STOP).
            if client is not None and (self._press is None or self._press[0] is not client):
                return
            self._expired_press = self._press
            self.intent, self.at = 'idle', -math.inf
            self.lift_hold_authorized = False
            self.stop_generation += 1

    def snapshot(self):
        with self.lock:
            self._expire(time.monotonic())
            return dict(intent=self.intent, serial=self.serial, stop_generation=self.stop_generation,
                        lift_hold_authorized=self.lift_hold_authorized,
                        fresh=self.intent != 'idle', test=dict(self.test) if self.test else None)


def test_settings(intent=None):
    return dict(seconds=config.HAMMER_TEST_WIND_DEFAULT_SECONDS if intent == 'wind' else config.HAMMER_TEST_DEFAULT_SECONDS,
                speed=(config.HAMMER_TEST_WIND_DEFAULT_SPEED if intent == 'wind' else
                       config.HAMMER_TEST_LATCH_DEFAULT_SPEED if intent in ('drop', 'rearm') else
                       config.HAMMER_TEST_DEFAULT_SPEED),
                reverse=config.HAMMER_TEST_WIND_DEFAULT_REVERSE if intent == 'wind' else False,
                center_us=config.HAMMER_TEST_SERVO_CENTER_US)


def validate_test(value, intent=None):
    """Normalize optional fields; malformed values never block an input."""
    value = value if isinstance(value, dict) else {}
    defaults = test_settings(intent)
    def bounded(name, low, high):
        number = value.get(name)
        if type(number) not in (int, float) or (type(number) is float and not math.isfinite(number)):
            number = defaults[name]
        return max(low, min(high, number))
    return dict(seconds=defaults['seconds'] if intent in ('wind', 'lift', 'lift_hold') else
                    bounded('seconds', config.HAMMER_TEST_MIN_SECONDS, config.HAMMER_TEST_MAX_SECONDS),
                speed=bounded('speed', 0, 1),
                center_us=config.HAMMER_TEST_SERVO_CENTER_US if intent in ('lift', 'lift_hold') else
                    round(bounded('center_us', config.HAMMER_TEST_SERVO_CENTER_MIN_US, config.HAMMER_TEST_SERVO_CENTER_MAX_US)),
                reverse=defaults['reverse'] if intent == 'wind' or type(value.get('reverse')) is not bool else value['reverse'])


def settings_ready():
    return dict(lift=True, wind=True, latch=True)


class Controller:
    def __init__(self, event=lambda text: None, test_mode=False, allow_lift=True):
        self.event = event
        self.active, self.latch, self.notice = 'idle', 'unknown', 'Ready'
        self.lift_holding = False
        self.serial, self.stop_generation = -1, 0
        self.started = 0.0
        self.test_mode, self.allow_lift = True, True
        self.test = test_settings()
        self.next_latch = 'rearm'

    def update(self, command, board, lift_ready, estop=False, now=None):
        now = time.monotonic() if now is None else now
        intent, serial = command['intent'], command['serial']
        generation = command.get('stop_generation', self.stop_generation)
        if generation != self.stop_generation:
            self.active, self.lift_holding = 'idle', False
            self.stop_generation = generation
        if estop or not command['fresh'] or intent == 'idle':
            self.active, self.lift_holding, self.serial = 'idle', False, serial
            self.notice = 'E-STOP' if estop else 'Stopped'
            return 0, 0., 0.
        if serial != self.serial:
            previous = self.active
            settings = validate_test(command.get('test'), intent)
            self.serial = serial
            if intent == 'lift_hold':
                self.lift_holding = previous == 'lift' or self.lift_holding or command.get('lift_hold_authorized', False)
                intent = 'lift_hold' if self.lift_holding else 'idle'
            elif intent == 'lift':
                self.lift_holding = False
            self.test, self.active, self.started = settings, intent, now
            self.notice = {'drop': 'RELEASE', 'rearm': 'RE-ARM', 'lift_hold': 'HOLD'}.get(intent, intent.upper())
        if self.active in ('drop', 'rearm') and now - self.started >= self.test['seconds']:
            self.next_latch = 'drop' if self.active == 'rearm' else 'rearm'
            self.active = 'lift_hold' if self.lift_holding else 'idle'
            self.notice = 'HOLD' if self.lift_holding else 'Ready'
        speed = self.test['speed'] * (-1 if self.test['reverse'] else 1)
        return (round(config.CYLINDER_TEST_MAX_RAW * speed) if self.active == 'lift' else 0,
                speed if self.active == 'wind' else 0.,
                speed if self.active == 'drop' else -speed if self.active == 'rearm' else 0.)

    def state(self, lift):
        return dict(configured=settings_ready(), lift=lift, active=self.active, lift_holding=self.lift_holding,
                    latch=self.latch, notice=self.notice, test_mode=True, next_latch=self.next_latch,
                    wind_settings=test_settings('wind'), latch_settings=test_settings('drop'),
                    wind_direction_locked=True,
                    test_settings=dict(test_settings(), min_seconds=config.HAMMER_TEST_MIN_SECONDS,
                        max_seconds=config.HAMMER_TEST_MAX_SECONDS, min_center_us=config.HAMMER_TEST_SERVO_CENTER_MIN_US,
                        max_center_us=config.HAMMER_TEST_SERVO_CENTER_MAX_US,
                        cylinder_motion_allowed=True, position_hold_on_release=True,
                        lift_max_raw=config.CYLINDER_TEST_MAX_RAW, lift_hold_to_run=True,
                        lift_up_sign=config.CYLINDER_TEST_UP_SIGN, servo_offset_us=config.HAMMER_TEST_SERVO_OFFSET_US))

    def test_pulses(self, pan):
        import pico_link
        pulses = pico_link.servo_pulses(pan=pan)
        pulses[0] = pulses[1] = 0
        offset = round(config.HAMMER_TEST_SERVO_OFFSET_US * self.test['speed']) * (-1 if self.test['reverse'] else 1)
        if self.active == 'wind' and offset:
            pulses[0] = self.test['center_us'] + offset
        elif self.active in ('drop', 'rearm') and offset:
            pulses[1] = self.test['center_us'] + (offset if self.active == 'drop' else -offset)
        return pulses
