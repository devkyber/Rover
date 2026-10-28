"""Rover PDB Pico 2 firmware (MicroPython), four independent PWM channels.

Standalone main.py; generated settings originate in rover/config.py. Current
native CAD has GP0/1/2/3 servo outputs, I2C0 GP16/17, no connected INA ALERT,
and USB-only Pico power. Optional J9 GPIOs are left as reset inputs.

Protocol v2 (ASCII lines; v1 three-channel commands are rejected):
  ?                         -> H,version,min_us,max_us,failsafe_ms,channels,config_id
  N,wind,release,pan,aux     -> install stop pulses, stop immediately
  S,wind,release,pan,aux     -> output microseconds (0 disables PWM); requires N
  Z                         -> zero net charge; gaps remain flagged if INA offline
  P,ms,bus,shunt,charge,wind,release,pan,aux,flags,bad,age_ms

Raw bus/shunt units: 1.25 mV / 2.5 uV. charge = signed shunt-count * ms.
Flags: 1 failsafe, 2 unavailable INA reading, 4 waiting for N, 8 charge gap.
Trim, direction, mechanical durations, shunt and conversion to SI are owned
by rover/config.py and rover/pico_link.py. No autonomous hammer sequencing.
The host must refresh S; invalid lines/queries/N never renew that deadman.
Hardware-free verification: python tools/check_pdb_io.py.
"""
import sys

try:
    import machine                      # exists only on the Pico
    import select
    from time import sleep_ms, ticks_diff, ticks_ms
except ImportError:                     # desktop Python: the tests and the simulator
    machine = None

    def ticks_diff(a, b):
        return a - b

# BEGIN GENERATED SETTINGS
# Source: rover/config.py; do not edit this block.
VERSION = 2
SERVO_PINS = (0, 1, 2, 3)
I2C_SDA_PIN = 16
I2C_SCL_PIN = 17
I2C_HZ = 100000
I2C_TIMEOUT_US = 5000
INA226_ADDRESS = 64
INA226_CONFIG = 17703
INA_RETRY_MS = 1000
INA_CHECK_REPORTS = 20
INA_STALE_MS = 200
PWM_HZ = 50
PULSE_MIN_US = 500
PULSE_MAX_US = 2500
FAILSAFE_MS = 300
REPORT_MS = 50
WATCHDOG_MS = 2000
LINE_MAX = 64
BOOT_STOP_US = (0, 0, 0, 0)
CONFIG_ID = 'e6ff3bd5919c3865'
# END GENERATED SETTINGS

# INA226 register definitions (TI SBOS547); tunable settings are generated above.
REG_CONFIG, REG_SHUNT, REG_BUS = 0x00, 0x01, 0x02
REG_MASK, REG_MANUFACTURER, REG_DIE = 0x06, 0xFE, 0xFF
MANUFACTURER_TI = 0x5449
CONVERSION_READY = 1 << 3

FLAG_FAILSAFE = 1
FLAG_NO_BATTERY = 2
FLAG_DEFAULT_STOP = 4
FLAG_CHARGE_GAP = 8


def parse_pulses(fields):
    """Exactly four unsigned pulse widths, or None if any field is invalid."""
    if len(fields) != len(SERVO_PINS):
        return None
    pulses = []
    for field in fields:
        if not field.isdigit() or len(field) > 4:
            return None
        us = int(field)
        if us != 0 and not PULSE_MIN_US <= us <= PULSE_MAX_US:
            return None
        pulses.append(us)
    return pulses


class Firmware:
    """Everything the Pico does, given an object that reaches the pins."""

    def __init__(self, hw, start_stopped=False):
        self.hw = hw
        now = hw.ticks_ms()
        self.stop_us = list(BOOT_STOP_US)
        self.pulse_us = [-1] * len(SERVO_PINS)    # unknown until the first _output() below
        self.failsafe = True            # nothing commanded yet
        self.default_stop = True        # no N line yet
        self.driven = False             # True once any servo has been sent pulses
        self.t_command = now
        self.t_report = now

        self.battery_ok = False
        self.battery_configured = False
        self.charge_gap = False
        self.sample_seen = False
        self.bus = 0
        self.shunt = 0
        self.charge = 0                 # sum of shunt counts x milliseconds
        self.t_sample = now
        self.t_retry = now
        self.reports = 0

        self.bad_lines = 0
        self._wrong_chip = None         # id of a chip that answered but is not an INA226
        self._line = bytearray(LINE_MAX)
        self._length = 0
        self._garbage = False           # this line is too long or not plain text

        if start_stopped:
            self.driven = True
            self._output(self.stop_us)
        else:
            self._output([0] * len(SERVO_PINS))
        self._setup_battery_monitor()
        self._hello()

    # ── one pass of the main loop ──────────────────────────────────

    def step(self):
        hw = self.hw
        now = hw.ticks_ms()

        # At most 64 bytes per pass, so a flood of bytes cannot hold up
        # the failsafe check below.
        for _ in range(64):
            byte = hw.read_byte()
            if byte < 0:
                break
            self._on_byte(byte, now)

        if not self.failsafe and ticks_diff(now, self.t_command) > FAILSAFE_MS:
            self.stop_servos()

        if ticks_diff(now, self.t_report) >= REPORT_MS:
            self.t_report = now
            self._measure(now)
            now = hw.ticks_ms()
            if not self.failsafe and ticks_diff(now, self.t_command) > FAILSAFE_MS:
                self.stop_servos()
            self._report(now)
            # LED: steady while commands arrive, blinking once a second in failsafe.
            hw.set_led(not self.failsafe or (now // 500) % 2 == 0)

        hw.feed_watchdog()

    def stop_servos(self):
        """
        The failsafe: stop pulses on all four outputs. If no output has
        carried a pulse since power-on, they stay silent.
        """
        self.failsafe = True
        self._output(self.stop_us if self.driven else [0] * len(SERVO_PINS))

    # ── lines from the Jetson ──────────────────────────────────────

    def _on_byte(self, byte, now):
        if byte == 10:                              # \n ends the line
            length, garbage = self._length, self._garbage
            self._length, self._garbage = 0, False
            if garbage:
                self.bad_lines += 1
            elif length:
                self._on_line(bytes(self._line[:length]).decode(), now)
        elif byte == 13:                            # \r: ignore
            pass
        elif byte < 32 or byte > 126 or self._length >= LINE_MAX:
            self._garbage = True
        else:
            self._line[self._length] = byte
            self._length += 1

    def _on_line(self, line, now):
        fields = line.split(",")
        kind = fields[0]
        if kind in ("S", "N"):
            pulses = parse_pulses(fields[1:])
            if pulses is None or (kind == "S" and self.default_stop):
                self.bad_lines += 1
            elif kind == "S":
                self.t_command = now
                self.failsafe = False
                if any(pulses):
                    self.driven = True
                self._output(pulses)
                # From the first command on, a stuck firmware would leave
                # a servo turning. Armed here and not at start, so that a
                # Pico nobody is commanding can never restart in a loop.
                self.hw.arm_watchdog()
            else:
                self.stop_us = pulses
                self.default_stop = False
                self.stop_servos()  # changing trim must never retain a moving command
        elif line == "Z":
            self.charge = 0
            self.charge_gap = not self.battery_ok
            self.t_sample = now
        elif line == "?":
            self._hello()
        else:
            self.bad_lines += 1

    def _output(self, pulses):
        for i in range(len(SERVO_PINS)):
            if pulses[i] != self.pulse_us[i]:
                self.pulse_us[i] = pulses[i]
                self.hw.set_pulse(i, pulses[i])

    # ── battery monitor ────────────────────────────────────────────

    def _battery_fault(self, now):
        self.battery_ok = self.battery_configured = False
        self.charge_gap = True
        self.t_retry = now

    def _setup_battery_monitor(self):
        hw = self.hw
        now = hw.ticks_ms()
        self.t_retry = now
        self.battery_ok = False  # never present zero/stale registers as a sample
        try:
            maker, die = hw.ina_read(REG_MANUFACTURER), hw.ina_read(REG_DIE)
            if maker != MANUFACTURER_TI or die not in (0x2260, 0x2261):
                identity = (maker, die)
                if identity != self._wrong_chip:
                    self._wrong_chip = identity
                    hw.write_line("E,INA226 identity mismatch %04x/%04x" % identity)
                self._battery_fault(now)
                return
            hw.ina_write(REG_CONFIG, INA226_CONFIG)
            if hw.ina_read(REG_CONFIG) != INA226_CONFIG:
                raise OSError("INA226 config readback")
            hw.ina_read(REG_MASK)  # clear old conversion-ready flag
            self.battery_configured = True
            self.t_sample = now
            self.sample_seen = False
        except OSError:
            self._battery_fault(now)

    def _measure(self, now):
        hw = self.hw
        if not self.battery_configured:
            if ticks_diff(now, self.t_retry) >= INA_RETRY_MS:
                self._setup_battery_monitor()
            return
        try:
            self.reports += 1
            if self.reports % INA_CHECK_REPORTS == 0:
                if (hw.ina_read(REG_CONFIG) != INA226_CONFIG
                        or hw.ina_read(REG_MANUFACTURER) != MANUFACTURER_TI
                        or hw.ina_read(REG_DIE) not in (0x2260, 0x2261)):
                    raise OSError("INA226 reset or identity changed")
            if not hw.ina_read(REG_MASK) & CONVERSION_READY:
                if ticks_diff(now, self.t_sample) > INA_STALE_MS:
                    raise OSError("INA226 conversions stopped")
                return
            shunt = hw.ina_read(REG_SHUNT, signed=True)
            bus = hw.ina_read(REG_BUS)
            if shunt in (-32768, 32767) or not 0 <= bus <= 28800:
                raise OSError("INA226 out of measurement range")
        except OSError:
            self._battery_fault(now)
            return
        self.shunt, self.bus = shunt, bus
        elapsed = ticks_diff(now, self.t_sample)
        if self.sample_seen and 0 <= elapsed <= INA_STALE_MS:
            self.charge += shunt * elapsed
        elif self.sample_seen:
            self.charge_gap = True
        self.sample_seen = self.battery_ok = True
        self.t_sample = now

    # ── lines to the Jetson ────────────────────────────────────────

    def _hello(self):
        self.hw.write_line("H,%d,%d,%d,%d,%d,%s" % (
            VERSION, PULSE_MIN_US, PULSE_MAX_US, FAILSAFE_MS, len(SERVO_PINS), CONFIG_ID))

    def _report(self, now):
        flags = ((FLAG_FAILSAFE if self.failsafe else 0)
                 | (0 if self.battery_ok else FLAG_NO_BATTERY)
                 | (FLAG_DEFAULT_STOP if self.default_stop else 0)
                 | (FLAG_CHARGE_GAP if self.charge_gap else 0))
        bus, shunt = (self.bus, self.shunt) if self.battery_ok else (0, 0)
        age = ticks_diff(now, self.t_sample) if self.sample_seen else -1
        values = [now, bus, shunt, self.charge] + self.pulse_us + [flags, self.bad_lines, age]
        self.hw.write_line("P," + ",".join(str(value) for value in values))


class PicoHardware:
    """The real pins. The only part of this file that needs a Pico."""

    # RP2350 watchdog control register, and the address that clears bits in
    # it. From pico-sdk: src/rp2350/hardware_regs, addressmap.h and watchdog.h.
    _WATCHDOG_CTRL_CLEAR = 0x400D8000 + 0x3000
    _WATCHDOG_ENABLE = 0x40000000

    def __init__(self):
        # duty_ns=0 holds the pin low: no pulses until a command says so.
        self._pwm = [machine.PWM(machine.Pin(pin), freq=PWM_HZ, duty_ns=0)
                     for pin in SERVO_PINS]
        self._i2c = machine.I2C(0, sda=machine.Pin(I2C_SDA_PIN),
                                scl=machine.Pin(I2C_SCL_PIN), freq=I2C_HZ, timeout=I2C_TIMEOUT_US)
        self._led = machine.Pin("LED", machine.Pin.OUT)
        # USB serial is the same stream as the Python prompt. poll() says
        # whether a byte is waiting, so reading never blocks the loop.
        self._poll = select.poll()
        self._poll.register(sys.stdin, select.POLLIN)
        self._watchdog = None

    def ticks_ms(self):
        return ticks_ms()

    def read_byte(self):
        if self._poll.poll(0):
            return sys.stdin.buffer.read(1)[0]
        return -1

    def write_line(self, text):
        sys.stdout.write(text + "\n")

    def set_pulse(self, index, microseconds):
        self._pwm[index].duty_ns(microseconds * 1000)

    def set_led(self, on):
        self._led.value(on)

    def ina_read(self, register, signed=False):
        data = self._i2c.readfrom_mem(INA226_ADDRESS, register, 2)
        value = data[0] << 8 | data[1]              # INA226 sends the high byte first
        if signed and value & 0x8000:
            value -= 0x10000
        return value

    def ina_write(self, register, value):
        self._i2c.writeto_mem(INA226_ADDRESS, register, bytes((value >> 8, value & 0xFF)))

    def arm_watchdog(self):
        if self._watchdog is None:
            self._watchdog = machine.WDT(timeout=WATCHDOG_MS)

    def feed_watchdog(self):
        if self._watchdog is not None:
            self._watchdog.feed()

    def disarm_watchdog(self):
        # MicroPython has no call for this. Without it the Pico would
        # restart two seconds after Ctrl-C, in the middle of a file copy.
        if self._watchdog is not None:
            machine.mem32[self._WATCHDOG_CTRL_CLEAR] = self._WATCHDOG_ENABLE


def main():
    hw = PicoHardware()
    # After a power-on nothing was moving, so the outputs start silent.
    # After any other restart (the watchdog) a servo may have been turning
    # when the firmware stopped, so start by sending the stop pulse.
    firmware = Firmware(hw, start_stopped=machine.reset_cause() != machine.PWRON_RESET)
    try:
        while True:
            try:
                firmware.step()
            except KeyboardInterrupt:
                raise
            except Exception as error:              # a bug must not leave a servo turning
                firmware.stop_servos()
                hw.write_line("E," + repr(error))
                sleep_ms(200)                       # watchdog not fed: if this keeps failing, the Pico restarts
            sleep_ms(1)
    except KeyboardInterrupt:
        # Ctrl-C from mpremote or a serial terminal: stop, and hand over the prompt.
        firmware.stop_servos()
        hw.disarm_watchdog()
        hw.write_line("E,stopped by Ctrl-C")


if __name__ == "__main__":
    main()
