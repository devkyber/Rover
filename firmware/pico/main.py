"""
Pico 2 firmware (MicroPython): three servo outputs and the battery monitor.

    Jetson --USB serial--> Pico 2 --PWM--> Lift A, Lift B, camera pan
                                  <--I2C-> INA226 on the battery input

Copy this file to the Pico as main.py (firmware/pico/README.md). It then starts by
itself whenever the Pico has power.

WHAT THE PICO DECIDES AND WHAT IT DOES NOT
------------------------------------------
The Pico knows nothing about the rover. It is told pulse widths in
microseconds and it reports raw INA226 register values. Servo trim, servo
direction and the shunt resistance all live in rover/config.py, so they
can be changed without touching the Pico.

The one decision made here is the failsafe, because it has to work when
the Jetson does not: if no valid S line arrives for FAILSAFE_MS, every
servo gets its stop pulse. A continuous-rotation servo keeps turning for
as long as it is told to, so "the last command" must never outlive the
program that sent it.

LINES ON THE WIRE (ASCII, comma separated, one per line)
--------------------------------------------------------
Jetson -> Pico
    S,<a>,<b>,<pan>    pulse width for each servo in microseconds; 0 = no pulses
    N,<a>,<b>,<pan>    the stop pulse for each servo, used by the failsafe
    Z                  set the charge counter to zero
    ?                  send the H line again

Pico -> Jetson
    H,<version>,<pulse min>,<pulse max>,<failsafe ms>       once at start
    P,<ms>,<bus>,<shunt>,<charge>,<a>,<b>,<pan>,<flags>,<bad>   20 a second
    E,<text>                                                something is wrong

    ms      time since the Pico started
    bus     INA226 bus voltage register, 1.25 mV per count
    shunt   INA226 shunt voltage register, 2.5 uV per count, signed
    charge  sum of shunt x milliseconds since start or the last Z
    a b pan the pulse width each output is producing now
    flags   1 = failsafe (no S line for FAILSAFE_MS)
            2 = battery reading unavailable (bus and shunt are then 0)
            4 = no N line received yet; the stop pulse is DEFAULT_STOP_US
    bad     lines from the Jetson that were rejected

A line that is not exactly right (wrong field count, not a number, a pulse
outside PULSE_MIN_US..PULSE_MAX_US) is rejected whole and counted in `bad`.

The same file runs on a desktop for firmware/pico/test_firmware.py and the
simulated Pico in rover/pico_link.py: only PicoHardware touches the pins.
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

VERSION = 1

# Pins, from docs/electronics/PDB_Design_Handover.md section 4.
SERVO_PINS = (0, 1, 2)                  # GP0 Lift A (J2), GP1 Lift B (J3), GP2 camera pan (J4)
I2C_SDA_PIN = 16
I2C_SCL_PIN = 17

PWM_HZ = 50                             # hobby servo frame rate
# Sanity limits only: anything outside is not a servo pulse at all. The
# range the servos are actually driven over is set in rover/config.py.
PULSE_MIN_US = 500
PULSE_MAX_US = 2500
DEFAULT_STOP_US = 1500                  # nominal stop pulse, until an N line says better

FAILSAFE_MS = 300                       # no S line for this long -> stop pulses
REPORT_MS = 50                          # one P line every 50 ms
# Firmware stuck for this long -> the Pico restarts. Longer than the worst
# case of a healthy loop: a USB write to a Jetson that has stopped reading
# can wait 0.5 s before MicroPython gives up on it.
WATCHDOG_MS = 2000
LINE_MAX = 40                           # longest valid line is 21 characters

# ── INA226 (TI datasheet SBOS547, section 7.6) ─────────────────────
INA226_ADDRESS = 0x40                   # A0 and A1 to GND
REG_CONFIG = 0x00
REG_SHUNT = 0x01
REG_BUS = 0x02
REG_MANUFACTURER = 0xFE
MANUFACTURER_TI = 0x5449                # "TI"
# Average 16 conversions of 1.1 ms each, shunt and bus, continuously: one
# new reading every 35 ms, each the mean over those 35 ms. Motor current is
# spiky, and a mean is what the charge counter needs. (Reset value 0x4127
# is the same without averaging.)
INA226_CONFIG = 0x4527
INA_RETRY_MS = 1000                     # after a failed read, try again this often
INA_CHECK_REPORTS = 20                  # re-check the configuration every 20 reports (1 s)

FLAG_FAILSAFE = 1
FLAG_NO_BATTERY = 2
FLAG_DEFAULT_STOP = 4


def parse_pulses(fields):
    """['1500', '1500', '0'] -> [1500, 1500, 0], or None if anything is off."""
    if len(fields) != 3:
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
        self.stop_us = [DEFAULT_STOP_US] * 3
        self.pulse_us = [-1, -1, -1]    # unknown until the first _output() below
        self.failsafe = True            # nothing commanded yet
        self.default_stop = True        # no N line yet
        self.driven = False             # True once any servo has been sent pulses
        self.t_command = now
        self.t_report = now

        self.battery_ok = False
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
            self._output((0, 0, 0))
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
            self._report(now)
            # LED: steady while commands arrive, blinking once a second in failsafe.
            hw.set_led(not self.failsafe or (now // 500) % 2 == 0)

        hw.feed_watchdog()

    def stop_servos(self):
        """
        The failsafe: stop pulses on all three outputs. If no output has
        carried a pulse since power-on, they stay silent.
        """
        self.failsafe = True
        self._output(self.stop_us if self.driven else (0, 0, 0))

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
            if pulses is None:
                self.bad_lines += 1
            elif kind == "S":
                self.t_command = now
                self.failsafe = False
                if pulses != [0, 0, 0]:
                    self.driven = True
                self._output(pulses)
                # From the first command on, a stuck firmware would leave
                # a servo turning. Armed here and not at start, so that a
                # Pico nobody is commanding can never restart in a loop.
                self.hw.arm_watchdog()
            else:
                self.stop_us = pulses
                self.default_stop = False
                if self.failsafe and self.driven:
                    self._output(pulses)
        elif line == "Z":
            self.charge = 0
        elif line == "?":
            self._hello()
        else:
            self.bad_lines += 1

    def _output(self, pulses):
        for i in range(3):
            if pulses[i] != self.pulse_us[i]:
                self.pulse_us[i] = pulses[i]
                self.hw.set_pulse(i, pulses[i])

    # ── battery monitor ────────────────────────────────────────────

    def _setup_battery_monitor(self):
        hw = self.hw
        try:
            maker = hw.ina_read(REG_MANUFACTURER)
            if maker != MANUFACTURER_TI:
                if maker != self._wrong_chip:       # say it once, not on every retry
                    self._wrong_chip = maker
                    hw.write_line("E,chip at 0x40 is not an INA226 (id 0x%04x)" % maker)
                self.battery_ok = False
                return
            hw.ina_write(REG_CONFIG, INA226_CONFIG)
            self.battery_ok = True
        except OSError:
            self.battery_ok = False

    def _measure(self, now):
        hw = self.hw
        if not self.battery_ok:
            if ticks_diff(now, self.t_retry) < INA_RETRY_MS:
                return
            self.t_retry = now
            self._setup_battery_monitor()
            self.t_sample = now         # charge is not counted across the outage
            return
        try:
            self.reports += 1
            if self.reports % INA_CHECK_REPORTS == 0 and hw.ina_read(REG_CONFIG) != INA226_CONFIG:
                # The INA226 lost power and came back with its reset
                # settings. Its readings stay valid; only the averaging is gone.
                hw.ina_write(REG_CONFIG, INA226_CONFIG)
            shunt = hw.ina_read(REG_SHUNT, signed=True)
            bus = hw.ina_read(REG_BUS)
        except OSError:
            self.battery_ok = False
            self.t_retry = now
            return
        self.shunt, self.bus = shunt, bus
        self.charge += shunt * ticks_diff(now, self.t_sample)
        self.t_sample = now

    # ── lines to the Jetson ────────────────────────────────────────

    def _hello(self):
        self.hw.write_line("H,%d,%d,%d,%d" % (VERSION, PULSE_MIN_US, PULSE_MAX_US, FAILSAFE_MS))

    def _report(self, now):
        flags = ((FLAG_FAILSAFE if self.failsafe else 0)
                 | (0 if self.battery_ok else FLAG_NO_BATTERY)
                 | (FLAG_DEFAULT_STOP if self.default_stop else 0))
        bus, shunt = (self.bus, self.shunt) if self.battery_ok else (0, 0)
        self.hw.write_line("P,%d,%d,%d,%d,%d,%d,%d,%d,%d" % (
            now, bus, shunt, self.charge,
            self.pulse_us[0], self.pulse_us[1], self.pulse_us[2], flags, self.bad_lines))


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
                                scl=machine.Pin(I2C_SCL_PIN), freq=100_000)
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
