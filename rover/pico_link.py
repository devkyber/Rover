"""
The Jetson's side of the Pico 2 I/O board (firmware: firmware/pico/main.py).

    pico = pico_link.open_pico("usb", on_event=print)
    pico.start()
    pico.set_servos(lift, pan)   # both -1..+1; call it every control cycle
    pico.state()                 # battery and servo outputs, for telemetry
    pico.close()

Bench check, with the Pico on USB:

    python pico_link.py                          # watch the battery reading
    python pico_link.py --lift 0.2 --seconds 3   # run the lift servos for 3 s
    python pico_link.py --pulse 1500 1500 1500   # raw pulse widths, to find each stop pulse
    python pico_link.py fake                     # no Pico: this computer plays one

A thread owns the serial port, the same way camera.py owns the camera. If
the Pico is unplugged it keeps retrying, so plugging it back in needs no
restart, and the control loop never waits on USB.

THE SERVO DEADMAN HAS TWO HALVES
--------------------------------
The Pico stops the servos when S lines stop arriving (300 ms). That covers
a dead Jetson or a pulled cable. It does not cover a control loop that has
hung while this thread is still alive: so the thread sends a command only
while set_servos() has been called within the last SETPOINT_MAX_AGE_S.
A loop that stops calling stops the servos.

UNITS
-----
The Pico sends raw INA226 register counts. They become volts and amps
here, with the shunt resistance from config.py, so the numbers can be
checked in one place and the firmware never needs editing for a
different shunt.
"""
import argparse
import collections
import importlib.util
import math
import os
import queue
import threading
import time

import config

# INA226 register scale (TI datasheet SBOS547). Facts about the chip, not
# settings: the shunt resistance, which is a setting, is in config.py.
BUS_VOLTS_PER_COUNT = 1.25e-3
SHUNT_VOLTS_PER_COUNT = 2.5e-6

FIRMWARE_VERSION = 1             # the H line this file was written against
FLAG_FAILSAFE, FLAG_NO_BATTERY, FLAG_DEFAULT_STOP = 1, 2, 4

SETPOINT_MAX_AGE_S = 0.2         # set_servos() older than this is not sent
REPORT_TIMEOUT_S = 0.5           # the Pico reports every 50 ms; this long without = lost
RETRY_S = 2

FIRMWARE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "firmware", "pico", "main.py")


def servo_pulses(lift, pan):
    """Operator commands, -1..+1, to a pulse width for each servo (microseconds)."""
    pulses = []
    for (_, stop, offset, direction), command in zip(config.SERVOS, (lift, lift, pan)):
        command = max(-1.0, min(1.0, command)) if math.isfinite(command) else 0.0
        pulses.append(round(stop + direction * command * offset))
    return pulses


def stop_pulses():
    return [servo[1] for servo in config.SERVOS]


class PicoLink:
    def __init__(self, port_factory, on_event=print):
        self._port_factory = port_factory    # () -> (name, open port) or None if absent
        self.on_event = on_event
        self._lock = threading.Lock()
        self._pulses = None                  # what the control loop wants, microseconds
        self._pulses_at = -math.inf
        self._zero_charge = False
        self._report = None                  # newest P line, as a dict
        self._report_at = -math.inf
        self._running = False
        self._thread = None
        self.bad_lines = 0                   # lines from the Pico that did not parse

    # ── called by the control loop ─────────────────────────────────

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_servos(self, lift, pan):
        """Speed commands, -1..+1: both lift servos together, and the camera pan."""
        self.set_pulses(servo_pulses(lift, pan))

    def set_pulses(self, pulses):
        with self._lock:
            self._pulses = list(pulses)
            self._pulses_at = time.monotonic()

    def zero_charge(self):
        """Restart the mAh count, e.g. after fitting a fresh battery."""
        with self._lock:
            self._zero_charge = True

    def state(self):
        """
        For telemetry. 'battery' is None when the Pico is there but its
        INA226 is not answering: the reading is shown as unavailable, never
        as a stale or zero value.
        """
        with self._lock:
            report = self._report
            fresh = time.monotonic() - self._report_at < REPORT_TIMEOUT_S
        if report is None or not fresh:
            return {"status": "lost", "battery": None, "servo_us": None, "failsafe": None}
        return {"status": "ok", "battery": report["battery"],
                "servo_us": report["servo_us"], "failsafe": report["failsafe"]}

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    # ── the port thread ────────────────────────────────────────────

    def _run(self):
        while self._running:
            try:
                opened = self._port_factory()
            except OSError:
                opened = None                # not plugged in, or held by another program
            if opened is None:
                time.sleep(RETRY_S)
                continue
            name, port = opened
            try:
                self._talk(name, port)
            except OSError:                  # unplugged mid-run
                pass
            finally:
                try:
                    port.close()
                except OSError:
                    pass
            if self._running:
                self.on_event("pico: connection lost, retrying")
                time.sleep(RETRY_S)

    def _talk(self, name, port):
        port.write(b"?\n")                   # ask who is there; the answer is an H line
        announced = False
        next_send = 0.0
        last_ms = None
        try:
            while self._running:
                raw = port.readline()        # returns within the port's short timeout
                if raw:
                    kind, value = self._parse(raw)
                    if kind == "P":
                        if last_ms is not None and value["ms"] < last_ms:
                            self.on_event("pico: restarted")
                        last_ms = value["ms"]
                        with self._lock:
                            self._report, self._report_at = value, time.monotonic()
                    elif kind == "H":
                        if not announced:
                            announced = True
                            self.on_event(f"pico: connected on {name}, firmware v{value}")
                        if value != FIRMWARE_VERSION:
                            self.on_event(f"pico: firmware v{value}, this program expects "
                                          f"v{FIRMWARE_VERSION} -- copy firmware/pico/main.py to the Pico")
                    elif kind == "E":
                        self.on_event("pico: " + value)
                    else:
                        self.bad_lines += 1

                now = time.monotonic()
                if now < next_send:
                    continue
                next_send = now + 1.0 / config.PICO_COMMAND_HZ
                with self._lock:
                    report = self._report
                    pulses = (self._pulses
                              if now - self._pulses_at < SETPOINT_MAX_AGE_S else None)
                    zero, self._zero_charge = self._zero_charge, False
                # A Pico that has just (re)started only knows the nominal
                # stop pulse. Give it the trimmed ones before anything else.
                if report is None or report["default_stop"]:
                    port.write(_line("N", stop_pulses()))
                if zero:
                    port.write(b"Z\n")
                if pulses is not None:
                    port.write(_line("S", pulses))
        finally:
            # Stop now rather than 300 ms from now, when the failsafe would.
            try:
                port.write(_line("S", stop_pulses()))
            except OSError:
                pass
            with self._lock:
                self._report_at = -math.inf

    def _parse(self, raw):
        """One line from the Pico -> ('P', report) | ('H', version) | ('E', text) | (None, None)."""
        try:
            fields = raw.decode("ascii").strip().split(",")
        except UnicodeDecodeError:
            return None, None
        try:
            if fields[0] == "P" and len(fields) == 10:
                ms, bus, shunt, charge, a, b, pan, flags, bad = (int(f) for f in fields[1:])
                return "P", {
                    "ms": ms,
                    "battery": (None if flags & FLAG_NO_BATTERY
                                else battery_reading(bus, shunt, charge)),
                    "servo_us": [a, b, pan],
                    "failsafe": bool(flags & FLAG_FAILSAFE),
                    "default_stop": bool(flags & FLAG_DEFAULT_STOP),
                    "rejected": bad,         # our lines the Pico refused
                }
            if fields[0] == "H" and len(fields) == 5:
                return "H", int(fields[1])
        except ValueError:
            return None, None
        if fields[0] == "E":
            return "E", ",".join(fields[1:])
        return None, None


def _line(kind, pulses):
    return (kind + "," + ",".join(str(int(p)) for p in pulses) + "\n").encode("ascii")


def battery_reading(bus, shunt, charge):
    """Raw INA226 counts -> volts, amps, watts and mAh used."""
    volts = bus * BUS_VOLTS_PER_COUNT
    amps = shunt * SHUNT_VOLTS_PER_COUNT / config.BATTERY_SHUNT_OHM
    # charge is shunt counts x milliseconds. In amps that is A*ms;
    # 1 mAh = 3600 A*ms.
    mah = charge * SHUNT_VOLTS_PER_COUNT / config.BATTERY_SHUNT_OHM / 3600.0
    return {"v": round(volts, 3), "a": round(amps, 3), "w": round(volts * amps, 2),
            "mah": round(mah, 1), "cell_v": round(volts / config.BATTERY_CELLS, 3)}


class NoPico:
    """Stands in when the Pico is switched off with --pico off."""

    def start(self):
        pass

    def set_servos(self, lift, pan):
        pass

    def zero_charge(self):
        pass

    def state(self):
        return {"status": "off", "battery": None, "servo_us": None, "failsafe": None}

    def close(self):
        pass


# ── the real port ──────────────────────────────────────────────────

def _open_usb():
    import serial                    # local import: 'fake' and 'off' need no pyserial
    from serial.tools import list_ports

    name = config.PICO_PORT
    if name is None:
        name = next((p.device for p in list_ports.comports()
                     if (p.vid, p.pid) == config.PICO_USB_ID), None)
        if name is None:
            return None
    # The baud rate means nothing on USB serial; pyserial just wants one.
    port = serial.Serial(name, 115200, timeout=0.02, write_timeout=0.5)
    port.reset_input_buffer()
    return name, port


# ── a Pico played by this computer ─────────────────────────────────

class _SimulatedHardware:
    """The pins firmware/pico/main.py expects, with a battery under a steady load behind them."""

    def __init__(self, firmware):
        self._fw = firmware
        self._t0 = time.monotonic()
        self._config = 0x4127                # INA226 reset value
        self.to_pico = collections.deque()   # bytes, host -> firmware
        self.to_host = queue.Queue()         # whole lines, firmware -> host
        self.pulse_us = [0, 0, 0]

    def ticks_ms(self):
        return int((time.monotonic() - self._t0) * 1000)

    def read_byte(self):
        try:
            return self.to_pico.popleft()
        except IndexError:
            return -1

    def write_line(self, text):
        self.to_host.put(text.encode("ascii") + b"\r\n")

    def set_pulse(self, index, microseconds):
        self.pulse_us[index] = microseconds

    def set_led(self, on):
        pass

    def ina_read(self, register, signed=False):
        fw = self._fw
        if register == fw.REG_MANUFACTURER:
            return fw.MANUFACTURER_TI
        if register == fw.REG_CONFIG:
            return self._config
        seconds = time.monotonic() - self._t0
        if register == fw.REG_BUS:           # a full 6S pack, sagging slowly
            return round((25.0 - 0.002 * seconds) / BUS_VOLTS_PER_COUNT)
        # 1.5 A for the Jetson, plus 0.3 A for each servo that is turning.
        turning = sum(1 for us, servo in zip(self.pulse_us, config.SERVOS)
                      if us and us != servo[1])
        amps = 1.5 + 0.3 * turning
        return round(amps * config.BATTERY_SHUNT_OHM / SHUNT_VOLTS_PER_COUNT)

    def ina_write(self, register, value):
        self._config = value

    def arm_watchdog(self):
        pass

    def feed_watchdog(self):
        pass


class _SimulatedPort:
    """
    Looks like a serial port; behind it the real firmware file runs on a
    simulated Pico. So `--pico fake` exercises the same line protocol and
    the same failsafe the rover will meet, not a second copy of them.
    """

    def __init__(self, timeout=0.02):
        self._timeout = timeout
        fw = load_firmware()
        self.hardware = _SimulatedHardware(fw)
        self.firmware = fw.Firmware(self.hardware)
        self._running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self._running:
            self.firmware.step()
            time.sleep(0.001)

    def readline(self):
        try:
            return self.hardware.to_host.get(timeout=self._timeout)
        except queue.Empty:
            return b""

    def write(self, data):
        self.hardware.to_pico.extend(data)

    def close(self):
        self._running = False


def load_firmware():
    """Import firmware/pico/main.py as a module. It only starts by itself on a Pico."""
    spec = importlib.util.spec_from_file_location("pico_firmware", FIRMWARE_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ── factory ────────────────────────────────────────────────────────

def open_pico(kind="usb", on_event=print):
    """kind: 'usb' | 'fake' | 'off'"""
    if kind == "off":
        return NoPico()
    if kind == "fake":
        return PicoLink(lambda: ("simulated Pico", _SimulatedPort()), on_event)
    if kind == "usb":
        return PicoLink(_open_usb, on_event)
    raise ValueError(f"unknown Pico kind: {kind}")


# ── bench check ────────────────────────────────────────────────────

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Bench check for the Pico 2 I/O board.")
    ap.add_argument("kind", nargs="?", default="usb", choices=["usb", "fake"])
    ap.add_argument("--lift", type=float, default=0.0, help="lift command, -1..+1")
    ap.add_argument("--pan", type=float, default=0.0, help="pan command, -1..+1")
    ap.add_argument("--pulse", type=int, nargs=3, metavar=("A", "B", "PAN"),
                    help="raw pulse widths in microseconds instead of --lift/--pan")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="how long to send the command (0 = send nothing, just watch)")
    ap.add_argument("--zero", action="store_true", help="restart the mAh count")
    args = ap.parse_args()

    pico = open_pico(args.kind)
    pico.start()
    if args.zero:
        pico.zero_charge()
    driving = args.seconds > 0
    wanted = args.pulse or servo_pulses(args.lift, args.pan)
    if driving:
        print(f"Sending {wanted} us for {args.seconds:.1f} s, then nothing. "
              "The servos must stop by themselves 0.3 s later.")
    print("Ctrl+C to quit.\n")
    t0 = time.monotonic()
    next_print = 0.0
    try:
        while True:
            now = time.monotonic() - t0
            if driving and now < args.seconds:
                pico.set_pulses(wanted)
            if now >= next_print:
                next_print = now + 0.5
                s = pico.state()
                if s["status"] != "ok":
                    print(f"{now:6.1f} s  no Pico yet (is main.py on it? is another "
                          "program holding the port?)")
                else:
                    b = s["battery"]
                    battery = (f"{b['v']:6.2f} V ({b['cell_v']:.2f}/cell) {b['a']:+7.3f} A "
                               f"{b['w']:6.1f} W {b['mah']:7.1f} mAh" if b
                               else "battery reading unavailable")
                    print(f"{now:6.1f} s  {battery}   servos {s['servo_us']} us"
                          f"{'  FAILSAFE' if s['failsafe'] else ''}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        pico.close()
