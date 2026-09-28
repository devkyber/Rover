"""
IMU readers. Three of them, all with the same interface:

    imu.read() -> {'timestamp': float,   # host clock, seconds
                   'device_t':  float,   # sensor's own clock, seconds
                   'accel': np.array(3), # m/s^2, rover body frame
                   'gyro':  np.array(3), # rad/s,  rover body frame
                   'frame': 'body'}

  FakeIMU     -- synthetic, for desktop work with no hardware
  ArduinoIMU  -- Arduino Nano 33 IoT (LSM6DS3) over USB   <- using this now
  RealIMU     -- BMI088 on the Jetson                     <- not written yet

Sign convention: a sensor lying flat reads accel = [0, 0, +g]. It
measures SPECIFIC FORCE (the reaction pushing up against gravity), not
the gravity vector. The ESKF assumes this everywhere.
"""
import time

import numpy as np

import config


def arduino_sensor_to_body(vector):
    """Rotate one Arduino sensor-frame vector into rover body axes."""
    return np.asarray(config.ARDUINO_IMU_TO_BODY, dtype=float) @ np.asarray(
        vector, dtype=float)


# ── Fake IMU for desktop testing ───────────────────────────────────

class FakeIMU:
    """
    Simulates a stationary IMU sitting flat on a table.
    Accel reads [0, 0, +g], gyro reads noise around zero.
    """
    def __init__(self):
        self.rng = np.random.default_rng(42)
        self.t0 = time.monotonic()

    def read(self):
        gyro_noise = self.rng.normal(0, config.GYRO_NOISE_DENSITY, size=3)
        accel_noise = self.rng.normal(0, config.ACCEL_NOISE_DENSITY, size=3)
        now = time.monotonic()

        return {
            'timestamp': now,
            'device_t':  now - self.t0,
            'accel': np.array([0.0, 0.0, config.GRAVITY]) + accel_noise,
            'gyro':  np.array([0.0, 0.0, 0.0]) + gyro_noise,
            'frame': 'body',
        }

    def close(self):
        pass


# ── Arduino Nano 33 IoT over USB ───────────────────────────────────

class ArduinoIMU:
    """
    Reads the CSV stream from arduino/imu_stream/imu_stream.ino.

    Line format:  I,<micros>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>
    Units and axes on the wire are the LSM6DS3 native ones. We convert
    units and rotate from the board's sensor frame into the rover body
    frame here. The measured mounting matrix is in config.py.

    read() DRAINS the serial buffer and returns the newest sample.
    That matters: the board streams at ~104 Hz whether we ask or not,
    so if we read one line per call the buffer backs up and the "IMU
    data" silently becomes older and older. Draining keeps it fresh and
    the dropped count tells us how much we are throwing away.
    """

    def __init__(self, port=None, baud=None, timeout=1.0):
        import serial  # local import so FakeIMU works without pyserial

        self.port = port or config.IMU_PORT
        self.baud = baud or config.IMU_BAUD
        self.ser = serial.Serial(self.port, self.baud, timeout=timeout)

        self.last = None          # newest good sample
        self.n_parsed = 0         # lines successfully parsed
        self.n_bad = 0            # lines that did not parse
        self.n_dropped = 0        # samples skipped because we drained past them
        self.n_stale = 0          # cycles that had to reuse the previous sample

        self._us_prev = None      # for unwrapping the 32-bit micros() counter
        self._us_wraps = 0

        # Opening the port toggles DTR, which RESETS the board. It then
        # runs the bootloader, starts the sketch, and waits for the USB
        # CDC port to come up. That takes a variable 1-3 s, so a fixed
        # sleep is a race -- wait for real data instead.
        self._wait_for_stream()

    def _wait_for_stream(self, timeout=15.0):
        """Block until the board is actually streaming samples."""
        self.ser.reset_input_buffer()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            if raw.startswith(b'E,'):
                raise RuntimeError(
                    f"IMU reports an error: {raw.decode('ascii', 'replace').strip()}")
            if self._parse(raw) is not None:
                # Got one good sample. Drop whatever queued up during
                # startup so we begin from fresh data.
                self.ser.reset_input_buffer()
                self._us_prev = None
                self._us_wraps = 0
                return
        raise RuntimeError(
            f"No IMU data on {self.port} after {timeout:.0f} s.\n"
            f"  - Is arduino/imu_stream/imu_stream.ino flashed on the board?\n"
            f"  - Does IMU_BAUD ({self.baud}) match SERIAL_BAUD in the sketch?\n"
            f"  - Is another program (Serial Monitor) holding {self.port}?")

    def _device_seconds(self, micros):
        """
        Arduino micros() is uint32 and wraps every ~71.6 minutes.
        Unwrap it into a continuously increasing time in seconds.
        """
        if self._us_prev is not None and micros < self._us_prev - (1 << 31):
            self._us_wraps += 1
        self._us_prev = micros
        return (micros + self._us_wraps * (1 << 32)) / 1e6

    def _parse(self, raw):
        """Parse one line. Returns a sample dict, or None if unusable."""
        try:
            text = raw.decode('ascii', errors='strict').strip()
        except UnicodeDecodeError:
            return None

        if not text.startswith('I,'):
            return None                      # banner ('#') or error ('E,')

        parts = text.split(',')
        if len(parts) != 8:
            return None                      # truncated line

        try:
            micros = int(parts[1])
            vals = [float(v) for v in parts[2:]]
        except ValueError:
            return None

        accel_sensor = np.array(vals[0:3]) * config.ACCEL_G_TO_MS2
        gyro_sensor = np.array(vals[3:6]) * config.GYRO_DPS_TO_RADS
        return {
            'device_t': self._device_seconds(micros),
            'accel': arduino_sensor_to_body(accel_sensor),
            'gyro':  arduino_sensor_to_body(gyro_sensor),
            'frame': 'body',
        }

    def read(self):
        """Drain the buffer, return the newest sample (host-stamped)."""
        newest = None

        # Drain whatever has arrived since the last call.
        while self.ser.in_waiting:
            raw = self.ser.readline()
            if not raw.endswith(b'\n'):
                break                        # partial line; leave it for next time
            sample = self._parse(raw)
            if sample is None:
                self.n_bad += 1
                continue
            if newest is not None:
                self.n_dropped += 1          # we are skipping past this one
            newest = sample
            self.n_parsed += 1

        if newest is None:
            # Nothing new this cycle. Block for one line so we never
            # return None to the filter.
            raw = self.ser.readline()
            newest = self._parse(raw) if raw else None
            if newest is not None:
                self.n_parsed += 1
            elif self.last is not None:
                # Board produced nothing in time. Hand back the previous
                # sample with a fresh host stamp -- device_t is unchanged,
                # which is exactly how the audit spots a stale sample.
                self.n_stale += 1
                return dict(self.last, timestamp=time.monotonic())
            else:
                raise RuntimeError(
                    f"No IMU data on {self.port} (stream stopped after "
                    f"{self.n_parsed} samples). Was the board unplugged?")

        newest['timestamp'] = time.monotonic()
        self.last = newest
        return newest

    def stats(self):
        return {'parsed': self.n_parsed, 'bad': self.n_bad,
                'dropped': self.n_dropped, 'stale': self.n_stale}

    def close(self):
        self.ser.close()


# ── Real IMU (BMI088 on Jetson -- TODO) ────────────────────────────

class RealIMU:
    """
    TODO: Read BMI088 via SPI or I2C on Jetson.

    BMI088 register map:
      Accel: WHO_AM_I = 0x00 (expect 0x1E), data starts at 0x12 (6 bytes, LE)
      Gyro:  WHO_AM_I = 0x00 (expect 0x0F), data starts at 0x02 (6 bytes, LE)

    Accel range +-6g:    1 LSB = 6/32768 * 9.81 m/s^2
    Gyro range +-500dps: 1 LSB = 500/32768 * pi/180 rad/s
    """
    def __init__(self):
        raise NotImplementedError(
            "BMI088 driver not written yet. Use ArduinoIMU for now.")


# ── Factory ────────────────────────────────────────────────────────

def create_imu(kind="arduino"):
    """kind: 'fake' | 'arduino' | 'bmi088'"""
    if kind == "fake":
        print("IMU: FakeIMU (synthetic, no hardware)")
        return FakeIMU()
    if kind == "arduino":
        imu = ArduinoIMU()
        print(f"IMU: Arduino Nano 33 IoT on {imu.port} @ {imu.baud}")
        return imu
    if kind == "bmi088":
        return RealIMU()
    raise ValueError(f"unknown IMU kind: {kind}")


# ── Quick test ─────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    kind = sys.argv[1] if len(sys.argv) > 1 else "arduino"
    imu = create_imu(kind)

    print("\nReading 20 samples...\n")
    prev_dev = None
    for i in range(20):
        d = imu.read()
        a, g = d['accel'], d['gyro']
        gap = "" if prev_dev is None else f"  dt_dev={1000*(d['device_t']-prev_dev):6.2f} ms"
        prev_dev = d['device_t']
        print(f"  [{i:2d}] accel=[{a[0]:+7.3f},{a[1]:+7.3f},{a[2]:+7.3f}] m/s^2  "
              f"gyro=[{g[0]:+7.4f},{g[1]:+7.4f},{g[2]:+7.4f}] rad/s{gap}")
        time.sleep(0.01)

    if hasattr(imu, 'stats'):
        print(f"\nstats: {imu.stats()}")

    d = imu.read()
    mag = np.linalg.norm(d['accel'])
    print(f"\n|accel| = {mag:.3f} m/s^2  (should be ~{config.GRAVITY:.2f} when still)")
    print(f"  {'OK' if abs(mag - config.GRAVITY) < 1.0 else 'CHECK THIS -- not 1 g'}")
    imu.close()
