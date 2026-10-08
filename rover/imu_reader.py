"""
IMU readers. Three of them, all with the same interface:

    imu.read() -> {'timestamp': float,   # host clock, seconds
                   'device_t':  float,   # sensor's own clock, nominal seconds
                   'accel': np.array(3), # m/s^2, rover body frame
                   'gyro':  np.array(3), # rad/s,  rover body frame
                   'frame': 'body'}

  FakeIMU     -- synthetic, for desktop work with no hardware
  ArduinoIMU  -- Arduino Nano 33 IoT (LSM6DS3) over USB   <- using this now
  RealIMU     -- BMI088 over SPI, gyro FIFO + latest filtered accel

BMI088 also returns dt and gyro_samples (ordered FIFO rates, rad/s).
Use each rate for dt / len(gyro_samples), not just the last rate. Its
timestamp is the host FIFO-count read midpoint; device_t is the separate
accelerometer clock and MUST NOT time gyro integration. See RealIMU.

Sign convention: a sensor lying flat reads accel = [0, 0, +g]. It
measures SPECIFIC FORCE (the reaction pushing up against gravity), not
the gravity vector. The ESKF assumes this everywhere.
"""
import hashlib
from pathlib import Path
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
    Reads the CSV stream from firmware/imu_stream/imu_stream.ino.

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
            f"  - Is firmware/imu_stream/imu_stream.ino flashed on the board?\n"
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


# ── BMI088 on Jetson ──────────────────────────────────────────────

class IMUError(RuntimeError):
    """Acquisition failed; restart the reader and stationary alignment."""

class RealIMU:
    """
    BMI088 SPI reader. read() drains every queued gyro frame and returns
    their mean for bias/stillness observations plus the ordered gyro_samples
    for quaternion prediction. Bias remains included for startup/ZARU.

    FIFO frames have no timestamps. We distribute them uniformly across
    the measured host interval between FIFO-count snapshots. This preserves
    total host elapsed time and all rates, but has about one ODR period of
    phase uncertainty (plus bus/scheduling delay). It is not hardware sync.
    Accel is the latest 40 Hz low-pass reading, held across the batch.

    Fail closed on reset/config drift, stale accel, FIFO overrun, clipping,
    excessive gap or transport error. No automatic reset/hidden lost motion.
    sensor_frame=True is for bench diagnostics ONLY and reports that frame.
    """
    def __init__(self, *, sensor_frame=False, spi_factory=None,
                 clock=time.monotonic, sleep=time.sleep):
        self._clock, self._sleep = clock, sleep
        self._devices = []
        self._closed = False
        self._fault = None
        self.n_parsed = self.n_frames = self.n_bad = 0
        self._max_gap = 0.0
        self._startup_discarded = 0
        self._startup_id_mismatches = 0
        self._last_tick = None
        self._ticks = 0
        self.frame = 'bmi088_sensor' if sensor_frame else 'body'
        matrix = np.eye(3) if sensor_frame else config.BMI088_IMU_TO_BODY
        if matrix is None:
            raise ValueError("Set verified BMI088_IMU_TO_BODY in config.py first; "
                             "use imu_reader.py bmi088 --sensor-frame for bench diagnostics.")
        self._rotation = np.asarray(matrix, dtype=float)
        if (self._rotation.shape != (3, 3)
                or not np.isfinite(self._rotation).all()
                or not np.allclose(self._rotation @ self._rotation.T, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(self._rotation), 1.0, atol=1e-6)):
            raise ValueError("BMI088_IMU_TO_BODY must be a finite proper rotation (det +1)")
        if config.IMU_RATE_HZ != 400 or config.BMI088_ACCEL_CONF != 0x8A or config.BMI088_GYRO_BANDWIDTH != 3:
            raise ValueError("This FIFO profile requires 400 Hz accel OSR4 and 400 Hz gyro / 47 Hz")
        accel_ranges = {3: 0, 6: 1, 12: 2, 24: 3}
        gyro_ranges = {2000: 0, 1000: 1, 500: 2, 250: 3, 125: 4}
        self._acc_range = accel_ranges[config.BMI088_ACCEL_RANGE_G]
        self._gyr_range = gyro_ranges[config.BMI088_GYRO_RANGE_DPS]
        self._acc_scale = config.BMI088_ACCEL_RANGE_G * 9.80665 / 32768.0
        self._gyr_scale = np.deg2rad(config.BMI088_GYRO_RANGE_DPS) / 32768.0
        try:
            if spi_factory is None:
                import spidev
                spi_factory = spidev.SpiDev
                self._lock_devices = True
            else:
                self._lock_devices = False  # injected test transport
            self.acc = self._open(spi_factory, config.BMI088_ACCEL_CS)
            self.gyr = self._open(spi_factory, config.BMI088_GYRO_CS)
            self._initialize()
        except BaseException:
            self.close()
            raise

    def _open(self, factory, cs):
        dev = factory()
        self._devices.append(dev)
        dev.open(config.BMI088_SPI_BUS, cs)
        if self._lock_devices:
            import fcntl
            fcntl.flock(dev.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        dev.mode = config.BMI088_SPI_MODE
        dev.max_speed_hz = config.BMI088_SPI_HZ
        dev.bits_per_word = 8
        dev.lsbfirst = False
        dev.cshigh = False
        return dev

    def _read(self, dev, address, count=1):
        dummy = 2 if dev is self.acc else 1
        # spidev 3.8 mutates the TX list. A fresh buffer on EVERY transfer.
        rx = list(dev.xfer2([address | 0x80] + [0] * (count + dummy - 1)))
        if len(rx) != count + dummy:
            raise IMUError("BMI088 short SPI read")
        return rx[dummy:]

    @staticmethod
    def _write(dev, address, value):
        dev.xfer2([address & 0x7F, value])

    def _wait_ids(self):
        # After opening/setting the controller, initial reads on this Jetson
        # sometimes return 00/FF before valid IDs. Bound startup readiness;
        # require two consecutive good pairs before any configuration write.
        # This is NOT a retry/recovery policy for a running estimator.
        consecutive = 0
        for _ in range(12):
            ids = (self._read(self.acc, 0)[0], self._read(self.gyr, 0)[0])
            if ids == (0x1E, 0x0F):
                consecutive += 1
                if consecutive == 2:
                    return
            else:
                consecutive = 0
                self._startup_id_mismatches += 1
            self._sleep(.01)
        raise IMUError(f"BMI088 startup IDs not stable: {ids}; check wiring/power")

    def _initialize(self):
        self._read(self.acc, 0)  # first read selects accel SPI
        self._sleep(0.01)
        self._wait_ids()
        self._write(self.acc, 0x7E, 0xB6)
        self._write(self.gyr, 0x14, 0xB6)
        self._sleep(0.05)
        self._read(self.acc, 0)
        self._sleep(0.01)
        self._wait_ids()
        for dev, address, value in (
            (self.acc, 0x7C, 0), (self.acc, 0x7D, 4),
            (self.acc, 0x40, config.BMI088_ACCEL_CONF),
            (self.acc, 0x41, self._acc_range),
            (self.gyr, 0x11, 0), (self.gyr, 0x0F, self._gyr_range),
            (self.gyr, 0x10, config.BMI088_GYRO_BANDWIDTH),
        ):
            self._write(dev, address, value)
            self._sleep(0.01)
        self._sleep(0.10)  # filter settling; not a thermal/bias calibration
        self._write(self.gyr, 0x3E, 0x40)  # FIFO, XYZ, six bytes/frame
        self._health()
        self._write(self.gyr, 0x3E, 0x40)  # discard startup samples, anchor host interval
        self._previous_host = self._clock()

    def _health(self):
        # Burst adjacent registers to bound SPI ioctl overhead at 100 Hz.
        acc_status = self._read(self.acc, 0, 4)
        gyro_status = self._read(self.gyr, 0, 18)
        if (acc_status[0], gyro_status[0]) != (0x1E, 0x0F):
            raise IMUError("BMI088 chip identity lost during acquisition")
        if acc_status[2] != 0:
            raise IMUError("BMI088 accelerometer error flag")
        if (gyro_status[0x0F] != self._gyr_range
                or gyro_status[0x10] & 0x7F != config.BMI088_GYRO_BANDWIDTH
                or gyro_status[0x11] != 0):
            raise IMUError("BMI088 gyro config/reset")
        expected = ((self.acc, 0x7C, [0, 4]),
                    (self.acc, 0x40, [config.BMI088_ACCEL_CONF, self._acc_range]),
                    (self.gyr, 0x3E, [0x40]))
        for dev, address, values in expected:
            actual = self._read(dev, address, len(values))
            if actual != values:
                raise IMUError(f"BMI088 config/reset at 0x{address:02X}: {actual} != {values}")

    def read(self):
        if self._closed or self._fault:
            raise IMUError(f"BMI088 reader unavailable: {self._fault or 'closed'}")
        try:
            return self._acquire()
        except (OSError, RuntimeError, ValueError) as exc:
            self.n_bad += 1
            self._fault = str(exc)
            raise IMUError(f"BMI088 stopped: {exc}; restart and realign") from exc

    def _acquire(self):
        start = self._clock()
        if self.n_parsed == 0 and start - self._previous_host > config.BMI088_MAX_GAP_S:
            # Camera/link/log setup can take seconds after construction.
            # The first estimator sample starts acquisition; data preceding
            # it is startup history, not part of an integrated interval.
            self._health()
            self._startup_discarded = self._read(self.gyr, 0x0E)[0] & 0x7F
            self._write(self.gyr, 0x3E, 0x40)
            self._previous_host = start = self._clock()
        if start - self._previous_host > config.BMI088_MAX_GAP_S:
            raise IMUError("acquisition gap exceeds limit")
        self._health()
        deadline = start + config.BMI088_READ_TIMEOUT_S
        while True:
            before = self._clock()
            status = self._read(self.gyr, 0x0E)[0]
            host = (before + self._clock()) / 2.0
            count = status & 0x7F
            if status & 0x80 or count >= 100:
                raise IMUError("gyro FIFO overflow/full; motion was lost")
            if count and self._read(self.acc, 3)[0] & 0x80:
                break
            if self._clock() >= deadline:
                raise IMUError("timeout waiting for fresh gyro/accel")
            self._sleep(0.0002)
        dt = host - self._previous_host
        if not 0 < dt <= config.BMI088_MAX_GAP_S:
            raise IMUError("invalid host acquisition interval")
        # +/- two frames covers count-snapshot phase and startup. Larger
        # discrepancy indicates a stall/reset/rate error, not a scale fix.
        if abs(count - dt * config.IMU_RATE_HZ) > 2.0:
            raise IMUError("gyro FIFO count inconsistent with host elapsed time")
        gyro_bytes = self._read(self.gyr, 0x3F, count * 6)
        gyro_raw = np.frombuffer(bytes(gyro_bytes), dtype='<i2').reshape(count, 3).astype(int)
        before = self._clock()
        accel_bytes = self._read(self.acc, 0x12, 9)
        accel_host = (before + self._clock()) / 2.0
        accel_raw = np.frombuffer(bytes(accel_bytes[:6]), dtype='<i2').astype(int)
        tick = int.from_bytes(bytes(accel_bytes[6:]), 'little')
        if self._last_tick is not None:
            delta = (tick - self._last_tick) & 0xFFFFFF
            if delta == 0 or delta * 39.0625e-6 > config.BMI088_MAX_GAP_S * 1.5:
                raise IMUError("accel sensor clock stopped/reset or excessive gap")
            self._ticks += delta
        else:
            self._ticks = tick
        if np.any(np.abs(gyro_raw) >= 32760) or np.any(np.abs(accel_raw) >= 32760):
            raise IMUError("sensor saturation (near full-scale)")
        temp = self._read(self.acc, 0x22, 2)
        value = (temp[0] << 3) | (temp[1] >> 5)
        temperature = 23.0 + (value - 2048 if value >= 1024 else value) * 0.125
        # Detect reset/config changes during acquisition before publishing.
        self._health()
        if self._read(self.gyr, 0x0E)[0] & 0x80:
            raise IMUError("gyro FIFO overrun during read")
        if self._clock() - start > config.BMI088_READ_TIMEOUT_S:
            raise IMUError("SPI acquisition latency exceeds limit")
        rates = (gyro_raw * self._gyr_scale) @ self._rotation.T
        self._previous_host, self._last_tick = host, tick
        self.n_parsed += 1
        self.n_frames += count
        self._max_gap = max(self._max_gap, dt)
        return {'timestamp': host, 'device_t': self._ticks * 39.0625e-6,
                'accel': self._rotation @ (accel_raw * self._acc_scale),
                'gyro': rates.mean(axis=0), 'frame': self.frame,
                'dt': dt, 'gyro_samples': rates.tolist(),
                'accel_timestamp': accel_host, 'temperature_c': temperature,
                'sensor_tick': tick, 'accel_raw': accel_raw.tolist(),
                'gyro_raw': gyro_raw.tolist(), 'source': 'bmi088'}

    def metadata(self):
        """Run provenance; snapshots allow later interpretation of raw counts."""
        import platform
        from importlib.metadata import PackageNotFoundError, version
        try:
            spi_version = version('spidev')
        except PackageNotFoundError:
            spi_version = None
        sources = {}
        for name in ('imu_reader.py', 'config.py', 'odometry.py', 'eskf.py', 'logger.py'):
            path = Path(__file__).with_name(name)
            data = path.read_bytes()
            sources[name] = {'sha256': hashlib.sha256(data).hexdigest(),
                             'text': data.decode('utf-8')}
        return {'sensor': 'BMI088', 'chip_ids': [30, 15], 'frame': self.frame,
                'runtime': {'python': platform.python_version(), 'numpy': np.__version__,
                            'spidev': spi_version, 'platform': platform.platform()},
                'sensor_to_output': self._rotation.tolist(),
                'spi_bus': config.BMI088_SPI_BUS,
                'chip_selects': [config.BMI088_ACCEL_CS, config.BMI088_GYRO_CS],
                'spi_requested_hz': config.BMI088_SPI_HZ, 'spi_mode': config.BMI088_SPI_MODE,
                'odr_hz': config.IMU_RATE_HZ, 'accel_conf': config.BMI088_ACCEL_CONF,
                'gyro_bandwidth': config.BMI088_GYRO_BANDWIDTH,
                'accel_range_g': config.BMI088_ACCEL_RANGE_G,
                'gyro_range_dps': config.BMI088_GYRO_RANGE_DPS,
                'gyro_timing': 'host FIFO-count intervals divided by frame count; estimated',
                'accel_timing': 'latest filtered sample; independent unsynchronized clock',
                'bias_removed_in_driver': False, 'scale_calibration': 'datasheet only',
                'startup_id_mismatches': self._startup_id_mismatches,
                'sources': sources}

    def stats(self):
        return {'parsed': self.n_parsed, 'gyro_frames': self.n_frames,
                'bad': self.n_bad, 'dropped': None if self._fault else 0, 'stale': 0,
                'startup_fifo_discarded': self._startup_discarded,
                'startup_id_mismatches': self._startup_id_mismatches,
                'max_gap_s': self._max_gap, 'fault': self._fault}

    def close(self):
        self._closed = True
        for dev in self._devices:
            try:
                dev.close()
            except OSError:
                pass
        self._devices.clear()


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
        imu = RealIMU()
        print("IMU: BMI088 SPI, 400 Hz gyro FIFO, configured rover body axes")
        return imu
    raise ValueError(f"unknown IMU kind: {kind}")


# ── Quick test ─────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('kind', nargs='?', default='arduino', choices=['fake', 'arduino', 'bmi088'])
    parser.add_argument('--sensor-frame', action='store_true', help='BMI088 bench only, no odometry')
    args = parser.parse_args()
    if args.sensor_frame and args.kind != 'bmi088':
        parser.error('--sensor-frame is only for BMI088')
    imu = RealIMU(sensor_frame=True) if args.sensor_frame else create_imu(args.kind)

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
