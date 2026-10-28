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
from copy import deepcopy
import hashlib
import math
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


class IMUSampleError(IMUError):
    """A sample interval was lost; realign, but keep the healthy reader open."""


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

    Invalid batches never reach the estimator. Clipping, FIFO loss and
    inconsistent clock/count evidence explicitly report rejected intervals.
    A slow read alone is diagnostic, not proof of sample loss: retained FIFO
    rates keep their original count-snapshot timestamp and elapsed interval.
    The delivery service separately reports original sample age and backlog.
    Identity/configuration, stopped sensor clock and transport faults still
    require a full reopening. No estimate may span either kind of fault.
    sensor_frame=True is for bench diagnostics ONLY and reports that frame.
    """
    def __init__(self, *, sensor_frame=False, spi_factory=None,
                 clock=time.monotonic, sleep=time.sleep):
        self._clock, self._sleep = clock, sleep
        self._devices = []
        self._closed = False
        self._fault = None
        self.n_parsed = self.n_frames = self.n_bad = 0
        self.n_resync = 0
        self._last_rejection = None
        self._last_rejected_batch = None
        self._batch_trace = {}
        self._last_health_check = None
        self._saturation_raw_limit = 32760
        self._max_gap = 0.0
        self._slow_reads = 0
        self._acquisition_latency = self._max_acquisition_latency = 0.0
        self._delivery_lag = self._max_delivery_lag = 0.0
        self._fifo_timestamp_uncertainty = self._accel_gyro_skew = 0.0
        self._previous_count_read_width = 0.0
        self._timing_tolerated_batches = 0
        self._startup_discarded = 0
        self._startup_id_mismatches = 0
        self._last_tick = None
        self._previous_accel_host = None
        self._previous_accel_read_width = 0.0
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
        # Capture facts from these existing reads, never perform diagnostic
        # SPI transfers. A failed/partial check must not appear as a pass.
        check = {'started_host_s': self._clock(), 'passed': False, 'registers': {}}
        self._last_health_check = check
        try:
            acc_status = self._read(self.acc, 0, 4)
            check.update(accel_chip_id=acc_status[0], accel_error=acc_status[2])
            gyro_status = self._read(self.gyr, 0, 18)
            check.update(gyro_chip_id=gyro_status[0], gyro_range=gyro_status[0x0F],
                         gyro_bandwidth=gyro_status[0x10], gyro_mode=gyro_status[0x11])
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
                name = 'accel' if dev is self.acc else 'gyro'
                check['registers'][f'{name}_0x{address:02X}'] = actual
                if actual != values:
                    raise IMUError(f"BMI088 config/reset at 0x{address:02X}: {actual} != {values}")
            check['passed'] = True
            return check
        finally:
            check['finished_host_s'] = self._clock()

    def _record_rejection(self, exc):
        """Bounded sensor-frame evidence from this read, before recovery."""
        trace = self._batch_trace
        now = self._clock()
        raw = {}
        for name in ('accel', 'gyro'):
            values = trace.get(name + '_raw')
            if values is None:
                raw[name] = None  # Rejected before this transfer; not zero.
                continue
            rows = np.asarray(values).reshape(-1, 3)
            clipped = np.abs(rows) >= self._saturation_raw_limit
            raw[name] = {'frames': len(rows), 'first': rows[0].tolist(),
                         'last': rows[-1].tolist(), 'min': rows.min(axis=0).tolist(),
                         'max': rows.max(axis=0).tolist(),
                         'saturated_axis_counts': clipped.sum(axis=0).tolist(),
                         'saturated_frames': int(np.any(clipped, axis=1).sum())}
        diagnostic = {k: v for k, v in trace.items() if not k.endswith('_raw')}
        diagnostic.update(
            schema=1, rejection_id=self.n_bad + 1, accepted_batches_before=self.n_parsed,
            rejected_host_s=now, reason=str(exc)[:240], raw_sensor_axes=raw,
            saturation_threshold_raw=self._saturation_raw_limit,
            accel_range_g=config.BMI088_ACCEL_RANGE_G,
            gyro_range_dps=config.BMI088_GYRO_RANGE_DPS,
            acquisition_latency_s=now - trace['started_host_s'],
            latest_health_check=self._last_health_check)
        self._last_rejected_batch = deepcopy(diagnostic)

    def read(self):
        if self._closed or self._fault:
            raise IMUError(f"BMI088 reader unavailable: {self._fault or 'closed'}")
        try:
            try:
                return self._acquire()
            except IMUSampleError as exc:
                # This does not reset either sensor or change its ranges.
                # Check identity/configuration before touching the FIFO. A
                # real bus/reset fault during recovery takes the fatal path.
                self._record_rejection(exc)
                try:
                    self._health()
                finally:
                    self._last_rejected_batch['recovery_health_check'] = deepcopy(self._last_health_check)
                self._write(self.gyr, 0x3E, 0x40)
                self._previous_host = self._clock()
                self._previous_count_read_width = 0.0
                self._last_tick = None
                self._previous_accel_host = None
                self._previous_accel_read_width = 0.0
                self._ticks = 0
                self.n_bad += 1
                self.n_resync += 1
                self._last_rejection = str(exc)
                raise
        except IMUSampleError:
            raise
        except (OSError, RuntimeError, ValueError) as exc:
            if (self._last_rejected_batch is not None
                    and self._last_rejected_batch['rejection_id'] == self.n_bad + 1):
                self._last_rejected_batch['recovery_error'] = str(exc)[:240]
            else:
                self._record_rejection(exc)
            self.n_bad += 1
            self._fault = str(exc)
            raise IMUError(f"BMI088 stopped: {exc}; restart and realign") from exc

    def _acquire(self):
        start = self._clock()
        self._batch_trace = {'started_host_s': start, 'previous_fifo_host_s': self._previous_host,
                             'previous_sensor_tick': self._last_tick}
        trace = self._batch_trace
        trace['health_before'] = self._health()
        deadline = start + config.BMI088_READ_TIMEOUT_S
        while True:
            before = self._clock()
            status = self._read(self.gyr, 0x0E)[0]
            after = self._clock()
            host = (before + after) / 2.0
            count_read_width = after - before
            count = status & 0x7F
            trace.update(fifo_status=status, fifo_count=count, fifo_host_s=host,
                         fifo_count_read_width_s=count_read_width,
                         previous_fifo_count_read_width_s=self._previous_count_read_width,
                         dt_s=host - self._previous_host,
                         expected_fifo_count=(host - self._previous_host) * config.IMU_RATE_HZ)
            if status & 0x80 or count >= 100:
                raise IMUSampleError("gyro FIFO overflow/full; motion was lost")
            if count and self._read(self.acc, 3)[0] & 0x80:
                break
            if self._clock() >= deadline:
                raise IMUError("timeout waiting for fresh gyro/accel")
            self._sleep(0.0002)
        dt = host - self._previous_host
        if not math.isfinite(dt) or dt <= 0:
            raise IMUSampleError("invalid host acquisition interval")
        # The actual snapshot can fall anywhere within each SPI-call bracket.
        # Midpoint elapsed time therefore has both half-widths of uncertainty.
        # A slow call widens measured timestamp uncertainty; it is not itself
        # evidence of lost FIFO frames. Keep coherent data and its brackets.
        phase = config.BMI088_FIFO_PHASE_TOLERANCE_FRAMES
        timing_slack = config.IMU_RATE_HZ * (count_read_width + self._previous_count_read_width) / 2.0
        mismatch = abs(count - dt * config.IMU_RATE_HZ)
        trace.update(fifo_count_mismatch=mismatch, fifo_count_tolerance=phase + timing_slack)
        if (not math.isfinite(count_read_width) or count_read_width < 0
                or mismatch > phase + timing_slack):
            raise IMUSampleError("gyro FIFO count inconsistent with host elapsed time")
        gyro_bytes = self._read(self.gyr, 0x3F, count * 6)
        gyro_raw = np.frombuffer(bytes(gyro_bytes), dtype='<i2').reshape(count, 3).astype(int)
        trace['gyro_raw'] = gyro_raw
        before = self._clock()
        accel_bytes = self._read(self.acc, 0x12, 9)
        after = self._clock()
        accel_host = (before + after) / 2.0
        accel_read_width = after - before
        accel_raw = np.frombuffer(bytes(accel_bytes[:6]), dtype='<i2').astype(int)
        tick = int.from_bytes(bytes(accel_bytes[6:]), 'little')
        trace.update(accel_raw=accel_raw, accel_host_s=accel_host, sensor_tick=tick)
        if (not math.isfinite(accel_host) or not math.isfinite(accel_read_width)
                or accel_read_width < 0):
            raise IMUError("invalid accel acquisition timestamp bracket")
        if self._last_tick is not None:
            delta = (tick - self._last_tick) & 0xFFFFFF
            tick_period = 39.0625e-6
            sensor_interval = delta * tick_period
            host_interval = accel_host - self._previous_accel_host
            # Distinguish a stopped/reset clock from a long, observed interval.
            # Use the sensor-clock allowance and actual SPI-call brackets;
            # elapsed wall time alone does not invalidate a coherent clock.
            tolerance = (config.BMI088_SENSOR_CLOCK_TOLERANCE_S
                         + (accel_read_width + self._previous_accel_read_width) / 2.
                         + tick_period)
            trace.update(sensor_tick_delta=delta, sensor_interval_s=sensor_interval,
                         accel_host_interval_s=host_interval, sensor_clock_tolerance_s=tolerance)
            if (delta == 0 or not math.isfinite(host_interval) or host_interval <= 0
                    or not math.isfinite(accel_read_width) or accel_read_width < 0
                    or abs(sensor_interval - host_interval) > tolerance):
                raise IMUError("accel sensor clock stopped/reset or inconsistent with host interval")
            self._ticks += delta
        else:
            self._ticks = tick
        if (np.any(np.abs(gyro_raw) >= self._saturation_raw_limit)
                or np.any(np.abs(accel_raw) >= self._saturation_raw_limit)):
            raise IMUSampleError("sensor saturation (near full-scale)")
        temp = self._read(self.acc, 0x22, 2)
        value = (temp[0] << 3) | (temp[1] >> 5)
        temperature = 23.0 + (value - 2048 if value >= 1024 else value) * 0.125
        # Detect reset/config changes during acquisition before publishing.
        trace['health_after'] = self._health()
        trace['fifo_status_after'] = self._read(self.gyr, 0x0E)[0]
        if trace['fifo_status_after'] & 0x80:
            raise IMUSampleError("gyro FIFO overrun during read")
        # READ_TIMEOUT bounds waiting for fresh data above. Total wall-clock
        # duration also contains scheduler pauses and post-capture health
        # checks; it cannot establish that already acquired frames were lost.
        # Keep the acquisition stamp (never re-stamp on delivery). FIFO/count,
        # sensor-clock, overflow and health checks still decide integrity;
        # IMUService reports original sample age without delivery restamping.
        finished = self._clock()
        self._acquisition_latency = finished - start
        self._delivery_lag = finished - host
        self._fifo_timestamp_uncertainty = count_read_width / 2.0
        self._accel_gyro_skew = accel_host - host
        self._max_acquisition_latency = max(self._max_acquisition_latency, self._acquisition_latency)
        self._max_delivery_lag = max(self._max_delivery_lag, self._delivery_lag)
        self._slow_reads += int(self._acquisition_latency > config.BMI088_READ_TIMEOUT_S)
        rates = (gyro_raw * self._gyr_scale) @ self._rotation.T
        self._previous_host, self._last_tick = host, tick
        self._previous_accel_host = accel_host
        self._previous_accel_read_width = accel_read_width
        self._previous_count_read_width = count_read_width
        self._timing_tolerated_batches += int(mismatch > phase)
        self.n_parsed += 1
        self.n_frames += count
        self._max_gap = max(self._max_gap, dt)
        return {'timestamp': host, 'device_t': self._ticks * 39.0625e-6,
                'accel': self._rotation @ (accel_raw * self._acc_scale),
                'gyro': rates.mean(axis=0), 'frame': self.frame,
                'dt': dt, 'gyro_samples': rates.tolist(),
                'accel_timestamp': accel_host, 'temperature_c': temperature,
                'acquisition_latency_s': self._acquisition_latency,
                'delivery_lag_s': self._delivery_lag,
                'fifo_timestamp_uncertainty_s': self._fifo_timestamp_uncertainty,
                'accel_gyro_skew_s': self._accel_gyro_skew,
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
        for name in ('imu_reader.py', 'imu_service.py', 'rover_main.py', 'config.py',
                     'odometry.py', 'eskf.py', 'logger.py'):
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
                'sensor_clock_tolerance_s': config.BMI088_SENSOR_CLOCK_TOLERANCE_S,
                'gyro_timing': 'host FIFO-count intervals divided by frame count; estimated',
                'accel_timing': 'latest filtered sample; independent unsynchronized clock',
                'bias_removed_in_driver': False, 'scale_calibration': 'datasheet only',
                'startup_id_mismatches': self._startup_id_mismatches,
                'sample_loss_policy': 'reject batch, verify health, resync FIFO; new estimator generation',
                'latency_policy': 'measured SPI-call brackets included in count/clock coherence gates; elapsed time alone does not reject retained FIFO data; original acquisition time retained',
                'sources': sources}

    def stats(self):
        return {'parsed': self.n_parsed, 'gyro_frames': self.n_frames,
                'timing_tolerated_batches': self._timing_tolerated_batches,
                'bad': self.n_bad, 'dropped': None if self.n_bad else 0, 'stale': 0,
                'resyncs': self.n_resync, 'last_rejection': self._last_rejection,
                'last_rejected_batch': deepcopy(self._last_rejected_batch),
                'slow_reads': self._slow_reads,
                'acquisition_latency_s': self._acquisition_latency,
                'max_acquisition_latency_s': self._max_acquisition_latency,
                'delivery_lag_s': self._delivery_lag,
                'max_delivery_lag_s': self._max_delivery_lag,
                'fifo_timestamp_uncertainty_s': self._fifo_timestamp_uncertainty,
                'accel_gyro_skew_s': self._accel_gyro_skew,
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
