"""One background owner for IMU initialization, acquisition and recovery.

The control loop only polls already acquired samples. A missing or stalled
IMU never blocks commands, camera or Pico. Gaps invalidate the estimate and
start a new acquisition generation. A separate alignment epoch distinguishes
verified BMI088 FIFO resynchronization from hard/unclassified loss. Lost samples
do not close a healthy reader; hardware faults still reopen it with backoff.
Waiting for the next batch can make the displayed pose stale without proving
lost motion. That is 'delayed': withhold pose validity, preserve generation.
Validated, ordered acquisition history is delivered even when it arrives late.
Its original timestamps and delivery ages are retained. Waiting for newer data
does not discard history or create a loss/recovery generation.
"""
from collections import deque
import math
import sys
import threading
import time

import config
import imu_reader


class IMUService:
    def __init__(self, kind, *, factory=None, retry_delay=None):
        self.kind = kind
        self._factory = factory or (lambda: self._create_reader(kind))
        self._retry_delay = config.IMU_RETRY_DELAY_S if retry_delay is None else retry_delay
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._queue = deque()
        self._status = 'starting'
        self._error = None
        self._attempt = self._generation = 0
        self._alignment_epoch = 0
        self._recovery = None
        self._last_sample = None
        self._last_device_t = None
        self._gaps = 0
        self._delivery_delays = 0
        self._delivery_backlogs = 0
        self._arrival_delays = 0
        self._last_arrival_age = self._max_arrival_age = 0.0
        self._consumer_dropped_batches = 0
        self._last_gap = None
        self._metadata = None
        self._stats = {}
        self._thread = threading.Thread(target=self._run, daemon=True, name='imu')

    def _create_reader(self, kind):
        if kind == 'bmi088' and sys.platform.startswith('linux'):
            from imu_process import BMI088ProcessReader
            return BMI088ProcessReader(stop_event=self._stop)
        return imu_reader.create_imu(kind)

    def start(self):
        self._thread.start()

    def invalidate(self, reason):
        """Invalidate the estimate without interrupting hardware acquisition."""
        with self._lock:
            self._gap_locked(reason)

    def _gap_locked(self, reason, *, can_resume=False, diagnostic=None):
        self._queue.clear()
        # Generation changes even if the control loop misses the brief
        # resyncing status. It cannot bridge a rejected interval in odometry.
        self._generation += 1
        if not can_resume:
            # Monotonic barrier: a later qualified gap must not hide a hard
            # break that the consumer did not happen to poll in between.
            self._alignment_epoch += 1
        self._gaps += 1
        self._last_gap = self._error = reason
        self._status = 'resyncing'
        self._recovery = {
            'generation': self._generation, 'alignment_epoch': self._alignment_epoch,
            'reader_attempt': self._attempt, 'can_resume': bool(can_resume),
            'reason': reason, 'detected_host_s': time.monotonic(),
            'last_accepted_host_s': self._last_sample,
            'rejection_id': (diagnostic or {}).get('rejection_id'),
        }

    def poll(self):
        now = time.monotonic()
        with self._lock:
            age = None if self._last_sample is None else now - self._last_sample
            backlog = len(self._queue)
            oldest_age = None if not self._queue else now - self._queue[0][0]
            if self._status in ('ok', 'delayed') and (
                    age is None or age > config.IMU_SAMPLE_MAX_AGE_S):
                # Waiting is a display/current-context condition, not evidence
                # of acquisition loss. Deliver validated history below anyway.
                if self._status != 'delayed':
                    self._delivery_delays += 1
                self._status = 'delayed'
                self._error = 'Waiting for fresh IMU batch; pose temporarily stale'
            elif self._status == 'delayed':
                self._status, self._error = 'ok', None
            samples = ([sample for _, sample in self._queue]
                       if self._status in ('ok', 'delayed') else [])
            if samples:
                # Oldest age measures consumer lag, not lost acquisition. Each
                # queued sample passed shape/dt/clock validation; a real
                # rejection or overflow clears the queue in _gap_locked.
                if oldest_age > config.IMU_SAMPLE_MAX_AGE_S:
                    self._delivery_backlogs += 1
                self._queue.clear()
            return dict(status=self._status, error=self._error, attempt=self._attempt,
                        generation=self._generation, age_s=age, samples=samples,
                        alignment_epoch=self._alignment_epoch,
                        recovery=dict(self._recovery) if self._recovery else None,
                        metadata=self._metadata,
                        stats=dict(self._stats, gaps=self._gaps, last_gap=self._last_gap,
                                   delivery_delays=self._delivery_delays,
                                   oldest_delivery_age_s=oldest_age, backlog_samples=backlog,
                                   delivered_samples=len(samples), delivery_backlogs=self._delivery_backlogs,
                                   arrival_delays=self._arrival_delays,
                                   last_arrival_age_s=self._last_arrival_age,
                                   max_arrival_age_s=self._max_arrival_age,
                                   consumer_dropped_batches=self._consumer_dropped_batches))

    @staticmethod
    def _sample_timestamp(sample, received):
        """Validate integrity without an arrival-age or duration cutoff."""
        try:
            stamp = float(sample['timestamp'])
            valid = math.isfinite(stamp) and math.isfinite(received) and stamp <= received
            for key in ('accel', 'gyro'):
                vector = sample[key]
                valid = valid and len(vector) == 3 and all(math.isfinite(float(v)) for v in vector)
            if 'gyro_samples' in sample:
                rates = sample['gyro_samples']
                valid = valid and len(rates) > 0 and all(
                    len(rate) == 3 and all(math.isfinite(float(v)) for v in rate) for rate in rates)
            if 'dt' in sample:
                interval = float(sample['dt'])
                valid = valid and math.isfinite(interval) and interval > 0
            if 'device_t' in sample:
                valid = valid and math.isfinite(float(sample['device_t']))
        except (KeyError, TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            raise imu_reader.IMUSampleError('Invalid IMU sample shape, clock or interval; estimate invalidated')
        return stamp

    @staticmethod
    def _reader_stats(reader):
        try:
            return reader.stats() if hasattr(reader, 'stats') else {}
        except Exception as exc:
            # A diagnostic accessor must never kill the recovery worker.
            return {'stats_error': f'{type(exc).__name__}: {exc}'}

    def _run(self):
        period = 1.0 / config.EKF_RATE_HZ
        while not self._stop.is_set():
            reader = None
            with self._lock:
                self._attempt += 1
            try:
                reader = self._factory()
                metadata = reader.metadata() if hasattr(reader, 'metadata') else {'sensor': self.kind}
                with self._lock:
                    self._generation += 1
                    self._alignment_epoch += 1
                    self._recovery = None
                    self._metadata = metadata
                    self._queue.clear()
                    self._last_sample = None
                    self._last_device_t = None
                last_reader_rejection = 0
                next_tick = time.monotonic()
                while not self._stop.is_set():
                    try:
                        reading = True
                        sample = reader.read()
                        reading = False
                        if sample is None:
                            raise imu_reader.IMUSampleError('No fresh IMU sample')
                        received = time.monotonic()
                        stamp = self._sample_timestamp(sample, received)
                        stats = self._reader_stats(reader)
                        with self._lock:
                            if self._last_sample is not None and stamp <= self._last_sample:
                                raise imu_reader.IMUSampleError('IMU acquisition timestamp did not advance')
                            if 'device_t' in sample:
                                device_t = float(sample['device_t'])
                                previous = self._last_device_t
                                self._last_device_t = device_t
                                if previous is not None and device_t <= previous:
                                    raise imu_reader.IMUSampleError('IMU sensor timestamp stopped/reset')
                            if len(self._queue) >= config.IMU_QUEUE_MAX_SAMPLES:
                                self._consumer_dropped_batches += len(self._queue)
                                self._gap_locked('IMU consumer queue overflow; estimate invalidated')
                            arrival_age = received - stamp
                            self._last_arrival_age = arrival_age
                            self._max_arrival_age = max(self._max_arrival_age, arrival_age)
                            self._arrival_delays += int(arrival_age > config.IMU_SAMPLE_MAX_AGE_S)
                            self._queue.append((stamp, sample))
                            self._last_sample = stamp
                            self._stats = stats
                            self._status, self._error = 'ok', None
                    except imu_reader.IMUSampleError as exc:
                        stats = self._reader_stats(reader)
                        diagnostic = stats.get('last_rejected_batch') or {}
                        rejection_id = diagnostic.get('rejection_id')
                        new_rejection = (type(rejection_id) is int
                                         and rejection_id > last_reader_rejection)
                        # Only an exception from this actual read, with fresh
                        # post-rejection health evidence, may retain an aligned
                        # prior. Sample validation failures must never borrow
                        # an older successful recovery diagnostic.
                        qualified = bool(
                            reading and self.kind == 'bmi088' and new_rejection
                            and diagnostic.get('reason') == str(exc)
                            and (diagnostic.get('recovery_health_check') or {}).get('passed') is True
                            and not diagnostic.get('recovery_error'))
                        if new_rejection:
                            last_reader_rejection = rejection_id
                        with self._lock:
                            self._gap_locked(str(exc), can_resume=qualified,
                                             diagnostic=diagnostic if qualified else None)
                            if qualified:
                                # RealIMU deliberately resets its derived accel
                                # tick anchor after FIFO resync. Hardware clock
                                # failures take the fatal/unqualified path.
                                self._last_device_t = None
                            self._stats = stats
                    next_tick += period
                    if self._stop.wait(max(0.0, next_tick - time.monotonic())):
                        break
                    if time.monotonic() - next_tick > period:
                        next_tick = time.monotonic()
            except Exception as exc:
                with self._lock:
                    self._gap_locked(f'{type(exc).__name__}: {exc}')
                    self._status = 'retrying'
                    self._error = f'{type(exc).__name__}: {exc}'
                    self._queue.clear()
                    if reader is not None:
                        self._stats = self._reader_stats(reader)
            finally:
                if reader is not None:
                    try:
                        reader.close()
                    except Exception as exc:
                        with self._lock:
                            self._error = f'IMU close failed: {exc}'
            if self._stop.wait(self._retry_delay):
                break
        with self._lock:
            self._status = 'off'
            self._queue.clear()

    def close(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
