"""
CSV logger for sensor data, and a replayer for offline processing.

Log format: one row per timestep, all sensor data in SI units.
Replay reads the CSV back as dicts -- feed into ESKF for offline tuning.

Log files are named automatically (date_time_tag) so that a run is never
overwritten and the file itself says when it happened. See new_log_path.
"""
import copy
import csv
import datetime
import hashlib
import json
import math
import os
import queue
import threading

import config


def _file_identity(filename):
    """Stream a CSV identity without retaining its rows or file in memory."""
    digest = hashlib.sha256()
    size = 0
    with open(filename, 'rb') as stream:
        while True:
            block = stream.read(64 * 1024)
            if not block:
                break
            size += len(block)
            digest.update(block)
    return size, digest.hexdigest()

# Raw logs go here. They are INPUTS: the only way to get one back is to
# run the experiment again. Derived things (plots) go in ../data/.
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       os.pardir, "logs")


def new_log_path(tag="run", directory=None):
    """
    A fresh, never-colliding log path: logs/2026-09-06_1432_drive.csv

    Timestamped on purpose. A fixed name like run1.csv means the second
    run silently destroys the first, and you find out days later when the
    log you wanted to re-tune against is gone.

    The minute resolution is enough to sort a session; if two runs start
    inside the same minute a -2, -3 suffix keeps them apart.
    """
    directory = os.path.abspath(directory or LOG_DIR)
    os.makedirs(directory, exist_ok=True)

    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
    tag = "".join(c if c.isalnum() or c in "-_" else "-" for c in tag)

    path = os.path.join(directory, f"{stamp}_{tag}.csv")
    n = 2
    while os.path.exists(path):
        path = os.path.join(directory, f"{stamp}_{tag}-{n}.csv")
        n += 1
    return path

# Column names for the CSV. Built from config.MOTOR_IDS so that going
# from the 2-motor bench setup to the 4-wheel rover needs no edit here.
N_WHEELS = len(config.MOTOR_IDS)

# 't'     = host loop time (seconds since the run started)
# 'imu_t' = the IMU's own nominal clock, with arbitrary offset/rate. Its
# difference from host time is NOT directly latency. BMI088 uses the separate
# host FIFO interval imu_dt for integration, preserving nominal accel time.
COLUMNS = (['t', 'imu_t', 'imu_frame']
           + [f'w{i+1}_{f}' for i in range(N_WHEELS)
              for f in ('pos', 'vel', 'cur')]
           + ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
           + ['imu_source', 'imu_dt', 'gyro_samples', 'imu_host_t',
              'accel_host_t', 'imu_temp_c', 'imu_tick', 'accel_raw', 'gyro_raw'])


class Logger:
    """Write sensor data to CSV, one row per timestep."""

    # Flush Python's user-space buffer every 100 rows (about 1 s at 100 Hz).
    # This reduces data lost on a process crash and makes new rows visible
    # to readers. It does not fsync the OS cache or guarantee durability
    # after power loss; fewer samples per second also means longer gaps.
    FLUSH_EVERY = 100          # rows == 1 s at 100 Hz

    def __init__(self, filename, metadata=None):
        self.filename = filename
        self.file = open(filename, 'x', newline='')
        try:
            if metadata is not None:
                with open(str(filename) + '.meta.json', 'x', encoding='utf-8') as f:
                    json.dump(metadata, f, indent=2, ensure_ascii=False)
                    f.write('\n')
            self.writer = csv.writer(self.file)
            self.writer.writerow(COLUMNS)
            self.file.flush()      # header on disk before the first row
            self.count = 0
        except BaseException:
            self.file.close()
            raise

    def log(self, timestamp, wheel_data, imu_data):
        """
        wheel_data: list of 4 dicts with 'position_rad', 'velocity_rads', 'current_A'
        imu_data:   dict with 'accel' (3,) and 'gyro' (3,) arrays
        """
        row = [f"{timestamp:.6f}",
               f"{imu_data.get('device_t', 0.0):.6f}",
               imu_data.get('frame', 'body')]

        for w in wheel_data:
            row.append(f"{w['position_rad']:.6f}")
            row.append(f"{w['velocity_rads']:.6f}")
            row.append(f"{w['current_A']:.6f}")

        a = imu_data['accel']
        g = imu_data['gyro']
        row.extend([f"{a[0]:.6f}", f"{a[1]:.6f}", f"{a[2]:.6f}"])
        row.extend([f"{g[0]:.6f}", f"{g[1]:.6f}", f"{g[2]:.6f}"])
        row.extend([imu_data.get('source', ''), imu_data.get('dt', ''),
                    json.dumps(imu_data['gyro_samples'], separators=(',', ':')) if 'gyro_samples' in imu_data else '',
                    imu_data.get('timestamp', ''), imu_data.get('accel_timestamp', ''),
                    imu_data.get('temperature_c', ''), imu_data.get('sensor_tick', ''),
                    json.dumps(imu_data['accel_raw']) if 'accel_raw' in imu_data else '',
                    json.dumps(imu_data['gyro_raw']) if 'gyro_raw' in imu_data else ''])

        self.writer.writerow(row)
        self.count += 1
        if self.count % self.FLUSH_EVERY == 0:
            self.file.flush()

    def close(self):
        self.file.close()
        print(f"Logged {self.count} rows -> {self.filename}")


class _AsyncCSVProxy:
    """One segment; count is accepted rows, not a claim of disk durability."""
    def __init__(self, owner, identity, filename):
        self._owner, self._identity = owner, identity
        self.filename = filename
        self.count = 0
        self._closed = False
        self.written_rows = self.dropped_rows = self.failed_rows = 0
        self.capture_saved = False

    def log(self, timestamp, wheel_data, imu_data):
        try:
            payload = copy.deepcopy((timestamp, wheel_data, imu_data))
        except Exception as exc:
            self._owner._producer_error('CSV input snapshot: ' + str(exc), self)
            return False
        return self._owner._submit('row', self, payload)

    def close(self):
        self._owner._close_proxy(self)


class AsyncCSVWriter:
    """One bounded disk worker for all CSV segments in a Run.

    Producers snapshot inputs and enqueue ordered open/row/close jobs. All
    formatting, JSON serialization, file opens, flushes and closes belong to
    the worker. Errors and missing jobs remain visible through state(). Segment
    close never waits; writer close may perform a bounded shutdown drain.
    """
    def __init__(self, *, queue_max=None, close_timeout=None):
        capacity = (getattr(config, 'CSV_LOG_QUEUE_MAX_RECORDS', 2048)
                    if queue_max is None else queue_max)
        timeout = (getattr(config, 'CSV_LOG_CLOSE_TIMEOUT_S', 3.0)
                   if close_timeout is None else close_timeout)
        if type(capacity) is not int or capacity <= 0:
            raise ValueError('CSV queue capacity must be a positive integer')
        if not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout < 0:
            raise ValueError('CSV close timeout must be finite and nonnegative')
        self._queue = queue.Queue(maxsize=capacity)
        self._close_timeout = float(timeout)
        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._finished = threading.Event()
        self._reserved = set()
        self._segments = {}
        self._pending_by_proxy = {}
        self._close_requested = set()
        self._accepted = self._written = self._dropped = self._failed = 0
        self._inflight = self._next_identity = 0
        self._fatal = False
        self._errors = []
        self._error_count = 0
        self._thread = threading.Thread(target=self._run, name='csv-writer', daemon=True)
        self._thread.start()

    def _error_locked(self, message, *, fatal=False):
        self._error_count += 1
        if message not in self._errors and len(self._errors) < 20:
            self._errors.append(message)
        self._fatal = self._fatal or fatal

    def _producer_error(self, message, proxy=None):
        with self._lock:
            self._dropped += 1
            if proxy is not None and not proxy._closed:
                proxy.dropped_rows += 1
            self._error_locked(message)

    def open(self, filename, metadata=None):
        """Reserve a unique segment name now; return before its disk open."""
        requested = os.path.abspath(os.fspath(filename))
        stem, extension = os.path.splitext(requested)
        with self._lock:
            if self._closing.is_set() or self._fatal:
                # Do not mutate the worker's finalization set after its barrier.
                # A rejected proxy has no queued jobs or reserved disk identity.
                proxy = _AsyncCSVProxy(self, -1, requested)
                proxy._closed = True
                self._dropped += 1
                self._error_locked('CSV writer closed; segment not accepted')
                return proxy
            candidate, suffix = requested, 2
            while (os.path.normcase(candidate) in self._reserved or os.path.exists(candidate)
                   or os.path.exists(candidate + '.meta.json') or os.path.exists(candidate + '.capture.json')):
                candidate = f'{stem}-{suffix}{extension}'
                suffix += 1
            self._reserved.add(os.path.normcase(candidate))
            self._next_identity += 1
            proxy = _AsyncCSVProxy(self, self._next_identity, candidate)
            self._segments[proxy._identity] = proxy
        try:
            snapshot = copy.deepcopy(metadata)
            snapshot = dict(snapshot or {}, async_csv_capture={
                'schema': 'rover_csv_capture_v1',
                'status_file': os.path.basename(candidate) + '.capture.json',
                'replay_requires_complete_capture': True})
        except Exception as exc:
            proxy._closed = True
            self._producer_error('CSV metadata snapshot: ' + str(exc))
            self._retire_proxy(proxy)
            return proxy
        if not self._submit('open', proxy, snapshot):
            proxy._closed = True
            self._retire_proxy(proxy)
        return proxy

    def _retire_proxy(self, proxy):
        """Forget completed segment bookkeeping; the caller still owns its proxy."""
        with self._lock:
            if self._pending_by_proxy.get(proxy._identity, 0):
                return
            self._segments.pop(proxy._identity, None)
            self._pending_by_proxy.pop(proxy._identity, None)
            self._close_requested.discard(proxy._identity)
            self._reserved.discard(os.path.normcase(proxy.filename))

    def _submit(self, kind, proxy, payload=None):
        with self._lock:
            if self._closing.is_set() or self._fatal or (proxy._closed and kind != 'close'):
                self._dropped += 1
                self._error_locked('CSV writer/segment closed; record not accepted')
                return False
            try:
                self._queue.put_nowait((kind, proxy, payload))
            except queue.Full:
                self._dropped += 1
                if kind == 'row' and not proxy._closed:
                    proxy.dropped_rows += 1
                self._error_locked('CSV writer queue overflow; records dropped')
                return False
            self._accepted += 1
            self._pending_by_proxy[proxy._identity] = self._pending_by_proxy.get(proxy._identity, 0) + 1
            if kind == 'row':
                proxy.count += 1
            return True

    def _close_proxy(self, proxy):
        with self._lock:
            if proxy._closed:
                return
            proxy._closed = True
            self._close_requested.add(proxy._identity)
            if self._closing.is_set():
                return  # The writer's final drain owns all remaining closes.
        # If the queue is full, the worker still closes this handle once all
        # its already accepted jobs have finished. The dropped job is explicit.
        self._submit('close', proxy)

    def state(self):
        with self._lock:
            status = ('error' if self._errors else 'closed' if self._finished.is_set()
                      else 'stopping' if self._closing.is_set() else 'recording')
            return dict(status=status,
                        active=not (self._closing.is_set() or self._finished.is_set() or self._fatal),
                        accepted=self._accepted, written=self._written,
                        dropped=self._dropped, failed=self._failed,
                        pending=self._accepted - self._written - self._failed,
                        error='; '.join(self._errors) or None,
                        errors=list(self._errors), error_count=self._error_count,
                        close_complete=self._finished.is_set())

    def close(self, wait=True):
        # Serialize the acceptance barrier with _submit so shutdown cannot
        # observe an empty queue immediately before an accepted producer put.
        with self._lock:
            self._closing.set()
        if wait:
            self._thread.join(timeout=self._close_timeout)
            if self._thread.is_alive():
                with self._lock:
                    self._error_locked('CSV writer close timed out; queued data may not be durable')
        return self.state()

    def _open_logger(self, filename, metadata):
        return Logger(filename, metadata=metadata)

    def _close_logger(self, handle):
        """Worker-only: persist CSV contents before certifying its identity."""
        try:
            stream = getattr(handle, 'file', None)
            if stream is not None and not stream.closed:
                stream.flush()
                os.fsync(stream.fileno())
        finally:
            # An fsync failure must not leak the open CSV handle.
            handle.close()

    def _save_capture(self, proxy, closed):
        """Worker-only durable loss identity; no incomplete CSV replays silently."""
        if proxy.capture_saved or not os.path.exists(proxy.filename + '.meta.json'):
            return
        csv_bytes, csv_sha256 = _file_identity(proxy.filename)
        with self._lock:
            status = dict(schema='rover_csv_capture_v1', file=os.path.basename(proxy.filename),
                          csv_bytes=csv_bytes, csv_sha256=csv_sha256,
                          accepted_rows=proxy.count, written_rows=proxy.written_rows,
                          dropped_rows=proxy.dropped_rows, failed_rows=proxy.failed_rows,
                          closed=bool(closed), complete=bool(closed and not self._fatal
                              and not proxy.dropped_rows and not proxy.failed_rows
                              and proxy.count == proxy.written_rows),
                          errors=list(self._errors),
                          durability='CSV flushed and fsynced before capture status; not sensor continuity or a device power-loss guarantee')
        path = proxy.filename + '.capture.json'
        with open(path, 'x', encoding='utf-8') as stream:
            json.dump(status, stream, ensure_ascii=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        proxy.capture_saved = True

    def _close_idle_segments(self, handles):
        with self._lock:
            ready = [identity for identity in self._close_requested
                     if not self._pending_by_proxy.get(identity, 0)]
            self._close_requested.difference_update(ready)
        for identity in ready:
            proxy = self._segments[identity]
            handle = handles.get(identity)
            if handle is not None:
                self._close_logger(handle)
                handles.pop(identity)
                self._save_capture(proxy, True)
            self._retire_proxy(proxy)

    def _run(self):
        handles = {}
        try:
            while not (self._closing.is_set() and self._queue.empty()):
                try:
                    kind, proxy, payload = self._queue.get(timeout=.02)
                except queue.Empty:
                    self._close_idle_segments(handles)
                    continue
                with self._lock:
                    self._inflight = 1
                    fatal = self._fatal
                try:
                    if fatal:
                        with self._lock:
                            self._failed += 1
                            if kind == 'row': proxy.failed_rows += 1
                    else:
                        if kind == 'open':
                            handles[proxy._identity] = self._open_logger(proxy.filename, payload)
                        elif kind == 'row':
                            handles[proxy._identity].log(*payload)
                            with self._lock: proxy.written_rows += 1
                        elif kind == 'close':
                            handle = handles.get(proxy._identity)
                            if handle is not None:
                                self._close_logger(handle)
                                handles.pop(proxy._identity)
                                self._save_capture(proxy, True)
                        with self._lock:
                            self._written += 1
                except Exception as exc:
                    with self._lock:
                        self._failed += 1
                        if kind == 'row': proxy.failed_rows += 1
                        self._error_locked(type(exc).__name__ + ': ' + str(exc), fatal=True)
                    self._closing.set()
                finally:
                    with self._lock:
                        self._inflight = 0
                        self._pending_by_proxy[proxy._identity] -= 1
                    self._queue.task_done()
                self._close_idle_segments(handles)
        except Exception as exc:
            with self._lock:
                self._error_locked(type(exc).__name__ + ': ' + str(exc), fatal=True)
            self._closing.set()
        finally:
            # Close every opened file even after failure; abandoned queued jobs
            # are counted so a closed worker never hides accepted pending work.
            while True:
                try:
                    kind, proxy, _payload = self._queue.get_nowait()
                except queue.Empty:
                    break
                self._queue.task_done()
                with self._lock:
                    self._failed += 1
                    if kind == 'row': proxy.failed_rows += 1
                    self._pending_by_proxy[proxy._identity] -= 1
            for handle in handles.values():
                try:
                    self._close_logger(handle)
                except Exception as exc:
                    with self._lock:
                        self._error_locked('CSV close: ' + str(exc), fatal=True)
            with self._lock:
                segments = list(self._segments.values())
            for proxy in segments:
                try:
                    self._save_capture(proxy, not self._fatal)
                except Exception as exc:
                    with self._lock:
                        self._error_locked('CSV capture status: ' + str(exc), fatal=True)
                finally:
                    self._retire_proxy(proxy)
            self._finished.set()


def replay(filename, *, allow_recovery=False, allow_incomplete=False, wheel_measurements_available=None,
           position_integration=None, wheel_distance_scale=None):
    """
    Generator: yields one dict per row from a log CSV.

    Each dict has:
        't':      float   (seconds)
        'wheels': list of 4 dicts {position_rad, velocity_rads, current_A}
        'accel':  [ax, ay, az]
        'gyro':   [gx, gy, gz]

    Integration and distance calibration must match the recorded estimator.
    Logs predating these fields use endpoint integration and scale 1.0. When
    those differ from current defaults, explicitly construct that historical
    Odometry mode and pass the same options to replay; they are never inferred
    from today's default or silently applied to the raw wheel measurements.
    """
    metadata = {}
    sidecar = str(filename) + '.meta.json'
    if os.path.exists(sidecar):
        with open(sidecar, encoding='utf-8') as f:
            metadata = json.load(f)
        if not isinstance(metadata, dict):
            raise ValueError('Log metadata must be an object')
        capture_policy = metadata.get('async_csv_capture')
        if capture_policy is not None:
            expected = os.path.basename(str(filename)) + '.capture.json'
            if (not isinstance(capture_policy, dict) or capture_policy.get('schema') != 'rover_csv_capture_v1'
                    or capture_policy.get('status_file') != expected):
                raise ValueError('Invalid CSV capture identity')
            try:
                with open(str(filename)+'.capture.json', encoding='utf-8') as stream:
                    capture_status = json.load(stream)
            except (OSError, ValueError) as exc:
                if not allow_incomplete:
                    raise ValueError('CSV capture did not close cleanly; completion evidence unavailable') from exc
                capture_status = {}
            if (not allow_incomplete and (not isinstance(capture_status,dict)
                    or capture_status.get('schema') != 'rover_csv_capture_v1'
                    or capture_status.get('file') != os.path.basename(str(filename))
                    or capture_status.get('complete') is not True
                    or capture_status.get('dropped_rows') != 0 or capture_status.get('failed_rows') != 0)):
                raise ValueError('Incomplete CSV capture; dropped/failed rows cannot be replayed as continuous motion')
            if not allow_incomplete:
                expected_bytes = capture_status.get('csv_bytes')
                expected_hash = capture_status.get('csv_sha256')
                if (type(expected_bytes) is not int or expected_bytes < 0
                        or not isinstance(expected_hash, str) or len(expected_hash) != 64
                        or any(character not in '0123456789abcdef' for character in expected_hash)):
                    raise ValueError('CSV capture byte/hash identity unavailable')
                actual_bytes, actual_hash = _file_identity(filename)
                if (actual_bytes, actual_hash) != (expected_bytes, expected_hash):
                    raise ValueError('CSV capture integrity mismatch; stored rows were changed or truncated')
        options = metadata.get('estimator_options', {})
        if not isinstance(options, dict):
            raise ValueError('Recorded estimator_options must be an object')
        checkpoint = metadata.get('estimator_initial_state')
        if checkpoint is not None and not isinstance(checkpoint, dict):
            raise ValueError('Recovery checkpoint must be an object')
        wheels_measured = options.get('wheel_measurements_available',
                                      (checkpoint or {}).get('wheel_measurements_available'))
        if wheels_measured is not None and not isinstance(wheels_measured, bool):
            raise ValueError('Recorded wheel_measurements_available must be a boolean')
        if wheels_measured is False and wheel_measurements_available is not False:
            raise ValueError('This log has no wheel measurements. Construct Odometry with '
                             'wheel_measurements_available=False and explicitly use '
                             'replay(..., wheel_measurements_available=False); zero placeholders are not measurements.')
        if wheels_measured is True and wheel_measurements_available is False:
            raise ValueError('Requested replay wheel mode disagrees with recorded estimator_options')
        if checkpoint is not None:
            if checkpoint.get('schema') != 'odometry_recovery_v1':
                raise ValueError('Unsupported recovery checkpoint; do not replay at a fresh origin')
            if not allow_recovery:
                raise ValueError('This segment retains an earlier estimator state. Restore '
                                 'Odometry.from_recovery_checkpoint(metadata["estimator_initial_state"]) '
                                 'and keep degraded quality before replay(..., allow_recovery=True).')
    options = metadata.get('estimator_options', {})
    checkpoint = metadata.get('estimator_initial_state') or {}
    recorded_mode = options.get('position_integration', checkpoint.get('position_integration', 'endpoint'))
    recorded_scale = options.get('wheel_distance_scale', checkpoint.get('wheel_distance_scale', 1.0))
    if recorded_mode not in ('endpoint', 'gyro_midpoint'):
        raise ValueError('Unsupported recorded position_integration')

    def checked_scale(value):
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError('Wheel distance scale must be a finite positive scalar') from exc
        if isinstance(value, bool) or not math.isfinite(number) or number <= 0:
            raise ValueError('Wheel distance scale must be a finite positive scalar')
        return number

    recorded_scale = checked_scale(recorded_scale)
    if checkpoint:
        if (checkpoint.get('position_integration', 'endpoint') != recorded_mode
                or checked_scale(checkpoint.get('wheel_distance_scale', 1.0)) != recorded_scale
                or ('wheel_measurements_available' in options and
                    options['wheel_measurements_available'] != checkpoint.get('wheel_measurements_available', True))):
            raise ValueError('Recovery checkpoint disagrees with recorded estimator_options')
    selected_mode = config.ODOMETRY_POSITION_INTEGRATION if position_integration is None else position_integration
    selected_scale = checked_scale(config.WHEEL_DISTANCE_SCALE if wheel_distance_scale is None else wheel_distance_scale)
    if selected_mode != recorded_mode:
        raise ValueError(f'Restore recorded position_integration={recorded_mode!r} in Odometry and '
                         f'explicitly use replay(..., position_integration={recorded_mode!r}); '
                         'current defaults must not change historical integration semantics.')
    if selected_scale != recorded_scale:
        raise ValueError(f'Restore recorded wheel_distance_scale={recorded_scale!r} in Odometry and '
                         f'explicitly use replay(..., wheel_distance_scale={recorded_scale!r}); '
                         'raw CSV wheel values are not already calibrated.')
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            wheels = []
            for i in range(1, N_WHEELS + 1):
                wheels.append({
                    'position_rad':  float(row[f'w{i}_pos']),
                    'velocity_rads': float(row[f'w{i}_vel']),
                    'current_A':     float(row[f'w{i}_cur']),
                })

            accel = [float(row['ax']), float(row['ay']), float(row['az'])]
            gyro = [float(row['gx']), float(row['gy']), float(row['gz'])]

            # Logs made before 2026-09-11 have no imu_frame column. The
            # Arduino driver then logged board axes directly even though the
            # filter expected rover body axes. Preserve the raw CSV, but fix
            # that known legacy format at the replay boundary.
            source_frame = row.get('imu_frame') or 'arduino_sensor_legacy'
            imu_frame = source_frame
            if imu_frame == 'arduino_sensor_legacy':
                from imu_reader import arduino_sensor_to_body
                accel = arduino_sensor_to_body(accel)
                gyro = arduino_sensor_to_body(gyro)
                imu_frame = 'body'
            elif imu_frame != 'body':
                raise ValueError(f"Unsupported IMU frame in log: {imu_frame}")

            extra = {}
            if row.get('imu_source') == 'bmi088':
                # No fallback to loop time or one selected gyro frame.
                extra = {'imu_dt': float(row['imu_dt']),
                         'gyro_samples': json.loads(row['gyro_samples'])}
            yield {
                't':      float(row['t']),
                'imu_t':  float(row.get('imu_t', 0.0)),
                'wheels': wheels,
                'accel':  accel,
                'gyro':   gyro,
                'frame':  imu_frame,
                'source_frame': source_frame,
                **extra,
            }


# ── Quick test ─────────────────────────────────────────────────────

if __name__ == "__main__":
    import numpy as np

    test_file = "test_log.csv"

    # Write some fake data
    log = Logger(test_file)
    for i in range(5):
        wheels = [{'position_rad': i * 0.1, 'velocity_rads': 1.0,
                   'current_A': 0.5}] * N_WHEELS
        imu = {'accel': np.array([0, 0, 9.81]), 'gyro': np.array([0, 0, 0]),
               'device_t': i * 0.0096, 'frame': 'body'}
        log.log(i * 0.01, wheels, imu)
    log.close()

    # Read it back
    print("\nReplay:")
    for row in replay(test_file, position_integration='endpoint', wheel_distance_scale=1.0):
        print(f"  t={row['t']:.3f}  w1_pos={row['wheels'][0]['position_rad']:.3f}  "
              f"accel_z={row['accel'][2]:.2f}")

    os.remove(test_file)
    print("\nTest passed -- CSV write and read match.")
