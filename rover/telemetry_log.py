"""Asynchronous, append-only session evidence independent of the IMU CSV.

JSONL is the complete telemetry record. The paired CSV is only a convenient
selection of scalar fields; missing/invalid values remain blank, never zero.
All filesystem operations belong to the worker. Producer calls only snapshot
plain data and attempt a bounded queue insertion, including during shutdown.
"""
import csv
from datetime import datetime, timezone
import io
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid

import numpy as np

import config


SCHEMA = 'rover_session_v1'

# The authoritative full field set is JSONL, so adding dashboard telemetry does
# not require changing this extraction table. Tuple components index arrays.
SCALAR_FIELDS = {
    'source_run_id': ('run_id',), 't_s': ('t',),
    'pico_status': ('pico', 'status'),
    'pico_generation': ('pico', 'generation'),
    'pico_age_s': ('pico', 'age_s'),
    'ina_age_ms': ('pico', 'ina_age_ms'),
    'battery_v': ('pico', 'battery', 'v'),
    'battery_a': ('pico', 'battery', 'a'),
    'battery_w': ('pico', 'battery', 'w'),
    'battery_mah': ('pico', 'battery', 'mah'),
    'battery_cell_average_v': ('pico', 'battery', 'cell_v'),
    'battery_charge_complete': ('pico', 'charge_complete'),
    'pico_failsafe': ('pico', 'failsafe'),
    'lift_connected': ('hammer', 'lift', 'connected'),
    'lift_ready': ('hammer', 'lift', 'ready'),
    'lift_position_raw': ('hammer', 'lift', 'position_raw'),
    'lift_mode': ('hammer', 'lift', 'mode'),
    'lift_torque_enabled': ('hammer', 'lift', 'torque_enabled'),
    'lift_holding': ('hammer', 'lift', 'holding'),
    'lift_goal_position_raw': ('hammer', 'lift', 'goal_position_raw'),
    'lift_hold_position_raw': ('hammer', 'lift', 'hold_position_raw'),
    'lift_baseline_position_raw': ('hammer', 'lift', 'baseline_position_raw'),
    'lift_sample_age_s': ('hammer', 'lift', 'sample_age_s'),
    'lift_hardware_error': ('hammer', 'lift', 'hardware_error'),
    'lift_error': ('hammer', 'lift', 'error'),
    'hammer_active': ('hammer', 'active'),
    'cmd_forward': ('cmd', 'forward'), 'cmd_turn': ('cmd', 'turn'),
    'cmd_lift': ('cmd', 'lift'), 'cmd_pan': ('cmd', 'pan'),
    'cmd_stale': ('cmd', 'stale'), 'cmd_estop': ('cmd', 'estop'),
    'motors_status': ('motors',),
    'imu_status': ('imu', 'status'), 'imu_generation': ('imu', 'generation'),
    'imu_age_s': ('imu', 'age_s'), 'imu_error': ('imu', 'error'),
    'pose_valid': ('pose_valid',),
    'pose_estimate_available': ('pose_estimate_available',),
    'pose_frame_id': ('pose_frame_id',), 'pose_quality': ('pose_quality',),
    'pose_loss_count': ('pose_continuity', 'loss_count'),
    'loop_hz': ('loop', 'hz'), 'loop_dropped': ('loop', 'dropped'),
    'cpu_temp_c': ('temp_c',), 'wifi_dbm': ('wifi_dbm',),
    'camera_fps': ('camera', 'fps'), 'camera_mbps': ('camera', 'mbps'),
}
for _i, _name in enumerate(('fl', 'fr', 'rl', 'rr')):
    SCALAR_FIELDS['wheel_' + _name + '_speed_mps'] = ('wheel_speed', _i)
    SCALAR_FIELDS['wheel_' + _name + '_current_a'] = ('wheel_current', _i)
    SCALAR_FIELDS['wheel_' + _name + '_sent_raw'] = ('cmd', 'sent', _i)
for _i, _name in enumerate(('wind', 'release', 'pan', 'aux')):
    SCALAR_FIELDS['servo_' + _name + '_us'] = ('pico', 'servo_us', _i)
for _key, _unit in (('pos', 'm'), ('vel', 'mps'), ('rpy', 'deg')):
    for _i, _axis in enumerate(('roll', 'pitch', 'yaw') if _key == 'rpy' else ('x', 'y', 'z')):
        SCALAR_FIELDS[_key + '_' + _axis + '_' + _unit] = (_key, _i)
        SCALAR_FIELDS['estimate_' + _key + '_' + _axis + '_' + _unit] = ('pose_estimate', _key, _i)
CSV_COLUMNS = ['seq', 'captured_host_utc', 'captured_monotonic_s', 'run_id'] + list(SCALAR_FIELDS)


def _snapshot(value):
    """Copy JSON-like data, preserving nulls and counting nonfinite numbers."""
    nonfinite = 0
    parents = set()

    def copy(item):
        nonlocal nonfinite
        if isinstance(item, np.ndarray):
            return copy(item.tolist())
        if isinstance(item, np.generic):
            return copy(item.item())
        if item is None or isinstance(item, (str, bool, int)):
            return item
        if isinstance(item, float):
            if not math.isfinite(item):
                nonfinite += 1
                return None
            return item
        if isinstance(item, (dict, list, tuple)):
            identity = id(item)
            if identity in parents:
                raise ValueError('Cyclic value in telemetry record')
            parents.add(identity)
            try:
                if isinstance(item, dict):
                    if any(not isinstance(key, str) for key in item):
                        raise TypeError('Telemetry object keys must be strings')
                    return {key: copy(child) for key, child in item.items()}
                return [copy(child) for child in item]
            finally:
                parents.remove(identity)
        raise TypeError('Unsupported telemetry value: ' + type(item).__name__)

    return copy(value), nonfinite


def _json_line(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n'


def _scalar(data, path):
    try:
        for part in path:
            data = data[part]
        return data if data is None or isinstance(data, (str, bool, int, float)) else None
    except (KeyError, IndexError, TypeError):
        return None


def _csv_line(values):
    stream = io.StringIO(newline='')
    csv.writer(stream).writerow(values)
    return stream.getvalue()


def _jumper_record(data):
    return any(data.get(key) == 'jumper' for key in ('subsystem', 'source', 'type'))


class SessionRecorder:
    """Nonblocking producer, one disk worker, explicit loss/error status.

    ``close(wait=False)`` starts a drain without delaying control. Shutdown may
    call ``close()`` again after actuators are stopped for the bounded join.
    A timeout latches an error; it never pretends a pending write completed.
    """
    def __init__(self, log_root, run_id, metadata):
        self.run_id = str(run_id)
        now = datetime.now(timezone.utc)
        slug = ''.join(c for c in self.run_id if c.isalnum() or c in '-_')[:20] or 'run'
        name = now.strftime('%Y%m%dT%H%M%S.%fZ') + '_' + slug + '_' + uuid.uuid4().hex[:8]
        self.directory = str(Path(log_root).absolute() / config.TELEMETRY_LOG_SUBDIR / name)
        self._created_utc = now.isoformat()
        self._created_mono = time.monotonic()
        self._lock = threading.Lock()
        self._queue = queue.Queue(maxsize=config.TELEMETRY_LOG_QUEUE_MAX)
        self._closing = threading.Event()
        self._finished = threading.Event()
        self._ready = False
        self._fatal = False
        self._seq = self._accepted = self._written = self._flushed = self._fsynced = 0
        self._dropped = self._failed = self._inflight = self._nonfinite = 0
        self._counts = {'telemetry': 0, 'sensors': 0, 'event': 0, 'command': 0}
        self._errors = []
        self._error_count = 0
        self._paths = []
        self._last_written_seq = self._last_flushed_seq = self._last_fsynced_seq = None
        self._close_complete = False
        self._metadata, self._nonfinite = _snapshot(metadata)
        self._thread = threading.Thread(target=self._run, name='session-recorder', daemon=True)
        self._thread.start()

    def _error_locked(self, message, fatal=False):
        self._error_count += 1
        if message not in self._errors and len(self._errors) < 20:
            self._errors.append(message)
        self._fatal = self._fatal or fatal

    def _submit(self, kind, data, *, owned=False):
        # No worker holds this lock while touching disk. Serialize producers
        # here so sequence order also equals queue/file order.
        with self._lock:
            self._seq += 1
            if self._closing.is_set() or self._fatal:
                self._dropped += 1
                return False
            try:
                # An owned tree is never changed again by its caller. Reserve
                # sequence/time/order now; normalize on the writer outside
                # this producer lock and the control thread.
                snapshot, nonfinite = (data, 0) if owned else _snapshot(data)
                if not isinstance(snapshot, dict):
                    raise TypeError('Record must be an object')
                self._nonfinite += nonfinite
                record = dict(schema=SCHEMA, seq=self._seq, run_id=self.run_id,
                              captured_host_utc=datetime.now(timezone.utc).isoformat(),
                              captured_monotonic_s=time.monotonic(), kind=kind,
                              nonfinite_values=nonfinite, data=snapshot)
                if owned:
                    record['_owned'] = True
                self._queue.put_nowait(record)
            except queue.Full:
                self._dropped += 1
                self._error_locked('Recorder queue overflow; snapshots dropped')
                return False
            except Exception as exc:
                self._dropped += 1
                self._error_locked(str(exc))
                return False
            self._accepted += 1
            return True

    def record_telemetry(self, telemetry):
        return self._submit('telemetry', telemetry)

    def record_telemetry_owned(self, telemetry):
        """Transfer a tree; the caller must never mutate it after return.

        Admission, capture timestamps and FIFO order are synchronous and bounded.
        Recursive validation/copying belongs to the writer. Rejected/failed
        records remain visible in recorder accounting, including during close.
        """
        return self._submit('telemetry', telemetry, owned=True)

    def event(self, event_dict):
        return self._submit('event', event_dict)

    def command(self, command_dict):
        return self._submit('command', command_dict)

    def sensors(self, payload):
        """Raw acquisition data before estimation; may contain numpy arrays."""
        return self._submit('sensors', payload)

    def state(self):
        with self._lock:
            if self._fatal:
                status = 'error'
            elif self._finished.is_set():
                status = 'closed'
            elif self._closing.is_set():
                status = 'stopping'
            elif self._errors:
                status = 'degraded'
            else:
                status = 'recording' if self._ready else 'starting'
            return dict(status=status, active=status in ('starting', 'recording', 'degraded'), schema=SCHEMA,
                        run_id=self.run_id, directory=self.directory, session_dir=self.directory,
                        accepted=self._accepted, written=self._written, flushed=self._flushed,
                        fsynced=self._fsynced, queued=self._queue.qsize(),
                        pending=self._queue.qsize() + self._inflight,
                        dropped=self._dropped, failed=self._failed,
                        nonfinite_values=self._nonfinite, errors=list(self._errors),
                        error='; '.join(self._errors) or None, error_count=self._error_count,
                        records_written=dict(self._counts), paths=list(self._paths),
                        last_written_seq=self._last_written_seq,
                        last_flushed_seq=self._last_flushed_seq,
                        last_fsynced_seq=self._last_fsynced_seq,
                        close_complete=self._close_complete)

    def close(self, wait=True):
        with self._lock:
            self._closing.set()
        if wait:
            self._thread.join(timeout=config.TELEMETRY_LOG_CLOSE_TIMEOUT_S)
            if self._thread.is_alive():
                with self._lock:
                    self._error_locked('Recorder close timed out; queued data may not be durable', fatal=True)
        return self.state()

    def _open(self, relative):
        handle = open(Path(self.directory) / relative, 'x', encoding='utf-8', newline='')
        self._handles[relative] = handle
        self._sizes[relative] = 0
        with self._lock:
            self._paths.append(relative)
        return handle

    def _write(self, relative, text):
        self._handles[relative].write(text)
        self._sizes[relative] += len(text.encode('utf-8'))

    def _flush(self, sync=False):
        for handle in self._handles.values():
            handle.flush()
            if sync:
                os.fsync(handle.fileno())
        with self._lock:
            self._flushed = self._written
            self._last_flushed_seq = self._last_written_seq
            if sync:
                self._fsynced = self._written
                self._last_fsynced_seq = self._last_written_seq

    def _open_telemetry(self):
        self._telemetry_paths = ('rover/telemetry_%04d.jsonl' % self._part,
                                 'jumper/telemetry_%04d.jsonl' % self._part,
                                 'rover/system_%04d.csv' % self._part)
        for relative in self._telemetry_paths:
            self._open(relative)
        self._write(self._telemetry_paths[2], _csv_line(CSV_COLUMNS))
        self._part_rows = 0

    def _telemetry_lines(self, record):
        rover_path, jumper_path, _ = self._telemetry_paths
        rover_data = {key: value for key, value in record['data'].items() if key != 'jumper'}
        common = {key: value for key, value in record.items() if key != 'data'}
        rover = dict(common, stream='rover', paired_file=jumper_path,
                     paired_seq=record['seq'], data=rover_data)
        jumper = dict(common, stream='jumper', paired_file=rover_path,
                      paired_seq=record['seq'], present='jumper' in record['data'],
                      data=record['data'].get('jumper'))
        cells = [record[key] for key in CSV_COLUMNS[:4]]
        cells.extend(_scalar(rover_data, path) for path in SCALAR_FIELDS.values())
        return _json_line(rover), _json_line(jumper), _csv_line(cells)

    def _write_scout(self, scout_id, filename, record):
        # IDs are fixed by config; never interpret arbitrary received IDs as paths.
        if scout_id not in config.JUMPER_SCOUT_IDS:
            return
        relative = f'jumper/{scout_id}/{filename}'
        if relative not in self._handles:
            (Path(self.directory) / 'jumper' / scout_id).mkdir(exist_ok=True)
            self._open(relative)
        self._write(relative, _json_line(dict(record, stream='jumper', scout_id=scout_id)))

    def _write_record(self, record):
        if record['kind'] == 'telemetry':
            lines = self._telemetry_lines(record)
            if self._part_rows and any(self._sizes[path] + len(line.encode('utf-8')) >
                                       config.TELEMETRY_LOG_ROTATE_BYTES
                                       for path, line in zip(self._telemetry_paths, lines)):
                self._flush(sync=True)
                for relative in self._telemetry_paths:
                    self._handles.pop(relative).close()
                for relative in list(self._handles):
                    if relative.startswith('jumper/scout_') and '/telemetry_' in relative:
                        self._handles.pop(relative).close()
                self._part += 1
                self._open_telemetry()
                lines = self._telemetry_lines(record)
            for relative, line in zip(self._telemetry_paths, lines):
                self._write(relative, line)
            jumper = record['data'].get('jumper')
            slots = jumper.get('scouts', {}) if isinstance(jumper, dict) else {}
            for scout_id, data in slots.items():
                common = {key: value for key, value in record.items() if key != 'data'}
                self._write_scout(scout_id, 'telemetry_%04d.jsonl' % self._part,
                                  dict(common, data=data, paired_seq=record['seq'],
                                       paired_file=self._telemetry_paths[0]))
            self._part_rows += 1
        elif record['kind'] == 'sensors':
            line = _json_line(dict(record, stream='rover'))
            if self._sensor_rows and (self._sizes[self._sensor_path] + len(line.encode('utf-8')) >
                                      config.TELEMETRY_LOG_ROTATE_BYTES):
                self._flush(sync=True)
                self._handles.pop(self._sensor_path).close()
                self._sensor_part += 1
                self._sensor_path = 'rover/sensors_%04d.jsonl' % self._sensor_part
                self._open(self._sensor_path)
                self._sensor_rows = 0
            self._write(self._sensor_path, line)
            self._sensor_rows += 1
        else:
            stream = 'jumper' if _jumper_record(record['data']) else 'rover'
            relative = stream + '/' + ('events.jsonl' if record['kind'] == 'event' else 'commands.jsonl')
            self._write(relative, _json_line(dict(record, stream=stream)))
            if stream == 'jumper':
                scout_id = record['data'].get('scout_id')
                if isinstance(scout_id, str):
                    self._write_scout(scout_id, relative.split('/')[-1], record)
        with self._lock:
            self._written += 1
            self._counts[record['kind']] += 1
            self._last_written_seq = record['seq']

    def _run(self):
        self._handles, self._sizes = {}, {}
        self._part = 1
        current = None
        try:
            root = Path(self.directory)
            root.parent.mkdir(parents=True, exist_ok=True)
            root.mkdir()  # Never reuse or truncate another session, even after a collision.
            (root / 'rover').mkdir()
            (root / 'jumper').mkdir()
            self._open('metadata.json')
            self._write('metadata.json', _json_line(dict(
                schema=SCHEMA, run_id=self.run_id, created_host_utc=self._created_utc,
                created_monotonic_s=self._created_mono, metadata=self._metadata,
                clock_note='Host UTC is recorded as observed; clock synchronization is not assumed.',
                telemetry_note='One paired rover/jumper snapshot per producer call; join on run_id and seq. Multi-Scout snapshots also have jumper/scout_1 and jumper/scout_2 streams with the same sequence.',
                csv_note='Scalar convenience extract; complete fields and nulls are in JSONL. Cell voltage is a pack average.',
                scalar_columns={key: list(path) for key, path in SCALAR_FIELDS.items()},
                durability=dict(flush_s=config.TELEMETRY_LOG_FLUSH_S,
                                fsync_s=config.TELEMETRY_LOG_FSYNC_S,
                                queue_max=config.TELEMETRY_LOG_QUEUE_MAX,
                                rotate_bytes=config.TELEMETRY_LOG_ROTATE_BYTES))))
            for stream in ('rover', 'jumper'):
                for name in ('events.jsonl', 'commands.jsonl'):
                    self._open(stream + '/' + name)
            self._open_telemetry()
            self._sensor_part, self._sensor_rows = 1, 0
            self._sensor_path = 'rover/sensors_0001.jsonl'
            self._open(self._sensor_path)
            self._flush(sync=True)
            with self._lock:
                self._ready = True
            last_flush = last_sync = time.monotonic()
            while not (self._closing.is_set() and self._queue.empty()):
                try:
                    current = self._queue.get(timeout=min(0.05, config.TELEMETRY_LOG_FLUSH_S))
                except queue.Empty:
                    current = None
                if current is not None:
                    with self._lock:
                        self._inflight = 1
                    if current.pop('_owned', False):
                        try:
                            snapshot, nonfinite = _snapshot(current['data'])
                        except Exception as exc:
                            with self._lock:
                                self._failed += 1
                                self._error_locked('Telemetry snapshot: ' + str(exc))
                                self._inflight = 0
                            self._queue.task_done()
                            current = None
                        else:
                            current['data'] = snapshot
                            current['nonfinite_values'] = nonfinite
                            with self._lock:
                                self._nonfinite += nonfinite
                    if current is not None:
                        self._write_record(current)
                        self._queue.task_done()
                        current = None
                    with self._lock:
                        self._inflight = 0
                now = time.monotonic()
                if now - last_sync >= config.TELEMETRY_LOG_FSYNC_S:
                    self._flush(sync=True)
                    last_sync = last_flush = now
                elif now - last_flush >= config.TELEMETRY_LOG_FLUSH_S:
                    self._flush()
                    last_flush = now
            self._flush(sync=True)
        except Exception as exc:
            with self._lock:
                self._error_locked(type(exc).__name__ + ': ' + str(exc), fatal=True)
                if current is not None:
                    self._failed += 1
            if current is not None:
                self._queue.task_done()
            while True:
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                    with self._lock:
                        self._dropped += 1
                except queue.Empty:
                    break
        finally:
            for handle in self._handles.values():
                try:
                    handle.close()
                except Exception as exc:
                    with self._lock:
                        self._error_locked('Close: ' + str(exc), fatal=True)
            with self._lock:
                self._inflight = 0
            # Best-effort final evidence. A disk failure may prevent this file;
            # live state retains the error in that case.
            try:
                summary = self.state()
                summary.update(schema=SCHEMA, status='error' if self._fatal else 'closed',
                               close_complete=not self._fatal,
                               closed_host_utc=datetime.now(timezone.utc).isoformat(),
                               closed_monotonic_s=time.monotonic())
                with open(Path(self.directory) / 'summary.json', 'x', encoding='utf-8') as handle:
                    handle.write(_json_line(summary))
                    handle.flush()
                    os.fsync(handle.fileno())
                with self._lock:
                    self._paths.append('summary.json')
                    self._close_complete = not self._fatal
            except Exception as exc:
                with self._lock:
                    self._error_locked('Summary: ' + str(exc), fatal=True)
            self._finished.set()
