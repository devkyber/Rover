"""Bounded, read-only transfer of Rover-owned Scout raw recordings.

The provider reads cached snapshots only. This server never opens an ESP socket,
claims a controller, or sends commands. Clients name a configured Scout slot and
an opaque source cursor; they cannot name files. One newline-delimited JSON
request receives one response and the TCP connection closes.
"""
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import stat as file_stat
import threading
import time
import uuid

import config


def _scalar(value):
    if value is None or type(value) in (bool, int):
        return value
    if type(value) is float:
        return value if math.isfinite(value) else None
    return value[:256] if isinstance(value, str) else None


def _context(value):
    """Expose pose provenance, never arbitrary snapshot/configuration fields."""
    if not isinstance(value, dict):
        return {}
    scalar_keys = (
        'host_utc', 'host_monotonic_s', 'run_id', 'run_elapsed_s', 'pose_valid',
        'imu_generation', 'pose_frame_id', 'pose_quality', 'odom_frame_id',
        'pose_continuity_loss_count', 'position_uncertainty_m', 'frame_id',
        'position_evidence', 'heading_rad', 'heading_evidence',
        'stationary_prelaunch_asserted', 'stationary_evidence', 'context_age_s',
        'context_fresh',
    )
    result = {key: _scalar(value[key]) for key in scalar_keys if key in value}
    for key in ('rover_position_m', 'rover_rpy_deg', 'scout_landing_position_m',
                'scout_to_rover_offset_m', 'position_world_m'):
        vector = value.get(key)
        if isinstance(vector, (list, tuple)) and len(vector) == 3:
            result[key] = [_scalar(item) if type(item) in (int, float) else None
                           for item in vector]
        elif key in value:
            result[key] = None
    sources = value.get('input_sources')
    if isinstance(sources, dict):
        result['input_sources'] = {key: _scalar(sources[key])
                                   for key in ('imu', 'wheels') if key in sources}
    return result


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate_key')
        result[key] = value
    return result


class ScoutPlotFeed:
    """Serve only current per-slot SDK ``.records.bin`` files beneath root_dir.

    provider() returns Scouts.snapshot(), optionally with top-level ``context``.
    A slot may add ``last_fire`` containing the latest acknowledged successful
    FIRE request and its connection_id/context. That retained field is preferred
    to last_result, which subsequent commands may replace. The client must first
    baseline each live connection before treating a changed FIRE as a trigger.
    """
    def __init__(self, provider, *, root_dir, bind=None, port=None, scout_ids=None):
        if not callable(provider):
            raise TypeError('provider must be callable')
        self.provider = provider
        self.root_dir = Path(root_dir).resolve()
        self.bind = config.SCOUT_PLOT_FEED_BIND if bind is None else bind
        self.port = config.SCOUT_PLOT_FEED_PORT if port is None else port
        self.scout_ids = tuple(config.JUMPER_SCOUT_IDS if scout_ids is None else scout_ids)
        self.record_bytes = 40  # SDK binary record wire layout, not a tuning value.
        self.chunk_bytes = (int(config.SCOUT_PLOT_FEED_CHUNK_BYTES) // self.record_bytes
                            * self.record_bytes)
        if self.chunk_bytes < self.record_bytes:
            raise ValueError('Scout feed chunk must hold a complete SDK record')
        self.request_bytes = int(config.SCOUT_PLOT_FEED_REQUEST_BYTES)
        self.response_bytes = int(config.SCOUT_PLOT_FEED_RESPONSE_BYTES)
        self.socket_timeout = float(config.SCOUT_PLOT_FEED_SOCKET_TIMEOUT_S)
        self.close_timeout = float(config.SCOUT_PLOT_FEED_CLOSE_TIMEOUT_S)
        self.feed_session_id = uuid.uuid4().hex
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._source_lock = threading.Lock()
        self._clients = {}
        self._source_observations = {}
        self._slots = threading.BoundedSemaphore(config.SCOUT_PLOT_FEED_MAX_CLIENTS)
        self._socket = None
        self.worker = None
        self.address = None
        self.last_error = None

    def start(self):
        if self.worker is not None:
            return self
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((self.bind, self.port))
            listener.listen(config.SCOUT_PLOT_FEED_MAX_CLIENTS)
            listener.settimeout(config.SCOUT_PLOT_FEED_POLL_S)
        except BaseException:
            listener.close()
            raise
        self._socket = listener
        self.address = listener.getsockname()
        self.worker = threading.Thread(target=self._accept, name='scout-plot-feed', daemon=True)
        self.worker.start()
        return self

    def stop(self):
        """Interrupt idle/slow clients and bound total shutdown waiting."""
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
        with self._lock:
            clients = list(self._clients.items())
        for connection, _ in clients:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        deadline = time.monotonic() + self.close_timeout
        threads = [self.worker] + [thread for _, thread in clients]
        for thread in threads:
            if thread and thread is not threading.current_thread():
                thread.join(timeout=max(0., deadline - time.monotonic()))

    close = stop

    def _error(self, code, **fields):
        return dict(ok=False, read_only=True, error=code,
                    feed_session_id=self.feed_session_id, **fields)

    def _accept(self):
        while not self._stop.is_set():
            try:
                connection, _ = self._socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            connection.settimeout(self.socket_timeout)
            if self._stop.is_set():
                connection.close()
                break
            if not self._slots.acquire(blocking=False):
                # Tiny fixed reply fits a fresh TCP send buffer. Never wait for
                # an occupied client slot in Rover's producer or accept loop.
                try:
                    connection.setblocking(False)
                    connection.send(self._encode(self._error('busy')))
                except OSError:
                    pass
                connection.close()
                continue
            thread = threading.Thread(target=self._serve, args=(connection,),
                                      name='scout-plot-read', daemon=True)
            with self._lock:
                if self._stop.is_set():
                    connection.close()
                    self._slots.release()
                    break
                self._clients[connection] = thread
            thread.start()

    def _encode(self, value):
        encoded = (json.dumps(value, separators=(',', ':'), allow_nan=False) + '\n').encode('utf-8')
        if len(encoded) > self.response_bytes:
            encoded = (json.dumps(self._error('response_too_large')) + '\n').encode('utf-8')
        return encoded

    def _serve(self, connection):
        try:
            raw = bytearray()
            while b'\n' not in raw:
                part = connection.recv(min(1024, self.request_bytes + 1 - len(raw)))
                if not part:
                    return
                raw.extend(part)
                if len(raw) > self.request_bytes:
                    connection.sendall(self._encode(self._error('request_too_large')))
                    return
            line, extra = raw.split(b'\n', 1)
            try:
                if extra.strip():
                    raise ValueError('one_request_only')
                request = json.loads(line.decode('utf-8'), object_pairs_hook=_unique_object,
                                     parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite')))
            except (ValueError, UnicodeError):
                response = self._error('invalid_json')
            else:
                response = self.read(request)
            connection.sendall(self._encode(response))
        except socket.timeout:
            try:
                connection.sendall(self._encode(self._error('request_timeout')))
            except OSError:
                pass
        except OSError:
            pass  # A closed laptop cannot interrupt recording or Rover controls.
        finally:
            connection.close()
            with self._lock:
                self._clients.pop(connection, None)
            self._slots.release()

    def _source(self, scout_id, slot):
        recording = slot.get('recording')
        prefix = recording.get('prefix') if isinstance(recording, dict) else None
        connection_id = slot.get('connection_id')
        if not isinstance(prefix, str) or not prefix or not isinstance(connection_id, str) or not connection_id:
            return None
        try:
            path = Path(prefix).with_suffix('.records.bin').resolve()
            relative = path.relative_to(self.root_dir)
        except (ValueError, OSError, RuntimeError):
            raise ValueError('source_outside_root') from None
        try:
            stream = path.open('rb')
        except FileNotFoundError:
            return None
        try:
            with self._source_lock:
                # Observe file growth in lock order. An older concurrent fstat
                # must not make an append look like a truncation.
                stat = os.fstat(stream.fileno())
                if not file_stat.S_ISREG(stat.st_mode):
                    raise ValueError('source_not_regular')
                first = stream.read(self.record_bytes)
                total = stat.st_size // self.record_bytes * self.record_bytes
                key = (connection_id, str(relative), stat.st_dev, stat.st_ino)
                prior = self._source_observations.get(scout_id)
                generation = prior['generation'] if prior else 0
                if prior and (prior['key'] != key or total < prior['total'] or
                              (prior['first'] and first[:len(prior['first'])] != prior['first'])):
                    generation += 1
                self._source_observations[scout_id] = dict(
                    key=key, generation=generation, total=total,
                    first=first if len(first) == self.record_bytes else b'')
            identity = json.dumps([scout_id, *key, generation], separators=(',', ':'))
            source_id = hashlib.sha256(identity.encode('utf-8')).hexdigest()
            return stream, source_id, total
        except BaseException:
            stream.close()
            raise

    def read(self, request):
        """Return a bounded protocol response. Exposed for hardware-free checks."""
        if not isinstance(request, dict) or set(request) - {'type', 'scout_id', 'cursor'}:
            return self._error('invalid_request')
        scout_id = request.get('scout_id')
        if request.get('type') != 'scout_plot_read' or not isinstance(scout_id, str) or scout_id not in self.scout_ids:
            return self._error('invalid_request')
        cursor = request.get('cursor')
        if cursor is not None and (
                not isinstance(cursor, dict) or set(cursor) != {'source_id', 'offset'} or
                not isinstance(cursor.get('source_id'), str) or len(cursor['source_id']) != 64 or
                type(cursor.get('offset')) is not int or cursor['offset'] < 0 or
                cursor['offset'] % self.record_bytes):
            return self._error('invalid_cursor', scout_id=scout_id)
        try:
            snapshot = self.provider()
            slots = snapshot.get('scouts', {}) if isinstance(snapshot, dict) else {}
            slot = slots.get(scout_id)
            if not isinstance(slot, dict):
                return self._error('scout_unavailable', scout_id=scout_id)
            connection_id = _scalar(slot.get('connection_id'))
            result = slot.get('last_fire', slot.get('last_result'))
            fire = None
            if (isinstance(result, dict) and result.get('action') == 'fire' and result.get('ok') is True and
                    isinstance(result.get('request_id'), str) and 0 < len(result['request_id']) <= 100 and
                    result.get('connection_id') == connection_id and connection_id):
                fire = dict(request_id=result['request_id'], action='fire', ok=True,
                            connection_id=connection_id)
                if 'context' in result:
                    fire['context'] = _context(result['context'])
            stream_info = slot.get('stream')
            stream_info = ({key: _scalar(stream_info[key]) for key in
                            ('receiving', 'status_age_s', 'data_age_s', 'silence_timeout_s')
                            if key in stream_info} if isinstance(stream_info, dict) else {})
            manager = slot.get('manager')
            manager = manager if isinstance(manager, dict) else {}
            diagnostics = manager.get('diagnostics')
            if isinstance(diagnostics, dict):
                stream_info['diagnostics'] = {key: diagnostics[key] for key in (
                    'received_records', 'raw_received_records', 'record_gaps', 'bad_records',
                    'client_dropped', 'record_order_errors', 'sensor_timestamp_errors',
                    'estimated_timestamp_overlaps', 'status_timestamp_errors', 'status_duplicates',
                    'frame_gaps', 'crc_errors', 'discarded_bytes', 'sequence_errors', 'tx_dropped')
                    if type(diagnostics.get(key)) is int and diagnostics[key] >= 0}
            stream_info['data_integrity_error'] = bool(manager.get('data_integrity_error'))
            recording = slot.get('recording')
            stream_info['recording_error'] = bool(recording.get('error')) if isinstance(recording, dict) else False
            metadata = dict(api=_scalar(manager.get('api')),
                            units_verified=manager.get('units_verified') is True,
                            motion_compatible=manager.get('motion_compatible') is True,
                            field_validated=False, record_bytes=self.record_bytes)
            hashes = slot.get('source_sha256')
            if isinstance(hashes, dict):
                metadata['source_sha256'] = {key: value for key, value in hashes.items()
                    if isinstance(key, str) and len(key) <= 128 and isinstance(value, str)
                    and len(value) == 64 and all(c in '0123456789abcdef' for c in value)}
            response = dict(ok=True, read_only=True, scout_id=scout_id,
                            feed_session_id=self.feed_session_id,
                            source_id=None, source_state='waiting', offset=0, next_offset=0,
                            total_bytes=0, records_base64='', fire=fire,
                            connection_id=connection_id,
                            connection_status=_scalar(slot.get('connection_status')),
                            stream=stream_info, metadata=metadata,
                            context=_context(snapshot.get('context')))
            source = self._source(scout_id, slot)
            if source is None:
                return response
            stream, source_id, total = source
            with stream:
                if cursor is not None and cursor['source_id'] != source_id:
                    return self._error('source_changed', scout_id=scout_id, source_id=source_id,
                                       total_bytes=total, connection_id=connection_id)
                offset = 0 if cursor is None else cursor['offset']
                if offset > total:
                    return self._error('offset_out_of_range', scout_id=scout_id,
                                       source_id=source_id, total_bytes=total)
                stream.seek(offset)
                payload = stream.read(min(self.chunk_bytes, total - offset))
                if len(payload) % self.record_bytes:
                    return self._error('source_changed_during_read', scout_id=scout_id)
            response.update(source_id=source_id, source_state='ready', offset=offset,
                            next_offset=offset + len(payload), total_bytes=total,
                            records_base64=base64.b64encode(payload).decode('ascii'))
            return response
        except ValueError as exc:
            code = str(exc)
            return self._error(code if code in ('source_outside_root', 'source_not_regular') else 'source_unavailable',
                               scout_id=scout_id)
        except Exception:
            self.last_error = 'Scout plot provider or raw recording unavailable'
            return self._error('source_unavailable', scout_id=scout_id)
