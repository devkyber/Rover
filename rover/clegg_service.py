"""Wireless Clegg acquisition for the existing station, without Rover controls.

Every TCP connection owns a new evidence directory. An impact is a nominal
acceleration candidate, not proof of contact, calibrated acceleration or CIV.
"""
from collections import OrderedDict, deque
from contextlib import ExitStack
import copy
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import threading
import time
import uuid

import config
from clegg import ClockMap, Decoder, associate_pose

ROOT = Path(__file__).resolve().parents[1]
LOSS_FLAGS = frozenset({'sequence_gap', 'sequence_repeated_or_reversed', 'fifo_loss',
                        'ring_loss', 'transport_crc_error', 'time_reversed'})
# Reconnects can occur after an operator edits files. A running receiver must
# keep its import-time evidence rather than claiming those later disk bytes.
SOURCE_CAPTURE_UTC = datetime.now(timezone.utc).isoformat()
SOURCE_SNAPSHOTS = {p: (ROOT / p).read_bytes() for p in (
    'rover/clegg_service.py', 'rover/clegg.py', 'rover/config.py', 'station/station.py')}
SOURCE_SHA256 = {p: hashlib.sha256(raw).hexdigest() for p, raw in SOURCE_SNAPSHOTS.items()}


def _setting(name):
    return getattr(config, 'CLEGG_' + name)


def _write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


class ImpactCapture:
    """Keep full sample resolution; bound memory and flag incomplete windows."""
    def __init__(self, threshold_g=None, pre_s=None, post_s=None, auto_rearm=None):
        self.threshold_g = _setting('IMPACT_THRESHOLD_G') if threshold_g is None else threshold_g
        self.pre_s = _setting('IMPACT_PRE_S') if pre_s is None else pre_s
        self.post_s = _setting('IMPACT_POST_S') if post_s is None else post_s
        self.auto_rearm = _setting('IMPACT_AUTO_REARM') if auto_rearm is None else auto_rearm
        self.max_points = int(_setting('IMPACT_MAX_POINTS'))
        self.history = deque(maxlen=self.max_points)
        self.impacts = OrderedDict()
        self.active = None
        self.latest = None
        self.armed = True
        self.above = False
        self.next_trigger_s = -math.inf
        self.last_time = None

    def arm(self, threshold_g=None):
        if self.active is not None:
            raise ValueError('Impact capture is still collecting its post-trigger window')
        if threshold_g is not None:
            if isinstance(threshold_g, bool) or not isinstance(threshold_g, (int, float)) or not math.isfinite(threshold_g) or not 0 < threshold_g <= 1000:
                raise ValueError('threshold_g must be finite and within (0, 1000]')
            self.threshold_g = float(threshold_g)
        self.armed, self.above = True, False

    def _begin(self, row, source_ref, manual=False):
        if self.active is not None:
            raise ValueError('An impact capture is already in progress')
        trigger = row['t']
        points = [copy.deepcopy(p) for p in self.history if trigger-self.pre_s <= p['t'] <= trigger]
        if not points or points[-1]['seq'] != row['seq']:
            points.append(copy.deepcopy(row))
        flags = {'manual_marker'} if manual else set()
        # FIFO-poll backdating produces uneven estimated intervals even when
        # every sequence arrives. Keep the actual bracketing sample rather than
        # declaring loss from a multiple of the nominal sensor period.
        boundary = trigger-self.pre_s
        before = next((p for p in reversed(self.history) if p['t'] <= boundary+1e-10), None)
        truncated = before is None and points[0]['t'] > boundary+1e-10
        if truncated:
            flags.add('pre_window_incomplete')
        if before is not None and before['seq'] != points[0]['seq'] and points[0]['seq'] != before['seq']+1:
            flags.add('pre_window_gap')
            truncated = True
        if len(points) > self.max_points:
            points = points[-self.max_points:]
            flags.add('capture_point_limit')
            truncated = True
        impact_id = uuid.uuid4().hex
        self.active = dict(id=impact_id, kind='impact_candidate', state='capturing', trigger_s=trigger,
                           points=points, flags=flags, truncated=truncated, source_ref=source_ref,
                           threshold_g=self.threshold_g, trigger='manual' if manual else 'threshold_crossing')
        self.impacts[impact_id] = self.active
        while len(self.impacts) > int(_setting('IMPACT_HISTORY')):
            self.impacts.popitem(last=False)
        self.latest = self.active
        self.armed = False
        self.next_trigger_s = trigger + _setting('IMPACT_REFRACTORY_S')
        return impact_id

    def mark(self, source_ref):
        if not self.history:
            raise ValueError('No sensor samples are available for a manual capture')
        return self._begin(self.history[-1], source_ref, manual=True)

    def finish(self, reason=None):
        if self.active is None:
            return None
        impact = self.active
        if reason:
            impact['flags'].add(reason)
            impact['truncated'] = True
        for point in impact['points']:
            impact['flags'].update(point['issues'])
        if impact['flags'].intersection(LOSS_FLAGS):
            impact['truncated'] = True
        impact['state'] = 'ready'
        self.active = None
        self.armed = bool(self.auto_rearm)
        return self.as_json(impact['id'])

    def feed(self, row, source_ref):
        completed = []
        if self.last_time is not None and row['t'] <= self.last_time:
            result = self.finish('time_reversed')
            if result:
                completed.append(result)
            self.history.clear()
            self.above = False
        self.last_time = row['t']
        if self.active is not None:
            end = self.active['trigger_s'] + self.post_s
            previous = self.active['points'][-1]
            if row['t'] <= end + 1e-10:
                if len(self.active['points']) < self.max_points:
                    self.active['points'].append(copy.deepcopy(row))
                else:
                    completed.append(self.finish('capture_point_limit'))
            if self.active is not None and row['t'] >= end - 1e-10:
                boundary_losses = set(row['issues']).intersection(LOSS_FLAGS)
                self.active['flags'].update(boundary_losses)
                reason = 'post_window_gap' if row['seq'] != previous['seq']+1 or boundary_losses else None
                completed.append(self.finish(reason))
        self.history.append(row)
        # Preserve one point at/before the pre-window boundary for coverage.
        while len(self.history)>1 and self.history[1]['t'] < row['t']-self.pre_s-1e-9:
            self.history.popleft()
        above = row['mag'] >= self.threshold_g
        if self.active is None and self.armed and above and not self.above and row['t'] >= self.next_trigger_s:
            self._begin(row, source_ref)
        self.above = above
        return completed

    def as_json(self, impact_id=None):
        impact = self.impacts.get(impact_id) if impact_id else self.latest
        if impact is None:
            return None
        points = impact['points']
        flags = set(impact['flags'])
        for point in points:
            flags.update(point['issues'])
        return dict(id=impact['id'], kind=impact['kind'], state=impact['state'],
                    time_s=[p['t']-impact['trigger_s'] for p in points],
                    xyz_g=[p['xyz'] for p in points], magnitude_g=[p['mag'] for p in points],
                    peak_g=max((p['mag'] for p in points), default=0.),
                    flags=sorted(flags), truncated=impact['truncated'], source_ref=impact['source_ref'],
                    calibrated=False, threshold_g=impact['threshold_g'], trigger=impact['trigger'],
                    position=copy.deepcopy(impact.get('position', dict(available=False, reason='rover_pose_not_associated'))),
                    release_request=copy.deepcopy(impact.get('release_request')),
                    timing=copy.deepcopy(impact.get('timing')),
                    contact_validated=False, sensor_time_validated=False)

    def reset_connection(self):
        self.history.clear()
        self.last_time = None
        self.above = False
        self.next_trigger_s = -math.inf


class Recording:
    def __init__(self, log_root, endpoint, trigger):
        name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '_' + uuid.uuid4().hex[:8]
        self.path = Path(log_root) / _setting('LOG_SUBDIR') / name
        self.path.mkdir(parents=True, exist_ok=False)
        self.started = time.monotonic()
        (self.path / 'source').mkdir()
        for source, raw in SOURCE_SNAPSHOTS.items():
            (self.path / 'source' / (Path(source).name + '.snapshot')).write_bytes(raw)
        self.metadata = dict(schema='clegg_wireless_capture_v1', created_utc=datetime.now(timezone.utc).isoformat(),
                             transport='tcp', endpoint=endpoint, trigger=trigger, device=None,
                             source_sha256=dict(SOURCE_SHA256),
                             source_capture='clegg_service_module_import', source_capture_utc=SOURCE_CAPTURE_UTC,
                             configuration_capture='values_from_imported_config_module',
                             config={k: v for k, v in vars(config).items() if k.startswith('CLEGG_')},
                             interpretation='Uncalibrated nominal acceleration; estimated sensor time. Candidates do not establish contact, CIV or Rover position.')
        self.metadata_path = self.path / 'metadata.json'
        _write_json(self.metadata_path, self.metadata)
        self.summary = dict(samples=0, sequence_missing=0, sequence_repeated_or_reversed=0,
                            fifo_loss_samples=0, ring_loss_samples=0, raw_limit_samples=0,
                            timing_reversed_samples=0, crc_errors=0, discarded_bytes=0, impact_candidates=0)
        with ExitStack() as opened:
            self.raw = opened.enter_context((self.path / 'tcp.bin').open('wb'))
            self.events = opened.enter_context((self.path / 'events.jsonl').open('w', encoding='utf-8'))
            self.csv = opened.enter_context((self.path / 'samples.csv').open('w', newline='', encoding='utf-8'))
            self.writer = csv.writer(self.csv)
            self.writer.writerow(['boot_id', 'seq', 'estimated_us', 'fifo_poll_us', 'raw_x', 'raw_y', 'raw_z',
                                  'flags', 'fifo_depth', 'nominal_x_g', 'nominal_y_g', 'nominal_z_g'])
            opened.pop_all()

    def event(self, kind, data):
        self.events.write(json.dumps(dict(kind=kind, host_monotonic_s=time.monotonic(), data=data), ensure_ascii=False, allow_nan=False) + '\n')

    def flush(self):
        for stream in (self.raw, self.csv, self.events):
            stream.flush()

    def close(self, decoder, error):
        self.summary.update(duration_s=time.monotonic()-self.started, error=error,
                            crc_errors=decoder.crc_errors, discarded_bytes=decoder.discarded_bytes,
                            trailing_bytes=len(decoder.buffer), contact_or_position_accuracy_verified=False)
        self.event('connection_closed', dict(error=error, trailing_bytes=len(decoder.buffer)))
        for stream in (self.raw, self.csv, self.events):
            stream.close()
        _write_json(self.path / 'summary.json', self.summary)
        hashes = {str(p.relative_to(self.path)).replace('\\', '/'): _sha256(p) for p in self.path.rglob('*') if p.is_file()}
        _write_json(self.path / 'sha256.json', hashes)


class CleggService:
    def __init__(self, host=None, port=None, log_root=None, discovery_port=None,
                 context_host=None, context_port=None):
        self.host = host
        self.port = _setting('TCP_PORT') if port is None else port
        self.discovery_port = _setting('DISCOVERY_PORT') if discovery_port is None else discovery_port
        self.log_root = Path(log_root) if log_root else ROOT / 'logs'
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.worker = None
        self.sock = None
        self.capture = ImpactCapture()
        self.decoder = Decoder()
        self.clock = ClockMap()
        self.pending_sync = {}
        self.recording = None
        self.last_recording = None
        self.device = None
        self.sample_count = 0
        self.errors = dict(crc=0, sequence_missing=0, fifo=0, ring=0)
        self.current_g = None
        self.observed = deque(maxlen=32)
        self.device_observed = deque(maxlen=32)
        self.previous = None
        self.error = None
        self.state = 'disconnected' if host else 'discovering'
        self.connected = False
        self.last_sample_host = None
        self.context_host = context_host
        self.context_port = _setting('CONTEXT_PORT') if context_port is None else context_port
        self.context_worker = None
        self.context_clock = ClockMap()
        self.pose_history = OrderedDict()
        self.context_received = None
        self.context_error = None
        self.pending_impacts = OrderedDict()
        self.release_requests = OrderedDict()
        self.latest_release = None

    def start(self):
        if self.worker is None:
            self.worker = threading.Thread(target=self._run, name='clegg-receiver', daemon=True)
            self.worker.start()
        if self.context_host and self.context_worker is None:
            self.context_worker = threading.Thread(target=self._context_run, name='clegg-rover-context', daemon=True)
            self.context_worker.start()
        return self

    def stop(self):
        self.stop_event.set()
        sock = self.sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self.worker:
            self.worker.join(timeout=_setting('CONNECT_TIMEOUT_S')+2.)
        if self.context_worker:
            self.context_worker.join(timeout=2*_setting('CONTEXT_CONNECT_TIMEOUT_S')+1.)

    def status(self):
        with self.lock:
            self._expire_release(time.monotonic())
            recent = self.capture.as_json()
            hz = None
            if len(self.observed) > 1:
                dt = self.observed[-1][0] - self.observed[0][0]
                if dt > 0:
                    hz = (self.observed[-1][1]-self.observed[0][1])/dt
            device_hz = None
            if len(self.device_observed) > 1:
                dt = self.device_observed[-1][0]-self.device_observed[0][0]
                if dt > 0:
                    device_hz = (self.device_observed[-1][1]-self.device_observed[0][1])/dt
            state = self.state
            if self.connected and self.last_sample_host is not None and time.monotonic()-self.last_sample_host > _setting('STREAM_STALE_S'):
                state = 'waiting_samples'
            return dict(available=True, status=state, connected=self.connected, device=copy.deepcopy(self.device),
                        sample_count=self.sample_count, sample_hz_observed=hz,
                        sample_hz_estimated=device_hz, sample_rate_time_validated=False, current_g=self.current_g,
                        errors=dict(self.errors), trigger=dict(threshold_g=self.capture.threshold_g,
                        pre_s=self.capture.pre_s, post_s=self.capture.post_s, armed=self.capture.armed,
                        auto_rearm=self.capture.auto_rearm),
                        latest_impact=None if recent is None else dict(id=recent['id'], state=recent['state'],
                        peak_g=recent['peak_g'], flagged=bool(recent['flags']), truncated=recent['truncated']),
                        recording=str(self.recording.path if self.recording else self.last_recording) if self.recording or self.last_recording else None,
                        recording_errors=dict(self.recording.summary, crc_errors=self.decoder.crc_errors,
                                              discarded_bytes=self.decoder.discarded_bytes) if self.recording else None,
                        release_request=copy.deepcopy(self.latest_release),
                        rover_context=self._context_status(), error=self.error, calibrated=False)

    def impact(self, impact_id=None):
        with self.lock:
            return self.capture.as_json(impact_id)

    def arm(self, threshold_g=None):
        with self.lock:
            self.capture.arm(threshold_g)
            if self.recording:
                self.recording.event('arm', dict(threshold_g=self.capture.threshold_g))
            return self.status()

    def mark(self):
        with self.lock:
            if not self.connected or self.last_sample_host is None or time.monotonic()-self.last_sample_host > _setting('STREAM_STALE_S'):
                raise ValueError('The sensor must be receiving current samples before a manual capture')
            impact_id = self.capture.mark(str(self.recording.path / 'samples.csv'))
            self._attach_trigger_context(self.capture.active, self.last_sample_host, manual=True)
            self.recording.event('impact_started', dict(id=impact_id, trigger='manual'))
            return dict(id=impact_id, kind='impact_candidate', state='capturing')

    def release_request(self, request_id, hammer_seq, browser_time_ms=None):
        """Record operator intent only. This endpoint never sends a Rover command."""
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128 or not all(c.isalnum() or c in '-_.' for c in request_id):
            raise ValueError('request_id must contain 1–128 letters, digits, dots, hyphens or underscores')
        if type(hammer_seq) is not int or not 0 <= hammer_seq < 2**53:
            raise ValueError('hammer_seq must be a nonnegative safe integer')
        if browser_time_ms is not None and (isinstance(browser_time_ms, bool) or not isinstance(browser_time_ms, (int, float)) or not math.isfinite(browser_time_ms)):
            raise ValueError('browser_time_ms must be finite')
        with self.lock:
            if request_id in self.release_requests:
                return copy.deepcopy(self.release_requests[request_id])
            now = time.monotonic()
            fresh = bool(self.connected and self.recording and self.last_sample_host is not None
                         and 0 <= now-self.last_sample_host <= _setting('STREAM_STALE_S'))
            state = 'sensor_unavailable' if not fresh else 'capture_busy' if self.capture.active else 'waiting_acceleration'
            if state == 'waiting_acceleration':
                self.capture.arm()
            request = dict(kind='release_request', request_id=request_id, shot_id=uuid.uuid4().hex,
                           hammer_seq=hammer_seq, browser_time_ms=browser_time_ms,
                           received_host_monotonic_s=now, expires_host_monotonic_s=now+_setting('RELEASE_CONTEXT_TTL_S'),
                           sensor_boot_id=self.device.get('boot_id') if self.device else None,
                           association_state=state, armed=state == 'waiting_acceleration',
                           intent_only=True, command_accepted=None, actual_release_validated=False,
                           interpretation='Browser Release request; no command acknowledgement or contact evidence')
            if self.latest_release and self.latest_release['association_state'] == 'waiting_acceleration':
                self.latest_release['association_state'] = 'superseded'
            self.release_requests[request_id] = request
            while len(self.release_requests) > _setting('RELEASE_HISTORY'):
                self.release_requests.popitem(last=False)
            self.latest_release = request
            if self.recording:
                self.recording.event('release_request', request)
            return copy.deepcopy(request)

    def _expire_release(self, now):
        if (self.latest_release and self.latest_release['association_state'] == 'waiting_acceleration'
                and now > self.latest_release['expires_host_monotonic_s']):
            self.latest_release['association_state'] = 'expired'
            if self.recording:
                self.recording.event('release_context_expired', self.latest_release)

    def _context_status(self):
        age = None if self.context_received is None else time.monotonic()-self.context_received
        fresh = age is not None and 0 <= age <= _setting('SYNC_MAX_AGE_S')
        return dict(configured=bool(self.context_host), connected=fresh and self.context_error is None,
                    endpoint=None if not self.context_host else dict(host=self.context_host, port=self.context_port),
                    boot_id=self.context_clock.boot_id, history_count=len(self.pose_history),
                    response_age_s=age, error=self.context_error,
                    read_only=True, deployed_or_validated=False)

    def _attach_trigger_context(self, impact, received, manual=False):
        mapped = self.clock.convert(self.device['boot_id'], impact['trigger_s'], received) if self.device else None
        impact['timing'] = dict(sensor_boot_id=self.device.get('boot_id') if self.device else None,
                                sensor_estimated_s=impact['trigger_s'],
                                host_time_estimate_s=mapped[0] if mapped else None,
                                clock_error_s=mapped[1] if mapped else None,
                                rover_context_epoch=self.context_clock.boot_id,
                                sensor_time_validated=False,
                                timestamp_source='fifo_poll_backdated_estimate',
                                position_time_basis='session_recorder_capture_not_sensor_acquisition')
        invalid_timing = any(set(p['issues']).intersection({'time_reversed', 'sequence_repeated_or_reversed'}) for p in impact['points'])
        impact['timing']['valid_for_association'] = not invalid_timing
        impact['position'] = dict(available=False, reason='context_pending' if self.context_host else 'rover_context_not_configured')
        if manual or invalid_timing:
            return
        request = self.latest_release
        if not request or request['association_state'] != 'waiting_acceleration':
            return
        now = time.monotonic()
        if now > request['expires_host_monotonic_s']:
            request['association_state'] = 'expired'
            return
        if not self.device or request['sensor_boot_id'] != self.device['boot_id']:
            request['association_state'] = 'sensor_epoch_changed'
            return
        if mapped is None:
            return
        # Buffered pre-request samples and clock-uncertain overlaps cannot be
        # promoted to a release-associated acceleration candidate.
        host, error = mapped
        if host-error < request['received_host_monotonic_s'] or host+error > request['expires_host_monotonic_s']:
            return
        request.update(association_state='acceleration_candidate_linked', impact_id=impact['id'],
                       association='estimated_clock_interval_after_request', clock_error_s=error,
                       request_to_trigger_estimate_s=host-request['received_host_monotonic_s'])
        impact['release_request'] = copy.deepcopy(request)
        if self.recording:
            self.recording.event('release_candidate_link', impact['release_request'])

    def _associate_context(self, impact, now):
        if not self.context_host:
            return dict(available=False, reason='rover_context_not_configured')
        timing = impact.get('timing') or {}
        if timing.get('valid_for_association') is False or set(impact['flags']).intersection({'time_reversed', 'sequence_repeated_or_reversed'}):
            return dict(available=False, reason='sensor_timing_invalid')
        host = timing.get('host_time_estimate_s')
        error = timing.get('clock_error_s')
        epoch = timing.get('rover_context_epoch')
        if host is None or error is None:
            return dict(available=False, reason='sensor_clock_unavailable')
        if epoch is None:
            return dict(available=False, reason='rover_context_not_ready_at_trigger')
        if epoch != self.context_clock.boot_id:
            return dict(available=False, reason='rover_context_epoch_changed')
        if self.context_error or self.context_received is None or not 0 <= now-self.context_received <= _setting('SYNC_MAX_AGE_S'):
            return dict(available=False, reason='rover_context_stale_or_unreachable')
        clock = self.context_clock
        bound = error+clock.error_s+abs(host-clock.updated_s)*_setting('ASSUMED_DRIFT_PPM')*1e-6
        result = associate_pose(list(self.pose_history.values()), host-clock.offset_s, bound, _setting('CONTEXT_MAX_AGE_S'))
        result.update(context_epoch=epoch, context_endpoint=dict(host=self.context_host, port=self.context_port),
                      position_accuracy_validated=False, hammer_contact_position=None,
                      timing_accuracy_validated=False,
                      timestamp_source='session_recorder_capture_not_sensor_acquisition')
        return result

    def _accept_context(self, response, sent, received, nonce):
        if (not isinstance(response, dict) or response.get('type') != 'clegg_context' or response.get('nonce') != nonce
                or response.get('read_only') is not True or not isinstance(response.get('boot_id'), str) or not response['boot_id']
                or not isinstance(response.get('history'), list) or len(response['history']) > _setting('CONTEXT_HISTORY')):
            raise ValueError('Rover context protocol mismatch')
        poses = []
        for original in response['history']:
            if not isinstance(original, dict):
                raise ValueError('Invalid Rover pose record')
            pose = copy.deepcopy(original)
            stamp = pose.get('captured_monotonic_s')
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                raise ValueError('Invalid Rover pose timestamp')
            if not isinstance(pose.get('source_log'), str) or type(pose.get('source_seq')) is not int:
                raise ValueError('Rover pose source identity is required')
            age = pose.get('imu_age_s')
            if isinstance(age, bool) or not isinstance(age, (int, float)) or not math.isfinite(age) or not 0 <= age <= _setting('CONTEXT_MAX_AGE_S'):
                pose['pose_valid'] = False
            if (not isinstance(pose.get('run_id'), str) or not pose['run_id']
                    or not isinstance(pose.get('pose_frame_id'), str) or not pose['pose_frame_id']
                    or type(pose.get('imu_generation')) is not int or pose['imu_generation'] < 0):
                pose['pose_valid'] = False
            poses.append(pose)
        # All validation precedes changing the clock epoch or pose history.
        updated = ClockMap()
        updated.update(response['boot_id'], sent, response['rx_s'], response['tx_s'], received)
        if self.context_clock.boot_id != updated.boot_id:
            self.pose_history.clear()
        self.context_clock = updated
        self.context_received, self.context_error = received, response.get('error')
        for pose in poses:
            self.pose_history[(pose['source_log'], pose['source_seq'])] = pose
        while len(self.pose_history) > _setting('CONTEXT_HISTORY'):
            self.pose_history.popitem(last=False)
        if self.recording:
            self.recording.event('rover_context', dict(sent_s=sent, received_s=received, response=response,
                                                      offset_s=updated.offset_s, clock_error_s=updated.error_s))

    def _context_run(self):
        nonce = 0
        while not self.stop_event.is_set():
            try:
                sent = time.monotonic()
                with socket.create_connection((self.context_host, self.context_port), timeout=_setting('CONTEXT_CONNECT_TIMEOUT_S')) as peer:
                    peer.settimeout(_setting('CONTEXT_CONNECT_TIMEOUT_S'))
                    peer.sendall(json.dumps(dict(type='context', nonce=nonce)).encode()+b'\n')
                    with peer.makefile('rb') as stream:
                        raw = stream.readline(_setting('CONTEXT_RESPONSE_MAX_BYTES')+1)
                    received = time.monotonic()
                if len(raw) > _setting('CONTEXT_RESPONSE_MAX_BYTES') or not raw.endswith(b'\n'):
                    raise ValueError('Truncated or oversized Rover context response')
                response = json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite context number')))
                with self.lock:
                    self._accept_context(response, sent, received, nonce)
                    self._resolve_pending_context(received)
            except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
                with self.lock:
                    self.context_error = str(exc)
                    self._resolve_pending_context(time.monotonic())
            nonce += 1
            self.stop_event.wait(_setting('SYNC_INTERVAL_S'))

    def _resolve_pending_context(self, now, force=False):
        for impact_id, deadline in list(self.pending_impacts.items()):
            impact = self.capture.impacts.get(impact_id)
            if impact is None:
                self.pending_impacts.pop(impact_id, None)
                continue
            position = self._associate_context(impact, now)
            if force or position['available'] or now >= deadline:
                impact.update(position=position, state='ready')
                self.pending_impacts.pop(impact_id)
                self._write_impact(self.capture.as_json(impact_id))

    def _discover(self):
        with self.lock:
            self.state = 'discovering'
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(('', self.discovery_port))
            listener.settimeout(.5)
            while not self.stop_event.is_set():
                try:
                    raw, peer = listener.recvfrom(4096)
                    data = json.loads(raw)
                    if not isinstance(data, dict) or data.get('type') != 'clegg_announce' or data.get('protocol') != 'clegg_v1':
                        continue
                    port = data.get('port', self.port)
                    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
                        continue
                    return peer[0], port
                except socket.timeout:
                    continue
                except (ValueError, UnicodeError):
                    continue
        return None

    def _command(self, command):
        self.sock.sendall(command.encode('ascii') + b'\n')
        self.recording.event('command', dict(text=command))

    @staticmethod
    def _validate_info(info):
        if info.get('protocol') != 'clegg_v1' or info.get('record_bytes') != 34 or info.get('devid') != 0xe5:
            raise ValueError('Firmware layout or ADXL375 identity mismatch; no acquisition started')
        for key in ('nominal_g_per_lsb', 'sample_hz'):
            value = info.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid device ' + key)
        if info['nominal_g_per_lsb'] > 1 or info['sample_hz'] > 100000 or not isinstance(info.get('boot_id'), str) or not info['boot_id']:
            raise ValueError('Invalid device scale, sample rate or boot identity')

    def _write_impact(self, impact):
        if impact is None:
            return
        _write_json(self.recording.path / ('impact_' + impact['id'] + '.json'), impact)
        self.recording.event('impact_ready', dict(id=impact['id'], peak_g=impact['peak_g'],
                                                flags=impact['flags'], truncated=impact['truncated']))
        self.recording.summary['impact_candidates'] += 1

    def _save_impact(self, impact):
        if impact is None:
            return
        stored = self.capture.impacts[impact['id']]
        stored['position'] = self._associate_context(stored, time.monotonic())
        if self.context_host and stored['position'].get('reason') == 'pose_time_not_bracketed':
            stored['state'] = 'awaiting_context'
            self.pending_impacts[impact['id']] = time.monotonic()+_setting('CONTEXT_ASSOCIATION_WAIT_S')
        else:
            self._write_impact(self.capture.as_json(impact['id']))

    def _feed(self, raw, received):
        """Called by the receiver only; serialization also protects HTTP captures."""
        with self.lock:
            self.recording.raw.write(raw)
            prior_crc = self.decoder.crc_errors
            frames = self.decoder.feed(raw)
            crc_added = self.decoder.crc_errors-prior_crc
            self.errors['crc'] += crc_added
            for kind, data in frames:
                if kind == 1:
                    self._validate_info(data)
                    if self.device and self.device['boot_id'] != data['boot_id']:
                        raise ValueError('Device boot changed; reopening as a new recording')
                    self.device = data
                    self.recording.metadata['device'] = data
                    _write_json(self.recording.metadata_path, self.recording.metadata)
                    self.recording.event('info', data)
                    self.state = 'waiting_samples'
                    self._command('START')
                elif kind == 4:
                    self.recording.event('device_status', data)
                    if data.get('sensor_ok') is False:
                        raise ValueError('Device reports sensor_ok=false; check SPI wiring')
                elif kind == 3:
                    self.recording.event('device_sync', data)
                    sent = self.pending_sync.pop(data.get('nonce'), None)
                    if sent is not None and self.device and data.get('boot_id') == self.device.get('boot_id'):
                        self.clock.update(data['boot_id'], sent, data['rx_us']/1e6, data['tx_us']/1e6, received)
                elif kind == 2:
                    if not self.device or data['boot_id'] != self.device['boot_id']:
                        raise ValueError('Unexpected sample boot identity; reopening as a new recording')
                    for sample in data['samples']:
                        self._sample(sample, received, bool(crc_added))
                    self.observed.append((received, self.sample_count))
                    if data['samples']:
                        last = data['samples'][-1]
                        self.device_observed.append((last['estimated_us']/1e6, last['seq']))

    def _sample(self, sample, received, crc_added):
        seq, flags = sample['seq'], sample['flags']
        issues = []
        if self.previous is not None:
            if seq <= self.previous:
                self.recording.summary['sequence_repeated_or_reversed'] += 1
                issues.append('sequence_repeated_or_reversed')
            elif seq > self.previous + 1:
                missing = seq-self.previous-1
                self.errors['sequence_missing'] += missing
                self.recording.summary['sequence_missing'] += missing
                issues.append('sequence_gap')
        self.previous = seq
        if crc_added:
            issues.append('transport_crc_error')
        if flags & ~15:
            issues.append('unknown_sensor_flags')
        for bit, label, counter in ((1, 'fifo_loss', 'fifo_loss_samples'), (2, 'ring_loss', 'ring_loss_samples'),
                                     (4, 'raw_limit', 'raw_limit_samples'), (8, 'time_reversed', 'timing_reversed_samples')):
            if flags & bit:
                issues.append(label)
                self.recording.summary[counter] += 1
        self.errors['fifo'] += bool(flags & 1)
        self.errors['ring'] += bool(flags & 2)
        xyz = [sample['raw_' + axis]*self.device['nominal_g_per_lsb'] for axis in 'xyz']
        self.current_g = xyz
        self.sample_count += 1
        self.recording.summary['samples'] += 1
        self.last_sample_host = received
        self.state = 'connected'
        self.error = None
        self.recording.writer.writerow([self.device['boot_id']] + [sample[k] for k in ('seq', 'estimated_us', 'fifo_poll_us', 'raw_x', 'raw_y', 'raw_z', 'flags', 'fifo_depth')] + xyz)
        row = dict(t=sample['estimated_us']/1e6, seq=seq, xyz=xyz, mag=math.sqrt(sum(v*v for v in xyz)), issues=issues)
        if self.capture.last_time is not None and row['t'] <= self.capture.last_time and 'time_reversed' not in issues:
            issues.append('time_reversed')
        old_id = self.capture.latest['id'] if self.capture.latest else None
        for impact in self.capture.feed(row, str(self.recording.path / 'samples.csv')):
            self._save_impact(impact)
        if self.capture.latest and self.capture.latest['id'] != old_id:
            self._attach_trigger_context(self.capture.latest, received)
            self.recording.event('impact_started', dict(id=self.capture.latest['id'], trigger='threshold_crossing', sensor_estimated_s=row['t']))

    def _connection(self, endpoint):
        with self.lock:
            self.state, self.error = 'connecting', None
        sock = socket.create_connection(endpoint, timeout=_setting('CONNECT_TIMEOUT_S'))
        sock.settimeout(_setting('RECEIVE_TIMEOUT_S'))
        self.sock = sock
        failure = None
        try:
            with self.lock:
                self.decoder, self.clock = Decoder(), ClockMap()
                self.pending_sync, self.previous, self.device = {}, None, None
                self.current_g, self.last_sample_host = None, None
                self.observed.clear()
                self.device_observed.clear()
                self.capture.reset_connection()
                self.recording = Recording(self.log_root, dict(host=endpoint[0], port=endpoint[1]), self.status()['trigger'])
                self.last_recording = self.recording.path
                self.connected, self.state = True, 'waiting_samples'
            self._command('STOP')
            self._command('INFO')
            started = last_data = last_flush = time.monotonic()
            last_sync, nonce = 0., 0
            while not self.stop_event.is_set():
                now = time.monotonic()
                if self.device is None and now-started > 5:
                    raise ValueError('No valid Clegg firmware INFO received within 5 seconds')
                if now-last_data > _setting('STREAM_STALE_S')+2:
                    raise TimeoutError('No wireless sensor data; reconnecting')
                if self.device and now-last_sync >= _setting('SYNC_INTERVAL_S'):
                    self.pending_sync[nonce] = now
                    self._command('SYNC ' + str(nonce))
                    self.pending_sync = {k: v for k, v in self.pending_sync.items() if now-v < _setting('SYNC_MAX_AGE_S')}
                    last_sync, nonce = now, nonce+1
                try:
                    raw = sock.recv(65536)
                except socket.timeout:
                    raw = None
                if raw == b'':
                    raise ConnectionError('Sensor TCP connection closed')
                if raw:
                    received = time.monotonic()
                    self._feed(raw, received)
                    last_data = received
                if now-last_flush >= 1:
                    with self.lock:
                        self._resolve_pending_context(time.monotonic())
                        self.recording.flush()
                    last_flush = now
        except Exception as exc:
            failure = str(exc)
            raise
        finally:
            try:
                sock.sendall(b'STOP\n')
            except OSError:
                pass
            sock.close()
            self.sock = None
            with self.lock:
                try:
                    if self.recording:
                        self._save_impact(self.capture.finish('receiver_stopped' if self.stop_event.is_set() else 'connection_lost'))
                        self._resolve_pending_context(time.monotonic(), force=True)
                        if self.latest_release and self.latest_release['association_state'] == 'waiting_acceleration':
                            self.latest_release['association_state'] = 'sensor_connection_closed'
                            self.recording.event('release_context_closed', self.latest_release)
                finally:
                    try:
                        if self.recording:
                            self.recording.close(self.decoder, failure)
                    finally:
                        self.recording = None
                        self.connected = False
                        self.current_g = None
                        self.observed.clear()
                        self.device_observed.clear()
                        self.capture.reset_connection()
                        self.state = 'disconnected'

    def _run(self):
        while not self.stop_event.is_set():
            try:
                endpoint = (self.host, self.port) if self.host else self._discover()
                if endpoint is not None:
                    self._connection(endpoint)
            except Exception as exc:
                with self.lock:
                    self.error, self.state, self.connected = str(exc), 'disconnected', False
            self.stop_event.wait(_setting('RECONNECT_S'))
