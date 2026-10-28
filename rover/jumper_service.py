"""Nonblocking Rover adapter for the supplied ESP jumping-scout protocol.

The backend maintains the v12 transport independently of browser focus/lifetime.
Commands are never replayed after a timeout, disconnect or reconnect; stop
requests have a separate worker. Only current-connection requests may actuate.
"""
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid

import config


def _safe(value):
    """Copy telemetry/context to JSON values; unknown numeric values stay null."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe(item) for item in value]
    return str(value)


def _identifier(value):
    return isinstance(value, str) and 8 <= len(value) <= 100


def _ack_error(result):
    # v4 replies include error='none' even for successful ACKs. Preserve that
    # raw ACK, but do not turn its sentinel into a failed operator result.
    if result.get('ok', False):
        return None
    error = result.get('error')
    return error if error and error != 'none' else 'ESP did not acknowledge command success'


class JumperService:
    def __init__(self, host, *, log_dir, event=lambda text: None, scout_id=None,
                 manager_factory=None, client_factory=None, clock=time.monotonic):
        self.host = str(host or '')
        self.scout_id = scout_id
        self._event = event
        self._clock = clock
        self._lock = threading.RLock()
        self._close_lock = threading.Lock()
        self._wake = threading.Condition(self._lock)
        self._closing = threading.Event()
        self._audit_closing = threading.Event()
        self._priority_done = threading.Event()
        self._priority_done.set()
        self._started = False
        self._manager_factory = manager_factory
        self._client_factory = client_factory
        self._manager = None
        self._client = None
        self._connection_status = 'off' if not self.host else 'connecting'
        self._connection_id = ''
        self._manager_state = {}
        self._manager_updated_at = None
        self._stream_state = dict(receiving=False, status_age_s=None, data_age_s=None,
                                  silence_timeout_s=config.JUMPER_STREAM_SILENCE_TIMEOUT_S,
                                  error=None)
        self._connection_error = None
        self._owner_client = None
        self._owner_id = None
        self._owner_at = -math.inf
        self._policy_allowed = False
        self._policy_at = -math.inf
        self._context = {}
        self._epoch = 0
        self._pending = deque()
        self._active = None
        self._urgent = None
        self._urgent_active = None
        self._last_result = None
        self._last_fire = None
        self._seen = deque(maxlen=config.JUMPER_REQUEST_HISTORY_MAX)
        self._counters = dict(accepted=0, rejected=0, completed=0, failed=0,
                              cancelled=0, reconnects=0, lease_expirations=0,
                              audit_dropped=0, stream_timeouts=0)
        self._audit = queue.Queue(maxsize=config.JUMPER_EVENT_QUEUE_MAX)
        self._audit_error = None
        self._close_error = None
        self._close_complete = False
        self._sensor_prefix = None
        self._client_attempt = 0
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self._session_dir = Path(log_dir).resolve() / (stamp + '_jumper_' + uuid.uuid4().hex[:8])
        self._events_path = self._session_dir / 'events.jsonl'
        self._metadata_path = self._session_dir / 'metadata.json'
        self._audit_file = None
        self._threads = []
        if not self.host:
            return
        try:
            self._session_dir.mkdir(parents=True, exist_ok=False)
            self._audit_file = self._events_path.open('x', encoding='utf-8')
            source_root = Path(__file__).resolve().parent
            source_files = [source_root / name for name in
                            ('jumper_service.py', 'scouts.py', 'config.py', 'rover_main.py', 'station_link.py')]
            source_files += sorted((source_root / 'vendor' / 'jumper').glob('*.py'))
            metadata = dict(schema='rover-jumper-events-v1', host=self.host, scout_id=self.scout_id,
                            created_utc=datetime.now(timezone.utc).isoformat(),
                            source_sha256={str(p.relative_to(source_root)):
                                           hashlib.sha256(p.read_bytes()).hexdigest()
                                           for p in source_files if p.is_file()},
                            timing='ESP boot microseconds and host clocks are distinct; no clock synchronization inferred',
                            mapping='Rover pose is context, not scout landing position; unknown offsets/uncertainty remain null')
            self._metadata_path.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
        except Exception as exc:
            self._audit_error = 'Cannot initialize jumper recording: ' + str(exc)

    def _new_client(self, **kwargs):
        factory = self._client_factory
        if factory is None:
            from vendor.jumper import ScoutClient
            factory = ScoutClient
        with self._lock:
            self._client_attempt += 1
            prefix = self._session_dir / ('scout_%04d_%s' % (self._client_attempt, uuid.uuid4().hex[:8]))
            self._sensor_prefix = str(prefix)
        client = factory(output=None if self._audit_error else str(prefix), port=config.JUMPER_CONTROL_PORT,
                         data_port=config.JUMPER_DATA_PORT, shutdown_event=self._closing,
                         control_rx_timeout_s=config.JUMPER_CONTROL_SILENCE_TIMEOUT_S,
                         close_timeout_s=config.JUMPER_CLOSE_TIMEOUT_S, **kwargs)
        with self._lock:
            self._client = client
        return client

    def start(self):
        with self._lock:
            if self._started:
                return
            self._started = True
            if not self.host:
                return
        for name, target in (('jumper-audit', self._audit_loop),
                             ('jumper-stop', self._stop_loop),
                             ('jumper-command', self._command_loop),
                             ('jumper-context', self._context_loop),
                             ('jumper-link', self._connection_loop)):
            thread = threading.Thread(name=name, target=target, daemon=True)
            self._threads.append(thread)
            thread.start()

    def _live_locked(self, now=None):
        now = self._clock() if now is None else now
        return (not self._closing.is_set()
                and self._connection_status == 'ok'
                and self._policy_allowed and now - self._policy_at < config.JUMPER_MAINLOOP_TIMEOUT_S)

    def _sync_gate_locked(self):
        # Manager guarantees these short methods perform no socket/file I/O or
        # ACK waits. Serialize gate changes with policy/owner changes so a stale
        # monitor iteration cannot re-enable an already revoked permit.
        if self._manager is not None:
            live = self._live_locked()
            try:
                self._manager.set_control_allowed(live)
                if live:
                    self._manager.ping(self._connection_id)
            except Exception as exc:
                # A concurrent ESP disconnect must not escape into Rover's
                # motor/IMU loop. Revoke locally; the link worker reconnects.
                self._connection_error = str(exc)
                self._halt_locked('ESP permission update failed')

    def update_policy(self, allowed, context):
        """Called by Rover's real control loop; never waits for an ACK or I/O."""
        with self._lock:
            previously_allowed = self._policy_allowed
            self._policy_allowed = bool(allowed)
            self._policy_at = self._clock()
            self._context = _safe(context or {})
            if not self._policy_allowed and (previously_allowed or self._owner_client is not None or self._pending
                                            or (self._active and previously_allowed)):
                self._halt_locked('Rover control policy denied')
            self._sync_gate_locked()

    def _record_locked(self, kind, **fields):
        if not self.host:
            return True
        if self._audit_closing.is_set():
            self._counters['audit_dropped'] += 1
            return False
        context_age = None if not math.isfinite(self._policy_at) else max(0.0, self._clock() - self._policy_at)
        row = dict(kind=kind, host=self.host, scout_id=self.scout_id,
                   host_utc=datetime.now(timezone.utc).isoformat(),
                   host_monotonic_s=self._clock(), connection_id=self._connection_id,
                   controller_id=self._owner_id, context=_safe(self._context),
                   context_age_s=context_age,
                   context_fresh=context_age is not None and context_age < config.JUMPER_MAINLOOP_TIMEOUT_S,
                   **_safe(fields))
        try:
            self._audit.put_nowait(row)
            return True
        except queue.Full:
            self._counters['audit_dropped'] += 1
            self._audit_error = 'Jumper event recording queue full'
            return False

    def _result_locked(self, request, ok, error=None, **fields):
        result = dict(request_id=request.get('request_id'), action=request.get('action'),
                      connection_id=request.get('connection_id'), ok=bool(ok), **_safe(fields))
        if self.scout_id is not None:
            result['scout_id'] = self.scout_id
        if error:
            result['error'] = str(error)
        # A slow command/stop from the previous ESP connection remains audit
        # evidence, but must not replace the new connection's current result.
        if result['connection_id'] == self._connection_id:
            self._last_result = result
        # v12 has no FIRE command or automatic launch/plot trigger. Historical
        # last_fire telemetry stays null for older read-only consumers.
        self._record_locked('result', result=result)
        return result

    def _reject_locked(self, request, error):
        self._counters['rejected'] += 1
        self._record_locked('rejected', request=_safe(request), reason=str(error))
        return self._result_locked(request, False, error)

    def _cancel_pending_locked(self, reason):
        while self._pending:
            work = self._pending.popleft()
            self._counters['cancelled'] += 1
            self._result_locked(work['message'], False, reason, cancelled=True)

    def _halt_locked(self, reason, request=None):
        self._epoch += 1
        self._cancel_pending_locked(reason)
        self._owner_client = self._owner_id = None
        self._owner_at = -math.inf
        if self._manager is not None:
            try: self._manager.revoke_control()
            except Exception as exc: self._connection_error = str(exc)
        message = dict(request or dict(action='stop', request_id=None),
                       connection_id=self._connection_id)
        work = dict(message=message,
                    epoch=self._epoch, connection_id=self._connection_id, reason=reason)
        if self._urgent:
            self._result_locked(self._urgent['message'], False, reason, cancelled=True)
            self._counters['cancelled'] += 1
        self._urgent = work
        self._priority_done.clear()
        self._record_locked('control_revoked', reason=reason)
        self._wake.notify_all()

    def _expire_locked(self):
        if self._policy_allowed and self._clock() - self._policy_at >= config.JUMPER_MAINLOOP_TIMEOUT_S:
            self._policy_allowed = False
            self._counters['lease_expirations'] += 1
            self._halt_locked('Rover control loop expired')

    def submit(self, client, message):
        """Validate and queue once. Returned acceptance is not an ESP ACK."""
        with self._lock:
            self._expire_locked()
            if not isinstance(message, dict):
                return self._reject_locked({}, 'Jumper request must be an object')
            action = message.get('action')
            automatic_connect = action in ('heartbeat', 'configure', 'latch', 'prepare', 'launch', 'compress', 'decompress', 'clear_fault') and self._owner_client is None
            request = {key: value for key, value in message.items() if key != 'type'}
            error = None
            allowed_fields = {'action', 'request_id', 'connection_id', 'controller_id', 'scout_id'}
            if action == 'configure': allowed_fields.add('settings')
            if action == 'clear_fault': allowed_fields.add('mechanism_reset')
            actions = {'claim', 'heartbeat', 'release_control', 'configure', 'latch',
                       'prepare', 'launch', 'compress', 'decompress', 'compression_stop', 'stop', 'estop', 'clear_fault'}
            if not isinstance(action, str) or action not in actions or set(request) - allowed_fields:
                error = 'Unsupported jumper action or fields'
            elif request.get('scout_id') != self.scout_id:
                error = 'Scout identity does not match this connection'
            elif not _identifier(request.get('request_id')) or not _identifier(request.get('controller_id')):
                error = 'Valid request_id and controller_id are required'
            elif self._connection_status != 'ok' or request.get('connection_id') != self._connection_id:
                error = 'Current ESP connection is required; refresh connection state'
            elif request['request_id'] in self._seen:
                error = 'Duplicate request rejected; commands are never replayed'
            elif action == 'launch' and self._manager_state.get('launch_supported') is not True:
                error = 'ESP firmware does not support LAUNCH; install launch-capable firmware'
            elif action == 'claim' or automatic_connect:
                if self._manager_state.get('motion_compatible') is False:
                    error = self._manager_state.get('compatibility_error') or 'ESP firmware is read-only until the matching protocol is verified'
                elif self._owner_client is not None and (self._owner_client is not client or self._owner_id != request['controller_id']):
                    error = 'Another controller owns the jumper'
                elif not self._policy_allowed or self._clock() - self._policy_at >= config.JUMPER_MAINLOOP_TIMEOUT_S:
                    error = 'Rover control loop must be live and E-STOP released'
                elif self._active or self._pending or self._urgent or self._urgent_active:
                    error = 'Wait for the preceding command or stop result'
            elif action not in ('estop', 'stop', 'compression_stop') and (self._owner_client is not client or self._owner_id != request['controller_id']):
                error = 'Another controller owns the jumper'
            elif action not in ('estop', 'stop', 'compression_stop', 'release_control') and not self._live_locked():
                error = 'Rover control loop is not live'
            if error:
                return self._reject_locked(request, error)
            # Freeze queued input: later caller mutation cannot alter a command.
            try:
                request = json.loads(json.dumps(request, allow_nan=False))
            except (TypeError, ValueError, OverflowError):
                return self._reject_locked(request, 'Command must contain finite JSON values')
            if action in ('configure', 'latch', 'prepare', 'launch', 'compress', 'decompress', 'clear_fault') and (
                    self._active or self._pending or self._urgent or self._urgent_active):
                return self._reject_locked(request, 'Previous command pending; do not queue the next stage')
            if action == 'compression_stop' and (self._urgent or self._urgent_active):
                return self._reject_locked(request, 'Stop already pending')
            self._record_locked('accepted', request=request)
            self._seen.append(request['request_id'])
            self._counters['accepted'] += 1
            if action == 'claim' or automatic_connect:
                self._owner_client, self._owner_id = client, request['controller_id']
                self._owner_at = self._clock()
                self._sync_gate_locked()
                if action in ('claim', 'heartbeat'):
                    return self._result_locked(request, True, **({'claimed': True} if action == 'claim' else {'connected': True}))
            if action not in ('release_control', 'estop'):
                self._owner_at = self._clock()
            if action == 'heartbeat':
                self._owner_at = self._clock()
                self._sync_gate_locked()
                return {'ok': True, 'request_id': request['request_id'], 'action': action}
            if action == 'release_control':
                self._halt_locked('Controller released control', request=request)
                return self._result_locked(request, True, released=True)
            if action in ('estop', 'stop'):
                self._halt_locked('Operator ' + action.upper(), request=request)
            elif action == 'compression_stop':
                self._epoch += 1
                self._cancel_pending_locked('Compression stopped before execution')
                if self._manager is not None:
                    self._manager.cancel_pending()
                self._urgent = dict(message=request, epoch=self._epoch,
                                    connection_id=self._connection_id, reason='Operator ' + action)
                self._priority_done.clear()
            else:
                if len(self._pending) >= config.JUMPER_COMMAND_QUEUE_MAX:
                    return self._reject_locked(request, 'Command queue full')
                self._pending.append(dict(message=request, epoch=self._epoch,
                                          connection_id=self._connection_id))
            self._wake.notify_all()
            return {'ok': True, 'queued': True, 'request_id': request['request_id'], 'action': action}

    def release(self, client):
        """Detach a closed browser, preserving server-owned v12 output/heartbeat.

        Explicit release_control is the stop path. A lost browser must not
        replay queued commands or prevent a new browser taking control.
        """
        with self._lock:
            if self._owner_client is client:
                self._epoch += 1
                self._cancel_pending_locked('Browser connection closed before execution')
                self._owner_client = self._owner_id = None
                if self._manager is not None:
                    self._manager.cancel_pending()
                self._record_locked('browser_detached')

    def emergency_stop(self):
        with self._lock:
            self._halt_locked('Rover E-STOP or shutdown', request=dict(action='estop', request_id=None))

    def snapshot(self):
        """Return cached state only; never poll the ESP from Rover's main loop."""
        with self._lock:
            pending = self._urgent or self._urgent_active or self._active or (self._pending[0] if self._pending else None)
            return _safe(dict(host=self.host, scout_id=self.scout_id, connection_status=self._connection_status,
                              automatic_control=config.JUMPER_AUTOMATIC_CONTROL,
                              connection_id=self._connection_id, owner_id=self._owner_id,
                              manager_snapshot_age_s=None if self._manager_updated_at is None else max(0.0, self._clock() - self._manager_updated_at),
                              stream=self._stream_state,
                              controller_live=self._live_locked(), busy=bool(pending),
                              pending_action=pending['message']['action'] if pending else None,
                              last_result=self._last_result, last_fire=self._last_fire,
                              manager=self._manager_state,
                              error=self._audit_error or self._connection_error,
                              recording=dict(prefix=self._sensor_prefix, directory=str(self._session_dir) if self.host else None,
                                             events_path=str(self._events_path) if self.host else None,
                                             metadata_path=str(self._metadata_path) if self.host else None,
                                             error=self._audit_error, close_complete=self._close_complete,
                                             close_error=self._close_error), counters=dict(self._counters)))

    def _command_loop(self):
        while not self._closing.is_set():
            with self._wake:
                if not self._pending:
                    self._wake.wait(config.JUMPER_POLL_S)
                    continue
                work = self._pending.popleft()
                self._expire_locked()
                if work['epoch'] != self._epoch or work['connection_id'] != self._connection_id or not self._live_locked():
                    self._counters['cancelled'] += 1
                    self._result_locked(work['message'], False, 'Controller or connection changed before execution', cancelled=True)
                    continue
                manager = self._manager
                self._active = work
                # Slot/controller IDs are host routing metadata, outside the
                # vendored firmware command schema. Keep them in audit/results.
                request = {key: value for key, value in work['message'].items()
                           if key not in ('controller_id', 'scout_id')}
            try:
                result = manager.action(request)
                error = None
            except Exception as exc:
                result, error = {}, str(exc)
            with self._lock:
                self._active = None
                cancelled = work['epoch'] != self._epoch or work['connection_id'] != self._connection_id
                ok = not error and not cancelled and bool(result.get('ok', False))
                self._counters['cancelled' if cancelled else 'completed' if ok else 'failed'] += 1
                self._result_locked(work['message'], ok,
                                    'Cancelled; command outcome requires inspection' if cancelled else error or _ack_error(result),
                                    cancelled=cancelled, ack=_safe(result))
                self._wake.notify_all()

    def _stop_loop(self):
        while True:
            with self._wake:
                if not self._urgent:
                    if self._closing.is_set(): return
                    self._wake.wait(config.JUMPER_POLL_S)
                    continue
                work, self._urgent = self._urgent, None
                self._urgent_active = work
                manager = self._manager
                current = work['connection_id'] == self._connection_id
            try:
                if manager is None or not current:
                    result = dict(ok=False, error='ESP connection changed or unavailable; lease was revoked')
                elif work['message']['action'] == 'compression_stop':
                    result = manager.compression_stop(work['connection_id'])
                elif work['message']['action'] == 'estop' and hasattr(manager, 'emergency_stop'):
                    result = manager.emergency_stop()
                else:
                    result = manager.stop()
                error = _ack_error(result)
            except Exception as exc:
                result, error = {}, str(exc)
            with self._lock:
                self._urgent_active = None
                ok = not error and bool(result.get('ok', False))
                self._counters['completed' if ok else 'failed'] += 1
                self._result_locked(work['message'], ok, error, ack=_safe(result))
                if not self._urgent: self._priority_done.set()
                self._wake.notify_all()

    def _stream_health(self, state, connected_at):
        """Transport progress, independent of sample validity and ESP clocks.

        The SDK keeps these receipt ages even when records fail timestamp/order
        checks. Such data must remain captured without a reconnect storm. A
        silent control stream, including a half-open socket, instead requires
        a fresh connection. Sensor/data age is recorded independently.
        """
        timeout = config.JUMPER_CONTROL_SILENCE_TIMEOUT_S
        transport = state.get('ages', {}).get('transport', {})
        result = dict(receiving=True, status_age_s=None, data_age_s=None,
                      silence_timeout_s=timeout, error=None)
        for channel in ('status', 'data'):
            age = transport.get('control_s', transport.get('status_s')) if channel == 'status' else transport.get('data_s')
            valid_age = type(age) in (int, float) and math.isfinite(age) and age >= 0
            result[channel + '_age_s'] = float(age) if valid_age else None
            if channel == 'status' and (not valid_age or age >= timeout):
                result['receiving'] = False
                if result['error'] is None and (valid_age or self._clock() - connected_at >= timeout):
                    result['error'] = ('ESP %s stream silent for %.3f s; reconnect required' %
                                       (channel, age if valid_age else self._clock() - connected_at))
        return result

    def _connection_loop(self):
        if not self.host:
            return
        manager = None
        try:
            from vendor.jumper.scout_client import FinalizationError
            factory = self._manager_factory
            if factory is None:
                from vendor.jumper import Manager
                factory = Manager
            manager = factory(factory=self._new_client, command_timeout_s=config.JUMPER_COMMAND_ACK_TIMEOUT_S)
            with self._lock:
                self._manager = manager
                manager.set_control_allowed(False)
            while not self._closing.is_set():
                try:
                    with self._lock:
                        self._connection_status = 'connecting'
                        self._record_locked('connection_attempt')
                    manager.connect(host=self.host)
                    state = manager.snapshot()
                    connected_at = self._clock()
                    with self._lock:
                        self._epoch += 1
                        self._connection_id = state['connection_id']
                        self._last_result = None
                        self._last_fire = None
                        self._connection_status = 'ok'
                        self._connection_error = None
                        self._manager_state = _safe(state)
                        self._manager_updated_at = self._clock()
                        self._stream_state = self._stream_health(state, connected_at)
                        self._owner_client = self._owner_id = None
                        self._seen.clear()
                        self._counters['reconnects'] += 1
                        manager.revoke_control()
                        self._record_locked('connected', recording_prefix=self._sensor_prefix,
                                            manager=self._manager_state)
                    while not self._closing.wait(config.JUMPER_POLL_S):
                        with self._lock:
                            self._expire_locked()
                            self._sync_gate_locked()
                        state = manager.snapshot()
                        stream = self._stream_health(state, connected_at)
                        with self._lock:
                            self._manager_state = _safe(state)
                            self._manager_updated_at = self._clock()
                            self._stream_state = stream
                        if not state.get('connected'):
                            raise RuntimeError(state.get('error') or 'ESP connection lost')
                        if stream['error']:
                            with self._lock:
                                self._counters['stream_timeouts'] += 1
                                self._record_locked('stream_timeout', stream=stream,
                                                    diagnostics=state.get('diagnostics', {}))
                            raise RuntimeError(stream['error'])
                except Exception as exc:
                    if isinstance(exc, FinalizationError):
                        with self._lock:
                            self._close_error = str(exc)
                        raise
                    with self._lock:
                        self._connection_status = 'lost'
                        self._connection_error = str(exc)
                        self._stream_state = dict(self._stream_state, receiving=False, error=str(exc))
                        # close() already revoked the owner and queued one stop.
                        # A cancelled constructor must not enqueue a second stop
                        # after the priority worker has completed and exited.
                        if not self._closing.is_set():
                            self._halt_locked('ESP connection lost; no command replay')
                        self._record_locked('connection_lost', error=str(exc))
                    try: manager.disconnect()
                    except Exception as disconnect_error:
                        with self._lock:
                            self._close_error = str(disconnect_error)
                            self._record_locked('disconnect_error', error=str(disconnect_error))
                        raise RuntimeError('Jumper disconnect finalization failed: ' + str(disconnect_error)) from disconnect_error
                    with self._lock:
                        self._connection_id = ''
                        self._manager_state = _safe(manager.snapshot())
                        self._manager_updated_at = self._clock()
                    if self._closing.wait(config.JUMPER_RECONNECT_DELAY_S): break
        except Exception as exc:
            with self._lock:
                self._connection_status = 'lost'
                self._connection_error = str(exc)
                self._record_locked('service_error', error=str(exc))
        finally:
            if manager is not None:
                self._priority_done.wait(config.JUMPER_CLOSE_COORDINATOR_ALLOWANCE_S)
                try: manager.close()
                except Exception as exc:
                    with self._lock:
                        self._close_error = str(exc)
                        self._record_locked('close_error', error=str(exc))
            with self._lock:
                self._connection_status = 'off' if self._closing.is_set() else 'lost'
                self._connection_id = ''
                self._stream_state = dict(self._stream_state, receiving=False)

    def _audit_loop(self):
        while not self._audit_closing.is_set() or not self._audit.empty():
            try: row = self._audit.get(timeout=config.JUMPER_POLL_S)
            except queue.Empty: continue
            try:
                if self._audit_file is None: raise OSError(self._audit_error or 'No event recording file')
                self._audit_file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
                self._audit_file.flush()
                if row['kind'] in ('connected', 'connection_lost', 'service_error', 'control_revoked'):
                    try: self._event('JUMPER ' + row['kind'] + ': ' + str(row.get('reason', row.get('error', self.host))))
                    except Exception: pass
            except Exception as exc:
                with self._lock:
                    self._audit_error = 'Jumper event recording failed: ' + str(exc)
            finally:
                self._audit.task_done()
        if self._audit_file is not None:
            try:
                self._audit_file.close()
            except Exception as exc:
                with self._lock:
                    self._close_error = 'Jumper event file finalization failed: ' + str(exc)

    def _context_loop(self):
        # Independent of ACK/socket activity, including reconnect attempts.
        # Keep host pose/status context for post-flight association; the sensor
        # SDK owns raw records, so cached sensor samples are not duplicated here.
        while not self._closing.wait(config.JUMPER_CONTEXT_LOG_INTERVAL_S):
            with self._lock:
                state = self._manager_state
                self._record_locked('context', connection_status=self._connection_status,
                                           esp_status=state.get('status', {}),
                                           ages=state.get('ages', {}),
                                           diagnostics=state.get('diagnostics', {}),
                                           stream=self._stream_state,
                                           manager_snapshot_age_s=None if self._manager_updated_at is None else max(0.0, self._clock() - self._manager_updated_at),
                                           motion_compatible=state.get('motion_compatible'))

    def close(self):
        """Cancel connection attempts, stop, and finalize records before returning.

        Failure to finalize within the bounded SDK + coordinator budget is an
        explicit error, never a successful close of still-running daemon tasks.
        """
        with self._close_lock:
            with self._lock:
                if self._close_complete:
                    return
                if not self.host:
                    self._closing.set()
                    self._close_complete = True
                    return
                if not self._closing.is_set():
                    self._policy_allowed = False
                    if self._started:
                        self._halt_locked('Jumper service closing')
                    self._closing.set()
                    self._wake.notify_all()
            deadline = time.monotonic() + config.JUMPER_CLOSE_TIMEOUT_S + config.JUMPER_CLOSE_COORDINATOR_ALLOWANCE_S
            for thread in self._threads:
                if thread.name != 'jumper-audit':
                    thread.join(timeout=max(0, deadline - time.monotonic()))
            with self._lock:
                self._record_locked('shutdown', close_error=self._close_error,
                                    unfinished_workers=[t.name for t in self._threads
                                                        if t.name != 'jumper-audit' and t.is_alive()])
            self._audit_closing.set()
            if not self._started:
                self._audit_loop()
            for thread in self._threads:
                if thread.name == 'jumper-audit':
                    thread.join(timeout=max(0, deadline - time.monotonic()))
            with self._lock:
                unfinished = [t.name for t in self._threads if t.is_alive()]
                if unfinished:
                    self._close_error = 'Jumper finalization incomplete: ' + ', '.join(unfinished)
                self._close_complete = not unfinished and not self._close_error
                if not self._close_complete:
                    raise RuntimeError(self._close_error or 'Jumper finalization incomplete')
