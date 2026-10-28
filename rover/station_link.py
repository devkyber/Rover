"""
The rover's side of the link to the station laptop.

One TCP port (config.STATION_PORT), two WebSocket channels:

    /control   station -> rover   drive commands, e-stop, "new run"   (JSON)
               rover -> station   telemetry, 20 times a second        (JSON)
    /video     rover -> station   the camera's H.264, frame by frame  (binary)

They are separate connections on purpose. Video is 10 Mbit/s and telemetry
is a few kbit/s; on one connection a video backlog on a weak Wi-Fi link
would hold every drive command and telemetry message queued behind it.

The control loop never touches a socket. It calls four things:

    link.command()          what the operator wants right now (deadman applied)
    link.take_requests()    one-off requests since the last call
    link.publish_owned(dict) transfer fresh telemetry for recording/publication
    link.publish_video(...) one compressed camera frame

Each client has its own sender thread and its own small outbox, so one
stalled station can never block the control loop or another station.
"""
import collections
import copy
import json
import math
import socket
import socketserver
import threading
import time

import config
from battery import BatteryProtection
import hammer
import websocket

Command = collections.namedtuple("Command", "forward turn lift pan stale estop")

# A station sends a drive message 20 times a second for as long as its page
# is open. One that has been silent this long is gone (laptop asleep, Wi-Fi
# lost) even if TCP has not noticed yet. Also the limit for a blocked send.
CLIENT_TIMEOUT_S = 5.0

# Video frames a slow client may fall behind before we skip ahead (0.5 s).
VIDEO_QUEUE_MAX = 15


class _Outbox:
    """What one client's sender thread still has to send."""

    def __init__(self):
        self.cond = threading.Condition()
        self.telemetry = None                # newest only -- old telemetry is worthless
        self.frames = collections.deque()    # video, in order
        self.need_key = True                 # video must (re)start on a keyframe
        self.closed = False

    def close(self):
        with self.cond:
            self.closed = True
            self.cond.notify_all()


class StationLink:
    def __init__(self, port=config.STATION_PORT, hello=None, *, clock=time.monotonic):
        self.port = port
        self.hello = dict(hello or {}, type="hello")   # sent once to each new station
        self._lock = threading.Lock()
        self._control = set()                # outboxes of /control clients
        self._video = set()                  # outboxes of /video clients
        self._requests = []
        self._events = collections.deque(maxlen=20)
        self._event_id = 0
        self.recorder = None                # Optional nonblocking session writer.
        self._recording_error = None
        self._recording_failures = 0
        self._publish_cond = threading.Condition()
        self._publish_thread = None
        self._publish_pending = None       # Latest display only; recording is FIFO.
        self._publish_generation = 0
        self._publish_closed = False
        self._publish_inflight = 0
        self._publish_counts = dict(submitted=0, published=0, replaced=0,
                                    failed=0, discarded=0)
        self._publish_error = None
        self._client_ids = {}
        self._next_client_id = 0

        self._forward = 0.0
        self._turn = 0.0
        self._lift = 0.0
        self._pan = 0.0
        self._pan_source = None  # Last PAN input, only for release/disconnect.
        self._last_pan_command = -math.inf
        self._estop = False
        self.battery = BatteryProtection(self.hello.get('pico', 'off'))
        self._battery_event_key = None
        self._last_command = -math.inf       # nothing received yet = stale
        self._clock = clock
        self._drive_owner = None             # Last moving input, for disconnect/idle handling only.
        self._drive_ignored = 0
        self.hammer = hammer.Inbox()
        self.jumper = None  # Attached before start; worker owns ESP socket I/O.
        self.mapping = None  # Separate /mapping channel; never part of drive traffic.

        link = self

        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                link._handle(self.request)

        class Server(socketserver.ThreadingTCPServer):
            # Linux: lets a restart re-bind at once, yet still refuses a port
            # another program is listening on. Windows gives the same option
            # a different meaning -- two programs may share the port -- so
            # there it stays off.
            allow_reuse_address = not config.ON_WINDOWS
            daemon_threads = True

        # Binding here, not in a thread: a port that is already taken (a
        # second copy of this program) fails loudly before any motor moves.
        self._server = Server(("0.0.0.0", port), Handler)

    def start(self):
        self.start_telemetry()
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def start_telemetry(self):
        with self._publish_cond:
            if self._publish_thread is None and not self._publish_closed:
                self._publish_thread = threading.Thread(
                    target=self._publish_worker, name='telemetry-publisher', daemon=True)
                self._publish_thread.start()

    def telemetry_state(self):
        """Small counters; never waits for encoding or recorder work."""
        with self._publish_cond:
            return dict(self._publish_counts, pending=int(self._publish_pending is not None),
                        inflight=self._publish_inflight, error=self._publish_error,
                        closed=self._publish_closed)

    def close_telemetry(self, wait=True):
        """Drain the latest display update with a bounded post-actuator join."""
        with self._publish_cond:
            self._publish_closed = True
            self._publish_cond.notify_all()
            worker = self._publish_thread
        if wait and worker is not None:
            worker.join(timeout=config.TELEMETRY_PUBLISH_CLOSE_TIMEOUT_S)
            if worker.is_alive():
                with self._publish_cond:
                    self._publish_error = 'Telemetry publisher close timed out'
        return self.telemetry_state()

    # ── called by the control loop ─────────────────────────────────

    def set_recorder(self, recorder):
        """Replace the producer target; its owner handles closing the old one."""
        with self._lock:
            if recorder is not self.recorder:
                self.recorder = recorder
                self._recording_error = None
                self._recording_failures = 0
                with self._publish_cond:
                    self._publish_generation += 1
                    if self._publish_pending is not None:
                        self._publish_counts['discarded'] += 1
                        self._publish_pending = None

    def _recording_failed(self, recorder, operation, error):
        # Never emit another event here: the failing writer may be event().
        with self._lock:
            if self.recorder is recorder:
                self._recording_error = f'{operation}: {type(error).__name__}: {error}'
                self._recording_failures += 1

    def _record_to(self, recorder, operation, record):
        if recorder is None:
            return True
        try:
            # The writer may queue this record. Later caller changes must not
            # alter already-received commands or an observed telemetry sample.
            return getattr(recorder, operation)(copy.deepcopy(record)) is not False
        except Exception as error:
            self._recording_failed(recorder, operation, error)
            return False

    def _recording_state(self, recorder):
        try:
            state = copy.deepcopy(recorder.state())
            if not isinstance(state, dict):
                raise TypeError('Recorder state must be a dict')
        except Exception as error:
            self._recording_failed(recorder, 'state', error)
            state = {'active': False, 'status': 'error'}
        with self._lock:
            error, failures = self._recording_error, self._recording_failures
        if error:
            state['link_error'] = error
            state['link_failures'] = failures
            state['error'] = state.get('error') or error
        return state

    def _client_id(self, ws):
        with self._lock:
            if ws not in self._client_ids:
                self._next_client_id += 1
                self._client_ids[ws] = f'control-{self._next_client_id:06d}'
            return self._client_ids[ws]

    def command(self):
        """
        The drive command to act on now.

        Deadman: if no drive message has arrived for COMMAND_TIMEOUT_S the
        command is zero, whatever the last one said. A station that walks
        out of Wi-Fi range must not leave the wheels running.
        """
        with self._lock:
            now = self._clock()
            self._expire_drive_locked(now)
            drive_stale = now - self._last_command > config.COMMAND_TIMEOUT_S
            pan_stale = now - self._last_pan_command > config.COMMAND_TIMEOUT_S
            stale = drive_stale and pan_stale
            if self._stopped_locked():
                return Command(0.0, 0.0, 0.0, 0.0, stale, self._estop)
            return Command(0.0 if drive_stale else self._forward,
                           0.0 if drive_stale else self._turn,
                           0.0 if drive_stale else self._lift,
                           0.0 if pan_stale else self._pan, stale, False)

    def _stopped_locked(self):
        return self._estop

    def update_battery(self, board):
        """Update the voltage display; battery readings never block controls."""
        with self._lock:
            self.battery.update(board, self._clock())
            state = self.battery.state(self._clock())
            key = state['level']
            changed = key != self._battery_event_key
            self._battery_event_key = key
        if changed and state['level'] == 'low':
            volts = '--' if state['voltage_v'] is None else f"{state['voltage_v']:.3f} V"
            self.event(f"LOW BATTERY {volts}")
        return state

    def _clear_drive_locked(self):
        self._drive_owner = None
        self._forward = self._turn = self._lift = 0.0
        self._last_command = -math.inf

    def _clear_pan_locked(self):
        self._pan = 0.0
        self._pan_source = None
        self._last_pan_command = -math.inf

    def _expire_drive_locked(self, now):
        if self._drive_owner is not None and now - self._last_command > config.COMMAND_TIMEOUT_S:
            self._clear_drive_locked()
        if self._pan_source is not None and now - self._last_pan_command > config.COMMAND_TIMEOUT_S:
            self._clear_pan_locked()

    def _release_drive(self, ws):
        with self._lock:
            if self._drive_owner is ws:
                self._clear_drive_locked()
            if self._pan_source is ws:
                self._clear_pan_locked()

    def take_requests(self):
        """One-off requests ({'type': 'new_run', ...}) since the last call."""
        with self._lock:
            requests, self._requests = self._requests, []
        return requests

    def event(self, text, *, scout_id=None):
        """A line for the station's system log. Also printed here."""
        print(text)
        with self._lock:
            self._event_id += 1
            self._events.append([self._event_id, text])
            record = {'event_id': self._event_id, 'text': text,
                      'subsystem': 'jumper' if text.startswith('JUMPER') else 'rover'}
            if scout_id is not None:
                record['scout_id'] = scout_id
            recorder = self.recorder
        self._record_to(recorder, 'event', record)

    def publish(self, telemetry):
        """Defensive synchronous API for callers retaining mutable input.

        Runtime control uses publish_owned instead; keep this API's immediate
        defensive snapshot semantics for other callers.
        """
        with self._lock:
            now = self._clock()
            self._expire_drive_locked(now)
            telemetry["events"] = list(self._events)
            telemetry["stations"] = len(self._control)
            telemetry['battery_protection'] = self.battery.state(now)
            telemetry["drive_control"] = {
                "owner_active": self._drive_owner is not None,
                "command_age_s": now - self._last_command if math.isfinite(self._last_command) else None,
                "ignored_other_client_commands": self._drive_ignored,
            }
            clients = list(self._control)
            recorder = self.recorder
        if recorder is not None:
            telemetry['recording'] = self._recording_state(recorder)
            if telemetry['recording'].get('active') is not False:
                if not self._record_to(recorder, 'record_telemetry', telemetry):
                    telemetry['recording'] = self._recording_state(recorder)
        else:
            # The main loop may supply a final stopped-session state for UI.
            telemetry.setdefault('recording', None)
        text = json.dumps(telemetry, separators=(",", ":"))
        for box in clients:
            with box.cond:
                box.telemetry = text
                box.cond.notify()

    def publish_owned(self, telemetry):
        """Transfer a fresh complete tree; callers must never mutate it again.

        The recorder reserves its FIFO entry immediately without copying. Its
        writer handles validation/formatting. Only browser updates coalesce;
        the encoder owns one pending reference and never holds the control
        lock during JSON work. Detach cannot redirect an older recorded sample.
        """
        self.start_telemetry()
        with self._publish_cond:
            if self._publish_closed:
                return False
        with self._lock:
            now = self._clock()
            self._expire_drive_locked(now)
            telemetry['events'] = list(self._events)
            telemetry['stations'] = len(self._control)
            telemetry['battery_protection'] = self.battery.state(now)
            telemetry['drive_control'] = {
                'owner_active': self._drive_owner is not None,
                'command_age_s': now - self._last_command if math.isfinite(self._last_command) else None,
                'ignored_other_client_commands': self._drive_ignored,
            }
            recorder = self.recorder
            generation = self._publish_generation
            recording_error, recording_failures = self._recording_error, self._recording_failures
        # The builder already captured recorder + estimator diagnostics. Do
        # not overwrite that state with a second, less complete recorder view.
        if 'recording' not in telemetry:
            telemetry['recording'] = self._recording_state(recorder) if recorder else None
        if recording_error:
            state = telemetry['recording'] or {}
            state.update(link_error=recording_error, link_failures=recording_failures,
                         error=state.get('error') or recording_error)
            telemetry['recording'] = state
        telemetry['telemetry_transport'] = self.telemetry_state()
        if recorder is not None and (telemetry['recording'] or {}).get('active') is not False:
            try:
                if recorder.record_telemetry_owned(telemetry) is False:
                    self._recording_failed(recorder, 'record_telemetry_owned', RuntimeError('Recorder rejected telemetry'))
            except Exception as error:
                self._recording_failed(recorder, 'record_telemetry_owned', error)
        # The accepted recording and browser encoder only read this source.
        with self._publish_cond:
            self._publish_counts['submitted'] += 1
            if self._publish_closed or generation != self._publish_generation:
                self._publish_counts['discarded'] += 1
                return False
            if self._publish_pending is not None:
                self._publish_counts['replaced'] += 1
            self._publish_pending = (generation, telemetry)
            self._publish_cond.notify()
        return True

    def _publish_worker(self):
        while True:
            with self._publish_cond:
                self._publish_cond.wait_for(lambda: self._publish_closed or self._publish_pending is not None)
                if self._publish_pending is None:
                    return
                generation, telemetry = self._publish_pending
                self._publish_pending = None
                self._publish_inflight = 1
            try:
                text = json.dumps(telemetry, separators=(',', ':'))
                with self._lock:
                    clients = list(self._control)
                # Client conditions are independent of the command lock. Even
                # a wedged outbox must not hold up E-STOP/deadman/control reads.
                stale = False
                for box in clients:
                    with box.cond:
                        with self._publish_cond:
                            if generation != self._publish_generation:
                                stale = True
                                break
                            box.telemetry = text
                            box.cond.notify()
                with self._publish_cond:
                    if generation != self._publish_generation:
                        stale = True
                    self._publish_counts['discarded' if stale else 'published'] += 1
            except Exception as error:
                with self._publish_cond:
                    self._publish_counts['failed'] += 1
                    self._publish_error = type(error).__name__ + ': ' + str(error)
            finally:
                with self._publish_cond:
                    self._publish_inflight = 0
                    self._publish_cond.notify_all()

    def publish_video(self, frame, is_key):
        """
        Queue one compressed frame for every video client. Never blocks.

        H.264 frames build on the ones before, so a client that fell behind
        cannot just skip one: its queue is emptied and it waits for the next
        keyframe (the camera sends one every second).
        """
        message = (b"\x01" if is_key else b"\x00") + frame
        with self._lock:
            clients = list(self._video)
        for box in clients:
            with box.cond:
                if len(box.frames) >= VIDEO_QUEUE_MAX:
                    box.frames.clear()
                    box.need_key = True
                if box.need_key and not is_key:
                    continue
                box.need_key = False
                box.frames.append(message)
                box.cond.notify()

    # ── one thread per connection from here down ───────────────────

    def _handle(self, sock):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(CLIENT_TIMEOUT_S)
        try:
            ws = websocket.accept(sock)
        except OSError:
            return
        if ws is None:
            return
        try:
            if ws.path == "/control":
                self._serve_control(ws)
            elif ws.path == "/video":
                self._serve_video(ws)
            elif ws.path == "/mapping":
                self._serve_mapping(ws)
        except (OSError, ValueError):
            pass                             # client vanished or sent garbage
        finally:
            ws.close()

    def _serve_control(self, ws):
        box = _Outbox()
        client_id = self._client_id(ws)
        try:
            self.event(f'Station {client_id}: control connected')
            ws.send(json.dumps(self.hello))
            with self._lock:
                self._control.add(box)
            threading.Thread(target=self._send_telemetry, args=(ws, box),
                             daemon=True).start()
            while True:
                text = ws.recv()             # raises socket.timeout after 5 s of silence
                if text is None:
                    return
                self._on_message(ws, json.loads(text))
        finally:
            with self._lock:
                self._control.discard(box)
            self._release_drive(ws)
            self.hammer.stop(ws, forget=True)
            if self.jumper is not None:
                self.jumper.release(ws)
            box.close()
            self.event(f'Station {client_id}: control disconnected')
            with self._lock:
                self._client_ids.pop(ws, None)

    def _send_telemetry(self, ws, box):
        try:
            while True:
                with box.cond:
                    while box.telemetry is None and not box.closed:
                        box.cond.wait()
                    if box.closed:
                        return
                    text, box.telemetry = box.telemetry, None
                ws.send(text)
        except OSError:
            ws.close()                       # wakes the reader, which cleans up

    def _serve_video(self, ws):
        box = _Outbox()
        with self._lock:
            self._video.add(box)
        try:
            while True:
                with box.cond:
                    while not box.frames:
                        box.cond.wait()
                    frame = box.frames.popleft()
                ws.send(frame)
        finally:
            with self._lock:
                self._video.discard(box)

    def _on_message(self, ws, msg):
        kind = msg.get("type")
        if kind != 'ping':
            client_id = self._client_id(ws)
            with self._lock:
                recorder = self.recorder
            self._record_to(recorder, 'command', {
                'client_id': client_id, 'kind': kind, 'stage': 'received',
                'subsystem': 'jumper' if kind == 'jumper' else 'rover',
                **({'scout_id': msg['scout_id']} if kind == 'jumper' and 'scout_id' in msg else {}),
                'payload': msg,
            })
        if kind == "drive":
            forward, turn = float(msg["forward"]), float(msg["turn"])
            # Legacy lift input is intentionally ignored: hammer control has
            # its own owner/deadman, independent of phone drive heartbeats.
            lift, pan = 0.0, float(msg.get("pan", 0.0))
            if not all(math.isfinite(v) for v in (forward, turn, lift, pan)):
                return
            with self._lock:
                now = self._clock()
                self._expire_drive_locked(now)
                if self._stopped_locked():
                    return
                # PAN takes any fresh held input immediately, independently of
                # wheel control. Idle peers cannot cancel or renew that hold.
                if pan or self._pan_source is ws:
                    self._pan = max(-1.0, min(1.0, pan))
                    self._pan_source = ws if pan else None
                    self._last_pan_command = now
                moving = any((forward, turn))
                if not moving and self._drive_owner is not None and self._drive_owner is not ws:
                    # Idle dashboards cannot cancel or renew a held input.
                    # Any new moving input takes over immediately.
                    self._drive_ignored += 1
                    return
                self._drive_owner = ws if moving else None
                self._forward = max(-1.0, min(1.0, forward))
                self._turn = max(-1.0, min(1.0, turn))
                self._lift = max(-1.0, min(1.0, lift))
                self._last_command = now
        elif kind == "drive_release":
            self._release_drive(ws)
        elif kind == "estop":
            on = bool(msg.get("on", True))
            with self._lock:
                changed = self._estop != on
                self._estop = on
                self._clear_drive_locked()
                self._clear_pan_locked()
            if changed:
                self.event("E-STOP ON -- drive commands ignored" if on
                           else "E-STOP released")
            if on:
                self.hammer.stop()
                if self.jumper is not None:
                    self.jumper.emergency_stop()
        elif kind == "hammer":
            with self._lock:
                self.hammer.submit(ws, msg, self._stopped_locked())
        elif kind == "jumper":
            with self._lock:
                stopped = self._stopped_locked()
            if self.jumper is None:
                result = {"ok": False, "error": "Jumper connection is not enabled"}
            elif stopped and msg.get('action') not in ('estop', 'stop', 'compression_stop', 'release_control'):
                result = {"ok": False, "error": "Release Rover E-STOP before claiming jumper control"}
            else:
                result = self.jumper.submit(ws, msg)
            # This acknowledges queue acceptance only. The later ESP ACK and
            # fresh device state are exposed separately in jumper telemetry.
            if msg.get('action') != 'heartbeat':
                if 'scout_id' in msg:
                    result = dict(result, scout_id=msg['scout_id'])
                ws.send(json.dumps(dict(result, type='jumper_result',
                                        request_id=msg.get('request_id'))))
        elif kind == "ping":
            # Answered here, not by the control loop, so the round trip the
            # station measures is the network's and nothing else's.
            ws.send(json.dumps({"type": "pong", "id": msg.get("id")}))
        elif kind in ("new_run", "stop_log", "zero_battery", "video_start", "video_stop"):
            with self._lock:
                self._requests.append(msg)

    def _serve_mapping(self, ws):
        """A slow map export cannot occupy the driving socket or map worker.

        The service callback only queues a result. This connection's sender
        owns JSON encoding/socket waits; a full outbox disconnects this map
        client and the client can recover committed records with a snapshot.
        No messages received here can command hardware.
        """
        condition = threading.Condition()
        replies = collections.deque()
        closed = threading.Event()

        def enqueue(result):
            with condition:
                if closed.is_set():
                    return
                if len(replies) >= config.MAPPING_REPLY_QUEUE_MAX:
                    closed.set()
                else:
                    replies.append(result)
                condition.notify_all()

        def send_results():
            try:
                while True:
                    with condition:
                        condition.wait_for(lambda: closed.is_set() or replies)
                        if closed.is_set():
                            return
                        result = replies.popleft()
                    ws.send(json.dumps(result, separators=(",", ":"), allow_nan=False))
            except (OSError, ValueError, TypeError):
                pass
            finally:
                closed.set()
                ws.close()

        enqueue({"type": "hello", "channel": "mapping"})
        sender = threading.Thread(target=send_results, name='mapping-client', daemon=True)
        sender.start()
        try:
            while not closed.is_set():
                text = ws.recv()
                if text is None:
                    return
                msg = json.loads(text)
                if not isinstance(msg, dict):
                    continue
                if msg.get('type') == 'ping':
                    enqueue({"type": "pong", "id": msg.get('id')})
                elif msg.get('type') == 'mapping':
                    if self.mapping is None:
                        enqueue({"type": "mapping_result", "request_id": msg.get('request_id'),
                                 "action": msg.get('action'), "ok": False,
                                 "error": "Mission mapping service is unavailable"})
                    else:
                        self.mapping.submit(msg, enqueue)
        finally:
            with condition:
                closed.set()
                condition.notify_all()
