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
    link.publish(dict)      telemetry for every connected station
    link.publish_video(...) one compressed camera frame

Each client has its own sender thread and its own small outbox, so one
stalled station can never block the control loop or another station.
"""
import collections
import json
import math
import socket
import socketserver
import threading
import time

import config
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
    def __init__(self, port=config.STATION_PORT, hello=None):
        self.port = port
        self.hello = dict(hello or {}, type="hello")   # sent once to each new station
        self._lock = threading.Lock()
        self._control = set()                # outboxes of /control clients
        self._video = set()                  # outboxes of /video clients
        self._requests = []
        self._events = collections.deque(maxlen=20)
        self._event_id = 0

        self._forward = 0.0
        self._turn = 0.0
        self._lift = 0.0
        self._pan = 0.0
        self._estop = False
        self._last_command = -math.inf       # nothing received yet = stale

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
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    # ── called by the control loop ─────────────────────────────────

    def command(self):
        """
        The drive command to act on now.

        Deadman: if no drive message has arrived for COMMAND_TIMEOUT_S the
        command is zero, whatever the last one said. A station that walks
        out of Wi-Fi range must not leave the wheels running.
        """
        with self._lock:
            stale = time.monotonic() - self._last_command > config.COMMAND_TIMEOUT_S
            if stale or self._estop:
                return Command(0.0, 0.0, 0.0, 0.0, stale, self._estop)
            return Command(self._forward, self._turn, self._lift, self._pan, False, False)

    def take_requests(self):
        """One-off requests ({'type': 'new_run', ...}) since the last call."""
        with self._lock:
            requests, self._requests = self._requests, []
        return requests

    def event(self, text):
        """A line for the station's system log. Also printed here."""
        print(text)
        with self._lock:
            self._event_id += 1
            self._events.append([self._event_id, text])

    def publish(self, telemetry):
        """Queue one telemetry message for every station. Never blocks."""
        with self._lock:
            telemetry["events"] = list(self._events)
            telemetry["stations"] = len(self._control)
            clients = list(self._control)
        text = json.dumps(telemetry, separators=(",", ":"))
        for box in clients:
            with box.cond:
                box.telemetry = text
                box.cond.notify()

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
        except (OSError, ValueError):
            pass                             # client vanished or sent garbage
        finally:
            ws.close()

    def _serve_control(self, ws):
        box = _Outbox()
        ws.send(json.dumps(self.hello))
        with self._lock:
            self._control.add(box)
        threading.Thread(target=self._send_telemetry, args=(ws, box),
                         daemon=True).start()
        try:
            while True:
                text = ws.recv()             # raises socket.timeout after 5 s of silence
                if text is None:
                    return
                self._on_message(ws, json.loads(text))
        finally:
            with self._lock:
                self._control.discard(box)
            box.close()

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
        if kind == "drive":
            forward, turn = float(msg["forward"]), float(msg["turn"])
            # The hobby servos (string lift, camera pan) ride on the same
            # message, so the one deadman and the one e-stop cover them too.
            # A page that does not send them means zero: servos stopped.
            lift, pan = float(msg.get("lift", 0.0)), float(msg.get("pan", 0.0))
            if not all(math.isfinite(v) for v in (forward, turn, lift, pan)):
                return
            with self._lock:
                self._forward = max(-1.0, min(1.0, forward))
                self._turn = max(-1.0, min(1.0, turn))
                self._lift = max(-1.0, min(1.0, lift))
                self._pan = max(-1.0, min(1.0, pan))
                self._last_command = time.monotonic()
        elif kind == "estop":
            on = bool(msg.get("on", True))
            with self._lock:
                changed = on != self._estop
                self._estop = on
            if changed:
                self.event("E-STOP ON -- drive commands ignored" if on
                           else "E-STOP released")
        elif kind == "ping":
            # Answered here, not by the control loop, so the round trip the
            # station measures is the network's and nothing else's.
            ws.send(json.dumps({"type": "pong", "id": msg.get("id")}))
        elif kind in ("new_run", "stop_log", "zero_battery"):
            with self._lock:
                self._requests.append(msg)
