"""
Measure the link to the rover, the way the station page uses it.

    python station/link_test.py 192.168.0.200 --label "30 m, clear view"

Run it on the station laptop while rover_main.py runs on the rover. It
opens the same two connections the page does (/control and /video), does
nothing but listen and ping for 30 s, prints one line a second, then a
summary, and adds one row to data/link_tests.csv. Move the rover, run it
again with a new --label, and the rows make the range table.

It sends no drive commands. For a stationary radio test, run the rover with
--no-motors --imu fake --pico off --no-log and close other station pages.

What the numbers mean:

  telemetry gap   longest silence between two telemetry messages. The rover
                  sends 20 a second, so 50 ms is perfect.
  ping            round trip of a small message, station -> rover -> station.
                  This is application round-trip time, not one-way latency.
  video gap       longest silence between two camera frames (33 ms is perfect).
  wifi            how strongly the rover hears the router, in dBm.

The rover's 0.5 s deadman is used as a conservative screening threshold.
Telemetry and ping cannot prove drive-command timing or test the deadman.
Video arrival gaps do not measure decoded-picture latency. Startup delay
is excluded; gaps include silence from the last message to the test end.

Standard library only, like station.py.
"""
import argparse
import base64
import csv
import datetime
import json
import math
import os
import socket
import statistics
import struct
import threading
import time

DEADMAN_S = 0.5              # rover/config.py COMMAND_TIMEOUT_S
SOCKET_TIMEOUT_S = 3.0       # silence this long = connection counted as lost, reopened
CSV_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "data", "link_tests.csv")


class WebSocketClient:
    """The client half of WebSocket (RFC 6455): just enough for this test."""

    def __init__(self, host, port, path):
        self.sock = socket.create_connection((host, port), timeout=SOCKET_TIMEOUT_S)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        self.sock.sendall((f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\n"
                           "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                           f"Sec-WebSocket-Key: {key}\r\n"
                           "Sec-WebSocket-Version: 13\r\n\r\n").encode("ascii"))
        self._reader = self.sock.makefile("rb")
        status = self._reader.readline()
        if b" 101 " not in status:
            raise OSError(f"{host}:{port} did not accept a WebSocket: {status!r}")
        while self._reader.readline().strip():      # skip the rest of the reply
            pass
        self._send_lock = threading.Lock()

    def send(self, text):
        payload = text.encode("utf-8")              # always under 126 bytes here
        mask = os.urandom(4)                        # clients must mask what they send
        masked = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        with self._send_lock:
            self.sock.sendall(bytes([0x81, 0x80 | len(payload)]) + mask + masked)

    def recv(self):
        """Next message as (is_text, payload), or None when the rover closed."""
        head = self._read(2)
        opcode, n = head[0] & 0x0F, head[1] & 0x7F
        if n == 126:
            n = struct.unpack("!H", self._read(2))[0]
        elif n == 127:
            n = struct.unpack("!Q", self._read(8))[0]
        payload = self._read(n)
        if opcode == 0x8:
            return None
        return opcode == 0x1, payload

    def _read(self, n):
        data = self._reader.read(n)
        if len(data) < n:
            raise OSError("connection closed")
        return data

    def close(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._reader.close()
        self.sock.close()


class Channel:
    """One connection, kept open for the length of the test, reopened if it drops."""

    def __init__(self, host, port, path, on_message, on_open=None):
        self.args = (host, port, path)
        self.on_message = on_message
        self.on_open = on_open
        self.client = None
        self.arrivals = []           # time.monotonic() of every message counted
        self.connects = 0
        self.error = None
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        # Callbacks refer to both channels; create both before starting either.
        self.thread.start()

    def stop(self):
        self.running = False
        client = self.client
        if client:
            client.close()
        self.thread.join(timeout=SOCKET_TIMEOUT_S + 1.0)

    def _run(self):
        while self.running:
            try:
                self.client = WebSocketClient(*self.args)
                self.connects += 1
                if self.on_open:
                    self.on_open(self.client)
                while self.running:
                    message = self.client.recv()
                    if message is None:
                        break
                    if self.running:
                        self.on_message(time.monotonic(), *message)
            except (OSError, ValueError) as e:      # includes a 3 s silence
                self.error = str(e) or type(e).__name__
            if self.client:
                self.client.close()
            self.client = None
            time.sleep(0.5)

    def gaps_ms(self, since=0.0, until=None):
        times = [t for t in self.arrivals if t >= since]
        if until is not None:
            times = [t for t in times if t <= until]
            if times:
                times.append(until)  # include a dropout that never recovers
        return [1000 * (b - a) for a, b in zip(times, times[1:])]


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * fraction))] if ordered else None


def fmt(value, digits=0):
    return "--" if value is None else f"{value:.{digits}f}"


def main():
    ap = argparse.ArgumentParser(description="Measure the station-rover link.")
    ap.add_argument("rover", help="rover address, e.g. 192.168.0.200")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--label", default="", help='where the rover is, e.g. "30 m, clear view"')
    ap.add_argument("--no-csv", action="store_true", help="do not add a row to data/link_tests.csv")
    args = ap.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        ap.error("--seconds must be a positive finite number")

    lock = threading.Lock()
    pings = {}                       # id -> time sent
    rtts = []                        # (time received, ms)
    wifi, rover_fps = [], []
    video_bytes = [0]
    keyframes = [0]

    def on_control(now, is_text, payload):
        msg = json.loads(payload)
        if msg.get("type") == "telemetry":
            with lock:
                control.arrivals.append(now)
                if msg.get("wifi_dbm") is not None:
                    wifi.append(msg["wifi_dbm"])
                if msg.get("camera"):
                    rover_fps.append(msg["camera"]["fps"])
        elif msg.get("type") == "pong":
            with lock:
                sent = pings.pop(msg.get("id"), None)
                if sent is not None:
                    rtts.append((now, 1000 * (now - sent)))

    def on_video(now, is_text, payload):
        if is_text or not payload:
            raise ValueError("expected a video frame with a keyframe flag")
        with lock:
            video.arrivals.append(now)
            video_bytes[0] += len(payload) - 1          # first byte is the keyframe flag
            keyframes[0] += payload[0]

    host = args.rover.split(":")[0]
    control = Channel(host, args.port, "/control", on_control)
    video = Channel(host, args.port, "/video", on_video)

    print(f"Testing the link to {host}:{args.port} for {args.seconds:.0f} s"
          + (f"  [{args.label}]" if args.label else ""))
    print("   s   wifi dBm   telemetry Hz   ping ms   video fps   Mbit/s")

    t0 = time.monotonic()
    control.start()
    video.start()
    ping_id = sent_pings = 0
    next_ping, next_line = t0, t0 + 1.0
    seen_bytes = 0
    while time.monotonic() - t0 < args.seconds:
        now = time.monotonic()
        if now >= next_ping:
            next_ping += 0.2
            client = control.client
            if client:
                ping_id += 1
                with lock:
                    pings[ping_id] = time.monotonic()
                try:
                    client.send(json.dumps({"type": "ping", "id": ping_id}))
                    sent_pings += 1
                except OSError:
                    with lock:
                        pings.pop(ping_id, None)
        if now >= next_line:
            with lock:
                tlm = sum(1 for t in control.arrivals if t >= next_line - 1.0)
                frames = sum(1 for t in video.arrivals if t >= next_line - 1.0)
                recent = [ms for t, ms in rtts if t >= next_line - 1.0]
                mbit = (video_bytes[0] - seen_bytes) * 8 / 1e6
                seen_bytes = video_bytes[0]
                dbm = wifi[-1] if wifi else None
            print(f"{next_line - t0:4.0f}   {fmt(dbm):>8}   {tlm:12d}   "
                  f"{fmt(max(recent) if recent else None):>7}   {frames:9d}   {mbit:6.1f}")
            next_line += 1.0
        time.sleep(0.005)

    control.running = video.running = False
    t_end = time.monotonic()
    duration = t_end - t0
    control.stop()
    video.stop()
    with lock:
        tlm_gaps = control.gaps_ms(until=t_end)
        video_gaps = video.gaps_ms(until=t_end)
        overdue_pings = sum(t_end - sent > DEADMAN_S for sent in pings.values())
        rtt_values = [ms for _, ms in rtts]
        row = {
            "time": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "label": args.label,
            "rover": host,
            "seconds": round(duration, 1),
            "wifi_dbm_median": statistics.median(wifi) if wifi else None,
            "wifi_dbm_min": min(wifi) if wifi else None,
            "telemetry_hz": round(len(control.arrivals) / duration, 1),
            "telemetry_gap_p99_ms": percentile(tlm_gaps, 0.99),
            "telemetry_gap_max_ms": max(tlm_gaps) if tlm_gaps else None,
            "ping_median_ms": statistics.median(rtt_values) if rtt_values else None,
            "ping_p99_ms": percentile(rtt_values, 0.99),
            "ping_max_ms": max(rtt_values) if rtt_values else None,
            "pings_lost": sent_pings - len(rtt_values),
            "video_fps": round(len(video.arrivals) / duration, 1),
            "video_mbps": round(video_bytes[0] * 8 / duration / 1e6, 2),
            "video_gap_p99_ms": percentile(video_gaps, 0.99),
            "video_gap_max_ms": max(video_gaps) if video_gaps else None,
            "rover_camera_fps": statistics.median(rover_fps) if rover_fps else None,
            # A video channel that never carried a frame (camera off) times
            # out and reopens by design; that is not a lost connection.
            "reconnects": (max(0, control.connects - 1)
                           + (max(0, video.connects - 1) if video.arrivals else 0)),
        }
    for key, value in row.items():
        if isinstance(value, float):
            row[key] = round(value, 1)

    if not control.arrivals:
        print(f"\nNo telemetry from {host}:{args.port}"
              f" ({control.error or 'no answer'}). Is rover_main.py running there?")

    print(f"""
Summary{f' [{args.label}]' if args.label else ''}
  wifi at the rover  median {fmt(row['wifi_dbm_median'])} dBm, weakest {fmt(row['wifi_dbm_min'])} dBm
  telemetry          {row['telemetry_hz']} per second, gap 99% {fmt(row['telemetry_gap_p99_ms'])} ms, longest {fmt(row['telemetry_gap_max_ms'])} ms
  ping               median {fmt(row['ping_median_ms'], 1)} ms, 99% {fmt(row['ping_p99_ms'], 1)} ms, longest {fmt(row['ping_max_ms'], 1)} ms, {row['pings_lost']} of {sent_pings} unanswered
  video              {row['video_fps']} frames/s, {row['video_mbps']} Mbit/s, gap 99% {fmt(row['video_gap_p99_ms'])} ms, longest {fmt(row['video_gap_max_ms'])} ms, {keyframes[0]} keyframes
  camera at rover    {fmt(row['rover_camera_fps'], 1)} frames/s produced
  reconnects         {row['reconnects']}""")

    worst = max(row["telemetry_gap_max_ms"] or 0, row["ping_max_ms"] or 0,
                row["video_gap_max_ms"] or 0)
    if worst > 1000 * DEADMAN_S:
        print(f"\n  A gap or round trip of {worst:.0f} ms exceeds the "
              f"{1000 * DEADMAN_S:.0f} ms screening threshold.")
    if row["reconnects"]:
        print(f"\n  A connection went silent for {SOCKET_TIMEOUT_S:.0f} s or closed, and was "
              f"reopened, {row['reconnects']} time(s).")
    if not video.arrivals:
        print("\n  No video frames arrived (camera off on the rover, or --no-camera).")
    if overdue_pings:
        print(f"\n  {overdue_pings} ping(s) were still unanswered after {DEADMAN_S:.1f} s.")

    complete = len(control.arrivals) >= 2 and len(video.arrivals) >= 2 and bool(rtts)
    passed = complete and worst <= 1000 * DEADMAN_S and not row["reconnects"] and not overdue_pings
    result = "PASS" if passed else ("FAIL" if complete else "INCOMPLETE")
    print(f"\n  Link screen: {result}. This does not verify drive-command timing or video latency.")

    if not args.no_csv:
        os.makedirs(os.path.dirname(CSV_FILE), exist_ok=True)
        new_file = not os.path.exists(CSV_FILE)
        with open(CSV_FILE, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(row))
            if new_file:
                writer.writeheader()
            writer.writerow(row)
        print(f"\nRow added to {os.path.normpath(CSV_FILE)}")
    return 0 if passed else (2 if complete else 1)


if __name__ == "__main__":
    raise SystemExit(main())
