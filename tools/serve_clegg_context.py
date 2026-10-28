"""Read-only Clegg clock/pose endpoint; tails the running Rover session log.

No actuator or control socket is opened. Records are exposed only after their
source file grows during this server's lifetime, preventing reuse of old runs.
"""
import argparse
import collections
import json
import math
from pathlib import Path
import socketserver
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'rover'))
import config


def finite_number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def finite_vector(value):
    return isinstance(value, list) and len(value) == 3 and all(finite_number(v) for v in value)


class PoseTail:
    def __init__(self, root, start=True):
        self.root = Path(root)
        self.positions = {}
        self.buffers = {}
        self.history = collections.deque(maxlen=config.CLEGG_CONTEXT_HISTORY)
        self.lock = threading.Lock()
        self.error = None
        self.last_update = None
        self.seen = set()
        self.stop_event = threading.Event()
        self.worker = None
        # Existing bytes are old evidence; new appends establish liveness.
        for path in self.root.glob('*/rover/telemetry_*.jsonl'):
            self.positions[path] = path.stat().st_size
        if start:
            self.worker = threading.Thread(target=self._run, daemon=True)
            self.worker.start()

    def stop(self):
        self.stop_event.set()
        if self.worker:
            self.worker.join(timeout=2)

    def _run(self):
        while not self.stop_event.is_set():
            try:
                self.poll_once()
                self.error = None
            except (OSError, ValueError, TypeError) as exc:
                self.error = str(exc)
            self.stop_event.wait(config.CLEGG_CONTEXT_POLL_S)

    def poll_once(self):
        paths = sorted(self.root.glob('*/rover/telemetry_*.jsonl'), key=lambda p: p.stat().st_mtime_ns, reverse=True)[:4]
        for path in paths:
            offset = self.positions.get(path, 0)
            with path.open('rb') as stream:
                stream.seek(offset)
                chunk = stream.read(1024 * 1024)
                self.positions[path] = stream.tell()
            if not chunk:
                continue
            lines = (self.buffers.get(path, b'') + chunk).split(b'\n')
            self.buffers[path] = lines.pop()
            if len(self.buffers[path]) > 1024 * 1024:
                raise ValueError('Oversized session log record')
            for line in lines:
                try:
                    row = json.loads(line)
                except (ValueError, UnicodeError):
                    continue
                if not isinstance(row, dict) or row.get('kind') != 'telemetry':
                    continue
                data = row.get('data', {})
                if not isinstance(data, dict):
                    continue
                mono = row.get('captured_monotonic_s')
                age = time.monotonic() - mono if finite_number(mono) else -1
                if not 0 <= age <= config.CLEGG_CONTEXT_LOG_MAX_AGE_S:
                    continue
                seq = row.get('seq')
                if isinstance(seq, bool) or not isinstance(seq, int):
                    continue
                key = (str(path), seq)
                if key in self.seen:
                    continue
                self.seen.add(key)
                if len(self.seen) > 2048:
                    self.seen = {key}
                sample = {k: data.get(k) for k in ('run_id', 'pose_frame_id', 'pose_valid', 'pose_quality', 'pos', 'rpy')}
                # Keep invalid poses in the history so association cannot
                # bridge across them. Never emit JSON NaN/Infinity.
                sample['pose_valid'] = sample['pose_valid'] is True
                for name in ('run_id', 'pose_frame_id', 'pose_quality'):
                    if not isinstance(sample[name], str):
                        sample[name] = None
                if not finite_vector(sample['pos']):
                    sample['pos'], sample['pose_valid'] = None, False
                if not finite_vector(sample['rpy']):
                    sample['rpy'] = None
                imu = data.get('imu') if isinstance(data.get('imu'), dict) else {}
                imu_age, generation = imu.get('age_s'), imu.get('generation')
                if isinstance(generation, bool) or not isinstance(generation, int) or generation < 0:
                    generation = None
                sample.update(captured_monotonic_s=mono, source_log=str(path), source_seq=seq,
                              imu_generation=generation,
                              imu_age_s=imu_age if finite_number(imu_age) else None,
                              timestamp_source='session_recorder_capture_not_sensor_acquisition')
                with self.lock:
                    self.history.append(sample)
                    self.last_update = time.monotonic()

    def snapshot(self):
        with self.lock:
            return list(self.history), self.last_update, self.error


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--logs', required=True, help='Rover logs/sessions directory')
    ap.add_argument('--bind', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=config.CLEGG_CONTEXT_PORT)
    args = ap.parse_args()
    if not Path(args.logs).is_dir():
        ap.error('--logs must be the existing Rover logs/sessions directory')
    poses = PoseTail(args.logs)
    boot_id = uuid.uuid4().hex

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.request.settimeout(3)
            raw = self.rfile.readline(1025)
            rx = time.monotonic()
            if len(raw) > 1024 or not raw.endswith(b'\n'):
                return
            try:
                request = json.loads(raw)
            except (ValueError, UnicodeError):
                return
            if not isinstance(request, dict) or request.get('type') != 'context':
                return
            history, updated, error = poses.snapshot()
            response = dict(type='clegg_context', boot_id=boot_id, nonce=request.get('nonce'),
                            rx_s=rx, tx_s=time.monotonic(), history=history,
                            last_update_s=updated, error=error, read_only=True)
            raw_response = json.dumps(response, allow_nan=False).encode() + b'\n'
            while len(raw_response) > config.CLEGG_CONTEXT_RESPONSE_MAX_BYTES and response['history']:
                response['history'] = response['history'][len(response['history'])//2+1:]
                raw_response = json.dumps(response, allow_nan=False).encode() + b'\n'
            self.wfile.write(raw_response)

    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True
    with Server((args.bind, args.port), Handler) as server:
        print(f'Clegg read-only context: {args.bind}:{args.port}', flush=True)
        try:
            server.serve_forever()
        finally:
            poses.stop()


if __name__ == '__main__':
    main()
