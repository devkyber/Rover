"""Jetson camera recording, independent of browser/IMU/log sessions.

The camera only enqueues compressed access units. One worker muxes H.264 to
MPEG-TS (no encoder, no end-of-file index), writes and fsyncs. Complete TS
packets already saved remain usable after an interrupted process. Missing
frames are disclosed and the next file starts at an IDR, never at a delta.
"""
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
import threading
import time
import uuid

import config


def _utc():
    return datetime.now(timezone.utc).isoformat()


def _sync(file):
    file.flush()
    os.fsync(file.fileno())


def _sync_directory(path):
    if os.name == 'posix':
        fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


def _timestamp_ns(value):
    if isinstance(value, (int, float)) and math.isfinite(value) and value >= 0:
        return int(value)
    return None


def _source_stamp(frame):
    dts = _timestamp_ns(frame['dts_ns'])
    return dts if dts is not None else _timestamp_ns(frame['pts_ns'])


class TransportStream:
    """Bounded GStreamer remuxer; only the recorder worker calls this class."""
    def __init__(self, path):
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst
        self.Gst = Gst
        Gst.init(None)
        self.pipeline = None
        self.file = open(path, 'xb')
        self.bytes = 0
        self.eos = False
        try:
            self.pipeline = Gst.parse_launch(
                'appsrc name=source format=time is-live=true block=false ! '
                'h264parse config-interval=-1 ! '
                'video/x-h264,stream-format=byte-stream,alignment=au ! '
                'mpegtsmux alignment=7 ! '
                f'appsink name=output sync=false max-buffers={config.VIDEO_MUX_OUTPUT_BUFFERS} '
                'drop=false wait-on-eos=false')
            self.source = self.pipeline.get_by_name('source')
            self.source.set_property('caps', Gst.Caps.from_string(
                'video/x-h264,stream-format=byte-stream,alignment=au,'
                f'framerate={config.CAMERA_FPS}/1'))
            self.sink = self.pipeline.get_by_name('output')
            self.bus = self.pipeline.get_bus()
            if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError('Cannot start video muxer')
        except Exception:
            self.abort()
            raise

    def _error(self):
        while True:
            # pop_filtered discards other message types, including EOS. Read
            # both here so an idle drain cannot swallow completion.
            message = self.bus.pop_filtered(self.Gst.MessageType.ERROR | self.Gst.MessageType.EOS)
            if not message:
                return
            if message.type == self.Gst.MessageType.EOS:
                self.eos = True
            else:
                error, debug = message.parse_error()
                raise RuntimeError(f'Video muxer: {error.message}; {debug}')

    def drain(self):
        self._error()
        while True:
            sample = self.sink.emit('try-pull-sample', 0)
            if sample is None:
                break
            buffer = sample.get_buffer()
            data = buffer.extract_dup(0, buffer.get_size())
            self.file.write(data)
            self.bytes += len(data)
        return self.bytes

    def push(self, data, pts_ns, dts_ns, key):
        # appsrc's default nonblocking queue is otherwise unbounded. Never
        # let a broken muxer consume arbitrary memory or hang shutdown.
        deadline = time.monotonic() + config.VIDEO_MUX_TIMEOUT_S
        while self.source.get_property('current-level-bytes') > config.VIDEO_MUX_QUEUE_BYTES:
            self.drain()
            if time.monotonic() > deadline:
                raise TimeoutError('Video muxer stopped consuming frames')
            time.sleep(config.VIDEO_WORKER_POLL_S)
        buffer = self.Gst.Buffer.new_allocate(None, len(data), None)
        buffer.fill(0, data)
        buffer.pts, buffer.dts = pts_ns, dts_ns
        buffer.duration = self.Gst.SECOND // config.CAMERA_FPS
        if not key:
            buffer.set_flags(self.Gst.BufferFlags.DELTA_UNIT)
        if self.source.emit('push-buffer', buffer) != self.Gst.FlowReturn.OK:
            raise RuntimeError('Video muxer rejected a frame')
        self.drain()

    def sync(self):
        self.drain()
        _sync(self.file)
        return self.bytes

    def finish(self):
        try:
            self.source.emit('end-of-stream')
            deadline = time.monotonic() + config.VIDEO_MUX_TIMEOUT_S
            while True:
                self.drain()
                if self.eos:
                    self.drain()
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError('Video muxer did not finish')
                time.sleep(config.VIDEO_WORKER_POLL_S)
            _sync(self.file)
            return self.bytes
        finally:
            self.abort()

    def abort(self):
        try:
            if self.pipeline is not None:
                self.pipeline.set_state(self.Gst.State.NULL)
        finally:
            if not self.file.closed:
                try:
                    _sync(self.file)
                finally:
                    self.file.close()


class VideoRecorder:
    @staticmethod
    def _counters():
        return dict(queue_frames=0, queue_bytes=0, queue_highwater_frames=0,
                    queue_highwater_bytes=0, max_writer_lag_s=0., last_writer_lag_s=0.,
                    timing_anomalies=0, retimed_frames=0, dependency_losses=0,
                    camera_waits=0, abandoned_frames=0)

    def __init__(self, log_root, *, enabled=True, mux_factory=TransportStream):
        self.root = Path(log_root).absolute() / config.VIDEO_LOG_SUBDIR
        self.enabled = enabled
        self._mux_factory = mux_factory
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._queue = queue.Queue(maxsize=config.VIDEO_QUEUE_FRAMES)
        self._state = dict(status='idle' if enabled else 'unavailable', requested=False,
                           directory=None, file=None, frames=0, bytes=0, synced_bytes=0,
                           segments=0, dropped=0, skipped=0, gaps=0, error=None,
                           close_complete=True, elapsed_s=0, **self._counters())

    def _set(self, **fields):
        with self._lock:
            self._state.update(fields)

    def state(self):
        with self._lock:
            return dict(self._state, available=self.enabled)

    def start(self, context=None):
        with self._lock:
            if not self.enabled or (self._thread and self._thread.is_alive()):
                return False
            name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '_' + uuid.uuid4().hex[:8]
            self._queue = queue.Queue(maxsize=config.VIDEO_QUEUE_FRAMES)
            self._queued_bytes = self._seq = 0
            self._stop.clear()
            self._state = dict(status='starting', requested=True, directory=str(self.root / name),
                               file=None, frames=0, bytes=0, synced_bytes=0, segments=0,
                               dropped=0, skipped=0, gaps=0, error=None, close_complete=False,
                               elapsed_s=0, **self._counters())
            self._thread = threading.Thread(target=self._run, args=(dict(context or {}),),
                                            name='video-recorder', daemon=True)
            try:
                self._thread.start()
            except Exception as exc:
                self._thread = None
                self._state.update(status='error', requested=False, close_complete=True,
                                   error=f'Video worker start failed: {exc}')
                return False
        return True

    def stop(self):
        with self._lock:
            if self._state['requested']:
                self._state.update(requested=False, status='saving')
                self._stop.set()

    def close(self):
        self.stop()
        if self._thread:
            self._thread.join(config.VIDEO_CLOSE_TIMEOUT_S)
            if self._thread.is_alive():
                self._set(status='error', error='Video close timed out; saved files retained',
                          close_complete=False)

    def submit(self, data, key, generation, pts_ns=None, dts_ns=None, discontinuity=False,
               source_seq=None):
        """No I/O or waiting. source_seq is an optional verified source counter."""
        with self._lock:
            if not self._state['requested']:
                return False
            self._seq += 1
            if (len(data) > config.VIDEO_MAX_FRAME_BYTES
                    or self._queued_bytes + len(data) > config.VIDEO_QUEUE_BYTES):
                self._state['dropped'] += 1
                return False
            frame = dict(seq=self._seq, data=data, key=bool(key), generation=generation,
                         pts_ns=pts_ns, dts_ns=dts_ns, discontinuity=discontinuity,
                         source_seq=source_seq if type(source_seq) is int and source_seq >= 0 else None,
                         mono=time.monotonic(), wall_ns=time.time_ns())
            try:
                self._queue.put_nowait(frame)
                self._queued_bytes += len(data)
                depth = self._queue.qsize()
                self._state.update(queue_frames=depth, queue_bytes=self._queued_bytes,
                    queue_highwater_frames=max(depth, self._state['queue_highwater_frames']),
                    queue_highwater_bytes=max(self._queued_bytes, self._state['queue_highwater_bytes']))
                return True
            except queue.Full:
                self._state['dropped'] += 1
                return False

    def _run(self, context):
        directory = Path(self.state()['directory'])
        mux = journal = None
        segment = previous = None
        total_bytes = 0
        next_sync = 0.0
        started = time.monotonic()
        fatal = None
        gap_pending = waiting = False
        active_frame = None

        def event(kind, **fields):
            journal.write(json.dumps(dict(kind=kind, host_utc=_utc(),
                                          host_monotonic_s=time.monotonic(), **fields)) + '\n')

        def space_check():
            if shutil.disk_usage(directory).free < config.VIDEO_MIN_FREE_BYTES:
                raise OSError('Low disk space; video stopped, existing files retained')

        def sync():
            nonlocal next_sync
            space_check()
            size = mux.sync() if mux else 0
            _sync(journal)
            _sync_directory(directory)
            self._set(bytes=total_bytes + size, synced_bytes=total_bytes + size)
            next_sync = time.monotonic() + config.VIDEO_SYNC_S

        def finish_segment(reason):
            nonlocal mux, segment, total_bytes
            if mux is None:
                return
            size = mux.finish()
            mux = None
            total_bytes += size
            event('segment_closed', file=segment['file'], reason=reason, bytes=size)
            self._set(bytes=total_bytes, synced_bytes=total_bytes)
            segment = None

        try:
            directory.mkdir(parents=True, exist_ok=False)
            space_check()
            source_root = Path(__file__).resolve().parent
            sources = {}
            for name in ('config.py', 'camera.py', 'video_recording.py', 'rover_main.py'):
                raw = (source_root / name).read_bytes()
                sources[name] = dict(sha256=hashlib.sha256(raw).hexdigest(), text=raw.decode('utf-8'))
            with open(directory / 'metadata.json', 'x', encoding='utf-8') as file:
                json.dump(dict(schema='rover_video_v1', context=context, sources=sources,
                               created_utc=_utc(), format='MPEG-TS / H.264, no audio or dashboard overlay',
                               timing='Camera pipeline PTS/DTS; nominal-cadence output fallback. Host UTC unverified. '
                                      'Source timestamps and receipt monotonic/wall time are retained in the journal. '
                                      'Nonadvancing/missing/large-jump timestamps use nominal output cadence. '
                                      'writer_lag_s measures receipt to mux submission, not durable completion. '
                                      'Journal frames are submitted to muxer, not proof of sensor completeness.',
                               loss_policy='Recorder sequence holes, explicit DISCONT, generation changes and '
                                           'optional verified source sequence holes require a new IDR. '
                                           'Timing anomalies alone do not prove missing H.264 dependencies. '
                                           'gaps counts interrupted chains; dependency_losses counts every observed loss boundary.'), file)
                _sync(file)
            journal = open(directory / 'frames.jsonl', 'x', encoding='utf-8')
            _sync_directory(directory)
            _sync_directory(directory.parent)
            self._set(status='waiting')
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    frame = self._queue.get(timeout=config.VIDEO_WORKER_POLL_S)
                    with self._lock:
                        self._queued_bytes -= len(frame['data'])
                        self._state.update(queue_frames=self._queue.qsize(), queue_bytes=self._queued_bytes)
                except queue.Empty:
                    frame = None
                if frame:
                    active_frame = frame
                    frame['utc'] = datetime.fromtimestamp(frame['wall_ns'] / 1e9, timezone.utc).isoformat()
                    waiting = False
                    stamp = _source_stamp(frame)
                    prior_stamp = _source_stamp(previous) if previous else None
                    nominal = round(1e9 / config.CAMERA_FPS)
                    delta = stamp - prior_stamp if stamp is not None and prior_stamp is not None else None
                    reasons, timing = [], []
                    if previous:
                        if frame['seq'] != previous['seq'] + 1: reasons.append('recorder_sequence')
                        if frame['generation'] != previous['generation']: reasons.append('generation')
                        if frame['discontinuity']: reasons.append('source_discontinuity')
                        if (frame['source_seq'] is not None and previous['source_seq'] is not None
                                and frame['source_seq'] != previous['source_seq'] + 1):
                            reasons.append('source_sequence')
                        if frame['mono'] - previous['mono'] > config.VIDEO_GAP_S: timing.append('host_delay')
                        if (stamp is None) != (prior_stamp is None): timing.append('timestamp_source_changed')
                        if delta is not None and (delta <= 0 or delta > config.VIDEO_FRAME_GAP_FACTOR * nominal):
                            timing.append('nonadvancing_timestamp' if delta <= 0 else 'timestamp_jump')
                    if timing:
                        event('timing_anomaly', seq=frame['seq'], reasons=timing, source_delta_ns=delta,
                              host_delta_s=frame['mono'] - previous['mono'],
                              possible_upstream_loss='timestamp_jump' in timing,
                              dependency_loss_proven=False)
                        with self._lock:
                            self._state['timing_anomalies'] += 1
                    if reasons:
                        finish_segment('source_or_queue_gap')
                        event('gap_extended' if gap_pending else 'gap', reasons=reasons,
                              previous_seq=previous['seq'], next_seq=frame['seq'],
                              missing_submitted_frames=max(0, frame['seq'] - previous['seq'] - 1),
                              previous_source_seq=previous['source_seq'], source_seq=frame['source_seq'])
                        with self._lock:
                            self._state['dependency_losses'] += 1
                            self._state['gaps'] += int(not gap_pending)
                        gap_pending = True
                    previous = frame
                    if segment and frame['key'] and frame['mono'] - segment['mono'] >= config.VIDEO_SEGMENT_S:
                        finish_segment('rotation')
                    if mux is None and not frame['key']:
                        event('frame_skipped', reason='waiting_for_idr',
                              **{k: v for k, v in frame.items() if k != 'data'})
                        with self._lock:
                            self._state['skipped'] += 1
                    else:
                        if mux is None:
                            space_check()
                            number = self.state()['segments'] + 1
                            filename = f'video_{number:04d}.ts'
                            mux = self._mux_factory(directory / filename)
                            segment = dict(file=filename, mono=frame['mono'],
                                           origin_ns=stamp, last_stamp=None, last_dts_ns=None)
                            event('segment_opened', **segment, generation=frame['generation'])
                            self._set(file=filename, segments=number)
                        source_delta = stamp - segment['last_stamp'] if stamp is not None and segment['last_stamp'] is not None else None
                        retimed = segment['last_dts_ns'] is not None and not (
                            source_delta is not None and 0 < source_delta <= config.VIDEO_FRAME_GAP_FACTOR * nominal)
                        dts = 0 if segment['last_dts_ns'] is None else segment['last_dts_ns'] + (nominal if retimed else source_delta)
                        source_pts, source_dts = _timestamp_ns(frame['pts_ns']), _timestamp_ns(frame['dts_ns'])
                        pts = dts + max(0, source_pts - source_dts) if source_pts is not None and source_dts is not None else dts
                        mux.push(frame['data'], pts, dts, frame['key'])
                        segment.update(last_stamp=stamp, last_dts_ns=dts)
                        lag = max(0., time.monotonic() - frame['mono'])
                        event('frame', **{k: v for k, v in frame.items() if k != 'data'},
                              file=segment['file'], output_pts_ns=pts, output_dts_ns=dts,
                              timestamp_retimed=retimed, writer_lag_s=lag)
                        with self._lock:
                            self._state['frames'] += 1
                            self._state['retimed_frames'] += int(retimed)
                            self._state['last_writer_lag_s'] = lag
                            self._state['max_writer_lag_s'] = max(lag, self._state['max_writer_lag_s'])
                        gap_pending = False
                    active_frame = None
                elif previous and not waiting and time.monotonic() - previous['mono'] > config.VIDEO_GAP_S:
                    event('camera_wait', last_seq=previous['seq'], dependency_loss_proven=False)
                    waiting = True
                    with self._lock:
                        self._state['camera_waits'] += 1
                if time.monotonic() >= next_sync:
                    sync()
                with self._lock:
                    self._state['elapsed_s'] = round(time.monotonic() - started, 1)
                    if self._state['requested']:
                        self._state['status'] = 'recording' if mux and mux.bytes and not waiting else 'waiting'
            finish_segment('operator_or_shutdown')
            event('stopped', **self.state())
            _sync(journal)
        except Exception as exc:
            fatal = f'{type(exc).__name__}: {exc}'
        finally:
            with self._lock:
                self._state['requested'] = False
                abandoned = int(active_frame is not None)
                while True:
                    try:
                        self._queue.get_nowait()
                        abandoned += 1
                    except queue.Empty:
                        break
                self._queued_bytes = 0
                self._state.update(queue_frames=0, queue_bytes=0,
                                   abandoned_frames=self._state['abandoned_frames'] + abandoned)
            if mux:
                try:
                    mux.abort()
                except Exception as exc:
                    fatal = fatal or str(exc)
            if journal:
                try:
                    journal.close()
                except Exception as exc:
                    fatal = fatal or str(exc)
            self._set(requested=False, status='error' if fatal else 'saved', error=fatal,
                      close_complete=not fatal)
            # Last checkpoint is additive. An interrupted process may have no
            # summary; the TS and journal files are still the original evidence.
            try:
                with open(directory / 'summary.json', 'x', encoding='utf-8') as file:
                    json.dump(self.state(), file)
                    _sync(file)
                _sync_directory(directory)
            except Exception as exc:
                self._set(status='error', error=fatal or str(exc), close_complete=False)
