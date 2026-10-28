"""One spawned BMI088 owner, with ordered request/response acquisition.

Only the child opens SPI. Parent-side scheduling cannot lengthen a child's
SPI-call brackets. It can still delay the next request or delivery, so the
existing FIFO, acquisition-gap and freshness checks remain necessary.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import multiprocessing
from pathlib import Path
import pickle
import queue
import threading
import time

import config
import imu_reader


class IMUProcessCleanupError(imu_reader.IMUError):
    """A previous owner is not confirmed stopped; do not spawn another."""


_owners = set()
_owners_lock = threading.RLock()
_source = Path(__file__).read_bytes()
_source_identity = dict(sha256=hashlib.sha256(_source).hexdigest(), text=_source.decode('utf-8'))
_source_captured_utc = datetime.now(timezone.utc).isoformat()


def _send(connection, reply):
    payload = pickle.dumps(reply, protocol=pickle.HIGHEST_PROTOCOL)
    if len(payload) > config.IMU_PROCESS_MAX_REPLY_BYTES:
        raise imu_reader.IMUError('IMU process reply exceeds bounded payload size')
    connection.send_bytes(payload)


def _factory():
    return imu_reader.create_imu('bmi088')


def _stats(reader):
    try:
        return reader.stats() if hasattr(reader, 'stats') else {}
    except Exception as exc:
        return {'stats_error': f'{type(exc).__name__}: {exc}'}


def _worker(connection, stop, factory):
    reader = None
    sequence = 0
    try:
        reader = factory()
        metadata = deepcopy(reader.metadata()) if hasattr(reader, 'metadata') else {'sensor': 'BMI088'}
        metadata.setdefault('sources', {})['imu_process.py'] = deepcopy(_source_identity)
        metadata['acquisition_process_source_captured_utc'] = _source_captured_utc
        _send(connection, dict(protocol=1, sequence=0, kind='ready',
                               metadata=metadata, stats=_stats(reader)))
        while not stop.is_set():
            if not connection.poll(.05):
                continue
            request = connection.recv()
            if (not isinstance(request, dict) or request.get('kind') != 'read'
                    or type(request.get('sequence')) is not int
                    or request['sequence'] != sequence + 1):
                raise imu_reader.IMUError('Invalid IMU process request sequence')
            sequence = request['sequence']
            try:
                sample = reader.read()
            except imu_reader.IMUSampleError as exc:
                # Preserve exact reason and the same read's fresh diagnostic.
                _send(connection, dict(protocol=1, sequence=sequence, kind='sample_error',
                                       error=str(exc), stats=_stats(reader)))
            else:
                _send(connection, dict(protocol=1, sequence=sequence, kind='sample',
                                       sample=sample, stats=_stats(reader)))
    except (EOFError, BrokenPipeError):
        pass
    except BaseException as exc:
        try:
            _send(connection, dict(protocol=1, sequence=sequence, kind='fatal_error',
                                   error=f'{type(exc).__name__}: {exc}',
                                   stats=_stats(reader) if reader is not None else {}))
        except (OSError, EOFError, imu_reader.IMUError):
            pass
    finally:
        if reader is not None:
            try:
                reader.close()
            except Exception:
                pass
        connection.close()


class BMI088ProcessReader:
    def __init__(self, *, factory=None, stop_event=None, start_timeout=None,
                 read_timeout=None, close_timeout=None):
        self._external_stop = stop_event
        self._start_timeout = config.IMU_PROCESS_START_TIMEOUT_S if start_timeout is None else start_timeout
        self._read_timeout = config.IMU_PROCESS_READ_TIMEOUT_S if read_timeout is None else read_timeout
        self._close_timeout = config.IMU_PROCESS_CLOSE_TIMEOUT_S if close_timeout is None else close_timeout
        if min(self._start_timeout, self._read_timeout, self._close_timeout) <= 0:
            raise ValueError('IMU process deadlines must be positive')
        self._closed = False
        self._cleanup_complete = False
        self._sequence = 0
        self._stats = {}
        self._metadata = None
        self._responses = queue.Queue(maxsize=2)
        self._receive_error = None
        self._close_lock = threading.RLock()
        context = multiprocessing.get_context('spawn')
        self._stop = context.Event()
        self._connection, child_connection = context.Pipe(duplex=True)
        self._process = context.Process(target=_worker,
                                        args=(child_connection, self._stop, factory or _factory),
                                        name='bmi088-spi', daemon=True)
        self._receiver = None
        try:
            with _owners_lock:
                previous_owners = list(_owners)
            for previous in previous_owners:
                if not previous._closed:
                    raise IMUProcessCleanupError('Another BMI088 process owner is still active')
                previous.close()  # Raises while an old owner remains unreaped.
            with _owners_lock:
                if _owners:
                    raise IMUProcessCleanupError('Another BMI088 process owner is still active')
                _owners.add(self)
        except BaseException:
            child_connection.close()
            self._connection.close()
            raise
        try:
            self._process.start()
            # Close this unused endpoint, so child exit always exposes EOF.
            child_connection.close()
            # Connection.recv can wait for a partial frame after poll(). Keep
            # that blocking operation outside the deadline-enforcing caller.
            self._receiver = threading.Thread(target=self._receive_loop,
                                              name='imu-process-ipc', daemon=True)
            self._receiver.start()
            reply = self._receive(0, self._start_timeout)
            if reply['kind'] != 'ready' or not isinstance(reply.get('metadata'), dict):
                self._fatal(reply.get('error') or 'Invalid IMU process startup reply')
            self._metadata = deepcopy(reply['metadata'])
            self._metadata['acquisition_process'] = dict(
                pid=self._process.pid, start_method='spawn', transport='one read request and ordered response',
                parent_proxy_sha256=_source_identity['sha256'],
                parent_source_captured_utc=_source_captured_utc,
                timestamps='original child acquisition monotonic timestamps; no delivery restamping',
                buffering='one outstanding request; no autonomous sample queue or silent dropping')
            if self._metadata['sources']['imu_process.py']['sha256'] != _source_identity['sha256']:
                self._metadata['acquisition_process']['parent_proxy_source'] = deepcopy(_source_identity)
        except BaseException:
            child_connection.close()
            self.close()
            raise

    def _receive_loop(self):
        try:
            while True:
                payload = pickle.loads(self._connection.recv_bytes(config.IMU_PROCESS_MAX_REPLY_BYTES))
                try:
                    self._responses.put_nowait(payload)
                except queue.Full:
                    raise imu_reader.IMUError('Unexpected extra IMU process response')
        except BaseException as exc:
            self._receive_error = f'{type(exc).__name__}: {exc}'

    def _receive(self, sequence, timeout):
        deadline = time.monotonic() + timeout
        while True:
            if self._closed or (self._external_stop is not None and self._external_stop.is_set()):
                self._fatal('IMU process acquisition cancelled')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._fatal('IMU process response timeout')
            try:
                reply = self._responses.get(timeout=min(.02, remaining))
            except queue.Empty:
                if self._receive_error:
                    self._fatal('IMU process IPC ended: ' + self._receive_error)
                continue
            if (not isinstance(reply, dict) or reply.get('protocol') != 1
                    or type(reply.get('sequence')) is not int or reply['sequence'] != sequence
                    or reply.get('kind') not in ('ready', 'sample', 'sample_error', 'fatal_error')
                    or not isinstance(reply.get('stats'), dict)):
                self._fatal('Invalid IMU process response/sequence')
            self._stats = deepcopy(reply['stats'])
            return reply

    def _fatal(self, reason):
        self.close()
        raise imu_reader.IMUError(reason)

    def read(self):
        if self._closed:
            raise imu_reader.IMUError('BMI088 acquisition process is closed')
        self._sequence += 1
        try:
            self._connection.send(dict(kind='read', sequence=self._sequence))
        except (OSError, EOFError) as exc:
            self._fatal(f'IMU process request failed: {exc}')
        reply = self._receive(self._sequence, self._read_timeout)
        if reply['kind'] == 'sample':
            if reply.get('sample') is not None and not isinstance(reply['sample'], dict):
                self._fatal('Invalid IMU process sample payload')
            return reply['sample']
        if reply['kind'] == 'sample_error' and isinstance(reply.get('error'), str):
            raise imu_reader.IMUSampleError(reply['error'])
        self._fatal(reply.get('error') or 'Unexpected IMU process read reply')

    def metadata(self):
        return deepcopy(self._metadata)

    def stats(self):
        return deepcopy(self._stats)

    def close(self):
        with self._close_lock:
            if self._cleanup_complete:
                return
            self._closed = True
            self._stop.set()
            if self._process.pid is not None:
                self._process.join(self._close_timeout)
                if self._process.is_alive():
                    self._process.terminate()
                    self._process.join(self._close_timeout)
                if self._process.is_alive():
                    self._process.kill()
                    self._process.join(self._close_timeout)
                if self._process.is_alive():
                    raise IMUProcessCleanupError(
                        f'Previous BMI088 owner PID {self._process.pid} has not stopped; retry blocked')
            self._connection.close()
            if self._receiver is not None and self._receiver is not threading.current_thread():
                self._receiver.join(self._close_timeout)
                if self._receiver.is_alive():
                    raise IMUProcessCleanupError('Previous BMI088 IPC receiver has not stopped; retry blocked')
            self._cleanup_complete = True
            with _owners_lock:
                _owners.discard(self)
