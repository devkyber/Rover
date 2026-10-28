#!/usr/bin/env python3
"""Jetson Orin Nano client, Python 3.6+: Wi-Fi TCP or native USB; no automatic motion on connect."""
import argparse
import csv
import errno
import json
import math
import queue
import secrets
import select
import socket
import struct
import threading
import time
import zlib
from pathlib import Path

HEADER = struct.Struct('<4sHBBI')
RECORD = struct.Struct('<QIBBH20sI')
API_V12 = 'scout-process-v12'
FIRMWARE_V12 = 'scout-process-v12.2-usb-recover-j5'
API, FIRMWARE = API_V12, FIRMWARE_V12
CONTRACTS = {API: FIRMWARE}


def v12_mechanism(mechanism):
    """Match the frozen PCB settings schema; servo positions belong to firmware."""
    limits = {'motor_duty_pct': (1, 100), 'load_timeout_ms': (0, 0),
              'stall_ms': (0, 0), 'hold_tolerance_counts': (0, 0),
              'servo_neutral_us': (0, 0), 'prepare_ms': (0, 0),
              'latch_ms': (0, 0), 'release_ms': (0, 0)}
    keys = set(limits) | {'servo_model', 'servo_mode', 'motor_direction', 'encoder_sign'}
    if not isinstance(mechanism, dict) or set(mechanism) != keys:
        raise ValueError('v12 requires exactly 12 settings; servo pulses and count targets are firmware-owned')
    result = dict(mechanism)
    if not isinstance(result['servo_model'], str) or not result['servo_model']:
        raise ValueError('servo_model must be a nonempty string')
    if result['servo_mode'] != 'position':
        raise ValueError('v12 uses the fixed J5 position servo')
    for key, (low, high) in limits.items():
        if type(result[key]) is not int or not low <= result[key] <= high:
            raise ValueError('Invalid v12 mechanism setting: ' + key)
    for key in ('motor_direction', 'encoder_sign'):
        if type(result[key]) is not int or result[key] not in (-1, 1):
            raise ValueError('Invalid v12 mechanism direction: ' + key)
    return result


class ConnectionCancelled(RuntimeError):
    """Expected cancellation while establishing a client connection."""


class FinalizationError(RuntimeError):
    """Transport or recording workers did not finalize by the close deadline."""


def compatibility_error(api, status):
    """Select a known wire schema; firmware labels and sensor health are display data."""
    if api not in CONTRACTS:
        return 'Unsupported ESP API: expected '+', '.join(CONTRACTS)+', received '+str(api)+'; sensor-only connection'
    return ''


def motion_compatible(client):
    return bool(client and not compatibility_error(client.api,getattr(client,'status',{})))


def launch_supported(client):
    """The optional LAUNCH command requires an explicit current-session hello."""
    return bool(client and getattr(client, 'running', False) and client.api == API_V12 and
                getattr(client, 'hello', {}).get('launch_supported') is True)


class FrameDecoder:
    def __init__(self, hello_nonce=None, require_hello=False):
        self.hello_nonce=hello_nonce
        self.tracking=not require_hello
        self.pre_handshake_crc_errors=0
        self.buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0
        self.frame_gaps = 0
        self.sequence_errors = 0
        self.previous = None
        self.api = None

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            at = self.buffer.find(b'JSF1')
            if at < 0:
                keep = min(3, len(self.buffer))
                self.discarded_bytes += len(self.buffer) - keep
                if len(self.buffer) > keep:
                    del self.buffer[:len(self.buffer)-keep]
                break
            if at:
                self.discarded_bytes += at
                del self.buffer[:at]
            if len(self.buffer) < HEADER.size:
                break
            magic, size, kind, version, seq = HEADER.unpack_from(self.buffer)
            limit = 1536
            if size > limit or version != 1 or kind not in (1, 2, 3):
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            total = HEADER.size + size + 4
            if len(self.buffer) < total:
                break
            raw = bytes(self.buffer[:total])
            if zlib.crc32(raw[:-4]) != struct.unpack_from('<I', raw, total-4)[0]:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:total]
            payload = raw[HEADER.size:-4]
            # A hello is the start of a new connection epoch, whose frame sequence resets.
            is_hello = kind == 2 and b'"event":"hello"' in payload
            if is_hello and self.hello_nonce is not None:
                try: is_hello=json.loads(payload.decode('utf-8')).get('nonce')==self.hello_nonce
                except (ValueError,UnicodeDecodeError): is_hello=False
            if is_hello:
                try:self.api=json.loads(payload.decode('utf-8')).get('api')
                except (ValueError,UnicodeDecodeError):pass
                if not self.tracking:
                    self.pre_handshake_crc_errors=self.crc_errors
                    self.crc_errors=0
                self.tracking=True
            if self.tracking:
                if self.previous is not None and not is_hello:
                    advance = (seq-self.previous) & 0xffffffff
                    if advance == 0 or advance >= 0x80000000:
                        self.sequence_errors += 1
                        raise ValueError('duplicate or regressed frame sequence; reconnect explicitly')
                    self.frame_gaps += advance-1
                self.previous = seq
            frames.append((kind, seq, payload))
        return frames

def encode_frame(kind, sequence, payload):
    raw = HEADER.pack(b'JSF1', len(payload), kind, 1, sequence) + payload
    return raw + struct.pack('<I', zlib.crc32(raw))

def decode_record(raw, units_verified=True):
    if len(raw) != RECORD.size or zlib.crc32(raw[:36]) != struct.unpack_from('<I', raw, 36)[0]:
        raise ValueError('record CRC/size mismatch')
    stamp, seq, source, flags, size, payload, crc = RECORD.unpack(raw)
    expected = {1: 16, 2: 6, 3: 12}
    if source not in expected or size != expected[source]:
        raise ValueError('unexpected sensor payload')
    row = dict(time_us=stamp, sequence=seq, sensor={1:'ICM-45605',2:'ADXL375',3:'MMC5983MA'}[source],
               flags=flags, fifo_full=bool(flags & 1), estimated_timestamp=bool(flags & 2),
               saturated=bool(flags & 4), sensor_error=bool(flags & 16))
    if source == 1:
        ax, ay, az, gx, gy, gz, temp, header, tmst = struct.unpack_from('<6hbBH', payload)
        row.update(ax_raw=ax, ay_raw=ay, az_raw=az, gx_raw=gx, gy_raw=gy, gz_raw=gz,
                   ax_g=ax/2048.0, ay_g=ay/2048.0, az_g=az/2048.0,
                   gx_dps=gx/16.4, gy_dps=gy/16.4, gz_dps=gz/16.4,
                   temp_raw=temp, sensor_timestamp_raw=tmst)
    elif source == 2:
        for axis, value in zip('xyz', struct.unpack_from('<3h', payload)):
            row['a'+axis+'_raw'] = value
            row['a'+axis+'_g'] = value*0.049
    else:
        values=struct.unpack_from('<3i',payload)
        if any(value < -131072 or value > 131071 for value in values):
            raise ValueError('magnetometer payload out of signed 18-bit range')
        row.update(zip(('mx_raw','my_raw','mz_raw'),values))
        row['mag_qualified']=False
    if not units_verified:
        row = {key:value for key,value in row.items() if not key.endswith(('_g','_dps'))}
    row['units_verified'] = units_verified
    return row

FIELDS = ['time_us', 'sequence', 'sensor', 'flags', 'fifo_full', 'estimated_timestamp',
          'saturated', 'sensor_error', 'ax_raw', 'ay_raw', 'az_raw', 'gx_raw', 'gy_raw',
          'gz_raw', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps',
          'temp_raw', 'sensor_timestamp_raw', 'mx_raw', 'my_raw', 'mz_raw', 'mag_qualified', 'units_verified',
          'timing_valid', 'sequence_order_valid']

class ScoutClient:
    def __init__(self, host=None, port=9000, usb=None, output=None, data_port=9001, controller_lease=False,
                 shutdown_event=None, close_timeout_s=2.0, control_rx_timeout_s=1.5):
        if type(close_timeout_s) not in (int,float) or not math.isfinite(close_timeout_s) or close_timeout_s<=0:
            raise ValueError('close_timeout_s must be positive finite seconds')
        if type(control_rx_timeout_s) not in (int,float) or not math.isfinite(control_rx_timeout_s) or control_rx_timeout_s<=0:
            raise ValueError('control_rx_timeout_s must be positive finite seconds')
        self.control_rx_timeout_s=control_rx_timeout_s
        self.last_control_rx_at=None
        self.shutdown_event=shutdown_event
        self.close_timeout_s=close_timeout_s
        self.close_requested=threading.Event()
        self.recording_stop=threading.Event()
        self.recording_done=threading.Event()
        self.close_lock=threading.Lock()
        self.close_result=None
        self.recording_error=None
        if output:
            prefix = Path(output)
            for suffix in ('.csv', '.records.bin', '.meta.json'):
                if prefix.with_suffix(suffix).exists():
                    raise FileExistsError(str(prefix.with_suffix(suffix)))
        self.socket = None
        self.data_socket = None
        self.serial = None
        if usb:
            import serial
            self.serial = serial.Serial(port=None, baudrate=115200, timeout=0.1, write_timeout=0.2)
            self.serial.dtr = False
            self.serial.rts = False
            self.serial.port = usb
            self.serial.open()
        else:
            if not host:
                raise ValueError('Specify the ESP address on the Rover LAN, e.g. --host 192.168.0.201')
            self.socket = self._connect_tcp(host,port)
        self.hello_nonce=secrets.randbelow(0xffffffff)+1 if usb else None
        self.decoder = FrameDecoder(self.hello_nonce, require_hello=True)
        self.data_decoder = FrameDecoder()
        self.controller_lease = controller_lease
        self.lease_at = -math.inf
        self.lease_allowed = False
        self.lease_stopped = False
        self.session = None
        self.api = None
        self.hello = {}
        self.wire_compatible = False
        self.seq = 0
        self.status = {}
        self.status_received_at = None
        # Transport liveness is independent of accepted clock/sample freshness.
        self.status_transport_at = None
        self.data_transport_at = None
        self.running = True
        self.error = None
        self.first_failure = None
        self.received_records = 0
        self.raw_received_records = 0
        self.record_gaps = 0
        self.bad_records = 0
        self.client_dropped = 0
        self.record_order_errors = 0
        self.sensor_timestamp_errors = 0
        self.estimated_timestamp_overlaps = 0
        self.status_timestamp_errors = 0
        self.status_duplicates = 0
        self.data_integrity_error = ''
        self.previous_record = None
        self.latest = {}
        self.latest_received = {}
        self.condition = threading.Condition()
        self.send_lock = threading.Lock()
        self.awaiting = set()
        self.acks = {}
        self.pending_mechanisms = {}
        self.configured_mechanism = None
        self.log_results = {}
        self.download_file = None
        self.download_offset = 0
        self.download_complete = False
        self.batches = queue.Queue(maxsize=1024)
        self.output = Path(output) if output else None
        self.host, self.data_port = host, data_port
        self.threads = [threading.Thread(target=self._receive, name='scout-control', daemon=True),
                        threading.Thread(target=self._heartbeat, name='scout-heartbeat', daemon=True),
                        threading.Thread(target=self._records, name='scout-records', daemon=True)]
        self.receiver_threads=[self.threads[0]]
        for thread in self.threads:
            thread.start()
        try:
            deadline = time.monotonic()+4
            with self.condition:
                while self.session is None and self.running and not self._cancelled() and time.monotonic() < deadline:
                    if self.serial:
                        self.serial.write((json.dumps({'cmd':'hello','nonce':self.hello_nonce})+'\n').encode('ascii'))
                    self.condition.wait(0.05)
            if self.session is None:
                cancelled=self._cancelled()
                if cancelled:self.error=self.error or 'Connection cancelled before firmware hello'
                self.close()
                if cancelled:raise ConnectionCancelled(self.error)
                raise RuntimeError(self.error or 'No firmware hello received; check selected USB/Wi-Fi image and connection')
        except Exception:
            self.close()
            raise
        if self.socket:
            thread = threading.Thread(target=self._data_receive, name='scout-data', daemon=True)
            self.threads.append(thread)
            self.receiver_threads.append(thread)
            thread.start()

    def _cancelled(self):
        requested=getattr(self,'close_requested',None)
        external=getattr(self,'shutdown_event',None)
        return bool((requested and requested.is_set()) or (external and external.is_set()))

    def _connect_tcp(self,host,port):
        """Bounded, cancellation-aware connect for the adapter's literal IPv4."""
        sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        try:
            if self._cancelled():raise ConnectionCancelled('Connection cancelled')
            sock.setblocking(False)
            result=sock.connect_ex((host,port))
            pending={errno.EINPROGRESS,errno.EWOULDBLOCK,errno.EALREADY,10035,10036,10037}
            if result not in pending and result!=0:raise OSError(result,'ESP TCP connect failed')
            deadline=time.monotonic()+3
            while result:
                if self._cancelled():raise ConnectionCancelled('Connection cancelled')
                remaining=deadline-time.monotonic()
                if remaining<=0:raise TimeoutError('ESP TCP connect timed out')
                _,ready,failed=select.select([],[sock],[sock],min(.05,remaining))
                if ready or failed:
                    result=sock.getsockopt(socket.SOL_SOCKET,socket.SO_ERROR)
                    if result:raise OSError(result,'ESP TCP connect failed')
                    break
            if self._cancelled():raise ConnectionCancelled('Connection cancelled')
            sock.settimeout(.1)
            sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            return sock
        except Exception:
            sock.close()
            raise

    def _send(self, cmd, fields, wait, guard=None, lock_timeout=None):
        if any(key in fields for key in ('cmd', 'session', 'seq')):
            raise ValueError('reserved command envelope fields')
        if not self.send_lock.acquire(timeout=-1 if lock_timeout is None else max(0,lock_timeout)):
            raise TimeoutError('ESP send lock unavailable before close deadline')
        try:
            if not self.running or self.session is None:
                raise RuntimeError(self.error or 'not connected')
            if getattr(self,'close_requested',None) and self.close_requested.is_set() and cmd not in ('stop','estop'):
                raise RuntimeError('Connection is closing')
            if not motion_compatible(self) and cmd not in ('stop','estop'):
                raise RuntimeError(compatibility_error(self.api,self.status))
            fields = self._v12_fields(cmd, fields)
            if guard is not None:
                guard()
            self.seq += 1
            seq = self.seq
            message = dict(cmd=cmd, session=self.session, seq=seq)
            message.update(fields)
            raw = (json.dumps(message, separators=(',', ':'), allow_nan=False)+'\n').encode('ascii')
            if len(raw) >= 2048:
                raise ValueError('command too long')
            with self.condition:
                if wait:
                    self.awaiting.add(seq)
                    if cmd == 'configure':
                        self.pending_mechanisms[seq] = dict(fields['mechanism'])
            try:
                if self.socket:
                    self.socket.sendall(raw)
                elif self.serial.write(raw) != len(raw):
                    raise OSError('partial serial command write')
            except Exception:
                with self.condition:
                    self.awaiting.discard(seq)
                    getattr(self, 'pending_mechanisms', {}).pop(seq, None)
                raise
        finally:
            self.send_lock.release()
        return seq

    def _v12_fields(self, cmd, fields):
        schemas = {'heartbeat': set(), 'status': set(), 'latch': set(),
                   'prepare': set(), 'launch': set(), 'compress': set(), 'decompress': set(),
                   'compression_stop': set(), 'stop': set(), 'estop': set(),
                   'log_prepare': set(), 'log_stop': set(), 'log_export': set(),
                   'configure': {'mechanism'},
                   'log_start': {'seconds'}, 'clear_fault': {'mechanism_reset'}}
        if cmd not in schemas:
            raise ValueError('Unsupported v12 command; use latch, prepare, launch, compress, decompress or compression_stop')
        if set(fields) != schemas[cmd] and not (cmd == 'clear_fault' and not fields):
            raise ValueError('Invalid v12 command fields')
        if cmd == 'launch' and not launch_supported(self):
            raise ValueError('ESP firmware does not support LAUNCH; install launch-capable firmware')
        if cmd == 'configure':return {'mechanism': v12_mechanism(fields['mechanism'])}
        if cmd == 'clear_fault' and fields and type(fields['mechanism_reset']) is not bool:
            raise ValueError('mechanism_reset must be bool')
        for key, high in (('seconds', 10),):
            if key in fields and (type(fields[key]) is not int or not 1 <= fields[key] <= high):
                raise ValueError('Invalid v12 command integer: ' + key)
        return dict(fields)

    def command(self, cmd, timeout=2, **fields):
        seq = self._send(cmd, fields, True)
        return self._await_ack(seq,timeout)

    def guarded_command(self,cmd,guard,timeout=2,**fields):
        """Check caller cancellation under the wire send lock, before sending."""
        seq=self._send(cmd,fields,True,guard=guard)
        return self._await_ack(seq,timeout)

    def _await_ack(self,seq,timeout):
        deadline = time.monotonic()+timeout
        with self.condition:
            while seq not in self.acks and self.running and time.monotonic() < deadline:
                self.condition.wait(0.05)
            self.awaiting.discard(seq)
            ack = self.acks.pop(seq, None)
            mechanism = getattr(self, 'pending_mechanisms', {}).pop(seq, None)
        if ack is None:
            raise RuntimeError('No ACK: outcome unknown; no automatic retry; inspect output status')
        if not ack.get('ok'):
            raise RuntimeError(ack.get('error', 'command rejected'))
        if mechanism is not None:
            self.configured_mechanism = mechanism
        return ack

    def configure(self, mechanism):
        return self.command('configure', mechanism=mechanism)

    def latch(self):
        return self.command('latch')

    def compression_stop(self):
        return self.command('compression_stop')

    def prepare(self):
        return self.command('prepare')

    def launch(self):
        return self.command('launch')

    def compress(self):
        return self.command('compress')

    def decompress(self):
        return self.command('decompress')

    def stop(self):
        return self.command('stop')

    def keep_alive(self, allowed=True):
        """Renew the server transport lease, independently of browser focus."""
        self.lease_at = time.monotonic()
        self.lease_allowed = bool(allowed)

    def healthy_connection(self):
        received = self.last_control_rx_at
        return bool(self.running and self.session is not None and received is not None
                    and time.monotonic()-received < self.control_rx_timeout_s)

    def log_command(self, cmd, timeout=90, **fields):
        ack = self.command(cmd, **fields)
        seq = ack['seq']
        deadline = time.monotonic()+timeout
        with self.condition:
            while seq not in self.log_results and self.running and time.monotonic() < deadline:
                self.condition.wait(0.1)
            result = self.log_results.pop(seq, None)
        if result is None or not result.get('ok'):
            raise RuntimeError(result or 'Flash operation completion not received')
        return result

    def download_flash(self, filename):
        with Path(filename).open('xb') as file:
            with self.condition:
                self.download_file = file
                self.download_offset = 0
                self.download_complete = False
            try:
                result = self.log_command('log_export')
                deadline = time.monotonic()+10
                with self.condition:
                    while not self.download_complete and self.running and time.monotonic() < deadline:
                        self.condition.wait(0.1)
                    if not self.download_complete:
                        raise RuntimeError('Flash data end marker not received; file is incomplete')
                return result
            finally:
                with self.condition:
                    self.download_file = None

    def _heartbeat(self):
        while self.running and not self._cancelled():
            if self.session is not None:
                try:
                    if not self.healthy_connection():
                        raise RuntimeError('ESP control responses silent; reconnect required; no command replay')
                    live = motion_compatible(self) and (not self.controller_lease or (self.lease_allowed and time.monotonic()-self.lease_at < 0.2))
                    if live:
                        self.lease_stopped = False
                        self._send('heartbeat', {}, False)
                    elif not self.lease_stopped:
                        self._send('stop', {}, False)
                        self.lease_stopped = True
                except Exception as error:
                    if self.running:
                        self._fail(error, channel='control', operation='heartbeat')
            time.sleep(0.1)

    def _fail(self, error, channel=None, operation=None, eof=False, disconnect=True):
        with self.condition:
            # Both receiver workers can fail together. Keep the first cause;
            # shutdown of the other socket must not replace its evidence.
            if not self.running:
                return
            if self.first_failure is None:
                self.error = str(error)
                self.first_failure = dict(channel=channel, operation=operation,
                                      error_type=type(error).__name__, detail=str(error),
                                      errno=getattr(error, 'errno', None),
                                      winerror=getattr(error, 'winerror', None), eof=bool(eof),
                                      host_monotonic_s=time.monotonic(), host_utc_s=time.time())
            if disconnect:
                self.running = False
            self.condition.notify_all()

    def _data_receive(self):
        """Reconnect sensor capture independently of the motor command connection."""
        while self.running and not self._cancelled():
            try:
                self.data_socket = self._connect_tcp(self.host, self.data_port)
                self.data_decoder = FrameDecoder()
                self.data_socket.sendall((json.dumps({'session': self.session})+'\n').encode('ascii'))
                self._receive(data_channel=True)
            except Exception as error:
                if self.running and not self._cancelled():
                    self.recording_error = 'Sensor data connection: ' + str(error)
                    self._fail(error, channel='data', operation='connect', disconnect=False)
            finally:
                if self.data_socket:
                    self.data_socket.close()
                    self.data_socket = None
            self.close_requested.wait(.5)

    def _receive(self, data_channel=False):
        channel = 'data' if data_channel else ('control' if self.socket else 'usb')
        operation = 'receive'
        eof = False
        try:
            sock = self.data_socket if data_channel else self.socket
            decoder = self.data_decoder if data_channel else self.decoder
            while self.running and not self._cancelled():
                try:
                    operation = 'receive'
                    data = sock.recv(8192) if sock else self.serial.read(4096)
                except socket.timeout:
                    continue
                if not data:
                    if sock:
                        eof = True
                        raise EOFError('ESP '+channel+' TCP peer closed connection')
                    continue
                operation = 'decode'
                for kind, seq, payload in decoder.feed(data):
                    if kind in (1,3) and self.session is None:
                        continue
                    if kind == 1:
                        if not self.wire_compatible:
                            raise ValueError('Sensor wire format has not been verified by hello')
                        if len(payload) % 40:
                            self.bad_records += 1
                            continue
                        if not payload:
                            continue
                        self.data_transport_at = time.monotonic()
                        try:
                            # Keep actual socket receipt time through the worker queue;
                            # slow CSV writes must not make an old batch look fresh.
                            self.batches.put_nowait((self.data_transport_at,payload))
                        except queue.Full:
                            self.client_dropped += len(payload)//40
                    elif kind == 3:
                        if len(payload) < 4:
                            raise ValueError('invalid flash chunk')
                        offset = struct.unpack_from('<I', payload)[0]
                        with self.condition:
                            if self.download_file is not None:
                                if offset != self.download_offset:
                                    raise ValueError('flash chunk gap/order mismatch')
                                self.download_file.write(payload[4:])
                                self.download_offset += len(payload)-4
                                if len(payload) == 4:
                                    self.download_complete = True
                                    self.condition.notify_all()
                    else:
                        value = json.loads(payload.decode('utf-8'))
                        with self.condition:
                            event = value.get('event')
                            if event == 'hello':
                                if self.hello_nonce is not None and value.get('nonce')!=self.hello_nonce:
                                    continue
                                if self.session is not None and value['session'] != self.session:
                                    raise OSError('ESP session changed; reconnect explicitly')
                                if type(value.get('session')) is not int or not 1 <= value['session'] <= 0xffffffff:
                                    raise ValueError('invalid ESP hello session')
                                if type(value.get('version')) is not int or value['version'] != 1 or type(value.get('record_bytes')) is not int or value['record_bytes'] != RECORD.size:
                                    raise ValueError('Unsupported ESP wire version or sensor record size')
                                if self.hello and value.get('api') != self.api:
                                    raise ValueError('ESP API changed within connection')
                                self.hello = dict(value)
                                self.wire_compatible = True
                                self.session = value['session']
                                self.api=value.get('api')
                                self.last_control_rx_at = time.monotonic()
                            elif value.get('session') != self.session:
                                continue
                            elif event == 'status':
                                if value.get('api',self.api) != self.api:
                                    raise ValueError('ESP status API changed within connection')
                                stamp=value.get('time_us')
                                previous=self.status.get('time_us')
                                if type(stamp) is not int or stamp < 0:
                                    self.status_timestamp_errors += 1
                                    continue
                                self.status_transport_at = time.monotonic()
                                if stamp == previous:
                                    self.status_duplicates += 1
                                self.status = value
                                self.status_received_at = time.monotonic()
                                self.last_control_rx_at = time.monotonic()
                            elif event == 'ack':
                                if value.get('api', self.api) != self.api:
                                    raise ValueError('ESP ACK API changed within connection')
                                self.last_control_rx_at = time.monotonic()
                                if value.get('seq') in self.awaiting:
                                    self.acks[value['seq']] = value
                            elif event == 'log_result':
                                self.log_results[value['seq']] = value
                                if len(self.log_results) > 32:
                                    self.log_results.pop(next(iter(self.log_results)))
                            self.condition.notify_all()
        except Exception as error:
            if data_channel and self.running and not self._cancelled():
                self.recording_error = 'Sensor data connection: ' + str(error)
                self._fail(error, channel=channel, operation=operation, eof=eof, disconnect=False)
            elif not data_channel and self.running:
                self._fail(error, channel=channel, operation=operation, eof=eof)

    def _records(self):
        csvfile = rawfile = None
        try:
            if self.output:
                try:
                    self.output.parent.mkdir(parents=True, exist_ok=True)
                    csvfile = self.output.with_suffix('.csv').open('x', newline='')
                    rawfile = self.output.with_suffix('.records.bin').open('xb')
                    writer = csv.DictWriter(csvfile, fieldnames=FIELDS)
                    writer.writeheader()
                except Exception as error:
                    self.recording_error = str(error)
                    for recording in (csvfile, rawfile):
                        if recording:
                            try: recording.close()
                            except Exception: pass
                    csvfile = rawfile = None
            while not self.recording_stop.is_set() or not self.batches.empty():
                try:
                    received_at,payload = self.batches.get(timeout=0.1)
                except queue.Empty:
                    continue
                self.raw_received_records += len(payload)//40
                if rawfile:
                    try: rawfile.write(payload)
                    except Exception as error:
                        self.recording_error = str(error)
                        try: rawfile.close()
                        except Exception: pass
                        rawfile = None
                for offset in range(0, len(payload), 40):
                    raw = payload[offset:offset+40]
                    try:
                        row = decode_record(raw,units_verified=motion_compatible(self))
                    except ValueError:
                        self.bad_records += 1
                        continue
                    seq = row['sequence']
                    row['sequence_order_valid'] = True
                    row['timing_valid'] = True
                    if self.previous_record is not None:
                        advance=(seq-self.previous_record) & 0xffffffff
                        if advance == 0 or advance >= 0x80000000:
                            self.record_order_errors += 1
                            row['sequence_order_valid'] = False
                            self.data_integrity_error = self.data_integrity_error or 'duplicate or regressed sensor sequence; reconnect explicitly'
                        else:
                            self.record_gaps += advance-1
                    if row['sequence_order_valid']:
                        self.previous_record = seq
                    prior=self.latest.get(row['sensor'])
                    if prior is not None and row['time_us'] <= prior['time_us']:
                        self.sensor_timestamp_errors += 1
                        row['timing_valid'] = False
                        # Reviewed v4/v5 sensors.c reconstructs each FIFO batch from
                        # a new host poll time. Estimated overlap is not proof of
                        # a stopped sensor clock. Exclude the row; never retime it.
                        if row['estimated_timestamp'] and motion_compatible(self):
                            self.estimated_timestamp_overlaps += 1
                        else:
                            self.data_integrity_error = self.data_integrity_error or row['sensor']+' timestamp stopped/regressed; reconnect explicitly'
                    # Preserve every decodable record for analysis, but an invalid
                    # clock/order record never replaces the latest valid sample.
                    # Record timing errors are diagnostic and do not block controls.
                    if csvfile:
                        try: writer.writerow(row)
                        except Exception as error:
                            self.recording_error = str(error)
                            try: csvfile.close()
                            except Exception: pass
                            csvfile = None
                    if row['timing_valid'] and row['sequence_order_valid']:
                        self.received_records += 1
                        self.latest[row['sensor']] = row
                        self.latest_received[row['sensor']] = received_at
        except Exception as error:
            self.recording_error=str(error)
        finally:
            for recording in (csvfile,rawfile):
                if recording:
                    try:recording.close()
                    except Exception as error:
                        self.recording_error=str(error)
            self.recording_done.set()

    def close(self,timeout=None):
        """Stop without an ACK wait, drain recordings, then report finalization."""
        timeout=self.close_timeout_s if timeout is None else timeout
        if type(timeout) not in (int,float) or not math.isfinite(timeout) or timeout<=0:
            raise ValueError('close timeout must be positive finite seconds')
        deadline=time.monotonic()+timeout
        if not self.close_lock.acquire(timeout=timeout):
            raise FinalizationError('Concurrent client close has not completed')
        try:
            if self.close_result and self.close_result['complete']:return dict(self.close_result)
            self.close_requested.set()
            stop_sent=False;stop_error=None
            if self.running and self.session:
                try:
                    self._send('stop',{},False,lock_timeout=min(.1,max(0,deadline-time.monotonic())))
                    stop_sent=True
                except Exception as error:
                    stop_error=str(error)  # No ACK is claimed; link watchdog remains the fallback.
            self.running=False
            with self.condition:self.condition.notify_all()
            for sock in (self.socket,self.data_socket):
                if sock:
                    try:sock.shutdown(socket.SHUT_RDWR)
                    except OSError:pass
                    sock.close()
            if self.serial:self.serial.close()
            current=threading.current_thread()
            for thread in self.receiver_threads:
                if thread is not current:thread.join(timeout=max(0,deadline-time.monotonic()))
            # Do not let an empty queue terminate the writer before a receiver's
            # final already-received batch has been enqueued.
            if not any(thread.is_alive() for thread in self.receiver_threads):
                self.recording_stop.set()
            for thread in self.threads:
                if thread is not current:thread.join(timeout=max(0,deadline-time.monotonic()))
            unfinished=[thread.name for thread in self.threads if thread.is_alive()]
            finalized=self.recording_done.is_set() and not self.recording_error
            complete=not unfinished and self.recording_done.is_set() and self.batches.empty()
            result=dict(complete=bool(complete),recording_finalized=bool(finalized),recording_error=self.recording_error,
                        unfinished_workers=unfinished,pending_batches=self.batches.qsize(),
                        stop_sent=stop_sent,stop_acknowledged=False,stop_error=stop_error)
            try:self._write_metadata(result)
            except Exception as error:
                result['recording_error']='Recording metadata finalization failed: '+str(error)
                result['recording_finalized']=False
            self.close_result=result
            if not complete:
                raise FinalizationError('SDK shutdown incomplete: '+json.dumps(result,separators=(',',':')))
            return dict(result)
        finally:
            self.close_lock.release()

    def _write_metadata(self,close_status):
        if self.output:
            metadata = dict(records=self.received_records, raw_records=self.raw_received_records, record_gaps=self.record_gaps,
                            frame_gaps=self.decoder.frame_gaps+self.data_decoder.frame_gaps,
                            crc_errors=self.decoder.crc_errors+self.data_decoder.crc_errors,
                            bad_records=self.bad_records, client_dropped=self.client_dropped,
                             record_order_errors=self.record_order_errors,
                             sensor_timestamp_errors=self.sensor_timestamp_errors,
                             estimated_timestamp_overlaps=self.estimated_timestamp_overlaps,
                             status_timestamp_errors=self.status_timestamp_errors,
                             status_duplicates=self.status_duplicates,
                             frame_sequence_errors=self.decoder.sequence_errors+self.data_decoder.sequence_errors,
                            hello=self.hello, api=self.api, wire_compatible=self.wire_compatible,
                            units_verified=motion_compatible(self),
                            data_integrity_error=self.data_integrity_error,
                            finalization=close_status,
                            final_status=self.status, error=self.error,
                            first_failure=getattr(self,'first_failure',None), measurement_qualified=False,
                            configured_mechanism=getattr(self,'configured_mechanism',None),
                            notes=['FIFO host timestamps are estimated; raw ICM timestamp retained.',
                                   'Board axes, timing, scales and latch/compression settings require hardware validation.',
                                   'Magnetometer is centered signed raw data, uncalibrated and not a validated heading.'])
            self.output.with_suffix('.meta.json').write_text(json.dumps(metadata, indent=2))

def main():
    p = argparse.ArgumentParser(description=__doc__)
    link = p.add_mutually_exclusive_group()
    link.add_argument('--host', default=None)
    link.add_argument('--usb')
    p.add_argument('--port', type=int, default=9000)
    p.add_argument('--data-port', type=int, default=9001)
    p.add_argument('--output', help='New output prefix for CSV, raw records and metadata')
    args = p.parse_args()
    if not args.host and not args.usb:
        p.error('Supply --host ESP_IP for Wi-Fi, or --usb DEVICE for the debug image')
    client = ScoutClient(args.host, args.port, args.usb, args.output, args.data_port)
    controls = 'latch | prepare | launch | compress | decompress | compression_stop'
    print('Connected. Commands: status | configure FILE.json | ' + controls + ' | stop | estop | clear_fault | log_prepare | log_start SECONDS | log_stop | log_export FILE.bin | quit')
    try:
        while client.running:
            text = input('scout> ').strip().split(maxsplit=1)
            if not text:
                continue
            cmd = text[0]
            if cmd == 'quit':
                break
            try:
                if cmd == 'configure' and len(text) == 2:
                    print(client.configure(json.loads(Path(text[1]).read_text())))
                elif cmd in ('latch', 'prepare', 'launch', 'compress', 'decompress', 'compression_stop') and len(text)==1:
                    print(getattr(client, cmd)())
                elif cmd == 'log_export' and len(text) == 2:
                    print(client.download_flash(text[1]))
                elif cmd == 'log_start' and len(text) == 2:
                    print(client.log_command(cmd, seconds=int(text[1])))
                elif cmd in ('log_prepare', 'log_stop'):
                    print(client.log_command(cmd))
                elif cmd in ('status', 'stop', 'estop', 'clear_fault') and len(text)==1:
                    print(client.command(cmd))
                    if cmd == 'status':
                        print(json.dumps(client.status, indent=2))
                        print('IMU records:', client.received_records, 'gaps:', client.record_gaps)
                else:
                    print('Unknown/incomplete command')
            except Exception as error:
                print('ERROR:', error)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        client.close()

if __name__ == '__main__':
    main()
