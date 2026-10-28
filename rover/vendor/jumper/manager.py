"""Frozen v12 direct controls and server-owned transport heartbeat."""
from collections import deque
from datetime import datetime
import math
import threading
import time
import uuid
from .scout_client import ScoutClient, API_V12, compatibility_error, motion_compatible, launch_supported, v12_mechanism, FinalizationError


class Manager:
    def __init__(self, factory=ScoutClient, command_timeout_s=1.5, **unused):
        if type(command_timeout_s) not in (int, float) or not math.isfinite(command_timeout_s) or command_timeout_s <= 0:
            raise ValueError('command_timeout_s must be positive finite seconds')
        self.command_timeout_s=command_timeout_s
        self.factory=factory;self.lock=threading.RLock();self.command_lock=threading.Lock()
        self.control_allowed=False
        self.client=None;self.cid='';self.connecting=False;self.generation=0
        self.view_seen=self.status_seen=0;self.stamp=None;self.sensor_stamps={};self.sensor_seen={}
        self.cancel_epoch=0;self.profile_applied=False;self.error='';self.events=deque(maxlen=30)
        self.closing=threading.Event();self.thread=threading.Thread(target=self.watch,daemon=True);self.thread.start()
    def event(self,text):self.events.append(dict(time=datetime.now().strftime('%H:%M:%S'),text=text))
    def connection(self,cid):
        if not self.client or cid!=self.cid:raise ValueError('현재 연결에서 다시 시도하세요.')
    def connect(self, host=''):
        with self.lock:
            if self.client or self.connecting:raise ValueError('현재 연결을 먼저 해제하세요.')
            self.connecting=True;self.generation+=1;generation=self.generation
        c=None
        try:
            c=self.factory(host=host,controller_lease=True)
            with self.lock:
                if self.generation!=generation:raise ValueError('연결 요청이 취소되었습니다.')
                self.client=c;self.cid=uuid.uuid4().hex;self.host=host
                self.profile_applied=bool(c.status.get('configured', True));self.error='';self.stamp=None;self.status_seen=0;self.view_seen=time.monotonic()
                self.control_allowed=False
                self.sensor_stamps.clear();self.sensor_seen.clear();self.connecting=False
                self.event('Scout connected')
                if not motion_compatible(c):self.event(compatibility_error(c.api,c.status))
            self.refresh()
        except Exception:
            try:
                if c:c.close()
            finally:
                with self.lock:self.connecting=False
            raise
    def disconnect(self):
        with self.lock:
            self.generation+=1;self.cancel_epoch+=1;self.control_allowed=False;c=self.client;self.client=None;self.cid='';self.profile_applied=False;self.connecting=False;self.event('연결 해제')
        if c:
            c.keep_alive(False)
            return c.close()
    def direct_manual(self):
        return bool(self.client and self.client.api == API_V12)

    def set_control_allowed(self, allowed):
        with self.lock:
            self.control_allowed = bool(allowed and self.client and self.client.running)

    def cancel_pending(self):
        with self.lock:
            self.cancel_epoch += 1

    def revoke_control(self):
        with self.lock:
            self.cancel_pending()
            self.control_allowed = False

    def ping(self, cid):
        with self.lock:
            self.connection(cid)
            self.view_seen = time.monotonic()

    def refresh(self):
        with self.lock:
            c = self.client
            if not c:
                return
            self.stamp = c.status.get('time_us')
            self.status_seen = getattr(c, 'status_received_at', None) or time.monotonic()
            for name, row in c.latest.copy().items():
                self.sensor_stamps[name] = row.get('time_us', 0)
                self.sensor_seen[name] = getattr(c, 'latest_received', {}).get(name, time.monotonic())

    def ages(self):
        """Receipt age and same-ESP-clock age; never compare ESP time to UTC."""
        now=time.monotonic();status_age=None if not self.status_seen else now-self.status_seen
        estimate=None if self.stamp is None or status_age is None else self.stamp+max(0,status_age)*1e6
        sensors={}
        for name,stamp in self.sensor_stamps.items():
            received=self.sensor_seen.get(name)
            sensors[name]={'receipt_s':None if received is None else now-received,
                           'device_s':None if estimate is None else (estimate-stamp)/1e6}
        transport={}
        for name in ('status','data'):
            received=getattr(self.client,name+'_transport_at',None)
            transport[name+'_s']=None if received is None else now-received
        received=getattr(self.client,'last_control_rx_at',None)
        transport['control_s']=None if received is None else now-received
        return dict(status_s=status_age,view_s=None if not self.view_seen else now-self.view_seen,sensors=sensors,transport=transport)
    def diagnostics(self):
        c=self.client
        keys=('received_records','raw_received_records','record_gaps','bad_records','client_dropped','record_order_errors','sensor_timestamp_errors','estimated_timestamp_overlaps','status_timestamp_errors','status_duplicates')
        result={key:getattr(c,key,0) for key in keys}
        for key in ('frame_gaps','crc_errors','discarded_bytes','sequence_errors'):
            result[key]=sum(getattr(getattr(c,name,None),key,0) for name in ('decoder','data_decoder'))
        result['tx_dropped']=c.status.get('tx_dropped') if c else None
        result['output_prefix']=str(c.output) if c and getattr(c,'output',None) else None
        result['data_integrity_error']=getattr(c,'data_integrity_error','')
        result['recording_error']=getattr(c,'recording_error',None)
        failure=getattr(c,'first_failure',None)
        result['first_failure']=dict(failure) if failure else None
        return result
    def issues(self):
        if not self.client or not self.client.running:
            return ['ESP connection lost']
        if not motion_compatible(self.client):
            return [compatibility_error(self.client.api, self.client.status)]
        if not self.control_allowed:
            return ['Control connection required']
        return []

    def action(self, data):
        action = data.get('action')
        if action not in ('configure', 'latch', 'prepare', 'launch', 'compress', 'decompress', 'compression_stop', 'clear_fault'):
            raise ValueError('Use LATCH / PREPARE / LAUNCH / COMPRESS / DECOMPRESS / COMPRESSION STOP')
        allowed_fields = {'action', 'connection_id', 'request_id'}
        if action == 'configure': allowed_fields.add('settings')
        if action == 'clear_fault': allowed_fields.add('mechanism_reset')
        if set(data) - allowed_fields:
            raise ValueError('Unsupported action fields')
        if not self.command_lock.acquire(False):
            raise ValueError('Previous command pending')
        try:
            with self.lock:
                self.connection(data.get('connection_id'))
                c, epoch = self.client, self.cancel_epoch
                if self.issues():
                    raise ValueError(' / '.join(self.issues()))
                if action == 'launch' and not launch_supported(c):
                    raise ValueError('ESP firmware does not support LAUNCH; install launch-capable firmware')
                fields = {}
                if action == 'configure':
                    fields['mechanism'] = v12_mechanism(data.get('settings', {}))
                elif action == 'clear_fault':
                    reset = data.get('mechanism_reset', False)
                    if type(reset) is not bool:
                        raise ValueError('mechanism_reset must be bool')
                    fields['mechanism_reset'] = reset

            def guard():
                with self.lock:
                    if self.client is not c or epoch != self.cancel_epoch or not self.control_allowed:
                        raise ValueError('Control disconnected; command cancelled')

            if hasattr(c, 'guarded_command'):
                result = c.guarded_command(action, guard=guard, timeout=self.command_timeout_s, **fields)
            else:
                guard()
                result = c.command(action, timeout=self.command_timeout_s, **fields)
            with self.lock:
                if self.client is not c or epoch != self.cancel_epoch:
                    return dict(ok=False, cancelled=True)
                self.profile_applied = c.status.get('configured', True)
                self.error = ''
                self.event(action)
            return result
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            raise
        finally:
            self.command_lock.release()

    def compression_stop(self, cid):
        with self.lock:
            self.connection(cid)
            c = self.client
            self.cancel_pending()
        return c.command('compression_stop', timeout=self.command_timeout_s)

    def stop(self):
        self.revoke_control()
        with self.lock:c=self.client;self.event('STOP')
        if c and c.running:
            c.keep_alive(False)
            return c.command('stop',timeout=self.command_timeout_s)
        return {'ok':False,'error':'ESP unavailable; stop outcome unconfirmed','lease_revoked':True}
    def emergency_stop(self):
        self.revoke_control()
        with self.lock:c=self.client;self.event('E-STOP')
        return c.command('estop',timeout=self.command_timeout_s) if c and c.running else dict(ok=False,error='ESP disconnected')
    def snapshot(self):
        self.refresh()
        with self.lock:
            c=self.client;now=time.monotonic()
            compatible=motion_compatible(c)
            sensors={k:{key:value for key,value in v.items() if compatible or not key.endswith(('_g','_dps'))} for k,v in c.latest.copy().items()} if c else {}
            for row in sensors.values():row['units_verified']=compatible and row.get('units_verified',True)
            return dict(connected=bool(c and c.running),has_connection=bool(c),connecting=self.connecting,
                api=c.api if c else None,hello=dict(getattr(c,'hello',{})) if c else {},motion_compatible=compatible,units_verified=compatible,
                launch_supported=launch_supported(c),
                compatibility_error=compatibility_error(c.api,c.status) if c and not compatible else '',
                connection_id=self.cid,profile_applied=self.profile_applied,host=getattr(self,'host',''),
                status=dict(c.status) if c else {},sensors=sensors,
                fresh=bool(c and c.running and now-self.status_seen<=.6),waiting_status=False,issues=self.issues(),
                ages=self.ages(),diagnostics=self.diagnostics(),freshness_error='',control_allowed=self.control_allowed,
                data_integrity_error=getattr(c,'data_integrity_error',''),
                error=self.error or (c.error if c else ''),events=list(self.events)[-10:])
    def watch(self):
        while not self.closing.wait(.04):
            self.refresh()
            with self.lock:
                c = self.client
                if not c:
                    continue
                # Browser focus/closure and command ownership do not expire a
                # healthy server connection. Actual silence/close still stops
                # the firmware through the client's transport watchdog.
                healthy = c.healthy_connection() if hasattr(c, 'healthy_connection') else c.running
                c.keep_alive(healthy and motion_compatible(c))

    def close(self):
        self.closing.set()
        try:return self.disconnect()
        finally:
            self.thread.join(timeout=.4)
            if self.thread.is_alive():raise FinalizationError('Manager watchdog did not finalize')
