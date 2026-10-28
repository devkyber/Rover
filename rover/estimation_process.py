"""Optional estimation owner. Control-facing methods never perform IPC.

A background bridge owns process startup, serialization and blocking pipe I/O.
The estimator owns IMU acquisition, filtering and its separate raw recordings.
An unavailable estimator never creates a dependency for manual actuation.
"""
import copy
from collections import deque
import math
import multiprocessing
import os
from pathlib import Path
import queue
import signal
import threading
import time
from types import SimpleNamespace

import config


def _unavailable(run_id, status, error):
    imu = dict(status='unavailable', error=error, attempt=0, generation=0,
               age_s=None, samples=[], stats={})
    telemetry = dict(pose_valid=False, pose_estimate_available=False,
                     attitude_estimate_available=False, estimate_held=False,
                     held_estimate=None, attitude_estimate=None, pose_estimate=None,
                     pose_quality='unavailable', pose_quality_reason=error,
                     manual_gap_tolerated=False, pose_frame_id=None, odom_frame_id=None,
                     pos=[None] * 3, vel=[None] * 3, rpy=[None] * 3,
                     gyro_bias=[None] * 3, nis={}, imu=imu)
    context = dict(run_id=run_id, pose_valid=False, rover_position_m=None,
                   rover_rpy_deg=None, pose_frame_id=None, odom_frame_id=None,
                   pose_quality='unavailable', position_uncertainty_m=None)
    return dict(status=status, error=error, run_id=run_id, produced_monotonic_s=None,
                snapshot=imu, telemetry=telemetry, context=context,
                estimator_recording=None)


def _paired_interval(source_stamp, wheel_stamp):
    if not all(isinstance(value, (int, float)) and math.isfinite(value)
               for value in (source_stamp, wheel_stamp)):
        return None
    return source_stamp - wheel_stamp


def source_health(snapshot, source_age):
    """Acquisition health is distinct from estimator/wheel eligibility."""
    value = dict(snapshot)
    status = value.get('source_status', value.get('status', 'unavailable'))
    error = value.get('source_error', value.get('error'))
    value.update(source_status=status, source_error=error, age_s=source_age)
    if status == 'ok' and (not isinstance(source_age, (int, float))
            or not math.isfinite(source_age) or not 0 <= source_age <= config.IMU_SAMPLE_MAX_AGE_S):
        value.update(status='delayed', error='Waiting for a fresh IMU observation')
    else:
        value.update(status=status, error=error)
    return value


def precision_status(source_age, paired_interval, *, result_age=None):
    """Timing eligibility for precise context, independent of display transport."""
    imu_limit = getattr(config, 'IMU_PRECISION_MAX_AGE_S', .05)
    wheel_limit = getattr(config, 'ESTIMATION_PRECISION_WHEEL_MAX_AGE_S', .05)
    def within(value, limit):
        return (isinstance(value, (int, float)) and math.isfinite(value)
                and 0 <= value <= limit)
    reason = None
    if not within(source_age, imu_limit):
        reason = 'IMU observation exceeds precision age'
    elif not within(paired_interval, wheel_limit):
        reason = 'Wheel pairing exceeds precision interval'
    elif result_age is not None and not within(result_age, config.ESTIMATION_RESULT_MAX_AGE_S):
        reason = 'Estimator result exceeds display age'
    return dict(fresh=reason is None, reason=reason, source_age_s=source_age,
                paired_wheel_interval_s=paired_interval, imu_max_age_s=imu_limit,
                wheel_max_interval_s=wheel_limit)


def restrict_precision(reply, source_age, paired_interval, *, result_age=None):
    """Keep explicit display estimates while withholding precise pose/context.

    The caller owns this reply. No numeric filter state, acquisition generation
    or motion-loss accounting is changed merely because delivery is late.
    """
    telemetry = reply.setdefault('telemetry', {})
    precision = precision_status(source_age, paired_interval, result_age=result_age)
    telemetry['estimation_precision'] = precision
    if not precision['fresh']:
        # Some compact/test producers expose only the precise fields. Preserve
        # that explicitly identified estimate before clearing the precise view.
        if telemetry.get('pose_valid'):
            if not telemetry.get('pose_estimate'):
                telemetry['pose_estimate'] = {key: copy.deepcopy(telemetry.get(key))
                                              for key in ('pos', 'vel', 'rpy', 'gyro_bias')}
                telemetry['pose_estimate_available'] = True
            if not telemetry.get('attitude_estimate'):
                telemetry['attitude_estimate'] = {key: copy.deepcopy(telemetry.get(key))
                                                  for key in ('rpy', 'gyro_bias')}
                telemetry['attitude_estimate_available'] = True
        telemetry.update(pose_valid=False, pos=[None] * 3, vel=[None] * 3,
                         rpy=[None] * 3, gyro_bias=[None] * 3, nis={})
        if ((telemetry.get('pose_estimate_available') or telemetry.get('attitude_estimate_available'))
                and telemetry.get('pose_quality') not in ('degraded_gap', 'simulation', 'unverified_sources')):
            telemetry.update(pose_quality='delayed', pose_quality_reason=precision['reason'])
    if not telemetry.get('pose_valid'):
        context = reply.setdefault('context', {})
        context.update(pose_valid=False, rover_position_m=None, rover_rpy_deg=None,
                       pose_frame_id=None, odom_frame_id=None,
                       pose_quality=telemetry.get('pose_quality', 'unavailable'),
                       position_uncertainty_m=None)
    return reply


def _masked(reply, status, error, source_age=None, *, stale_reason='estimate'):
    """Keep diagnostics/source identity while removing every usable pose view."""
    result = copy.deepcopy(reply)
    empty = _unavailable(reply['run_id'], status, error)
    imu = result.get('telemetry', {}).get('imu', {})
    result.setdefault('telemetry', {}).update(empty['telemetry'])
    result['telemetry']['imu'] = source_health(imu, source_age)
    estimation = result['telemetry'].get('estimation_status')
    if estimation is not None:
        result['telemetry']['estimation_status'] = dict(estimation, position_source=None)
    result.setdefault('context', {}).update(empty['context'])
    result['snapshot'] = source_health(result.get('snapshot', {}), source_age)
    result['status'], result['error'] = status, error
    result['source_age_s'] = source_age
    result['stale_reason'] = stale_reason
    return result


def _entry(target, connection, kind, settings):
    # The BMI088 acquisition grandchild inherits this group. Forced shutdown
    # can therefore remove all estimation owners, rather than orphaning SPI.
    if hasattr(os, 'setsid'):
        os.setsid()
    try:
        target(connection, kind, settings)
    finally:
        connection.close()


class EstimationProcess:
    def __init__(self, kind, settings, *, worker_target=None):
        self.kind = kind
        self._settings = copy.deepcopy(settings)
        if not isinstance(self._settings.get('run_id'), str):
            raise ValueError('Estimator settings require a run_id')
        self._target = worker_target or _worker
        self._inputs = queue.Queue(maxsize=getattr(config, 'ESTIMATION_INPUT_QUEUE_MAX', 8))
        self._max_age = getattr(config, 'ESTIMATION_RESULT_MAX_AGE_S', .2)
        self._close_timeout = getattr(config, 'ESTIMATION_CLOSE_TIMEOUT_S', 3.)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._changed = threading.Event()
        self._revision = 1
        self._configuration = (self._revision, self._settings)
        self._result = None
        self._status, self._error = 'starting', None
        self._published = (self._status, self._error, None)
        self._dropped = 0
        self._thread = self._process = None
        self._cleanup_safe = True
        self._close_deadline = None
        self._close_forced = False
        self._recording_completion_unknown = False
        self._last_recording = None

    def start(self):
        with self._lock:
            if self._thread is None and not self._stop.is_set():
                self._thread = threading.Thread(target=self._bridge, name='estimation-bridge', daemon=True)
                try:
                    self._thread.start()
                except Exception as exc:
                    self._thread = None
                    self._status, self._error = 'error', type(exc).__name__ + ': ' + str(exc)
                    self._published = (self._status, self._error, None)
        return self

    def configure(self, settings):
        frozen = copy.deepcopy(settings)
        if not isinstance(frozen.get('run_id'), str):
            raise ValueError('Estimator settings require a run_id')
        # Publish one immutable tuple. The bridge never mutates these settings;
        # control must not wait for a bridge lock or its source-metadata copy.
        old_revision, old_settings = self._configuration
        old_status, old_error, _ = self._published
        self._revision = old_revision + 1
        self._settings = frozen
        self._configuration = (self._revision, frozen)
        self._published = (('error', old_error, None)
                           if old_status == 'error' and old_settings['run_id'] == frozen['run_id']
                           else ('starting', None, None))
        for _ in range(self._inputs.maxsize):
            try:
                self._inputs.get_nowait()
            except queue.Empty:
                break
        self._changed.set()

    def submit(self, wheel_reading, timestamp, run_id):
        if not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            return False
        packet = dict(kind='wheels', wheel_reading=copy.deepcopy(wheel_reading),
                      timestamp=float(timestamp), run_id=run_id)
        revision, settings = self._configuration
        if self._stop.is_set() or run_id != settings['run_id']:
            self._dropped += 1
            return False
        packet['revision'] = revision
        try:
            self._inputs.put_nowait(packet)
            return True
        except queue.Full:
            self._dropped += 1
            return False

    def poll(self):
        # Only a local reference/compact-copy operation. No connection.poll,
        # recv, unpickling, process startup or worker join is permitted here.
        revision, settings = self._configuration
        status, error, result = self._published
        run_id, dropped = settings['run_id'], self._dropped
        if result is not None and (result.get('revision') != revision or result.get('run_id') != run_id):
            result = None
        if self._stop.is_set():
            if self._close_forced and error is None:
                error = 'Estimator required forced shutdown; estimation recording durability unknown'
            result = _unavailable(run_id, 'error' if error else 'closed', error)
            result['estimator_recording'] = copy.deepcopy(self._last_recording)
        elif result is None:
            result = _unavailable(run_id, status, error or 'Estimator result not available')
        else:
            now = time.monotonic()
            age = now - result['produced_monotonic_s']
            stamp = result.get('source_sample_monotonic_s')
            source_age = now - stamp if isinstance(stamp, (int, float)) else None
            wheel_stamp = result.get('paired_wheel_monotonic_s')
            paired_interval = _paired_interval(stamp, wheel_stamp)
            usable = any(result.get('telemetry', {}).get(key) for key in
                         ('pose_valid', 'pose_estimate_available', 'attitude_estimate_available', 'estimate_held'))
            if not 0 <= age <= self._max_age:
                result = _masked(result, 'stale', 'Estimator result delayed; pose unavailable', source_age,
                                 stale_reason='result_age')
            elif ((usable and source_age is None)
                  or (source_age is not None and not 0 <= source_age <= config.IMU_SAMPLE_MAX_AGE_S)):
                result = _masked(result, 'stale', 'IMU observation delayed; pose unavailable', source_age,
                                 stale_reason='imu_age')
            elif ((usable and paired_interval is None)
                  or (paired_interval is not None and not 0 <= paired_interval <= config.ESTIMATION_WHEEL_MAX_AGE_S)):
                result = _masked(result, 'stale', 'IMU interval has no fresh preceding wheel observation', source_age,
                                 stale_reason='wheel_pair')
            else:
                result = copy.deepcopy(result)
                if result.get('status') != 'stale':
                    result['status'], result['error'] = 'running', None
                result['source_age_s'] = source_age
                result['snapshot'] = source_health(result.get('snapshot', {}), source_age)
                result['telemetry']['imu'] = source_health(result['telemetry'].get('imu', {}), source_age)
                result['result_age_s'] = age
            result['paired_wheel_interval_s'] = paired_interval
            restrict_precision(result, source_age, paired_interval, result_age=age)
        result['input_dropped'] = dropped
        result['close_forced'] = self._close_forced
        result['recording_completion_unknown'] = self._recording_completion_unknown
        return result

    def _fail(self, error):
        self._published = ('error', error, None)

    def _stop_process(self, process, deadline):
        if process is None or process.pid is None:
            return process is None
        try:
            forced = process.is_alive()
            # The leader may already have exited while its SPI grandchild is
            # still alive. Signal the group even when is_alive() is false.
            if hasattr(os, 'killpg'):
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    forced = True
                except ProcessLookupError:
                    pass
            if forced:
                self._close_forced = True
                self._recording_completion_unknown = True
            if process.is_alive():
                if not hasattr(os, 'killpg'):
                    process.terminate()
                else:
                    try:
                        if os.getpgid(process.pid) != process.pid:
                            process.terminate()  # Entry has not called setsid yet.
                    except ProcessLookupError:
                        pass
                process.join(max(0., min(.15, deadline - time.monotonic())))
            # A grandchild may ignore TERM even after the estimator exited.
            # KILL the remaining group independently of the leader's state.
            if hasattr(os, 'killpg'):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    if process.is_alive():
                        process.kill()
            elif process.is_alive():
                process.kill()
            process.join(max(0., deadline - time.monotonic()))
            if process.is_alive():
                return False
            if hasattr(os, 'killpg'):
                for entry in Path('/proc').iterdir():
                    if not entry.name.isdigit():
                        continue
                    try:
                        # comm may contain spaces/parentheses; fields after its
                        # final ')' begin with state, ppid and process group.
                        tail = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
                        if int(tail[2]) == process.pid and tail[0] != 'Z':
                            return False
                    except (OSError, ValueError, IndexError):
                        continue
            return True
        except (OSError, ValueError, AssertionError):
            return False

    def _bridge(self):
        failed_run = None
        while not self._stop.is_set():
            revision, settings = self._configuration
            if failed_run == settings['run_id']:
                self._changed.wait(.02)
                self._changed.clear()
                continue  # Never restart a failed estimator under the old origin.
            if not self._cleanup_safe:
                self._fail('Estimator owner cleanup unconfirmed; restart the runtime')
                return
            parent = child = process = None
            try:
                parent, child = multiprocessing.get_context('spawn').Pipe()
                initial_settings = dict(settings, _estimation_revision=revision)
                process = multiprocessing.get_context('spawn').Process(
                    target=_entry, args=(self._target, child, self.kind, initial_settings), daemon=False)
                with self._lock:
                    self._process = process
                if self._stop.is_set():
                    break
                process.start()
                child.close()
                while not self._stop.is_set():
                    latest_revision, latest_settings = self._configuration
                    if latest_revision != revision:
                        parent.send(dict(kind='configure', revision=latest_revision,
                                         settings=latest_settings))
                        revision = latest_revision
                        settings = latest_settings
                    # Both pipe operations, including their pickle work, belong
                    # exclusively to this background thread.
                    try:
                        packet = self._inputs.get_nowait()
                    except queue.Empty:
                        packet = None
                    if packet is not None and packet['revision'] == revision:
                        parent.send(packet)
                    if parent.poll(.002):
                        reply = parent.recv()
                        if not isinstance(reply, dict):
                            raise ValueError('Invalid estimator reply')
                        if reply.get('kind') == 'fatal':
                            raise RuntimeError(reply.get('error', 'Estimator failed'))
                        if 'estimator_recording' in reply:
                            self._last_recording = reply['estimator_recording']
                        current_revision, current_settings = self._configuration
                        if (reply.get('run_id') == current_settings['run_id']
                                and reply.get('revision') == current_revision
                                and isinstance(reply.get('produced_monotonic_s'), (int, float))):
                            self._published = ('running', None, reply)
                    elif not process.is_alive():
                        raise RuntimeError('Estimator process exited: ' + str(process.exitcode))
                if self._stop.is_set() and process.is_alive():
                    parent.send(dict(kind='stop'))
                    deadline = self._close_deadline or (time.monotonic() + self._close_timeout)
                    process.join(max(0., deadline - time.monotonic() - .2))
                    # A clean worker sends its final recorder accounting before
                    # exiting. Receive it on this bridge, never on control.
                    while True:
                        try:
                            if not parent.poll():
                                break
                            final = parent.recv()
                        except (EOFError, OSError):
                            break
                        if final.get('kind') == 'closed':
                            self._last_recording = final.get('estimator_recording')
                            if final.get('error'):
                                self._fail(final['error'])
            except (Exception, EOFError) as exc:
                self._fail(type(exc).__name__ + ': ' + str(exc))
                self._recording_completion_unknown = bool(settings.get('log_requested', True))
                failed_run = settings['run_id']
            finally:
                if process is not None and process.pid is not None:
                    self._cleanup_safe = self._stop_process(process, time.monotonic() + .3)
                if parent is not None:
                    parent.close()
                if child is not None:
                    child.close()

    def close(self, wait=True):
        if self._close_deadline is None:
            self._close_deadline = time.monotonic() + self._close_timeout
        self._stop.set()
        self._changed.set()
        if wait and self._thread is not None:
            deadline = self._close_deadline
            self._thread.join(max(0., deadline - time.monotonic() - .2))
            if self._thread.is_alive():
                with self._lock:
                    process = self._process
                self._cleanup_safe = self._stop_process(process, deadline)
                self._thread.join(max(0., deadline - time.monotonic()))
            if self._thread.is_alive() or not self._cleanup_safe:
                self._fail('Estimator close timed out or owner cleanup unconfirmed')
        return self.poll()


_ESTIMATOR_FIELDS = {
    'odom_frame_id', 'align', 'pose_valid', 'pose_estimate_available', 'estimate_held',
    'held_estimate', 'pose_frame_id', 'pose_quality', 'pose_quality_reason',
    'manual_gap_tolerated', 'attitude_estimate_available', 'attitude_estimate',
    'estimation_status', 'pose_estimate', 'pose_continuity', 'pos', 'vel', 'rpy',
    'gyro_bias', 'nis', 'imu_stale', 'imu',
}


def _close_run(run):
    if run is None:
        return
    run.stop_log()
    if run.csv_writer is not None:
        run.csv_writer.close()
    if run.recording is not None:
        run.recording.close()


def _worker(connection, kind, settings):
    # Import applies numeric thread policy inside this optional owner, before
    # NumPy initializes. Control does not import this estimator dependency tree.
    import rover_main

    service = rover_main.imu_service.IMUService(kind)
    run = None
    history = deque(maxlen=config.ESTIMATION_WHEEL_HISTORY)
    revision = settings.pop('_estimation_revision', 1)
    command = SimpleNamespace(forward=0., turn=0., lift=0., pan=0., stale=True, estop=True)
    raw_command = SimpleNamespace(forward=None, turn=None, lift=None, pan=None,
                                  stale=None, estop=None)
    pico = SimpleNamespace(state=lambda: None)

    def new_run(values):
        source_path = values.get('log_path')
        path = None
        metadata = copy.deepcopy(values.get('recording_metadata') or {})
        metadata.update(role='estimator', control_run_id=values['run_id'])
        if source_path and values.get('log_requested', True):
            safe_id = ''.join(c for c in values['run_id'] if c.isalnum() or c in '-_')
            directory = Path(source_path).absolute().parent / 'estimation' / safe_id
            directory.mkdir(parents=True, exist_ok=True)
            path = str(directory / Path(source_path).name)
        options = values.get('estimator_options') or {}
        return rover_main.Run(path, metadata if path else None,
            run_id=values['run_id'], t_start=values['t_start'],
            imu_source=values.get('imu_source', kind), wheel_source=values.get('wheel_source', 'encoders'),
            calibration_profile=values.get('calibration_profile'),
            **{key: options[key] for key in ('position_integration', 'wheel_distance_scale') if key in options})

    try:
        run = new_run(settings)
        service.start()
        next_output = 0.
        next_tick = time.monotonic()
        last_pair_stamp = None
        last_source_stamp = None
        while True:
            for _ in range(config.ESTIMATION_WHEEL_HISTORY):
                if not connection.poll():
                    break
                packet = connection.recv()
                if packet.get('kind') == 'stop':
                    return
                if packet.get('kind') == 'configure':
                    values = packet['settings']
                    revision = packet['revision']
                    if values['run_id'] != run.run_id:
                        _close_run(run)
                        run = new_run(values)
                        history.clear()
                        last_pair_stamp = None
                        last_source_stamp = None
                    elif not values.get('log_requested', True):
                        run.stop_log()
                    settings = values
                elif (packet.get('kind') == 'wheels' and packet.get('run_id') == run.run_id
                      and packet.get('revision') == revision):
                    if not history or packet['timestamp'] > history[-1]['timestamp']:
                        history.append(packet)
            snapshot = service.poll()
            polled_at = time.monotonic()
            if snapshot.get('samples'):
                last_source_stamp = snapshot['samples'][-1]['timestamp']
            output_snapshot = snapshot
            rejected = False
            if not snapshot.get('samples'):
                run.record_sensors(None, snapshot, raw_command,
                                   dict(estimation_only=True, pairing_reason='no_delivered_samples'))
                run.update_imu_state(snapshot)
            for sample in snapshot.get('samples', []):
                stamp = sample['timestamp']
                preceding = next((item for item in reversed(history) if item['timestamp'] <= stamp), None)
                paired = (preceding is not None and preceding['wheel_reading'] is not None
                          and 0 <= stamp - preceding['timestamp'] <= getattr(config, 'ESTIMATION_WHEEL_MAX_AGE_S', .05))
                wheels = preceding['wheel_reading'] if paired else None
                # Operator commands remain in the parent control journal. This
                # owner has no command observation. Preserve raw inputs before
                # processing so a rejected interval does not erase its evidence.
                run.record_sensors(wheels, dict(snapshot, samples=[sample]), raw_command,
                                   dict(estimation_only=True, paired_wheel_timestamp_s=(preceding['timestamp'] if paired else None)))
                if rejected:
                    continue
                if wheels is None:
                    last_pair_stamp = None
                    run.pause_imu('No fresh preceding wheel sample; IMU interval unpaired')
                else:
                    try:
                        # Acquisition already validated these ordered intervals.
                        # Delayed delivery is not a missing-motion barrier; keep
                        # its original status/timestamps in the raw record above.
                        state_snapshot = dict(snapshot, samples=[sample])
                        if state_snapshot['status'] == 'delayed':
                            state_snapshot.update(status='ok', error=None)
                        run.update_imu_state(state_snapshot)
                        run.integrate_imu(wheels, sample, 1. / rover_main.config.EKF_RATE_HZ)
                        last_pair_stamp = preceding['timestamp']
                    except ValueError as exc:
                        reason = 'Odometry input rejected: ' + str(exc)
                        service.invalidate(reason)
                        run.pause_imu(reason)
                        last_pair_stamp = None
                        rejected = True
                        output_snapshot = dict(snapshot, status='resyncing', error=reason, samples=[])
                        if run.recording is not None:
                            run.recording.event(dict(subsystem='estimator', kind='interval_rejected',
                                reason=reason, imu_timestamp_s=stamp, raw_input_preserved=True))
            run.cycles += 1
            now = time.monotonic()
            if now >= next_output:
                # A long computation cannot renew old acquisition timestamps.
                # Recompute source age at publication, then again in poll().
                source_stamp = last_source_stamp
                source_age = now - source_stamp if source_stamp is not None else None
                paired_interval = _paired_interval(source_stamp, last_pair_stamp)
                output_snapshot = dict(output_snapshot, age_s=source_age,
                                       paired_wheel_interval_s=paired_interval)
                wheels = history[-1]['wheel_reading'] if history and history[-1]['wheel_reading'] is not None else []
                full = rover_main.build_telemetry(run, now - run.t_start, wheels, command,
                    0., 0., 0., 0., [], 'off', output_snapshot, None, pico, None, None)
                compact = {key: output_snapshot[key] for key in
                           ('status', 'error', 'attempt', 'generation', 'age_s', 'alignment_epoch', 'recovery', 'stats')}
                recording = run.recording_state()
                reply = dict(run_id=run.run_id, revision=revision, produced_monotonic_s=now,
                    source_poll_monotonic_s=polled_at, source_sample_monotonic_s=source_stamp,
                    source_age_s=source_age, paired_wheel_monotonic_s=last_pair_stamp,
                    snapshot=compact, telemetry={key: full[key] for key in _ESTIMATOR_FIELDS if key in full},
                    context=rover_main.jumper_context(run, output_snapshot), estimator_recording=recording)
                reply['paired_wheel_interval_s'] = paired_interval
                if source_age is None or not 0 <= source_age <= config.IMU_SAMPLE_MAX_AGE_S:
                    reply = _masked(reply, 'stale', 'IMU observation delayed; pose unavailable', source_age,
                                    stale_reason='imu_age')
                elif paired_interval is None or not 0 <= paired_interval <= config.ESTIMATION_WHEEL_MAX_AGE_S:
                    reply = _masked(reply, 'stale', 'IMU interval has no fresh preceding wheel observation', source_age,
                                    stale_reason='wheel_pair')
                elif output_snapshot['status'] != 'ok':
                    reply = _masked(reply, 'stale', 'IMU acquisition unavailable', source_age,
                                    stale_reason='acquisition')
                restrict_precision(reply, source_age, paired_interval, result_age=0.)
                if run.recording is not None and run.log_requested:
                    run.recording.record_telemetry(dict(reply['telemetry'], run_id=run.run_id,
                        t=now - run.t_start, estimation_only=True,
                        source_sample_monotonic_s=source_stamp, source_age_s=source_age,
                        paired_wheel_monotonic_s=last_pair_stamp))
                connection.send(reply)
                next_output = max(next_output + 1. / config.ESTIMATION_OUTPUT_HZ, now)
            next_tick += 1. / config.ESTIMATION_POLL_HZ
            pause = next_tick - time.monotonic()
            if pause > 0:
                time.sleep(pause)
            else:
                next_tick = time.monotonic()
    except BaseException as exc:
        try:
            connection.send(dict(kind='fatal', error=type(exc).__name__ + ': ' + str(exc)))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        cleanup_error = None
        try:
            service.close()
        except Exception as exc:
            cleanup_error = type(exc).__name__ + ': ' + str(exc)
        try:
            _close_run(run)
        except Exception as exc:
            cleanup_error = (cleanup_error + '; ' if cleanup_error else '') + type(exc).__name__ + ': ' + str(exc)
        try:
            connection.send(dict(kind='closed', run_id=run.run_id if run else settings['run_id'],
                                 revision=revision, error=cleanup_error,
                                 estimator_recording=run.recording_state() if run else None))
        except (BrokenPipeError, EOFError, OSError):
            pass
