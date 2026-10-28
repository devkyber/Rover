"""
The rover program. Runs on the Jetson, needs no screen.

    python rover_main.py                          # motors + IMU + camera + log
    python rover_main.py --tag square             # log named logs/<date>_<time>_square.csv
    python rover_main.py --no-motors --imu fake   # nothing plugged in (camera still on)
    python rover_main.py --pico fake              # no Pico: simulate the servos and battery

Then on the station laptop:  python station/station.py --rover <this machine's address>

Every 10 ms the control loop does, in this order:

    station command -> read wheels -> copy optional estimate -> drive -> telemetry

IMU acquisition, filtering and CSV recording belong to an optional process.
The control loop only offers bounded wheel observations and reads a cached view;
it never waits for IMU initialization, recovery, calculation or storage.

The camera runs beside the loop in its own thread (camera.py), the Pico's
servos and battery monitor in theirs (pico_link.py), and the station
connections in theirs (station_link.py). The loop itself never
waits on the network: a stalled Wi-Fi link cannot delay a motor command.

A RUN starts with a fresh filter. After completed alignment, input interruptions
retain the prior in a new, explicitly degraded log segment. Missing motion is
not reconstructed. Neither changes the run clock or other services. The station's
NEW RUN button ends the current run and starts another, so a new test
needs no SSH and no restart. Keep the rover still for 5 s to initialize
attitude. Manual driving stays available while the IMU is unavailable or
aligning. Faults invalidate the pose; raw/status recording remains independent.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import math
import os
from pathlib import Path
import signal
import socket
import sys
import time
import uuid

import config

# Apply the runtime policy before NumPy first initializes its BLAS backend.
# An embedding process that already imported NumPy must set its own thread
# limits before that import; changing the environment here cannot reconfigure it.
for _thread_variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS',
                         'MKL_NUM_THREADS', 'BLIS_NUM_THREADS'):
    os.environ[_thread_variable] = str(config.NUMERIC_THREADS)
# This requests fairer Python scheduling; it is not a real-time deadline.
sys.setswitchinterval(config.PYTHON_SWITCH_INTERVAL_S)

import numpy as np

import camera
import video_recording
import imu_service
import estimation_process
from loop_timing import LoopTiming
import logger
import telemetry_log
import motors
import odometry
import odometry_calibration
import pico_link
import hammer
import hammer_lift
import jumper_service
import scouts
import scout_plot_feed
import mission_map
import station_link

# The live NIS mean is taken over this many recent samples (3 s at 100 Hz):
# long enough to be steady, short enough to react while still driving.
NIS_WINDOW = 300


class Run:
    """One run clock, with separate recording/filter segments after IMU gaps."""

    def __init__(self, log_path, recording_metadata=None, *, imu_source='unknown', wheel_source='unknown',
                 wheel_distance_scale=None, position_integration=None, calibration_profile=None,
                 run_id=None, t_start=None):
        self.imu_source = imu_source
        self.wheel_source = wheel_source
        self.run_id = run_id or uuid.uuid4().hex
        # NEW RUN starts a fresh local frame and stationary alignment.
        # Carried-state recovery segments below identify their exact initial
        # conditions explicitly; ordinary raw replay refuses them by default.
        self.odo = odometry.Odometry(wheel_measurements_available=self.wheel_source == 'encoders',
                                     wheel_distance_scale=wheel_distance_scale,
                                     position_integration=position_integration)
        # Resolve once so a reset/NEW RUN cannot pick up a changed default or
        # lose an explicitly selected calibration. Calibration affects only
        # estimation; raw wheel measurements and drive commands stay raw.
        self.estimator_options = dict(wheel_measurements_available=self.odo.wheel_measurements_available,
                                      position_integration=self.odo.position_integration,
                                      wheel_distance_scale=self.odo.wheel_distance_scale)
        self.calibration_profile = copy.deepcopy(calibration_profile)
        self.log = None
        self.log_requested = bool(log_path)
        self.pending_log_path = log_path
        self.log_directory = os.path.dirname(log_path) if log_path else None
        self.imu_generation = None
        self.imu_alignment_epoch = None
        self.imu_reader_attempt = None
        self.imu_active = False
        self.pose_frame_id = uuid.uuid4().hex
        self.pose_degraded = False
        self.pose_quality_reason = None
        self.pose_loss_count = 0
        self.gap_budget = odometry.GapErrorBudget(position_available=self.wheel_source == 'encoders')
        self._last_motion_input = None
        self.last_pose_loss = None
        self._recovery_snapshot = None
        self._recovery_anchor_host = None
        self._last_integrated_host = None
        self._previous_log = None
        self.csv_writer = None
        self._csv_segment = 0
        self.loop_timing = LoopTiming()
        self.estimation_view = None
        self._estimator_sources = {}
        for name in ('odometry.py', 'odometry_calibration.py', 'eskf.py', 'config.py', 'rover_main.py', 'imu_service.py', 'imu_process.py', 'logger.py', 'estimation_process.py', 'loop_timing.py'):
            data = Path(__file__).with_name(name).read_bytes()
            self._estimator_sources[name] = {'sha256': hashlib.sha256(data).hexdigest(),
                                             'text': data.decode('utf-8')}
        self.t_start = time.monotonic() if t_start is None else float(t_start)
        self.t_prev = 0.0
        self.cycles = 0
        self.dropped = 0            # cycles with no answer from the motor bus
        self.recording = (telemetry_log.SessionRecorder(
            self.log_directory, self.run_id,
            dict(recording_metadata, initial_imu_csv=log_path,
                 estimator_sources=self._estimator_sources,
                 estimator_options=dict(self.estimator_options),
                 odometry_calibration_profile=self.calibration_profile))
            if log_path and recording_metadata is not None else None)
        self._recorded_imu_identity = None

    def stop_log(self, *, keep_request=False):
        try:
            if self.log:
                self._previous_log = self.log.filename
                self.log.close()
                self.log = None
        finally:
            if not keep_request:
                self.log_requested = False
                if self.recording:
                    self.recording.close(wait=False)
                if self.csv_writer:
                    self.csv_writer.close(wait=False)

    def recording_state(self):
        state = self.recording.state() if self.recording else None
        if state is not None and self.csv_writer is not None:
            csv_state = self.csv_writer.state()
            state['csv'] = csv_state
            if csv_state['error']:
                state.update(status='error', error=csv_state['error'])
            state['dropped'] += csv_state['dropped']
            state['failed'] += csv_state['failed']
        return state

    def new_run(self, log_path, recording_metadata=None):
        """A fresh origin keeps this process's explicit estimator selection."""
        return Run(log_path, recording_metadata, imu_source=self.imu_source,
                   wheel_source=self.wheel_source,
                   wheel_distance_scale=self.estimator_options['wheel_distance_scale'],
                   position_integration=self.estimator_options['position_integration'],
                   calibration_profile=self.calibration_profile)

    def record_sensors(self, wheels, imu, command, outputs):
        """Preserve each delivered batch before any estimation rejection.

        Metadata is written once per acquisition identity; no copying a source
        bundle into every 100 Hz row. IMU/encoder failures remain explicit nulls.
        """
        if self.recording is None or not self.log_requested:
            return
        identity = (imu.get('generation'), imu.get('attempt'), imu.get('alignment_epoch'))
        if identity != self._recorded_imu_identity:
            if self.recording.event(dict(subsystem='rover', kind='imu_metadata',
                                         identity=identity, metadata=imu.get('metadata'))):
                self._recorded_imu_identity = identity
        self.recording.sensors(dict(
            run_elapsed_s=time.monotonic() - self.t_start, cycle=self.cycles,
            wheel_reading=wheels,
            imu={key: value for key, value in imu.items() if key != 'metadata'},
            imu_metadata_identity=identity,
            imu_metadata_logged=identity == self._recorded_imu_identity,
            command={key: getattr(command, key) for key in
                     ('forward', 'turn', 'lift', 'pan', 'stale', 'estop')},
            outputs=outputs))

    def pause_imu(self, reason='Input interval unavailable; aligned estimate held'):
        if self.imu_active and self.odo.ready:
            self._recovery_anchor_host = None
            if self._recovery_snapshot is None:
                self._retain_prior(dict(status='resyncing', error=reason, samples=[],
                    generation=self.imu_generation, alignment_epoch=self.imu_alignment_epoch,
                    attempt=self.imu_reader_attempt, metadata=None, recovery=None))
            self.gap_budget.unknown = True
            return
        self.stop_log(keep_request=True)
        if self.imu_active:
            self.odo = odometry.Odometry(**self.estimator_options)
            self.pose_frame_id = uuid.uuid4().hex
        self.imu_active = False
        self._recovery_snapshot = None
        self._last_motion_input = None
        if self.pose_degraded:
            self.gap_budget.unknown = True

    def begin_imu(self, snapshot):
        self.stop_log(keep_request=True)
        self.odo = odometry.Odometry(**self.estimator_options)
        self.pose_frame_id = uuid.uuid4().hex
        self.imu_generation = snapshot['generation']
        self.imu_alignment_epoch = snapshot.get('alignment_epoch')
        self.imu_reader_attempt = snapshot.get('attempt')
        self.imu_active = True
        self._recovery_snapshot = None
        self._last_motion_input = None
        if self.pose_degraded:
            self.gap_budget.unknown = True
            self.odo.use_zupt = False
        self._open_imu_log(snapshot)

    def _open_imu_log(self, snapshot, checkpoint=None):
        if self.log_requested:
            self._csv_segment += 1
            path = self.pending_log_path or logger.new_log_path(
                f'imu_recovered_{self.run_id[:8]}_{self._csv_segment:05d}', self.log_directory)
            metadata = dict(snapshot['metadata'] or {}, imu_generation=self.imu_generation,
                            imu_alignment_epoch=self.imu_alignment_epoch,
                            imu_reader_attempt=self.imu_reader_attempt,
                            run_id=self.run_id, pose_frame_id=self.pose_frame_id,
                            estimator_sources=self._estimator_sources,
                            input_sources={'imu': self.imu_source, 'wheels': self.wheel_source},
                            estimator_options=dict(self.estimator_options),
                            odometry_calibration_profile=self.calibration_profile,
                            replay_policy='restore estimator_options and source/config identity; disabled wheels are placeholders, never zero-motion measurements',
                            pose_degraded=self.pose_degraded,
                            pose_quality_reason=self.pose_quality_reason,
                            pose_loss_count=self.pose_loss_count,
                            estimator_runtime_policy={'use_zupt': self.odo.use_zupt,
                                                      'reason': self.pose_quality_reason},
                            manual_gap_budget=self.gap_budget.snapshot(),
                            prior_loss=dict(self.last_pose_loss) if self.last_pose_loss else None,
                            pose_quality='degraded_gap' if self.pose_degraded else 'initial_alignment_required',
                            scientific_pose_valid=False,
                            scientific_validity_policy='requires measured IMU and wheel inputs, fresh samples and completed alignment; degraded remains invalid until NEW RUN; accuracy is not certified',
                            run_clock='continuous across recovery segments; timestamp column does not restart',
                            run_log_origin=self.log_directory,
                            recovery_policy='fresh alignment and log after IMU gap; manual drive independent')
            if checkpoint is not None:
                metadata.update(
                    estimator_initial_state=checkpoint,
                    previous_segment=self._previous_log,
                    pose_quality='degraded_gap', scientific_pose_valid=False,
                    lost_interval=dict(self.last_pose_loss),
                    replay_policy='restore estimator_initial_state before first row; never start this segment at origin',
                    recovery_policy='retain once-aligned prior across all input interruptions; integrate fresh explicit intervals only; missing motion unknown')
            if self.csv_writer is None:
                self.csv_writer = logger.AsyncCSVWriter()
            self.log = self.csv_writer.open(path, metadata=metadata)
            self.pending_log_path = None

    def _can_retain_prior(self, snapshot):
        # Operator policy: only NEW RUN discards a completed alignment. Reader
        # restarts/clock resets are measurement barriers, not new local origins.
        return bool(self.imu_active and self.odo.ready and self.imu_generation is not None)

    def _retain_prior(self, snapshot):
        self._recovery_anchor_host = None
        first_reason = ((self.last_pose_loss or {}).get('initial_reason') or
                        (self.last_pose_loss or {}).get('reason')) if self._recovery_snapshot is not None else None
        first_identity = (self.last_pose_loss or {}).get('initial_identity') if self._recovery_snapshot is not None else None
        self.stop_log(keep_request=True)
        self.odo.interrupt_interval()
        recovery = snapshot.get('recovery') or {}
        reason = recovery.get('reason') or snapshot.get('error') or 'IMU acquisition identity changed; aligned estimate held'
        qualified = bool(recovery.get('can_resume') is True
                         and snapshot.get('alignment_epoch') == self.imu_alignment_epoch
                         and snapshot.get('attempt') == self.imu_reader_attempt
                         and recovery.get('generation') == snapshot['generation']
                         and recovery.get('alignment_epoch') == self.imu_alignment_epoch
                         and recovery.get('reader_attempt') == self.imu_reader_attempt)
        if (not qualified or snapshot['generation'] - self.imu_generation != 1
                or not self.gap_budget.timing_loss(reason)):
            # A latest benign reason cannot account for missed generations or
            # hide an earlier saturated/unclassified interval before resumption.
            self.gap_budget.unknown = True
        self.pose_degraded = True  # Only a new Run clears this scientific-validity latch.
        self.pose_quality_reason = reason
        self.pose_loss_count += max(1, snapshot['generation'] - self.imu_generation)
        self.imu_generation = snapshot['generation']
        self.imu_alignment_epoch = snapshot.get('alignment_epoch')
        self.imu_reader_attempt = snapshot.get('attempt')
        self.last_pose_loss = dict(recovery,
                                  generation=self.imu_generation,
                                  alignment_epoch=self.imu_alignment_epoch,
                                  reader_attempt=self.imu_reader_attempt,
                                  can_resume=qualified, reason=reason,
                                  initial_reason=first_reason or reason,
                                  initial_identity=first_identity or {
                                      'generation': self.imu_generation,
                                      'alignment_epoch': self.imu_alignment_epoch,
                                      'reader_attempt': self.imu_reader_attempt},
                                  alignment_retained=True,
                                  last_integrated_host_s=self._last_integrated_host,
                                  first_resumed_host_s=None,
                                  unintegrated_host_interval_s=None,
                                  uncertainty_bound=None)
        self._recovery_snapshot = snapshot

    def manual_gap_tolerated(self):
        return bool(self.pose_degraded and self._recovery_snapshot is None
                    and self.gap_budget.snapshot()['within_budget'])

    def update_imu_state(self, snapshot):
        """Keep completed alignment across interruptions until explicit NEW RUN."""
        # Consumer age is not lost acquisition. The optional owner verifies
        # historical wheel pairing for every delivered interval; precision is
        # evaluated at publication without latching a false continuity loss.
        identity_changed = (self.imu_generation != snapshot['generation']
                            or self.imu_alignment_epoch != snapshot.get('alignment_epoch')
                            or self.imu_reader_attempt != snapshot.get('attempt'))
        if (self._can_retain_prior(snapshot)
                and (identity_changed or (snapshot['status'] not in ('ok', 'delayed')
                                          and self._recovery_snapshot is None))):
            self._retain_prior(snapshot)
        if (self._recovery_snapshot is not None
                and self._can_retain_prior(snapshot)):
            # Keep the retained prior unavailable until a fresh batch has
            # actually passed Odometry.step. No five-second re-alignment.
            self._recovery_snapshot = snapshot
            return
        if snapshot['status'] not in ('ok', 'delayed'):
            self.pause_imu()
        elif snapshot['status'] == 'ok' and (
                not self.imu_active or self.imu_generation != snapshot['generation']):
            self.begin_imu(snapshot)
        # Delayed acquisition with no delivered interval retains the prior.
        # Validated delayed batches are processed explicitly by the owner.

    def integrate_imu(self, wheels, sample, fallback_dt):
        """Apply one accepted interval; never bridge a missing interval."""
        if sample['timestamp'] < self.t_start:
            return False
        recovering = self._recovery_snapshot is not None
        checkpoint = None
        if recovering:
            if 'dt' not in sample or 'gyro_samples' not in sample:
                if self.imu_source not in ('fake', 'arduino'):
                    raise ValueError('Recovery requires an explicit fresh FIFO interval')
                stamp = float(sample['timestamp'])
                if not math.isfinite(stamp):
                    raise ValueError('Recovery anchor requires a finite acquisition timestamp')
                for key in ('accel', 'gyro'):
                    vector = np.asarray(sample[key], dtype=float)
                    if vector.shape != (3,) or not np.isfinite(vector).all():
                        raise ValueError('Recovery anchor requires finite IMU vectors')
                if 'dt' in sample and (not math.isfinite(float(sample['dt'])) or not 0 < float(sample['dt']) <= config.IMU_SAMPLE_MAX_AGE_S):
                    raise ValueError('Invalid recovery sample interval')
                anchor = self._recovery_anchor_host
                if anchor is None:
                    self._recovery_anchor_host = stamp
                    return False  # First post-gap observation is an anchor, never gap motion.
                interval = stamp - anchor
                if interval <= 0:
                    raise ValueError('Recovery acquisition timestamp did not advance')
                if interval > config.IMU_SAMPLE_MAX_AGE_S:
                    self._recovery_anchor_host = stamp
                    return False  # Start a new short observed pair after another delay.
                sample = dict(sample, dt=interval, gyro_samples=[list(sample['gyro'])])
            checkpoint = self.odo.recovery_checkpoint()
        sample_t = sample['timestamp'] - self.t_start
        step = sample_t - self.t_prev if self.odo.cycles else fallback_dt
        interval = sample.get('dt', step if 0 < step < .5 else fallback_dt)
        before_bias = self.odo.gyro_bias
        self.odo.step(wheels, gyro=sample['gyro'], accel=sample['accel'],
                      dt=interval, gyro_samples=sample.get('gyro_samples'))
        self.t_prev = sample_t
        self._last_integrated_host = sample['timestamp']
        if recovering:
            previous_host = self.last_pose_loss['last_integrated_host_s']
            self.last_pose_loss['first_resumed_host_s'] = sample['timestamp']
            self.last_pose_loss['unintegrated_host_interval_s'] = (
                max(0.0, sample['timestamp'] - interval - previous_host)
                if previous_host is not None else None)
            previous = self._last_motion_input
            duration = (sample['timestamp'] - interval - previous_host
                        if previous_host is not None else None)
            if previous is None or duration is None:
                self.gap_budget.unknown = True
            else:
                self.gap_budget.charge(duration, previous['gyro'], sample['gyro_samples'][0], before_bias,
                                       previous['speed'], odometry.body_velocity(
                                           wheels, distance_scale=self.odo.wheel_distance_scale),
                                       self.last_pose_loss['reason'],
                                       timestamp_uncertainties=(previous['timing_uncertainty_s'],
                                           sample.get('fifo_timestamp_uncertainty_s', 0.0)))
            self.last_pose_loss['manual_gap_budget'] = self.gap_budget.snapshot()
        # Public displacement can be a short chord of a curved FIFO interval.
        # Charge heading uncertainty against accepted wheel travel; Odometry
        # owns the alignment/no-wheel/ZUPT gating and interval rollback.
        self.gap_budget.advance(self.odo.last_wheel_travel_m)
        self._last_motion_input = {'gyro': list(sample.get('gyro_samples', [sample['gyro']])[-1]),
                                   'speed': odometry.body_velocity(wheels, distance_scale=self.odo.wheel_distance_scale),
                                   'timing_uncertainty_s': sample.get('fifo_timestamp_uncertainty_s', 0.0)}
        if recovering:
            self._open_imu_log(self._recovery_snapshot, checkpoint)
            self._recovery_snapshot = None
            self._recovery_anchor_host = None
        if self.log:
            self.log.log(sample_t, wheels, sample)
        return True

    def estimate_held(self, imu):
        return bool(self.imu_active and self.odo.ready and not self.estimate_available(imu))

    def held_estimate(self, imu):
        if not self.estimate_held(imu):
            return None
        return dict(pos=self.odo.position.tolist(), vel=self.odo.velocity.tolist(),
                    rpy=self.odo.rpy_deg.tolist(), gyro_bias=np.degrees(self.odo.gyro_bias).tolist(),
                    gyro_bias_unit='deg/s', position_available=self.wheel_source == 'encoders',
                    last_integrated_host_s=self._last_integrated_host,
                    age_s=(max(0., time.monotonic() - self._last_integrated_host)
                           if self._last_integrated_host is not None else None),
                    reason=imu.get('error') or (self.last_pose_loss or {}).get('reason') or 'Waiting for fresh IMU interval',
                    current_measurement=False, missing_motion=True, frame_id=self.pose_frame_id)

    def estimate_available(self, imu):
        return bool(self.imu_active and imu['status'] == 'ok' and self.odo.ready
                    and self._recovery_snapshot is None)

    def scientific_pose_valid(self, imu):
        precision = estimation_process.precision_status(
            imu.get('age_s'), imu.get('paired_wheel_interval_s', 0.))
        return self.position_estimate_available(imu) and not self.pose_degraded and precision['fresh']

    def attitude_estimate_available(self, imu):
        return self.estimate_available(imu) and self.imu_source in ('bmi088', 'arduino')

    def position_estimate_available(self, imu):
        return self.attitude_estimate_available(imu) and self.wheel_source == 'encoders'

    def pose_quality(self, imu):
        if self.pose_degraded:
            return 'degraded_gap'
        if imu['status'] == 'delayed':
            return 'stale'
        if not self.imu_active or imu['status'] != 'ok':
            return 'unavailable'
        if not self.odo.ready:
            return 'aligning'
        if self.imu_source not in ('bmi088', 'arduino'):
            return 'simulation' if self.imu_source == 'fake' else 'unverified_sources'
        return 'local_odometry' if self.wheel_source == 'encoders' else 'attitude_only'


def describe_rover(args, camera_on, estimator_settings=None):
    """Sent once to each station when it connects: what this rover is."""
    estimator_settings = estimator_settings or {}
    return {
        # Changes when this program restarts, so a station that reconnects
        # can tell "same rover, link came back" from "rover was restarted".
        "boot": round(time.time(), 3),
        "loop_hz": config.EKF_RATE_HZ,
        "wheels": [{"id": i, "name": name, "side": side}
                   for i, name, side in config.WHEELS],
        "max_speed_mps": config.COMMAND_WHEEL_SPEED_LIMIT_MPS,
        "track_m": config.WHEELBASE_M,
        "wheel_radius_m": config.WHEEL_RADIUS_M,
        "odometry": {
            "position_integration": estimator_settings.get('position_integration', config.ODOMETRY_POSITION_INTEGRATION),
            "wheel_distance_scale": estimator_settings.get('wheel_distance_scale', config.WHEEL_DISTANCE_SCALE),
            "calibration_profile": estimator_settings.get('calibration_profile'),
            "scope": "estimation only; wheel_speed and command limits use raw encoders and configured radius",
        },
        "telemetry_hz": config.TELEMETRY_HZ,
        "command_timeout_s": config.COMMAND_TIMEOUT_S,
        "alignment_s": config.STARTUP_ALIGNMENT_SECONDS,
        "motors": not args.no_motors,
        "imu": args.imu,
        "camera": ({"width": config.CAMERA_WIDTH, "height": config.CAMERA_HEIGHT,
                    "fps": config.CAMERA_FPS} if camera_on else None),
        "pico": args.pico,                                       # "usb" | "fake" | "off"
        "jumper": {"host": next(iter(scouts.scout_hosts(getattr(args, 'jumper', None))), None)},
        "scouts": scouts.description(getattr(args, 'jumper', None)),
        "scout_postshot": {"max_import_bytes": config.SCOUT_REPORT_MAX_IMPORT_BYTES,
                           "capture_s": config.SCOUT_POSTSHOT_CAPTURE_S,
                           "max_telemetry_rows": config.SCOUT_POSTSHOT_MAX_TELEMETRY_ROWS},
        "battery": {"cells": config.BATTERY_CELLS, "low_v": config.BATTERY_LOW_V},
    }


def build_telemetry(run, t, wheels, command, forward, turn, lift, pan, raw_sent,
                    motor_state, imu, cam, pico, temp_c, wifi_dbm):
    """Everything the station shows, as plain numbers."""
    odo = run.odo
    r3 = lambda v: [round(float(x), 3) for x in v]

    nis = {}
    for name, samples in odo.nis.items():
        window = samples[-NIS_WINDOW:]
        if window:
            nis[name] = [round(float(np.mean(window)), 3),
                         config.NIS_DOF.get(name, 1), len(samples)]

    pose_valid = run.scientific_pose_valid(imu)
    estimate_available = run.position_estimate_available(imu)
    attitude_available = run.attitude_estimate_available(imu)
    state = "moving" if any(abs(v) > 0.005 for v in odometry.wheel_speeds_mps(wheels)) else "stopped"

    return {
        "type": "telemetry",
        "t": round(t, 3),
        "run_id": run.run_id,
        "odom_frame_id": run.pose_frame_id,
        "state": state,
        "align": round(float(odo.alignment_progress), 3),
        "pose_valid": pose_valid,
        "pose_estimate_available": estimate_available,
        "estimate_held": run.estimate_held(imu),
        "held_estimate": run.held_estimate(imu),
        "pose_frame_id": run.pose_frame_id,
        "pose_quality": run.pose_quality(imu),
        "pose_quality_reason": run.pose_quality_reason,
        "manual_gap_tolerated": run.manual_gap_tolerated(),
        "attitude_estimate_available": attitude_available,
        "attitude_estimate": ({
            "rpy": [round(float(x), 2) for x in odo.rpy_deg],
            "gyro_bias": r3(np.degrees(odo.gyro_bias)),
            "degraded": run.pose_degraded and not run.manual_gap_tolerated(),
        } if attitude_available else None),
        "estimation_status": {
            "imu_source": run.imu_source, "wheel_source": run.wheel_source,
            "position_source": "wheel_speed_and_eskf_attitude" if estimate_available else None,
            "position_integration": odo.position_integration,
            "wheel_distance_scale": odo.wheel_distance_scale,
            "calibration_profile": copy.deepcopy(run.calibration_profile),
            "heading_reference": "local_start_direction",
            "absolute_position_available": False,
            "position_uncertainty_m": None,
            "accuracy_validated": False,
            "internal_covariance_scope": "internal ESKF state; not public path uncertainty",
            "diagnostics": odo.diagnostics(),
        },
        "pose_estimate": ({
            "pos": [round(float(x), 4) for x in odo.position],
            "vel": r3(odo.velocity),
            "rpy": [round(float(x), 2) for x in odo.rpy_deg],
            "gyro_bias": r3(np.degrees(odo.gyro_bias)),
        } if estimate_available else None),
        "pose_continuity": {
            "degraded": run.pose_degraded, "loss_count": run.pose_loss_count,
            "last_loss": copy.deepcopy(run.last_pose_loss), "missing_motion": run.pose_loss_count > 0,
            "quality_reason": run.pose_quality_reason,
            "uncertainty_bound": None,
            "manual_gap_budget": run.gap_budget.snapshot(),
        },
        "pos": [round(float(x), 4) for x in odo.position] if pose_valid else [None] * 3,
        "vel": r3(odo.velocity) if pose_valid else [None] * 3,
        "rpy": [round(float(x), 2) for x in odo.rpy_deg] if pose_valid else [None] * 3,
        "gyro_bias": r3(np.degrees(odo.gyro_bias)) if pose_valid else [None] * 3,
        "wheel_speed": r3(odometry.wheel_speeds_mps(wheels)),    # raw encoder/radius m/s, + = forward
        "wheel_speed_source": "raw_encoder_velocity_with_configured_radius",
        "estimator_wheel_speed": r3(odometry.wheel_speeds_mps(wheels, distance_scale=odo.wheel_distance_scale)),
        "wheel_current": r3(w['current_A'] for w in wheels),     # A
        "cmd": {"forward": round(forward, 3), "turn": round(turn, 3),
                "lift": round(lift, 3), "pan": round(pan, 3),
                "sent": list(raw_sent),
                "stale": command.stale, "estop": command.estop},
        "loop": {"hz": round(run.cycles / max(t, 1e-6), 1),
                 "cycles": run.cycles, "dropped": run.dropped},
        "nis": nis if pose_valid else {},
        "motors": motor_state,                                   # "ok" | "lost" | "off"
        "imu_stale": imu['stats'].get('stale', 0),
        "imu": dict(
            {key: imu[key] for key in ('status', 'error', 'attempt', 'generation', 'age_s')},
            **{key: imu['stats'].get(key) for key in
               ('gaps', 'resyncs', 'last_gap', 'last_rejection', 'last_rejected_batch', 'slow_reads', 'delivery_delays', 'oldest_delivery_age_s', 'timing_tolerated_batches',
                'arrival_delays', 'last_arrival_age_s', 'max_arrival_age_s', 'consumer_dropped_batches',
                'acquisition_latency_s', 'max_acquisition_latency_s',
                'delivery_lag_s', 'max_delivery_lag_s',
                'fifo_timestamp_uncertainty_s', 'accel_gyro_skew_s')}),
        "camera": ({"fps": round(cam.fps, 1), "mbps": round(cam.mbps, 2),
                    "error": cam.error if isinstance(getattr(cam, 'error', None), str) else None,
                    "preview_dropped": cam.preview_dropped if type(getattr(cam, 'preview_dropped', None)) is int else 0,
                    "record_callback_errors": cam.record_errors if type(getattr(cam, 'record_errors', None)) is int else 0,
                    "delivery_errors": cam.delivery_errors if type(getattr(cam, 'delivery_errors', None)) is int else 0}
                   if cam else None),
        # status "ok" | "lost" | "off"; battery None = reading unavailable
        "pico": copy.deepcopy(pico.state()),
        "temp_c": temp_c,
        "wifi_dbm": wifi_dbm,                                    # None on a wired link
        "log": ({"file": os.path.basename(run.log.filename), "rows": run.log.count}
                if run.log else None),
        "logging_requested": run.log_requested,
        "recording": run.recording_state(),
    }


def recording_metadata(args, hello):
    """Identify the exact software/settings without opening any hardware."""
    source_root = Path(__file__).resolve().parent
    sources = {}
    for name in ('telemetry_log.py', 'pico_link.py', 'battery.py', 'hammer.py', 'hammer_lift.py',
                 'station_link.py', 'jumper_service.py', 'scouts.py', 'motors.py', 'dxl_reader.py',
                 'dxl_control.py', 'imu_reader.py', 'camera.py', 'estimation_process.py',
                 'loop_timing.py'):
        data = (source_root / name).read_bytes()
        sources[name] = dict(sha256=hashlib.sha256(data).hexdigest(), text=data.decode('utf-8'))
    return dict(arguments=vars(args), rover_description=hello, runtime_sources=sources,
                host_wall_clock_plausible=datetime.now(timezone.utc).year >= 2020,
                timing='Host UTC is unverified; use monotonic time and sequence for ordering. No device clock synchronization inferred.',
                coverage='Control session: raw wheels, received commands, applied outputs and cached estimator status each control loop; complete dashboard telemetry at telemetry rate. Estimator session under estimation/<run_id>: original delivered IMU batches with preceding wheel pairing and estimator telemetry. Camera status only, not video frames.',
                separate_jumper='Session Jumper snapshots/events/commands are separate. Original ESP CSV/binary capture remains independent under logs/jumper.')


def read_cpu_temp():
    try:
        with open(config.CPU_TEMP_FILE) as f:
            return round(int(f.read()) / 1000.0, 1)
    except (OSError, ValueError):
        return None


def jumper_context(run, imu):
    """Rover context at receipt/command time, never a scout landing position."""
    view = getattr(run, 'estimation_view', None)
    if view is not None:
        if _estimation_view_fresh(run, view, precision=True) and view.get('context'):
            return dict(view['context'])
        return dict(host_utc=datetime.now(timezone.utc).isoformat(),
                    host_monotonic_s=time.monotonic(), run_id=run.run_id,
                    run_elapsed_s=time.monotonic()-run.t_start, pose_valid=False,
                    imu_generation=imu.get('generation'), pose_frame_id=None,
                    odom_frame_id=None, pose_quality='unavailable',
                    rover_position_m=None, rover_rpy_deg=None,
                    scout_landing_position_m=None, scout_to_rover_offset_m=None,
                    position_uncertainty_m=None, esp_clock_sync=None,
                    input_sources={'imu':run.imu_source,'wheels':run.wheel_source})
    valid = run.scientific_pose_valid(imu)
    return {
        'host_utc': datetime.now(timezone.utc).isoformat(),
        'host_monotonic_s': time.monotonic(), 'run_id': run.run_id,
        'run_elapsed_s': time.monotonic() - run.t_start,
        'rover_log': run.log.filename if run.log else None,
        'pose_valid': valid, 'imu_generation': imu['generation'],
        'pose_frame_id': run.pose_frame_id, 'pose_quality': run.pose_quality(imu),
        'odom_frame_id': run.pose_frame_id,
        'pose_continuity_loss_count': run.pose_loss_count,
        'rover_position_m': [float(x) for x in run.odo.position] if valid else None,
        'rover_rpy_deg': [float(x) for x in run.odo.rpy_deg] if valid else None,
        'scout_landing_position_m': None, 'scout_to_rover_offset_m': None,
        'position_uncertainty_m': None, 'esp_clock_sync': None,
        'input_sources': {'imu': run.imu_source, 'wheels': run.wheel_source},
    }


def _estimation_view_fresh(run, view, *, precision=False):
    stamp = view.get('produced_monotonic_s')
    source_stamp = view.get('source_sample_monotonic_s')
    wheel_stamp = view.get('paired_wheel_monotonic_s')
    now = time.monotonic()
    fresh = (view.get('run_id') == run.run_id and view.get('status') == 'running'
            and all(isinstance(value, (float, int)) and math.isfinite(value)
                    for value in (stamp, source_stamp, wheel_stamp))
            and 0 <= now-stamp <= config.ESTIMATION_RESULT_MAX_AGE_S
            and 0 <= now-source_stamp <= config.IMU_SAMPLE_MAX_AGE_S
            and 0 <= source_stamp-wheel_stamp <= config.ESTIMATION_WHEEL_MAX_AGE_S)
    return fresh and (not precision or estimation_process.precision_status(
        now-source_stamp, source_stamp-wheel_stamp, result_age=now-stamp)['fresh'])


def estimation_settings(run, metadata):
    return dict(run_id=run.run_id, t_start=run.t_start,
                log_path=run.pending_log_path, recording_metadata=metadata,
                log_requested=run.log_requested, imu_source=run.imu_source,
                wheel_source=run.wheel_source, estimator_options=dict(run.estimator_options),
                calibration_profile=run.calibration_profile)


def merge_estimation(telemetry, run, view):
    """A stale/crashed optional worker can only remove estimation validity."""
    excluded = {'type','t','run_id','state','wheel_speed','wheel_current','wheel_speed_source',
                'estimator_wheel_speed','cmd','loop','motors','camera','pico','temp_c',
                'wifi_dbm','logging_requested','recording'}
    if _estimation_view_fresh(run, view):
        telemetry.update({k:v for k,v in view.get('telemetry',{}).items() if k not in excluded})
    else:
        source_view = view.get('telemetry', {}) if view.get('run_id') == run.run_id else {}
        for key in ('align', 'pose_continuity', 'estimation_status', 'imu_stale'):
            if key in source_view:
                telemetry[key] = copy.deepcopy(source_view[key])
        for key in ('pose_valid','pose_estimate_available','attitude_estimate_available','manual_gap_tolerated'):
            telemetry[key] = False
        telemetry.update(pose_quality='unavailable', pose_quality_reason=view.get('error') or 'Optional estimator unavailable',
                         pose_estimate=None, attitude_estimate=None, estimate_held=False,
                         held_estimate=None, pose_frame_id=None, odom_frame_id=None,
                         pos=[None]*3,vel=[None]*3,rpy=[None]*3,gyro_bias=[None]*3,nis={})
        telemetry['estimation_status']['position_source'] = None
        # Precision eligibility does not diagnose acquisition failure. Keep
        # the same-run source status and counters when only processing/pairing
        # is stale; never borrow source health from an older run.
        source = view.get('telemetry', {}).get('imu') if view.get('run_id') == run.run_id else None
        telemetry['imu'] = (dict(source) if isinstance(source, dict) else
            dict(status='unavailable', error=view.get('error') or 'Optional estimator unavailable',
                 generation=None, attempt=None, age_s=None))
    source_stamp = view.get('source_sample_monotonic_s')
    if view.get('run_id') == run.run_id and isinstance(source_stamp, (float, int)):
        telemetry['imu'] = estimation_process.source_health(telemetry['imu'], time.monotonic()-source_stamp)
        paired_interval = estimation_process._paired_interval(source_stamp, view.get('paired_wheel_monotonic_s'))
        estimation_process.restrict_precision(dict(telemetry=telemetry),
            time.monotonic()-source_stamp, paired_interval,
            result_age=(time.monotonic()-view['produced_monotonic_s']
                        if isinstance(view.get('produced_monotonic_s'), (float, int)) else None))
    telemetry['estimation_worker'] = {k:view.get(k) for k in (
        'status', 'error', 'produced_monotonic_s', 'input_dropped', 'source_age_s',
        'source_sample_monotonic_s', 'paired_wheel_monotonic_s', 'paired_wheel_interval_s',
        'stale_reason')}
    if telemetry.get('recording') is not None:
        child_recording = view.get('estimator_recording')
        telemetry['recording']['estimator'] = child_recording
        if isinstance(child_recording,dict) and (child_recording.get('error') or child_recording.get('dropped')):
            telemetry['recording'].update(status='degraded',error=child_recording.get('error') or 'Estimator recording gaps')


def read_wifi_dbm():
    """
    How strongly the rover hears the router, in dBm (-50 strong, -80 weak).
    None if there is no Wi-Fi link.
    """
    try:
        with open(config.WIFI_STATUS_FILE) as f:
            lines = f.read().splitlines()[2:]        # two header lines, then one per radio
        return int(float(lines[0].split()[3]))
    except (OSError, ValueError, IndexError):
        return None


def local_ip():
    """The address a station on the same network should connect to."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))       # no packet is sent; this only picks a route
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return socket.gethostname()


def stop_on_signal(signum, frame):
    """
    Turn `kill`, `systemctl stop` and a closed SSH window into Ctrl+C.

    By default those end Python at once, skipping the `finally` below.
    Goal Velocity latches inside the motor, so the wheels would keep
    turning at the last commanded speed with nothing left to stop them.
    """
    raise KeyboardInterrupt


def operator_control_policy(args):
    manual_profile = bool(not args.no_motors or getattr(args, 'hammer_test', False))
    allow_lift = bool(not args.no_motors or getattr(args, 'cylinder_test', False))
    return manual_profile, allow_lift


def load_odometry_options(args):
    """Validate the selected profile before opening hardware or a station port."""
    options = dict(position_integration=config.ODOMETRY_POSITION_INTEGRATION,
                   wheel_distance_scale=config.WHEEL_DISTANCE_SCALE,
                   calibration_profile=None)
    path = getattr(args, 'odometry_profile', config.WHEEL_CALIBRATION_PROFILE)
    if path is not None:
        loaded = odometry_calibration.load_profile(path, geometry={
            'wheel_radius_m': config.WHEEL_RADIUS_M,
            'wheelbase_m': config.WHEELBASE_M,
            'wheel_sign': list(config.WHEEL_SIGN)})
        options.update(wheel_distance_scale=loaded['wheel_distance_scale'],
                       calibration_profile=loaded['provenance'])
    # Resolve/validate config values even without a profile, before devices can
    # be opened. No estimator object here imports or touches a hardware driver.
    odometry.Odometry(position_integration=options['position_integration'],
                      wheel_distance_scale=options['wheel_distance_scale'])
    return options


def main(args):
    # Reject duplicate/invalid destinations before opening Rover hardware.
    scout_addresses = scouts.scout_hosts(getattr(args, 'jumper', None))
    estimator_settings = load_odometry_options(args)
    signal.signal(signal.SIGTERM, stop_on_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, stop_on_signal)

    camera_on = not args.no_camera and camera.available()
    if not args.no_camera and not camera_on:
        print("Camera off: GStreamer Python bindings not found on this machine.")

    # The link binds its port first. If another copy of this program holds
    # it, we stop here -- before torque goes on -- instead of running a
    # second rover that no station can reach.
    try:
        link = station_link.StationLink(port=args.port,
                                        hello=describe_rover(args, camera_on, estimator_settings))
    except OSError as e:
        raise SystemExit(
            f"\nCannot listen on port {args.port}: {e}\n"
            "Another rover_main.py is probably still running. Stop it first:\n"
            f"  {'taskkill /F /IM python.exe' if config.ON_WINDOWS else 'pkill -f [r]over_main.py'}\n")

    log_path = None if args.no_log else (args.log or logger.new_log_path(args.tag))
    session_metadata = dict(recording_metadata(args, link.hello), recording_role='control',
                            imu_raw_location='logs/estimation/<run_id>/; optional worker owns acquisition and estimation')
    run = Run(log_path, session_metadata, imu_source=args.imu,
              wheel_source='disabled' if args.no_motors else 'encoders', **estimator_settings)
    link.set_recorder(run.recording)
    retired_recordings = []

    mapping = mission_map.MissionMapService(
        os.path.join(os.path.dirname(os.path.dirname(__file__)),
                     'logs', config.MAPPING_LOG_SUBDIR))
    link.mapping = mapping
    # Startup can fail after torque is enabled but before the loop's finally.
    # Register each returned resource immediately; actuator stops run before
    # worker/log finalization, and a failed stop never skips the other devices.
    startup_stops = []
    startup_finalizers = [('telemetry publisher', link.close_telemetry),
                          ('mapping', mapping.close),
                          ('run log', run.stop_log)]
    if run.recording:
        startup_finalizers.append(('session recorder', run.recording.close))
    try:
        imu = estimation_process.EstimationProcess(args.imu, estimation_settings(run, session_metadata))
        startup_finalizers.insert(0, ('IMU', imu.close))
        wheels_hw = motors.open_motors(not args.no_motors)
        startup_stops.append(('wheels', wheels_hw.close))
        hammer_test, allow_lift = operator_control_policy(args)
        link.hammer.allow_lift = allow_lift
        # Wheel-connected operation shares the main thread and existing U2D2 port.
        # A standalone worker is used only when the wheel bus is disabled.
        lift_hw = (hammer_lift.TestLift(link.event, allow_motion=allow_lift) if hammer_test and not wheels_hw.connected
                   else hammer_lift.Lift(wheels_hw, link.event, test_mode=hammer_test, allow_motion=allow_lift))
        startup_stops.insert(0, ('cylinder', lift_hw.close))
        clegg = hammer.Controller(link.event, test_mode=hammer_test, allow_lift=allow_lift)
        jumper = scouts.Scouts(
            scout_addresses,
            log_dir=os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                 'logs', config.JUMPER_LOG_SUBDIR), event=link.event)
        link.jumper = jumper
        startup_stops.append(('Scout policy', lambda: jumper.update_policy(False, {})))
        startup_finalizers.insert(0, ('Scouts', jumper.close))
        pico = pico_link.open_pico(args.pico, on_event=link.event)
        startup_stops.append(('Pico', pico.close))
        pico.start()
        cam = None
        video_recorder = video_recording.VideoRecorder(
            Path(__file__).resolve().parents[1] / 'logs', enabled=camera_on)
        startup_finalizers.insert(0, ('video recorder', video_recorder.close))
        if camera_on:
            if config.VIDEO_AUTO_START:
                video_recorder.start(dict(run_id=run.run_id, arguments=vars(args),
                    telemetry_session=run.recording.directory if run.recording else None,
                    policy='automatic runtime recording; independent of station and NEW RUN'))
            cam = camera.Camera(on_frame=link.publish_video, on_event=link.event,
                                on_record_frame=video_recorder.submit)
            startup_finalizers.insert(0, ('camera', cam.stop))
            cam.start()
        mapping.start()
        link.start()

        imu.start()
        jumper.start()
        plot_feed = None
        if scout_addresses:
            try:
                plot_feed = scout_plot_feed.ScoutPlotFeed(
                    jumper.snapshot, root_dir=Path(__file__).resolve().parents[1] / 'logs').start()
                startup_finalizers.insert(0, ('Scout plot feed', plot_feed.close))
                link.event('Scout read-only plot feed ready')
            except Exception as exc:
                # Visualization availability never changes actuator policy/startup.
                link.event('Scout plot feed unavailable: ' + str(exc))

        print("=" * 60)
        print(f"  Station ->  python station/station.py --rover {local_ip()}")
        print(f"  Port    ->  {args.port}  (/control, /video)")
        print(f"  Log     ->  {log_path or 'NONE (--no-log)'}")
        print("=" * 60)

        dt = 1.0 / config.EKF_RATE_HZ
        next_tick = time.monotonic()
        next_telemetry = next_temp = 0.0
        temp_c = wifi_dbm = None
        last_wheel_read_at = next_tick       # monotonic deadline; failed reads can block
        motor_bus_lost = False
        wheels = motors.NoMotors().read()    # shown until the first real reading
        raw_sent = [0] * len(config.MOTOR_IDS)

    except BaseException:
        for label, close in startup_stops + startup_finalizers:
            try:
                close()
            except Exception as error:
                print(f'Startup cleanup failed for {label}: {error}')
        raise

    try:
        while True:
            loop_timing = run.loop_timing
            loop_timing.start()
            # ── 0. one-off requests from the station ───────────────
            for request in link.take_requests():
                if request["type"] == "new_run":
                    link.event('Ending recording for NEW RUN')
                    link.set_recorder(None)
                    run.stop_log()
                    if run.recording:
                        retired_recordings.append(run.recording)
                    path = logger.new_log_path(str(request.get("tag") or "run"))
                    run = run.new_run(path, session_metadata)
                    imu.configure(estimation_settings(run, session_metadata))
                    link.set_recorder(run.recording)
                    link.event(f"new run: {os.path.basename(path)} -- hold still, aligning")
                elif request["type"] == "stop_log":
                    link.event("Rover session recording stopped; independent Jumper raw capture continues")
                    link.set_recorder(None)
                    run.stop_log()
                    imu.configure(estimation_settings(run, session_metadata))
                elif request["type"] == "zero_battery":
                    pico.zero_charge()
                    link.event("battery mAh count set to zero")
                elif request["type"] == "video_start":
                    if video_recorder.start(dict(run_id=run.run_id, arguments=vars(args),
                                                 telemetry_session=run.recording.directory if run.recording else None)):
                        link.event("VIDEO: recording requested on Jetson")
                elif request["type"] == "video_stop":
                    # A delayed stop for an older recording cannot stop a new one.
                    if request.get('directory') == video_recorder.state()['directory']:
                        video_recorder.stop()
                        link.event("VIDEO: save requested on Jetson")

            loop_timing.mark('requests')
            now = time.monotonic()
            t = now - run.t_start

            # ── 1. what the operator wants ─────────────────────────
            # Worked out BEFORE the sensor read, so that every path below
            # -- including the failure path -- can act on it.
            battery_state = link.update_battery(pico.state())
            command = link.command()
            forward, turn = command.forward, command.turn
            lift, pan = command.lift, command.pan
            loop_timing.mark('command')
            # Manual control does not depend on IMU readiness. The independent
            # link deadman/E-STOP and motor-bus checks still apply.

            # ── 2. wheel read + optional cached estimator view ─────
            reading = wheels_hw.read()
            wheel_read_at = time.monotonic()
            loop_timing.mark('wheel_read')
            imu.submit(reading, wheel_read_at, run.run_id)
            run.estimation_view = imu.poll()
            snapshot = run.estimation_view['snapshot']
            received_snapshot = snapshot
            loop_timing.mark('imu_poll')
            # IMU status/error/loss details are already retained in telemetry.
            # Never synchronously print an IMU transition from the drive loop:
            # a blocked console must not delay the next actuator write.
            if reading is not None:
                last_wheel_read_at = wheel_read_at
                motor_bus_lost = False
                wheels = reading
            else:
                # A lost READ must not skip the WRITE below. Goal Velocity
                # latches in the motor: skipping the write on a bus that has
                # stopped answering would leave the wheels at their last
                # speed. So keep commanding, and once the bus has been
                # silent as long as the deadman allows, command a stop
                # whatever the stick says.
                run.dropped += 1
                motor_bus_lost = wheel_read_at - last_wheel_read_at > config.COMMAND_TIMEOUT_S
                if motor_bus_lost:
                    forward = turn = 0.0
                # The optional worker independently rejects unpaired intervals.
            run.cycles += 1
            loop_timing.mark('imu_state')

            # ── 4. drive ───────────────────────────────────────────
            raw_sent = wheels_hw.drive(forward, turn)
            loop_timing.mark('wheel_write')
            # Called every cycle on purpose: pico_link sends a servo command
            # only while these calls keep coming, so a loop that hangs
            # stops the servos as well.
            lift_raw, wind, release = clegg.update(link.hammer.snapshot(), pico.state(),
                                                  lift_hw.ready, command.estop, now)
            loop_timing.mark('hammer')
            lift_hw.drive(lift_raw, hold=clegg.lift_holding)
            loop_timing.mark('lift')
            if command.estop:
                # A nominal neutral pulse is not a measured mechanical stop.
                # Remove every Pico PWM output on E-STOP.
                pico.set_pulses([0] * len(config.SERVOS))
            elif hammer_test:
                pico.set_pulses(clegg.test_pulses(pan))
            else:
                pico.set_servos(wind, pan, release)
            loop_timing.mark('pico')
            # A live browser lease AND these main-loop updates are required.
            # Merely receiving ESP telemetry cannot keep motion authorized.
            jumper.update_policy(not command.estop, jumper_context(run, snapshot))
            loop_timing.mark('scout_policy')
            run.record_sensors(reading, received_snapshot, command, dict(
                wheel_goal_raw=list(raw_sent), applied_forward=forward, applied_turn=turn,
                cylinder_jog_raw=lift_raw, cylinder_hold=clegg.lift_holding,
                hammer_active=clegg.active, wind=wind, release=release, pan=pan,
                battery_protection=battery_state,
                test_servo_us=([0] * len(config.SERVOS) if command.estop
                               else clegg.test_pulses(pan) if hammer_test else None)))
            loop_timing.mark('sensor_record')

            # ── 5. tell the station ────────────────────────────────
            if now >= next_telemetry:
                # Step from the schedule, not from "now". The loop only looks
                # every 10 ms, so restarting the wait from now would stretch
                # each period by up to one tick: 18 Hz instead of 20.
                next_telemetry = max(next_telemetry + 1.0 / config.TELEMETRY_HZ, now)
                if now >= next_temp:
                    next_temp = now + 1.0
                    temp_c = read_cpu_temp()
                    wifi_dbm = read_wifi_dbm()
                if not wheels_hw.connected:
                    motor_state = "off"
                elif motor_bus_lost:
                    motor_state = "lost"
                else:
                    motor_state = "ok"
                telemetry = build_telemetry(run, t, wheels, command, forward, turn,
                                             lift, pan, raw_sent, motor_state, snapshot,
                                             cam, pico, temp_c, wifi_dbm)
                telemetry['hammer'] = clegg.state(lift_hw.state())
                telemetry['jumper'] = jumper.snapshot()
                telemetry['mapping'] = mapping.status()
                telemetry['video_recording'] = video_recorder.state()
                merge_estimation(telemetry, run, run.estimation_view)
                telemetry['loop']['timing'] = run.loop_timing.snapshot()
                loop_timing.mark('telemetry')
                # Ownership ends here; subsequent cycles build independent
                # views while the recorder and publisher retain this tree.
                link.publish_owned(telemetry)
                loop_timing.mark('publish')

            # ── hold the beat ──────────────────────────────────────
            # Never skipped, not even on a dropped cycle: free-running to
            # catch up turns one lost packet into a burst of jitter.
            loop_timing.end()
            next_tick += dt
            pause = next_tick - time.monotonic()
            if pause > 0:
                time.sleep(pause)
            else:
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        original_error = sys.exc_info()[1]
        scout_stop_error = None
        # The Scout adapter contains device errors, but a coordinator failure
        # must still never skip the Rover's wheel/cylinder/Pico cleanup.
        for label, stop in (('policy', lambda: jumper.update_policy(False, {})),):
            try:
                stop()
            except Exception as error:
                scout_stop_error = scout_stop_error or error
                print(f'Scout shutdown {label} failed: {error}')
        try:
            try:
                try:
                    lift_hw.close()          # shared bus must still be open
                finally:
                    try:
                        wheels_hw.close()    # zero the wheels and release torque
                    finally:
                        pico.close()        # also stop if bus shutdown fails
            finally:
                try:
                    if cam:
                        cam.stop()
                finally:
                    # An actuator/camera cleanup error cannot orphan IMU/SPI.
                    imu.close()
        finally:
            try:
                try:
                    video_recorder.close()  # after actuator cleanup, even on other cleanup failures
                finally:
                    try:
                        if plot_feed:
                            plot_feed.close()
                    finally:
                        jumper.close()      # finalize ESP recordings independently of hardware cleanup
            finally:
                try:
                    mapping.close()
                finally:
                    link.close_telemetry()
                    link.event('Runtime stopped; closing session recording')
                    link.set_recorder(None)
                    try:
                        run.stop_log()
                    finally:
                        # Actuators are stopped; drain even if legacy CSV close failed.
                        if run.recording:
                            run.recording.close()
                        for recorder in retired_recordings:
                            recorder.close()
        elapsed = time.monotonic() - run.t_start
        print(f"{run.cycles} cycles in {elapsed:.1f} s -> "
              f"{run.cycles / max(elapsed, 1e-6):.1f} Hz")
        if scout_stop_error is not None and original_error is None:
            raise scout_stop_error


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Rover program: drive, log, camera, station link.")
    ap.add_argument("--no-motors", action="store_true",
                    help="do not touch the motors (wheels read zero)")
    ap.add_argument("--imu", default=config.DEFAULT_IMU_SOURCE,
                    choices=["fake", "arduino", "bmi088"])
    ap.add_argument("--no-camera", action="store_true", help="do not open the camera")
    ap.add_argument("--hammer-test", action="store_true",
                    help="bounded manual string controls even with --no-motors; normal driving enables manual controls by default")
    ap.add_argument("--cylinder-test", action="store_true",
                    help="allow manual ID 5 calibration jogs with supported load; requires --hammer-test --no-motors")
    ap.add_argument("--jumper", action="append", default=None, metavar="ESP_IP",
                    help="Scout LAN IPv4; repeat for Scout 2 (different IP). Each Scout requires its own dashboard claim and calibrated settings")
    ap.add_argument("--pico", default="usb", choices=["usb", "fake", "off"],
                    help="servo + battery board: on USB (keeps looking if absent), "
                         "simulated, or not used")
    ap.add_argument("--tag", default="drive",
                    help="name fragment for the auto-named log")
    ap.add_argument("--log", default=None,
                    help="explicit CSV path, overriding the auto name")
    ap.add_argument("--odometry-profile", default=config.WHEEL_CALIBRATION_PROFILE, metavar="PATH",
                    help="optional geometry-matched wheel-distance calibration JSON; estimation only")
    ap.add_argument("--no-log", action="store_true",
                    help="do not start a log (NEW RUN from the station still does)")
    ap.add_argument("--port", type=int, default=config.STATION_PORT)
    args = ap.parse_args()
    if args.jumper is None:
        args.jumper = list(config.JUMPER_DEFAULT_HOSTS)
    try:
        scouts.scout_hosts(args.jumper)
    except ValueError as exc:
        ap.error(str(exc))
    if args.cylinder_test and not (args.hammer_test and args.no_motors):
        ap.error('--cylinder-test requires --hammer-test --no-motors')
    main(args)
