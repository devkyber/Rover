// The station page: wires the rover link to the panels.
//
//   link.js    telemetry in, commands out        video.js   camera picture
//   drive.js   keyboard / pad / gamepad          plots.js   strip charts
//   views.js   attitude, path, wheels            util.js    drawing helpers
//
// Data flow: a telemetry message arrives 20 times a second -> stored as
// `latest` and added to the plots, path and statistics -> the next screen
// refresh redraws everything from `latest`. Drive commands go the other way
// on their own 20 Hz timer, whether or not anything was drawn.

import { $, refreshColors, signed, fitCanvas, colors } from './util.js';
import { RoverLink } from './link.js';
import { VideoView } from './video.js';
import { HammerPanel } from './hammer.js';
import { JumperPanels } from './jumper.js';
import { MissionMapPanel } from './mapping.js';
import { ScoutLandings } from './scout_landings.js';
import { ShotPlots } from './scout_plots.js';
import { CylinderView } from './cylinder.js';
import { recordingDisplay } from './recording.js';
import { VideoRecordingControls } from './video_recording.js';
import { PanReferencePanel } from './pan_reference.js';
import { installCleggReleaseBridge } from './clegg_release.js';
import { setupOperationDetails } from './ui.js';
import { batteryLevel as classifyBattery, batteryNotice } from './battery.js';
import { DriveInput } from './drive.js';
import { StripChart } from './plots.js';
import { AttitudeView, PathView, shortName } from './views.js';

const RAD = Math.PI / 180;
const MAX_PLOTS = 4;
const FRESH_S = 0.5;        // telemetry newer than this: link is fine
const DEAD_S = 2.0;         // older than this: treat the link as lost
const params = new URLSearchParams(location.search);
const view = ['control', 'monitor', 'ground'].includes(params.get('view')) ? params.get('view') : 'dashboard';
const canDrive = view !== 'monitor';
const canControl = view !== 'monitor';
const canHammer = canControl;
document.body.dataset.view = view;
$('station-view').value = view;
if (view !== 'dashboard') {
  $('mode-note').hidden = false;
  $('mode-note').textContent = view === 'monitor'
    ? 'Monitor only. Driving and run controls are disabled on this screen.'
    : view === 'ground' ? 'W/S forward/reverse · A/D turn · ↑/↓ speed limit · ←/→ camera pan · Space E-STOP. Use one controller at a time.'
    : 'Keep the rover in sight. Use one controller at a time.';
}
if (view === 'control') {
  $('drive-hint').textContent = 'Hold and drag to drive. Release to stop.';
  $('speed-limit').value = '20';
  $('speed-limit-value').textContent = '20%';
}

// ── state ────────────────────────────────────────────────────────────
let rover = null;           // the rover's description ("hello"), null until first connected
let latest = null;          // newest telemetry message
let latestAt = 0;           // performance.now() when it arrived
let dirty = false;          // something changed since the last redraw
let received = 0, tlmHz = 0;
let seenEvent = 0;          // id of the newest rover event already in the log
let eventHistoryPending = true;  // first snapshot has no known occurrence timestamps
let lastRowT = -1;          // rover time of the last telemetry row written to the log
let imuStaleSeen = 0, imuStaleAt = -1e9;
let droppedSeen = 0, droppedAt = -1e9;
let stats = newStats();
let cameraExpanded = false;
let cameraRestoreSize = $('camera-size').value;
// Display memory never feeds commands, mapping, scientific validity or samples.
// Missing fields in a masked result do not identify a new local origin.
const displayMemory = {};
const videoMemory = { address: null, baseline: 0, seen: false };

function newStats() {
  return { distance: 0, moving: 0, maxSpeed: 0, maxCurrent: 0, maxTilt: 0, prev: null };
}

const age = () => (performance.now() - latestAt) / 1000;
const fresh = () => latest !== null && age() < FRESH_S;
const poseAvailable = t => t.pose_valid === true && t.rpy.every(Number.isFinite) && t.pos.every(Number.isFinite);
function manualGapTolerated(t) {
  const b = t?.pose_continuity?.manual_gap_budget;
  return t?.pose_quality === 'degraded_gap' && t.pose_valid === false
    && t.manual_gap_tolerated === true && b?.known === true && b.within_budget === true
    && [b.heading_error_deg, b.heading_budget_deg, b.position_budget_m]
      .every(v => Number.isFinite(v) && v >= 0)
    && b.heading_error_deg <= b.heading_budget_deg
    && (b.position_budget_applicable === false ? b.position_error_m === null
      : Number.isFinite(b.position_error_m) && b.position_error_m >= 0 && b.position_error_m <= b.position_budget_m);
}
function gapBudgetText(t) {
  if (t?.pose_quality === 'delayed')
    return 'Delayed estimate display only. Precision and map context follow current telemetry; delivery delay does not prove missing motion.';
  if (t?.pose_continuity?.missing_motion === false && t.pose_quality_reason)
    return 'Delayed processing retained all IMU samples. Wheel pairing during catch-up is unverified; precise mapping remains invalid. Alignment is kept.';
  const b = t?.pose_continuity?.manual_gap_budget;
  return b?.known === true && Number.isFinite(b.heading_error_deg)
    ? `Estimated gap error: ${b.heading_error_deg.toFixed(2)}° / ${Number.isFinite(b.position_error_m) ? (100 * b.position_error_m).toFixed(1) + ' cm' : 'position unavailable'}. Conditional motion model; total accuracy unverified. Precision mapping remains invalid.`
    : 'Missing motion was not reconstructed. Estimate only; uncertainty is unknown. NEW RUN requires stationary alignment.';
}
function displayHeld(t) {
  const p = t?.held_estimate;
  const finite3 = a => Array.isArray(a) && a.length === 3 && a.every(Number.isFinite);
  if (t?.estimate_held !== true || p?.current_measurement !== false || !finite3(p.rpy)) return null;
  return { pos: p.position_available === true && finite3(p.pos) ? p.pos : null,
    rpy: p.rpy, gyro_bias: p.gyro_bias, degraded: true, held: true };
}
function displayPose(t) {
  const held = displayHeld(t);
  if (held?.pos) return held;
  if (!t || t.pose_estimate_available === false) return null;
  if (t.pose_quality === 'delayed' && !['bmi088', 'arduino'].includes(t.estimation_status?.imu_source)) return null;
  const finite3 = a => Array.isArray(a) && a.length === 3 && a.every(Number.isFinite);
  if (t.pose_valid === true && finite3(t.pos) && finite3(t.rpy))
    return { pos: t.pos, rpy: t.rpy, gyro_bias: t.gyro_bias, degraded: false };
  const p = t.pose_estimate;
  if (t.pose_estimate_available === true && t.pose_valid === false
      && ['degraded_gap', 'delayed'].includes(t.pose_quality)
      && finite3(p?.pos) && finite3(p?.rpy))
    return { pos: p.pos, rpy: p.rpy, gyro_bias: p.gyro_bias,
      degraded: !manualGapTolerated(t), delayed: t.pose_quality === 'delayed' };
  return null;
}
function displayAttitude(t) {
  if (t?.pose_quality === 'delayed' && !['bmi088', 'arduino'].includes(t.estimation_status?.imu_source)) return null;
  const pose = displayPose(t);
  if (pose) return pose;
  const held = displayHeld(t);
  if (held) return held;
  const a = t?.attitude_estimate;
  if (t?.attitude_estimate_available === true && Array.isArray(a?.rpy)
      && a.rpy.length === 3 && a.rpy.every(Number.isFinite))
    return { rpy: a.rpy, gyro_bias: a.gyro_bias, degraded: a.degraded === true };
  return null;
}
function poseFrameChanged(previous, current) {
  if (previous.run_id != null && current.run_id != null && previous.run_id !== current.run_id) return true;
  if (previous.pose_frame_id != null && current.pose_frame_id != null)
    return previous.pose_frame_id !== current.pose_frame_id;
  if (previous.pose_frame_id != null || current.pose_frame_id != null) return false;
  return previous.imu?.generation != null && current.imu?.generation != null
    && previous.imu.generation !== current.imu.generation;
}
function poseIntervalBroken(previous, current, elapsed, readPose = displayPose) {
  return !!previous && (elapsed >= FRESH_S || poseFrameChanged(previous, current)
    || previous.estimate_held === true || current.estimate_held === true
    || !readPose(previous) || !readPose(current)
    || previous.pose_continuity?.loss_count !== current.pose_continuity?.loss_count);
}
function poseRecoveryResetsPath(previous, current) {
  return poseFrameChanged(previous, current);
}
function clearDisplayMemory(preserveVisual = false) {
  const visual = preserveVisual ? {
    pose: displayMemory.pose || displayMemory.visual?.pose,
    attitude: displayMemory.attitude || displayMemory.visual?.attitude,
  } : null;
  // The connection identity outlives an individual run/view reset.
  for (const key of Object.keys(displayMemory)) if (key !== 'address') delete displayMemory[key];
  if (visual?.pose || visual?.attitude) displayMemory.visual = visual;
}
function displayIdentityChanged(t) {
  const identity = { run: t.run_id, frame: t.pose_frame_id ?? t.odom_frame_id,
    imuSource: t.estimation_status?.imu_source, wheelSource: t.estimation_status?.wheel_source,
    generation: (t.pose_frame_id ?? t.odom_frame_id ?? displayMemory.frame) == null ? t.imu?.generation : null };
  return Object.entries(identity).some(([key, value]) => value != null
    && displayMemory[key] != null && displayMemory[key] !== value);
}
function interruptDisplay() {
  if (!displayMemory.pose && !displayMemory.attitude) return;
  displayMemory.interrupted = true;
  displayMemory.healthySince = null;
}
function rememberDisplay(t) {
  const now = performance.now();
  const identity = { run: t.run_id, frame: t.pose_frame_id ?? t.odom_frame_id,
    imuSource: t.estimation_status?.imu_source, wheelSource: t.estimation_status?.wheel_source,
    generation: (t.pose_frame_id ?? t.odom_frame_id ?? displayMemory.frame) == null ? t.imu?.generation : null };
  for (const [key, value] of Object.entries(identity)) if (value != null) displayMemory[key] = value;
  const sourceAge = t.estimation_worker?.source_age_s ?? t.imu?.age_s;
  const at = now - (Number.isFinite(sourceAge) && sourceAge >= 0 ? sourceAge * 1000 : 0);
  const pose = displayPose(t), tilt = displayAttitude(t);
  for (const [key, value] of [['pose', pose], ['attitude', tilt]]) {
    if (!value || value.held) continue;
    displayMemory[key] = { at, value: { ...value, pos: value.pos?.slice(), rpy: value.rpy.slice(),
      gyro_bias: Array.isArray(value.gyro_bias) ? value.gyro_bias.slice() : null } };
  }
  if ((!tilt || tilt.held) || (displayMemory.pose && (!pose || pose.held))) interruptDisplay();
  else if (displayMemory.interrupted && displayMemory.healthySince == null) displayMemory.healthySince = now;
}
function displayReadback(t, kind) {
  const current = t && fresh() ? (kind === 'pose' ? displayPose(t) : displayAttitude(t)) : null;
  const cached = displayMemory[kind] || displayMemory.visual?.[kind];
  if (current && !current.held) return { ...current,
    age_s: cached ? Math.max(0, (performance.now() - cached.at) / 1000) : 0 };
  if (cached) return { ...cached.value, held: true, degraded: true,
    visual_only: !displayMemory[kind],
    age_s: Math.max(0, (performance.now() - cached.at) / 1000) };
  if (current?.held) return { ...current, age_s: Math.max(0, age())
    + (Number.isFinite(t.held_estimate?.age_s) ? Math.max(0, t.held_estimate.age_s) : 0) };
  return null;
}
function displayStatus(t) {
  const pose = displayReadback(t, 'pose'), tilt = displayReadback(t, 'attitude');
  if (!pose && !tilt) return null;
  const held = pose?.held === true || tilt?.held === true;
  if (held) interruptDisplay();
  // A brief usable result must not turn the badges green between masked ones.
  // Values can update immediately; only the display warning has a 1 s recovery dwell.
  const waiting = held || displayMemory.interrupted
    && (displayMemory.healthySince == null || performance.now() - displayMemory.healthySince < 1000);
  const elapsed = Math.max(pose?.age_s || 0, tilt?.age_s || 0);
  return { held, waiting, age_s: elapsed,
    text: waiting ? `LAST ESTIMATE · ${elapsed.toFixed(1)} s` : null };
}
function updateDisplayHealth(t) {
  const status = displayStatus(t);
  chip($('h-imu'), 'IMU', 'idle');
  $('h-imu').title = `Current IMU health: ${t?.imu?.status || 'unavailable'}; estimate: ${t?.pose_quality || 'unavailable'}; precision valid: ${t?.pose_valid === true}. Display age: ${status ? status.age_s.toFixed(2) + ' s' : 'unavailable'}. Retained display values are not current measurements; precision and map context use live telemetry.`;
}
function steadyDisplayPresentation(t) {
  const status = displayStatus(t);
  const title = `Local estimate display. Current IMU: ${t?.imu?.status || 'unavailable'}; precision valid: ${t?.pose_valid === true}; display age: ${status ? status.age_s.toFixed(2) + ' s' : 'unavailable'}. Retained values are display only; cached positions do not add path samples. Mapping uses live telemetry.`;
  for (const id of ['attitude', 'path']) $(id).closest('.panel').dataset.poseQuality = '';
  for (const id of ['hud-pose-quality', 'plots-pose-quality']) $(id).hidden = true;
  setText('attitude-quality', 'ATTITUDE');
  setText('path-position-status', 'LOCAL ESTIMATE');
  $('attitude-quality').title = title;
  $('path-position-status').title = title;
}
function updatePoseReadouts(estimate, tilt) {
  const rpy = tilt?.rpy || [null, null, null], pos = estimate?.pos || [null, null, null];
  for (const [axis, index] of [['roll', 0], ['pitch', 1], ['yaw', 2]]) {
    setText('hud-' + axis, signed(rpy[index], 1));
    setText('att-' + axis, Number.isFinite(rpy[index]) ? signed(rpy[index], 1) + '°' : '--');
  }
  setText('pos-x', signed(pos[0], 3));
  setText('pos-y', signed(pos[1], 3));
  setText('pos-r', estimate ? Math.hypot(pos[0], pos[1]).toFixed(3) : '--');
}
function posePendingText(t) {
  return t.align >= 1 && (t.estimation_worker?.stale_reason || t.estimation_worker?.status === 'stale')
    ? 'WAITING ESTIMATE' : `ALIGNING ${Math.round(100 * t.align)}%`;
}
function poseWaitText(status, progress, duration) {
  if (status === 'delayed') return { attitude: 'IMU delayed\nPose unavailable', path: 'Waiting for fresh IMU data' };
  if (status === 'ok') {
    // Use stationary sample progress, so motion/reset restarts the displayed time.
    const timing = Number.isFinite(progress) && Number.isFinite(duration) && duration > 0
      ? `\n${(Math.max(0, Math.min(1, progress)) * duration).toFixed(1)} / ${duration.toFixed(1)} s` : '';
    return { attitude: 'Hold still to align IMU' + timing, path: 'Waiting for IMU alignment' };
  }
  return { attitude: 'IMU unavailable', path: 'Waiting for IMU recovery' };
}

// ── panels ───────────────────────────────────────────────────────────
refreshColors();
const attitude = new AttitudeView($('attitude'));
const pathView = new PathView($('path'));
const cylinder = new CylinderView($('cylinder-view'), FRESH_S);
const video = new VideoView($('video'));
const drive = new DriveInput({ pad: $('pad'), dot: $('pad-dot'), auxButtons: document.querySelectorAll('button[data-pan]'),
  onEstop: () => setEstop(true),
  onSpeedStep: direction => {
    if (!canDrive) return;
    const slider = $('speed-limit');
    slider.value = Number(slider.value) + direction * Number(slider.step);
    updateSpeedLimit();
  } });
const link = new RoverLink({ onHello, onTelemetry, onStatus, onJumperResult: result => {
  jumperPanel.onResult(result);
} });
const videoRecording = new VideoRecordingControls($('video-recording'), {
  send: message => link.send(message), canControl });
const panReference = new PanReferencePanel($('pan-reference'), {
  maxGapS: FRESH_S, canSetZero: canDrive });
const mappingLink = new RoverLink({ channel: 'mapping', onHello: () => {}, onTelemetry: () => {},
  onStatus: () => mappingPanel.connected(mappingLink.connected && link.connected),
  onMappingResult: result => { mappingPanel.result(result); dirty = true; } });
const mappingPanel = new MissionMapPanel($('mapping-panel'), {
  send: message => mappingLink.send(message), canEdit: canControl });
const hammerPanel = new HammerPanel({ root: $('hammer-panel'), send: message => { if (canHammer) link.send(message); },
  enabled: () => canHammer && fresh() && !latest?.cmd.estop });
installCleggReleaseBridge({ canRecord: canControl, onResult: (text, bad) => logEvent(text, bad) });
const scoutLandings = new ScoutLandings($('scout-landings'), {
  limits: () => rover?.scout_postshot, changed: () => { dirty = true; } });
const shotPlots = new ShotPlots($('scout-shot-window'), {
  onLandings: bundle => scoutLandings.load(bundle) });
$('scout-plots-open').addEventListener('click', () => shotPlots.open());
const jumperPanel = new JumperPanels({ root: $('jumper-panel'), send: message => { if (canHammer) link.send(message); },
  controls: canHammer, enabled: () => canHammer && link.connected && fresh() && !latest?.cmd.estop });
setupOperationDetails($('operation-panel'));
drive.speedLimit = Number($('speed-limit').value) / 100;

// Mean wheel speed the stick asks for, with the same scaling the rover
// applies (rover/dxl_control.drive): a full stick plus a turn is scaled down.
function commandedSpeed(t) {
  const { forward, turn } = t.cmd;
  return forward / Math.max(1, Math.abs(forward + turn), Math.abs(forward - turn)) * rover.max_speed_mps;
}
const meanSpeed = t => t.wheel_speed.reduce((a, b) => a + b, 0) / t.wheel_speed.length;

// The Pico's part of a telemetry message. A rover program from before the
// Pico was added sends none: treat that as "off".
const pico = t => t.pico || { status: 'off', battery: null, servo_us: null, failsafe: null };
const battery = t => pico(t).battery;       // null = no reading

// 'ok' | 'low' | 'empty', from the volts per cell the rover says to warn at.
function batteryLevel(b) {
  return classifyBattery(b, rover?.battery);
}

function batteryText(b, level = 'ok') {
  const volts = Number.isFinite(b?.v) ? b.v.toFixed(2) : '--';
  const amps = Number.isFinite(b?.a) ? b.a.toFixed(2) : '--';
  return `BAT ${volts} V · ${amps} A` + (b && level !== 'ok' ? ' ' + level.toUpperCase() : '');
}

function captureBadge(data, live, telemetryAgeS) {
  if (!live) return 'CAPTURE --';
  if (data?.recording?.error || data?.recording?.close_error) return 'CAPTURE ERROR';
  if (!data || data.connection_status === 'off') return 'CAPTURE OFF';
  const dataAge = data.manager?.ages?.transport?.data_s;
  const active = data.connection_status === 'ok' && data.manager?.connected === true
    && data.stream?.receiving === true && !!data.recording?.prefix
    && data.manager?.diagnostics?.raw_received_records > 0
    && Number.isFinite(dataAge) && dataAge >= 0 && dataAge + telemetryAgeS < FRESH_S;
  return active ? 'CAPTURE ON' : 'CAPTURE WAIT';
}

// Every plot the operator can switch on. `get` picks its numbers out of a
// telemetry message. All are fed all the time, so switching one on shows
// its last 30 s at once.
let plots = [];
function buildPlots() {
  const names = rover.wheels.map(w => shortName(w.name));
  plots = [
    { label: 'WHEEL SPEED', unit: 'm/s', series: names, digits: 3, minSpan: 0.05, on: true, get: t => t.wheel_speed },
    { label: 'WHEEL CURRENT', unit: 'A', series: names, digits: 2, minSpan: 0.2, on: true, get: t => t.wheel_current },
    { label: 'SPEED', unit: 'm/s', series: ['commanded', 'measured'], digits: 3, minSpan: 0.05, on: true,
      get: t => [commandedSpeed(t), meanSpeed(t)] },
    { label: 'ATTITUDE', unit: 'deg', series: ['roll', 'pitch', 'yaw'], estimate: true, digits: 1, minSpan: 5, on: true, get: t => displayAttitude(t)?.rpy },
    { label: 'GYRO BIAS', unit: 'deg/s', series: ['x', 'y', 'z'], estimate: true, digits: 3, minSpan: 0.05, on: false, get: t => displayAttitude(t)?.gyro_bias },
  ];
  if (rover.pico && rover.pico !== 'off') {
    // No reading -> nothing is added, and the line has a gap there.
    plots.find(plot => plot.label === 'SPEED').on = false;
    plots.push({ label: 'BATTERY', unit: 'V', series: ['voltage', 'current'], digits: 2, minSpan: 0.5,
      rightAxis: { unit: 'A', series: [1], minSpan: 0.5 }, on: true,
      get: t => battery(t) && [battery(t).v, battery(t).a] });
  }
  $('plot-row').replaceChildren();
  $('plot-toggles').replaceChildren();
  for (const plot of plots) {
    plot.chart = new StripChart($('plot-row'), { title: plot.label, ...plot });
    const label = document.createElement('label');
    plot.box = document.createElement('input');
    plot.box.type = 'checkbox';
    plot.box.checked = plot.on;
    plot.box.addEventListener('change', () => { plot.on = plot.box.checked; showPlots(); });
    const text = document.createElement('span');
    text.textContent = plot.label;
    label.append(plot.box, text);
    $('plot-toggles').append(label);
  }
  showPlots();
}

function showPlots() {
  const shown = plots.filter(p => p.on).length;
  for (const plot of plots) {
    plot.chart.root.style.display = plot.on ? '' : 'none';
    plot.box.disabled = !plot.on && shown >= MAX_PLOTS;    // at the limit: only un-ticking is possible
  }
  dirty = true;
}

// ── from the rover ───────────────────────────────────────────────────
function onHello(hello) {
  eventHistoryPending = true;
  const restarted = rover !== null && rover.boot !== hello.boot;
  const sourceChanged = rover !== null && displayMemory.address != null && displayMemory.address !== link.address;
  const sameWheels = rover !== null && JSON.stringify(rover.wheels) === JSON.stringify(hello.wheels)
    && rover.pico === hello.pico;            // the battery plots exist only with a Pico
  rover = hello;
  if (restarted || sourceChanged) { resetView(restarted && !sourceChanged); latest = null; latestAt = 0; }
  displayMemory.address = link.address;
  if (restarted) panReference.interrupt();
  if (restarted) { seenEvent = 0; logEvent('rover program was restarted'); }
  attitude.setRover(hello);
  pathView.setRover(hello);
  if (!sameWheels) { buildPlots(); buildLogHeader(); }
  logEvent('connected to rover at ' + link.address);
}

function onTelemetry(t) {
  panReference.observe(t);
  mappingPanel.update(t);
  const poseBreak = poseIntervalBroken(latest, t, age());
  const attitudeBreak = poseIntervalBroken(latest, t, age(), displayAttitude);
  const sensorChanged = ['imuSource', 'wheelSource'].some((key, index) => {
    const value = t.estimation_status?.[index ? 'wheel_source' : 'imu_source'];
    return value != null && displayMemory[key] != null && displayMemory[key] !== value;
  });
  // The rover's clock restarts at zero on NEW RUN: begin a clean view.
  if (latest && (t.t < latest.t - 1 || latest.run_id != null && t.run_id != null && latest.run_id !== t.run_id)) resetView(!displayMemory.operatorNewRun && !sensorChanged);
  if (displayIdentityChanged(t) || latest && poseFrameChanged(latest, t)) {
    pathView.clear();
    clearDisplayMemory(!displayMemory.operatorNewRun && !sensorChanged);
    stats.prev = null;  // a new IMU estimate has a new origin, not a traveled jump
  }
  if (latest && !displayPose(latest) && displayPose(t)) {
    if (poseRecoveryResetsPath(latest, t)) pathView.clear();
    stats.prev = null;  // also covers re-alignment after a missing wheel sample
  }
  if (poseBreak) {
    // A cached older view module may survive a frontend-only update.
    // Clear it rather than join a missing interval or abort telemetry handling.
    if (typeof pathView.break === 'function') pathView.break(); else pathView.clear();
    stats.prev = null;
  }
  latest = t;
  latestAt = performance.now();
  rememberDisplay(t);
  received++;

  for (const plot of plots) {
    if (plot.estimate && attitudeBreak) plot.chart.push(t.t, plot.series.map(() => null));
    const values = plot.estimate && t.estimate_held === true ? null : plot.get(t);
    plot.chart.push(t.t, values || plot.series.map(() => null));
    if (plot.estimate) plot.chart.root.dataset.poseQuality = '';
  }
  const estimate = displayPose(t);
  if (estimate && !estimate.held) {
    const lastPoint = pathView.points.at(-1) || pathView.points.at(-2);
    if (!lastPoint || Math.hypot(estimate.pos[0] - lastPoint[0], estimate.pos[1] - lastPoint[1]) >= .005)
      displayMemory.pathRevision = (displayMemory.pathRevision || 0) + 1;
    pathView.add(estimate.pos[0], estimate.pos[1]);
  }
  addToStats(t);

  receiveRoverEvents(t.events);
  if (t.t - lastRowT >= 0.5 || t.t < lastRowT) { lastRowT = t.t; logRow(t); }
  if (t.imu_stale > imuStaleSeen) imuStaleAt = performance.now();
  imuStaleSeen = t.imu_stale;
  if (t.loop.dropped > droppedSeen) droppedAt = performance.now();
  droppedSeen = t.loop.dropped;
  dirty = true;
}

function onStatus(status) {
  mappingPanel.connected(status === 'connected' && mappingLink.connected);
  if (view !== 'control') {
    if (status !== 'connected') video.disconnect();
    else if (video.address !== link.address) video.connect(link.address);
  }
  if (status !== 'connected') {
    panReference.interrupt();
    latestAt = -Infinity; dirty = true;
    eventHistoryPending = true; drive.reset('connection_' + status); hammerPanel.stop();
  }
  const label = $('link-label');
  if (status === 'connected') { label.textContent = 'CONNECTED'; label.className = 'status-text good'; }
  else if (status === 'connecting') { label.textContent = 'CONNECTING...'; label.className = 'status-text warn'; }
  else { label.textContent = 'DISCONNECTED'; label.className = 'status-text'; }
  $('connect').textContent = link.address === null ? 'CONNECT' : 'DISCONNECT';
}

function addToStats(t) {
  const prev = stats.prev;
  if (prev) {
    const dt = t.t - prev.t;
    if (dt > 0 && dt < 1) {
      if (poseAvailable(t) && poseAvailable(prev))
        stats.distance += Math.hypot(t.pos[0] - prev.pos[0], t.pos[1] - prev.pos[1]);
      // "Moving" here means the wheels are actually turning. The rover's own
      // state says "moving" for an instant after alignment while standing still.
      if (Math.abs(meanSpeed(t)) > 0.005) stats.moving += dt;
    }
  }
  stats.prev = t;
  stats.maxSpeed = Math.max(stats.maxSpeed, Math.abs(meanSpeed(t)));
  stats.maxCurrent = Math.max(stats.maxCurrent, ...t.wheel_current.map(Math.abs));
  // Tilt = angle between the rover's up and true up, from roll and pitch together.
  if (poseAvailable(t)) {
    const tilt = Math.acos(Math.cos(t.rpy[0] * RAD) * Math.cos(t.rpy[1] * RAD)) / RAD;
    stats.maxTilt = Math.max(stats.maxTilt, tilt);
  }
}

function resetView(preserveVisual = false) {
  for (const plot of plots) plot.chart.clear();
  pathView.clear();
  clearDisplayMemory(preserveVisual);
  stats = newStats();
  lastRowT = -1;
  dirty = true;
}

// ── log panel ────────────────────────────────────────────────────────
const logBody = $('log');
const MAX_LOG_LINES = 400;
logBody.closest('details').addEventListener('toggle', event => {
  if (event.currentTarget.open) logBody.scrollTop = logBody.scrollHeight;
});

function addLogLine(text, className) {
  const atBottom = logBody.scrollTop + logBody.clientHeight >= logBody.scrollHeight - 8;
  const line = document.createElement('div');
  line.textContent = text;
  if (className) line.className = className;
  logBody.append(line);
  while (logBody.childElementCount > MAX_LOG_LINES) logBody.firstElementChild.remove();
  if (atBottom) logBody.scrollTop = logBody.scrollHeight;    // follow, unless the operator scrolled up to read
}

function receiveRoverEvents(events) {
  // The snapshot repeats retained events; their IDs give order, not event time.
  // On reconnect, retain unseen diagnostics without assigning them "now".
  for (const [id, text] of events) {
    if (id > seenEvent) {
      seenEvent = id;
      logEvent(text, /e-stop on|lost/i.test(text), eventHistoryPending);
    }
  }
  eventHistoryPending = false;
}

function logEvent(text, bad = false, history = false) {
  const prefix = history ? '[history]'
    : new Date().toLocaleTimeString('en-GB', { timeZone: 'Asia/Seoul', hour12: false });
  addLogLine(prefix + '  ' + text, bad ? 'event bad' : 'event');
}

const col = (text, width) => String(text).padStart(width);

// Two rows a second of the numbers worth reading back later. Per-wheel
// values are in the selectable plots; everything, at 100 Hz,
// is in the rover's own CSV log.
function buildLogHeader() {
  $('log-header').textContent =
    col('T+', 7) + '  ' + 'state   ' + col('x m', 8) + col('y m', 8) + col('yaw', 8) + col('roll', 7) + col('pitch', 7) +
    col('m/s', 8) + col('A max', 7) + col('fwd', 7) + col('turn', 7);
}

function logRow(t) {
  addLogLine(
    col(t.t.toFixed(1), 7) + '  ' + t.state.padEnd(8) +
    col(signed(t.pos[0], 3), 8) + col(signed(t.pos[1], 3), 8) + col(signed(t.rpy[2], 1), 8) +
    col(signed(t.rpy[0], 1), 7) + col(signed(t.rpy[1], 1), 7) + col(signed(meanSpeed(t), 3), 8) +
    col(Math.max(...t.wheel_current.map(Math.abs)).toFixed(2), 7) +
    col(signed(t.cmd.forward, 2), 7) + col(signed(t.cmd.turn, 2), 7));
}

// ── drawing ──────────────────────────────────────────────────────────
function chip(el, text, kind) {
  if (el.textContent !== text) el.textContent = text;
  const className = 'chip ' + kind;
  if (el.className !== className) el.className = className;
}
const setText = (id, text) => { const el = $(id); if (el.textContent !== text) el.textContent = text; };

function unavailable(id, text) {
  const { g, w, h } = fitCanvas($(id));
  g.fillStyle = colors.muted;
  g.font = '14px sans-serif';
  g.textAlign = 'center';
  const lines = text.split('\n');
  lines.forEach((line, i) => g.fillText(line, w / 2, h / 2 + (i - (lines.length - 1) / 2) * 18));
}

function redraw() {
  const t = latest;
  const estimate = displayReadback(t, 'pose'), attitudeEstimate = displayReadback(t, 'attitude');
  steadyDisplayPresentation(t);
  updatePoseReadouts(estimate, attitudeEstimate);
  if (!t || !rover) {
    if (!attitudeEstimate) unavailable('attitude', 'Waiting for telemetry');
    if (!estimate) unavailable('path', 'Waiting for rover position');
    return;
  }
  const rpy = attitudeEstimate?.rpy || [null, null, null], pos = estimate?.pos || [null, null, null];
  setText('hud-speed', Math.abs(meanSpeed(t)).toFixed(2));
  setText('hud-yaw', signed(rpy[2], 1));
  setText('hud-roll', signed(rpy[0], 1));
  setText('hud-pitch', signed(rpy[1], 1));
  setText('hud-cmd', signed(t.cmd.forward, 2) + ' / ' + signed(t.cmd.turn, 2));

  const poseWait = fresh() ? poseWaitText(t.imu?.status, t.align, rover.alignment_s)
    : { attitude: 'Telemetry stale\nPose unavailable', path: 'Waiting for telemetry' };
  if (fresh() && t.estimation_status?.imu_source === 'fake') {
    poseWait.attitude = 'Simulated IMU\nNo measured attitude';
    poseWait.path = 'Simulation · no measured position';
  }
  const attitudeKey = JSON.stringify([rpy[0], rpy[1], $('attitude').clientWidth, $('attitude').clientHeight, colors]);
  if (attitudeEstimate) {
    if (!attitudeEstimate.visual_only && displayMemory.attitudeRender !== attitudeKey) {
      attitude.draw(rpy[0], rpy[1]);
      displayMemory.attitudeRender = attitudeKey;
    }
  } else unavailable('attitude', poseWait.attitude);

  const landings = scoutLandings.update(t, mappingPanel.state);
  pathView.setLandings(landings);
  const pathKey = JSON.stringify([pos[0], pos[1], estimate?.rpy[2], displayMemory.pathRevision, landings,
    $('path').clientWidth, $('path').clientHeight, colors]);
  if (estimate) {
    if (!estimate.visual_only && displayMemory.pathRender !== pathKey) {
      pathView.draw(pos[0], pos[1], estimate.rpy[2], false);
      displayMemory.pathRender = pathKey;
    }
  } else if (landings.length) pathView.drawLandingsOnly();
  else unavailable('path', attitudeEstimate ? 'Position unavailable\nNo wheel measurements' : 'Position unavailable');
  setText('path-scale', estimate ? 'grid ' + pathView.gridStep + ' m' : attitudeEstimate ? 'Attitude only' : poseWait.path);

  for (const plot of plots) if (plot.on) plot.chart.draw();
  drive.show(t.cmd.forward, t.cmd.turn);
  drive.showAux(t.cmd.lift ?? 0, t.cmd.pan ?? 0);

  const b = battery(t);
  setText('st-battery', b ? b.a.toFixed(2) + ' A' : '--');
  setText('st-mah', b ? b.mah.toFixed(0) + ' mAh' + (pico(t).charge_complete === false ? ' (gaps)' : '') : '--');
  setText('st-distance', stats.distance.toFixed(2) + ' m');
  setText('st-moving', stats.moving.toFixed(1) + ' s');
  setText('st-speed', stats.maxSpeed.toFixed(3) + ' m/s');
  setText('st-current', stats.maxCurrent.toFixed(2) + ' A');
  setText('st-tilt', stats.maxTilt.toFixed(1) + '°');

  drawNis(t.nis);
}

function drawNis(nis) {
  const body = $('nis');
  const names = Object.keys(nis).sort();
  body.replaceChildren();
  if (!names.length) return;
  for (const name of names) {
    const [mean, target] = nis[name];
    const ratio = mean / target;
    // Same heuristic band the old live panel used.
    const [word, kind] = ratio > 3 ? ['HIGH', 'bad'] : ratio < 0.33 ? ['LOW', 'warn'] : ['OK', 'good'];
    const row = document.createElement('tr');
    for (const [text, className] of [[name], [mean.toFixed(2)], [String(target), 'muted'],
                                     [ratio.toFixed(2) + 'x'], [word, kind]]) {
      const cell = document.createElement('td');
      cell.textContent = text;
      if (className) cell.className = className;
      row.append(cell);
    }
    body.append(row);
  }
}

// Things that must keep updating even when no telemetry arrives -- that is
// exactly when the operator needs them: the clock, link age, the banners.
function slowUpdate() {
  fitCameraLayout();
  const t = latest, wanted = link.address !== null, live = fresh();
  if (t && !live) dirty = true;  // expired pose readback must not look current
  cylinder.update(t?.hammer?.lift, live, t ? age() : 0);
  for (const panel of jumperPanel.panels) {
    const capture = captureBadge(panel.data, live, t ? age() : 0);
    const badge = panel.node('capture-badge');
    badge.textContent = capture;
    badge.dataset.active = String(capture === 'CAPTURE ON');
  }

  setText('clock', 'KST ' + new Date().toLocaleTimeString('en-GB', { timeZone: 'Asia/Seoul', hour12: false }));
  const runTime = $('run-time');
  if (t) {
    const m = Math.floor(t.t / 60), s = t.t - 60 * m;
    runTime.textContent = 'T+ ' + String(m).padStart(2, '0') + ':' + s.toFixed(1).padStart(4, '0');
  }
  runTime.className = 'mission-cell ' + (live ? 'live' : t && wanted ? 'bad' : 'muted');
  setText('tlm-rate', 'TLM ' + (live ? tlmHz : '--') + ' Hz');
  // Round trip, and how strongly the rover hears the router.
  setText('rtt', 'LINK ' + (link.rttMs === null ? '--' : link.rttMs.toFixed(0)) + ' ms'
    + (live && t.wifi_dbm != null ? `  ${t.wifi_dbm} dBm` : ''));

  const batteryCell = $('battery');
  const b = live ? battery(t) : null;
  const protection = live ? t.battery_protection : null;
  const level = protection?.level || (b ? batteryLevel(b) : null);
  setText('battery', batteryText(b, level));
  batteryCell.className = 'mission-cell ' + (level === 'stop' || level === 'empty' ? 'bad' : !b ? 'muted' : level === 'low' ? 'warn' : '');
  batteryCell.title = b ? `Voltage and signed current from the Pico battery monitor${Number.isFinite(b.cell_v) ? ', ' + b.cell_v.toFixed(2) + ' V per cell' : ''}` : 'Battery reading unavailable';
  const notice = live ? batteryNotice(protection, b, rover?.battery)
    : { text: '', tone: 'warn' };
  const batteryAlert = $('battery-alert');
  batteryAlert.textContent = notice.text;
  batteryAlert.className = 'battery-alert ' + notice.tone;
  batteryAlert.hidden = !notice.text;
  setText('video-rate', video.fps > 0 ? `VIDEO ${video.fps} fps  ${video.mbps.toFixed(1)} Mbit/s` : 'VIDEO -- fps');

  // The one big word.
  const state = $('state-chip');
  if (!wanted) chip(state, 'OFFLINE', 'idle');
  else if (!live) chip(state, 'NO LINK', 'bad');
  else if (t.cmd.estop) chip(state, 'E-STOP', 'bad');
  else if (t.motors === 'lost') chip(state, 'MOTORS LOST', 'bad');
  else if (t.state === 'aligning') chip(state, `ALIGNING ${Math.round(100 * t.align)}%`, 'warn');
  else if (t.state === 'moving') chip(state, 'DRIVING', 'good');
  else chip(state, 'READY', 'info');
  state.title = state.textContent === 'READY'
    ? 'Wheels stationary. This status does not lock manual controls.' : '';

  // Health.
  if (!wanted) chip($('h-link'), 'OFF', 'idle');
  else if (live) chip($('h-link'), 'FRESH', 'good');
  else if (t && age() < DEAD_S) chip($('h-link'), 'STALE', 'warn');
  else chip($('h-link'), 'DEAD', 'bad');

  if (!live) {
    // Old values would look current. Blank them.
    for (const id of ['h-control', 'h-motors', 'h-camera', 'h-loop', 'h-servos', 'h-battery']) chip($(id), '--', 'idle');
    setText('h-temp', '');
  } else {
    const now = performance.now();
    if (t.cmd.estop) chip($('h-control'), 'E-STOP', 'bad');
    else if (t.cmd.stale) chip($('h-control'), 'NO SIGNAL', 'warn');
    else chip($('h-control'), 'LIVE', 'good');

    if (t.motors === 'off') chip($('h-motors'), 'OFF', 'idle');
    else if (t.motors === 'lost') chip($('h-motors'), 'LOST', 'bad');
    else if (now - droppedAt < 2000) chip($('h-motors'), 'DROPPING', 'warn');
    else chip($('h-motors'), 'OK', 'good');

    if (!t.camera) chip($('h-camera'), 'OFF', 'idle');
    else if (t.camera.fps < 1) chip($('h-camera'), 'NO VIDEO', 'bad');
    else if (video.fps < 1) chip($('h-camera'), 'NOT SHOWN', 'warn');
    else chip($('h-camera'), video.fps + ' FPS', 'good');

    const loopOk = Math.abs(t.loop.hz - rover.loop_hz) <= 0.1 * rover.loop_hz;
    chip($('h-loop'), t.loop.hz.toFixed(1) + ' HZ', loopOk ? 'good' : 'bad');

    // The Pico drives the lift and pan servos and reads the battery.
    const board = pico(t);
    if (board.status === 'off') chip($('h-servos'), 'OFF', 'idle');
    else if (board.status === 'lost') chip($('h-servos'), 'NO PICO', 'bad');
    else if (board.failsafe) chip($('h-servos'), 'FAILSAFE', 'warn');     // the Pico is not being commanded
    else if (rover.pico === 'fake') chip($('h-servos'), 'FAKE', 'idle');
    else chip($('h-servos'), 'OK', 'good');

    if (protection?.blocked) chip($('h-battery'), 'STOP', 'bad');
    else if (board.status !== 'ok') chip($('h-battery'), '--', 'idle');
    else if (!b) chip($('h-battery'), 'NO READING', 'bad');
    else chip($('h-battery'), `${level.toUpperCase()} ${b.cell_v.toFixed(2)} V`,
              level === 'stop' || level === 'empty' ? 'bad' : level === 'low' ? 'warn' : 'good');
    setText('h-temp', t.temp_c === null ? '' : 'Jetson ' + t.temp_c.toFixed(1) + ' °C');
  }

  updateDisplayHealth(t);

  // Log status and the buttons that need a rover.
  const logLabel = $('log-label');
  const recording = recordingDisplay(t, live);
  logLabel.textContent = recording.text;
  logLabel.title = recording.title;
  for (const tone of ['good', 'warn', 'bad']) logLabel.classList.toggle(tone, recording.tone === tone);
  $('new-run').disabled = !canControl || !live;
  $('stop-log').disabled = !canControl || !recording.canStop;
  videoRecording.update(t?.video_recording, live);
  panReference.tick(live && link.connected);
  $('zero-battery').disabled = !canControl || !b;
  // Lift and pan need the Pico. Without it the buttons are greyed out.
  const servosReady = canDrive && live && !t.cmd.estop && pico(t).status === 'ok';
  const panBlocked = !canDrive ? 'Monitor only: PAN is disabled on this page.'
    : !wanted ? 'Connect to use PAN.' : !live ? 'Waiting for fresh telemetry before PAN is available.'
    : t.cmd.estop ? 'Release E-STOP before using PAN.'
    : pico(t).status !== 'ok' ? 'Pico unavailable: check the Pico connection.' : '';
  for (const button of drive.auxButtons) {
    button.disabled = !servosReady;
    button.dataset.inputHint ||= button.title;
    button.title = panBlocked || button.dataset.inputHint;
  }

  const estop = $('estop');
  estop.disabled = !canControl || !live;
  const engaged = live && t.cmd.estop;
  estop.textContent = engaged ? 'RELEASE' : 'E-STOP';
  estop.classList.toggle('engaged', engaged);
  $('camera-estop').disabled = !canControl || !live;
  $('camera-estop').textContent = engaged ? 'RELEASE E-STOP' : 'E-STOP';
  $('hammer-estop').disabled = !canHammer || !live;
  $('hammer-estop').textContent = engaged ? 'RELEASE E-STOP' : 'E-STOP';
  const inputHint = drive.gamepadName ? 'gamepad: sticks drive, D-pad pans, B stops'
    : view === 'control' ? 'hold to drive' : 'keyboard and pad';
  const driveNote = !wanted ? 'Connect to drive' : !live ? 'Waiting for telemetry'
    : t.cmd.estop ? 'E-STOP active'
    : t.motors === 'lost' ? 'Motors disconnected' : t.motors === 'off' ? 'Drive motors off' : inputHint;
  setText('gamepad-label', driveNote);
  $('gamepad-label').title = driveNote === inputHint ? inputHint : driveNote + '. ' + inputHint;
  $('gamepad-label').classList.toggle('control-unavailable', driveNote !== inputHint);

  updateVideoBanner(t, live, wanted, engaged);
}

function updateVideoBanner(t, live, wanted, engaged) {
  if (videoMemory.address !== link.address) {
    videoMemory.address = link.address;
    videoMemory.baseline = video.lastFrameAt;
    videoMemory.seen = false;
  }
  if (video.address === link.address && video.lastFrameAt > videoMemory.baseline) videoMemory.seen = true;
  const picture = videoMemory.seen;
  let message = '', alert = false;
  if (!video.supported) message = 'Use Chrome / Edge on localhost';
  else if (engaged || t?.cmd?.estop) { message = 'E-STOP'; alert = true; }
  else if (live && rover && !rover.camera) message = 'The camera is off on the rover';
  else if (!picture && !wanted) message = 'Not connected to the rover';
  else if (!picture) message = wanted && !live ? 'No link to the rover' : 'Waiting for video...';
  const banner = $('video-banner');
  banner.hidden = message === '';
  $('video-banner-message').textContent = message;
  banner.classList.toggle('alert', alert);
  banner.title = picture ? `Last video frame age: ${Math.max(0, (performance.now() - video.lastFrameAt) / 1000).toFixed(1)} s. Link ${live ? 'fresh' : 'unavailable'}; retained image is display only.` : 'Waiting for a video frame from this rover address.';
  setText('video-info', picture ? `${$('video').width} x ${$('video').height}` + (video.skipped ? `, ${video.skipped} skips` : '') : '');
}

// ── controls ─────────────────────────────────────────────────────────
function setEstop(on) {
  drive.reset(on ? 'estop_on' : 'estop_release');
  hammerPanel.stop();
  if (on) jumperPanel.suspend('E-STOP requested. Release the stop before operating.');
  sendControl({ type: 'estop', on });
}

function sendControl(message) {
  if (canControl) link.send(message.type === 'drive'
    ? { ...message, input_debug: drive.inputDebug({ telemetry_age_ms: latest === null ? null : age() * 1000 }) }
    : message);
}

function stopInput() {
  drive.reset('stop_input');
  hammerPanel.stop();
  if (canDrive) link.send({ type: 'drive', forward: 0, turn: 0, pan: 0, input_debug: drive.inputDebug() });
}

function remember(key, value) {
  try { localStorage.setItem(key, value); } catch (e) { /* private window: fine, just not remembered */ }
}
function recall(key) {
  try { return localStorage.getItem(key); } catch (e) { return null; }
}

function connect() {
  const address = $('rover-address').value.trim();
  if (!address) { $('rover-address').focus(); return; }
  if (link.address && link.address !== address) jumperPanel.suspend('Rover address changed; Scout output stop requested.');
  remember('rover-address', address);
  if (videoMemory.address !== address) {
    videoMemory.address = address; videoMemory.baseline = video.lastFrameAt; videoMemory.seen = false;
  }
  link.connect(address);
  mappingLink.connect(address);
  if (view !== 'control') video.connect(address);
}

function disconnect() {
  jumperPanel.suspend('Disconnected by operator; Scout output stop requested.');
  stopInput();
  link.disconnect();
  mappingLink.disconnect();
  video.disconnect();
  logEvent('disconnected');
}

$('connect').addEventListener('click', () => (link.address === null ? connect() : disconnect()));
$('rover-address').addEventListener('keydown', e => { if (e.key === 'Enter') { e.target.blur(); connect(); } });

$('new-run').addEventListener('click', () => {
  const tag = $('run-tag').value.trim() || 'run';
  displayMemory.operatorNewRun = true;
  sendControl({ type: 'new_run', tag });
});
$('stop-log').addEventListener('click', () => sendControl({ type: 'stop_log' }));
$('zero-battery').addEventListener('click', () => sendControl({ type: 'zero_battery' }));
$('reset-view').addEventListener('click', () => { resetView(); logEvent('view reset (plots, path, statistics)'); });
$('hammer-estop').addEventListener('click', () => setEstop(!(latest && latest.cmd.estop)));
$('estop').addEventListener('click', () => setEstop(!(latest && latest.cmd.estop)));
$('camera-estop').addEventListener('click', () => setEstop(!(latest && latest.cmd.estop)));

function cameraGeometry(width, height, header, aspect, size, narrow, expanded) {
  const ratio = Number.isFinite(aspect) && aspect > 0 ? aspect : 16 / 9;
  const scale = Math.max(.5, Math.min(1, Number(size) / 100 || 1));
  const innerWidth = Math.max(1, width - 12);
  const widthLimit = !expanded && !narrow ? Math.min(innerWidth * .64, innerWidth - 12 - 370) : innerWidth;
  const heightLimit = expanded ? height - 12 - header - 2
    : narrow ? Infinity : Math.max(300, Math.min(520, height * .62)) - header - 2;
  const videoWidth = Math.max(1, Math.min(widthLimit - 2, heightLimit * ratio)) * scale;
  return { width: videoWidth + 2, height: videoWidth / ratio + header + 2, ratio };
}

function fitCameraLayout() {
  if (view === 'control') return;
  const main = $('dashboard'), canvas = $('video');
  const overlay = $('video-recording'), hud = $('video-hud');
  const overlayBottom = (overlay?.offsetTop || 0) + (overlay?.offsetHeight || 0);
  const hudHeight = hud?.offsetHeight || 0;
  const layout = cameraGeometry(main.clientWidth, main.clientHeight, $('camera-head').offsetHeight,
    canvas.width / canvas.height, $('camera-size').value, window.innerWidth <= 1000, cameraExpanded);
  for (const [key, value] of Object.entries({ '--camera-width': layout.width + 'px',
    '--camera-row-height': layout.height + 'px', '--camera-aspect': String(layout.ratio),
    '--video-overlay-bottom': overlayBottom + 'px', '--video-hud-height': hudHeight + 'px' })) {
    if (main.style.getPropertyValue(key) !== value) { main.style.setProperty(key, value); dirty = true; }
  }
}

function setCameraExpanded(expanded) {
  if (expanded && !cameraExpanded) { cameraRestoreSize = $('camera-size').value; $('camera-size').value = '100'; }
  else if (!expanded && cameraExpanded) $('camera-size').value = cameraRestoreSize;
  cameraExpanded = expanded;
  document.body.dataset.cameraExpanded = String(expanded);
  $('camera-expand').textContent = expanded ? 'RESTORE' : 'EXPAND';
  $('camera-expand').setAttribute('aria-expanded', String(expanded));
  $('camera-estop').hidden = !expanded;
  fitCameraLayout();
}
function resizeCamera() { fitCameraLayout(); }
function onCameraSizeKey(event) {
  // Camera-size keystrokes belong to this selector, never driving or PAN.
  // Space retains the shared emergency-stop action.
  event.stopPropagation();
  if (event.code === 'Space') {
    event.preventDefault();
    if (!event.repeat) setEstop(true);
  }
}
function onCameraEscape(event) {
  if (event.code !== 'Escape' || !cameraExpanded || document.fullscreenElement) return;
  event.preventDefault(); event.stopPropagation();
  setCameraExpanded(false);
}
$('camera-expand').addEventListener('click', () => setCameraExpanded(!cameraExpanded));
$('camera-size').addEventListener('change', resizeCamera);
$('camera-size').addEventListener('keydown', onCameraSizeKey);
window.addEventListener('keydown', onCameraEscape, true);

function toggleFullscreen() {
  if (document.fullscreenElement === $('video-wrap')) document.exitFullscreen();
  else $('video-wrap').requestFullscreen();
}
function onFullscreenChange() {
  $('exit-fullscreen').hidden = document.fullscreenElement !== $('video-wrap');
}
$('fullscreen').addEventListener('click', toggleFullscreen);
$('exit-fullscreen').addEventListener('click', toggleFullscreen);

function updateSpeedLimit() {
  drive.speedLimit = Number($('speed-limit').value) / 100;
  $('speed-limit-value').textContent = $('speed-limit').value + '%';
}
$('speed-limit').addEventListener('input', updateSpeedLimit);

$('station-view').addEventListener('change', e => {
  stopInput();
  const next = new URL(location.href);
  next.searchParams.set('view', e.target.value);
  const address = $('rover-address').value.trim();
  if (address) next.searchParams.set('rover', address);
  location.href = next.href;
});
window.addEventListener('pagehide', stopInput);
document.addEventListener('fullscreenchange', onFullscreenChange);

function setTheme(theme) {
  document.documentElement.dataset.theme = theme;
  $('theme').textContent = theme === 'dark' ? '☀' : '☾';
  $('theme').setAttribute('aria-label', `Switch to ${theme === 'dark' ? 'light' : 'dark'} theme`);
  $('theme').title = $('theme').getAttribute('aria-label');
  document.querySelector('meta[name="theme-color"]').content = theme === 'dark' ? '#000000' : '#eeeeee';
  remember('theme', theme);
  refreshColors();
  dirty = true;
}
$('theme').addEventListener('click', () => setTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark'));

// ── timers ───────────────────────────────────────────────────────────

// Drive commands, 20 a second. This is also the rover's deadman signal:
// when these stop -- page closed, laptop asleep, Wi-Fi gone -- the rover
// stops within its command timeout.
setInterval(() => {
  hammerPanel.tick(fresh(), latest);
  jumperPanel.tick(link.connected && fresh(), latest);
  if (!canDrive) return;
  if (!fresh() || latest?.cmd.estop) drive.reset(latest?.cmd.estop ? 'estop' : 'telemetry_stale');
  const command = drive.read();
  sendControl({ type: 'drive', ...command });
  if (!fresh()) { drive.show(command.forward, command.turn); drive.showAux(command.lift, command.pan); }
}, 50);

setInterval(() => { tlmHz = received; received = 0; }, 1000);
setInterval(slowUpdate, 200);
window.addEventListener('resize', () => { fitCameraLayout(); dirty = true; });

(function frame() {
  mappingPanel.draw();
  if (dirty) { dirty = false; redraw(); }
  requestAnimationFrame(frame);
})();

// ── start ────────────────────────────────────────────────────────────
setTheme(recall('theme') === 'light' ? 'light' : 'dark');
const fromUrl = params.get('rover');
$('rover-address').value = fromUrl || recall('rover-address') || '';
logEvent('station page ready');
slowUpdate();
dirty = true;
if (fromUrl) connect();
