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

import { $, refreshColors, signed } from './util.js';
import { RoverLink } from './link.js';
import { VideoView } from './video.js';
import { DriveInput } from './drive.js';
import { StripChart } from './plots.js';
import { AttitudeView, PathView, WheelsView, shortName } from './views.js';

const RAD = Math.PI / 180;
const MAX_PLOTS = 4;
const FRESH_S = 0.5;        // telemetry newer than this: link is fine
const DEAD_S = 2.0;         // older than this: treat the link as lost

// ── state ────────────────────────────────────────────────────────────
let rover = null;           // the rover's description ("hello"), null until first connected
let latest = null;          // newest telemetry message
let latestAt = 0;           // performance.now() when it arrived
let dirty = false;          // something changed since the last redraw
let received = 0, tlmHz = 0;
let seenEvent = 0;          // id of the newest rover event already in the log
let lastRowT = -1;          // rover time of the last telemetry row written to the log
let imuStaleSeen = 0, imuStaleAt = -1e9;
let droppedSeen = 0, droppedAt = -1e9;
let stats = newStats();

function newStats() {
  return { distance: 0, moving: 0, maxSpeed: 0, maxCurrent: 0, maxTilt: 0, prev: null };
}

const age = () => (performance.now() - latestAt) / 1000;
const fresh = () => latest !== null && age() < FRESH_S;

// ── panels ───────────────────────────────────────────────────────────
refreshColors();
const attitude = new AttitudeView($('attitude'));
const pathView = new PathView($('path'));
const wheelsView = new WheelsView($('wheels'));
const video = new VideoView($('video'));
const drive = new DriveInput({ pad: $('pad'), dot: $('pad-dot'), auxButtons: $('aux').querySelectorAll('button'),
                               onEstop: () => setEstop(true) });
const link = new RoverLink({ onHello, onTelemetry, onStatus });

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
  const limits = rover && rover.battery;
  if (!limits) return 'ok';
  return b.cell_v < limits.empty_cell_v ? 'empty' : b.cell_v < limits.low_cell_v ? 'low' : 'ok';
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
    { label: 'TILT', unit: 'deg', series: ['roll', 'pitch'], digits: 1, minSpan: 2, on: true, get: t => [t.rpy[0], t.rpy[1]] },
    { label: 'YAW', unit: 'deg', series: ['yaw'], digits: 1, minSpan: 5, on: false, get: t => [t.rpy[2]] },
    { label: 'GYRO BIAS', unit: 'deg/s', series: ['x', 'y', 'z'], digits: 3, minSpan: 0.05, on: false, get: t => t.gyro_bias },
  ];
  if (rover.pico && rover.pico !== 'off') {
    // No reading -> nothing is added, and the line has a gap there.
    plots.push(
      { label: 'BATTERY', unit: 'V', series: ['battery'], digits: 2, minSpan: 0.5, on: false, get: t => battery(t) && [battery(t).v] },
      { label: 'BATTERY DRAW', unit: 'A', series: ['battery'], digits: 2, minSpan: 0.5, on: false, get: t => battery(t) && [battery(t).a] });
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
  const restarted = rover !== null && rover.boot !== hello.boot;
  const sameWheels = rover !== null && JSON.stringify(rover.wheels) === JSON.stringify(hello.wheels)
    && rover.pico === hello.pico;            // the battery plots exist only with a Pico
  rover = hello;
  if (restarted) { seenEvent = 0; logEvent('rover program was restarted'); }
  attitude.setRover(hello);
  pathView.setRover(hello);
  if (!sameWheels) { wheelsView.setRover(hello); buildPlots(); buildLogHeader(); }
  logEvent('connected to rover at ' + link.address);
}

function onTelemetry(t) {
  // The rover's clock restarts at zero on NEW RUN: begin a clean view.
  if (latest && t.t < latest.t - 1) resetView();
  latest = t;
  latestAt = performance.now();
  received++;

  for (const plot of plots) {
    const values = plot.get(t);
    if (values) plot.chart.push(t.t, values);
  }
  pathView.add(t.pos[0], t.pos[1]);
  addToStats(t);

  for (const [id, text] of t.events) {
    if (id > seenEvent) { seenEvent = id; logEvent(text, /e-stop on|lost/i.test(text)); }
  }
  if (t.t - lastRowT >= 0.5 || t.t < lastRowT) { lastRowT = t.t; logRow(t); }
  if (t.imu_stale > imuStaleSeen) imuStaleAt = performance.now();
  imuStaleSeen = t.imu_stale;
  if (t.loop.dropped > droppedSeen) droppedAt = performance.now();
  droppedSeen = t.loop.dropped;
  dirty = true;
}

function onStatus(status) {
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
  const tilt = Math.acos(Math.cos(t.rpy[0] * RAD) * Math.cos(t.rpy[1] * RAD)) / RAD;
  stats.maxTilt = Math.max(stats.maxTilt, tilt);
}

function resetView() {
  for (const plot of plots) plot.chart.clear();
  pathView.clear();
  stats = newStats();
  lastRowT = -1;
  dirty = true;
}

// ── log panel ────────────────────────────────────────────────────────
const logBody = $('log');
const MAX_LOG_LINES = 400;

function addLogLine(text, className) {
  const atBottom = logBody.scrollTop + logBody.clientHeight >= logBody.scrollHeight - 8;
  const line = document.createElement('div');
  line.textContent = text;
  if (className) line.className = className;
  logBody.append(line);
  while (logBody.childElementCount > MAX_LOG_LINES) logBody.firstElementChild.remove();
  if (atBottom) logBody.scrollTop = logBody.scrollHeight;    // follow, unless the operator scrolled up to read
}

function logEvent(text, bad = false) {
  const clock = new Date().toLocaleTimeString('en-GB', { timeZone: 'Asia/Seoul', hour12: false });
  addLogLine(clock + '  ' + text, bad ? 'event bad' : 'event');
}

const col = (text, width) => String(text).padStart(width);

// Two rows a second of the numbers worth reading back later. Per-wheel
// values are on the WHEELS panel and in the plots; everything, at 100 Hz,
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

function redraw() {
  const t = latest;
  if (!t || !rover) {
    // Nothing from a rover yet: show the empty views rather than blank boxes.
    attitude.draw(0, 0);
    pathView.draw(0, 0, 0);
    return;
  }

  setText('hud-speed', Math.abs(meanSpeed(t)).toFixed(2));
  setText('hud-yaw', signed(t.rpy[2], 1));
  setText('hud-roll', signed(t.rpy[0], 1));
  setText('hud-pitch', signed(t.rpy[1], 1));
  setText('hud-cmd', signed(t.cmd.forward, 2) + ' / ' + signed(t.cmd.turn, 2));

  attitude.draw(t.rpy[0], t.rpy[1]);
  setText('att-roll', signed(t.rpy[0], 1) + '°');
  setText('att-pitch', signed(t.rpy[1], 1) + '°');
  setText('att-yaw', signed(t.rpy[2], 1) + '°');

  pathView.draw(t.pos[0], t.pos[1], t.rpy[2]);
  setText('path-scale', 'grid ' + pathView.gridStep + ' m, start direction is up');
  setText('pos-x', signed(t.pos[0], 3));
  setText('pos-y', signed(t.pos[1], 3));
  setText('pos-r', Math.hypot(t.pos[0], t.pos[1]).toFixed(3));

  wheelsView.update(t.wheel_speed, t.wheel_current);
  for (const plot of plots) if (plot.on) plot.chart.draw();
  drive.show(t.cmd.forward, t.cmd.turn);
  drive.showAux(t.cmd.lift ?? 0, t.cmd.pan ?? 0);

  const b = battery(t);
  setText('st-battery', b ? b.a.toFixed(2) + ' A' : '--');
  setText('st-mah', b ? b.mah.toFixed(0) + ' mAh' : '--');
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
  if (!names.length) return;
  body.replaceChildren();
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
  const t = latest, wanted = link.address !== null, live = fresh();

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
  const level = b ? batteryLevel(b) : null;
  setText('battery', b ? `BAT ${b.v.toFixed(1)} V` + (level === 'ok' ? '' : ' ' + level.toUpperCase()) : 'BAT -- V');
  batteryCell.className = 'mission-cell ' + (!b ? 'muted' : level === 'empty' ? 'bad' : level === 'low' ? 'warn' : '');
  if (b) batteryCell.title = `${b.cell_v.toFixed(2)} V per cell, ${b.a.toFixed(2)} A, ${b.w.toFixed(0)} W`;
  setText('video-rate', video.fps > 0 ? `VIDEO ${video.fps} fps  ${video.mbps.toFixed(1)} Mbit/s` : 'VIDEO -- fps');

  // The one big word.
  const state = $('state-chip');
  if (!wanted) chip(state, 'OFFLINE', 'idle');
  else if (!live) chip(state, 'NO LINK', 'bad');
  else if (t.cmd.estop) chip(state, 'E-STOP', 'bad');
  else if (t.motors === 'lost') chip(state, 'MOTORS LOST', 'bad');
  else if (t.state === 'aligning') chip(state, `ALIGNING ${Math.round(100 * t.align)}%`, 'warn');
  else if (t.state === 'moving') chip(state, 'DRIVING', 'good');
  else chip(state, 'STOPPED', 'info');

  // Health.
  if (!wanted) chip($('h-link'), 'OFF', 'idle');
  else if (live) chip($('h-link'), 'FRESH', 'good');
  else if (t && age() < DEAD_S) chip($('h-link'), 'STALE', 'warn');
  else chip($('h-link'), 'DEAD', 'bad');

  if (!live) {
    // Old values would look current. Blank them.
    for (const id of ['h-control', 'h-motors', 'h-imu', 'h-camera', 'h-loop', 'h-servos', 'h-battery']) chip($(id), '--', 'idle');
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

    if (rover.imu === 'fake') chip($('h-imu'), 'FAKE', 'idle');
    else if (now - imuStaleAt < 2000) chip($('h-imu'), 'STALE', 'warn');
    else chip($('h-imu'), 'OK', 'good');

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

    if (board.status !== 'ok') chip($('h-battery'), '--', 'idle');
    else if (!b) chip($('h-battery'), 'NO READING', 'bad');
    else chip($('h-battery'), `${level.toUpperCase()} ${b.cell_v.toFixed(2)} V`,
              level === 'empty' ? 'bad' : level === 'low' ? 'warn' : 'good');
    setText('h-temp', t.temp_c === null ? '' : 'Jetson ' + t.temp_c.toFixed(1) + ' °C');
  }

  // Log status and the buttons that need a rover.
  const logLabel = $('log-label');
  if (live && t.log) { logLabel.textContent = `REC ${t.log.file}  ${t.log.rows} rows`; logLabel.className = 'status-text good'; }
  else { logLabel.textContent = 'NOT LOGGING'; logLabel.className = 'status-text'; }
  $('new-run').disabled = !live;
  $('stop-log').disabled = !(live && t.log);
  $('zero-battery').disabled = !b;
  // Lift and pan need the Pico. Without it the buttons are greyed out.
  const servosReady = live && pico(t).status === 'ok';
  for (const button of drive.auxButtons) button.disabled = !servosReady;

  const estop = $('estop');
  const engaged = live && t.cmd.estop;
  estop.textContent = engaged ? 'RELEASE' : 'E-STOP';
  estop.classList.toggle('engaged', engaged);
  setText('gamepad-label', drive.gamepadName ? 'gamepad: sticks drive, D-pad lifts and pans, B stops' : 'keyboard and pad');

  // What is over the picture.
  const banner = $('video-banner');
  const picture = performance.now() - video.lastFrameAt < 2000;
  let message = '', alert = false;
  if (!video.supported) message = 'This browser cannot show the video. Use Chrome or Edge, and open this page with station.py on this laptop (http://localhost).';
  else if (!wanted) message = 'Not connected to the rover';
  else if (engaged) { message = 'E-STOP'; alert = true; }
  else if (live && rover && !rover.camera) message = 'The camera is off on the rover';
  else if (!picture) message = wanted && !live ? 'No link to the rover' : 'Waiting for video...';
  banner.hidden = message === '';
  banner.textContent = message;
  banner.classList.toggle('alert', alert);
  setText('video-info', picture ? `${$('video').width} x ${$('video').height}` + (video.skipped ? `, ${video.skipped} skips` : '') : '');
}

// ── controls ─────────────────────────────────────────────────────────
function setEstop(on) {
  link.send({ type: 'estop', on });
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
  remember('rover-address', address);
  link.connect(address);
  video.connect(address);
}

function disconnect() {
  link.disconnect();
  video.disconnect();
  logEvent('disconnected');
}

$('connect').addEventListener('click', () => (link.address === null ? connect() : disconnect()));
$('rover-address').addEventListener('keydown', e => { if (e.key === 'Enter') { e.target.blur(); connect(); } });

$('new-run').addEventListener('click', () => {
  const tag = $('run-tag').value.trim() || 'run';
  link.send({ type: 'new_run', tag });
});
$('stop-log').addEventListener('click', () => link.send({ type: 'stop_log' }));
$('zero-battery').addEventListener('click', () => link.send({ type: 'zero_battery' }));
$('reset-view').addEventListener('click', () => { resetView(); logEvent('view reset (plots, path, statistics)'); });
$('estop').addEventListener('click', () => setEstop(!(latest && latest.cmd.estop)));
$('fullscreen').addEventListener('click', () => $('video-wrap').requestFullscreen());

$('speed-limit').addEventListener('input', e => {
  drive.speedLimit = e.target.value / 100;
  $('speed-limit-value').textContent = e.target.value + '%';
});

function setTheme(theme) {
  document.documentElement.dataset.theme = theme;
  $('theme').textContent = theme === 'dark' ? '☀' : '☾';
  remember('theme', theme);
  refreshColors();
  dirty = true;
}
$('theme').addEventListener('click', () => setTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark'));

// ── timers ───────────────────────────────────────────────────────────

// Drive commands, 20 a second. This is also the rover's deadman signal:
// when these stop -- page closed, laptop asleep, Wi-Fi gone -- the rover
// stops within its command timeout. A hidden tab sends nothing on purpose:
// nobody should be driving a rover they cannot see.
setInterval(() => {
  const command = drive.read();
  if (!document.hidden) link.send({ type: 'drive', ...command });
  if (!fresh()) { drive.show(command.forward, command.turn); drive.showAux(command.lift, command.pan); }
}, 50);

setInterval(() => { tlmHz = received; received = 0; }, 1000);
setInterval(slowUpdate, 200);
window.addEventListener('resize', () => { dirty = true; });

(function frame() {
  if (dirty) { dirty = false; redraw(); }
  requestAnimationFrame(frame);
})();

// ── start ────────────────────────────────────────────────────────────
setTheme(recall('theme') === 'light' ? 'light' : 'dark');
const fromUrl = new URLSearchParams(location.search).get('rover');
$('rover-address').value = fromUrl || recall('rover-address') || '';
logEvent('station page ready');
slowUpdate();
dirty = true;
if (fromUrl) connect();
