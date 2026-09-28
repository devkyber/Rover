"""
Live telemetry panel, served by the same Flask/SocketIO server the phone
joystick already uses.

    phone   ->  http://<ip>:5000/         joystick
    laptop  ->  http://<ip>:5000/panel    this dashboard

WHY A BROWSER PANEL AND NOT A BIGGER PYGAME WINDOW
--------------------------------------------------
The pygame window only exists on the machine holding the USB cables, and
during a drive that machine is riding on the rover. A browser page is
reachable from the phone in your hand and from a laptop on the desk at
the same time, over the socket we are already running. No new dependency
and no second process.

It shows only what you cannot get from the log afterwards: whether the
filter is behaving RIGHT NOW, so you can stop a bad run in the first ten
seconds instead of discovering it in replay.

The NIS table is the important half. It is the same consistency measure
replay.py prints, computed over a moving window, so you can watch a
tuning change take effect while driving instead of guessing.
"""
import time

import numpy as np

import config
from config import NIS_DOF

# How many recent samples the live NIS mean averages over. 300 at 100 Hz
# is about three seconds of driving -- long enough to be stable, short
# enough to react while you are still holding the joystick.
NIS_WINDOW = 300


class Telemetry:
    """
    Collects one snapshot per cycle and pushes it to the browser at a
    fixed, low rate.

    The filter runs at 100 Hz; a browser cannot draw that and does not
    need to. Emitting every cycle would make the socket, not the motors,
    the slowest thing in the loop -- so we build the payload only when
    it is actually time to send one.
    """

    def __init__(self, socketio, hz=15.0):
        self.socketio = socketio
        self.period = 1.0 / hz
        self.next_emit = 0.0
        self.path = []          # decimated x,y trace for the map panel
        self.last_path_t = -1.0

    def maybe_emit(self, *, t, odo, wheels, fwd, turn, raw_sent,
                   stale, stopped, ready, alignment_progress,
                   n, dropped, hz):
        now = time.monotonic()
        if now < self.next_emit:
            return
        self.next_emit = now + self.period

        state = odo.state
        p = odo.position
        v = odo.velocity
        rpy = odo.rpy_deg

        # Trace one point every 100 ms. At 100 Hz the raw path would be
        # 360 000 points in an hour and the browser would choke long
        # before that; the shape of the path survives decimation fine.
        if t - self.last_path_t > 0.1:
            self.last_path_t = t
            self.path.append([round(float(p[0]), 4), round(float(p[1]), 4)])
            if len(self.path) > 3000:
                del self.path[:1000]

        nis = {}
        for name, samples in state['nis'].items():
            window = samples[-NIS_WINDOW:]
            if window:
                nis[name] = [round(float(np.mean(window)), 3),
                             NIS_DOF.get(name, 1), len(samples)]

        self.socketio.emit('telemetry', {
            't': round(t, 2),
            'pos': [round(float(x), 4) for x in p],
            'vel': [round(float(x), 4) for x in v],
            'filter_pos': [round(float(x), 4) for x in odo.inertial_position],
            'rpy': [round(float(x), 2) for x in rpy],
            'bg': [round(float(np.degrees(x)), 3) for x in state['bg']],
            'ba': [round(float(x), 4) for x in state['ba']],
            'wheel_vel': [round(w['velocity_rads'], 3) for w in wheels],
            'wheel_cur': [round(w['current_A'], 3) for w in wheels],
            'ids': config.MOTOR_IDS,
            'stick': [round(fwd, 2), round(turn, 2)],
            'sent': list(raw_sent),
            'stale': bool(stale),
            'stopped': bool(stopped),
            'ready': bool(ready),
            'alignment_progress': round(float(alignment_progress), 3),
            'n': n, 'dropped': dropped, 'hz': round(hz, 1),
            'nis': nis,
            'path': self.path[-1200:],
        })


PANEL_HTML = """
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Rover live panel</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
<style>
  body { margin:0; background:#14181d; color:#dfe6ee;
         font-family: Consolas, ui-monospace, monospace; font-size:13px; }
  h1 { font-size:15px; margin:0; padding:10px 14px; background:#1c222a;
       border-bottom:1px solid #2b333d; font-weight:600; letter-spacing:.4px; }
  .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(330px,1fr));
          gap:10px; padding:10px; }
  .card { background:#1a1f26; border:1px solid #2b333d; border-radius:6px;
          padding:10px 12px; }
  .card h2 { font-size:11px; margin:0 0 8px; color:#8b98a7;
             text-transform:uppercase; letter-spacing:1px; font-weight:600; }
  canvas { width:100%; display:block; }
  table { width:100%; border-collapse:collapse; }
  td, th { padding:3px 6px; text-align:right; }
  th { color:#8b98a7; font-weight:600; font-size:11px; }
  td.k, th.k { text-align:left; color:#a9b6c4; }
  .big { font-size:19px; letter-spacing:.5px; }
  .ok   { color:#7ddc7d; } .warn { color:#ffcc55; } .bad { color:#ff6b6b; }
  .dim  { color:#6c7885; }
  .pill { display:inline-block; padding:2px 9px; border-radius:10px;
          font-size:11px; font-weight:600; }
  .pill.live { background:#1e4620; color:#7ddc7d; }
  .pill.dead { background:#4a1d1d; color:#ff6b6b; }
</style>
</head>
<body>
<h1>Rover live panel &nbsp;<span id="link" class="dim"></span></h1>
<div class="grid">

  <div class="card"><h2>status</h2>
    <table>
      <tr><td class="k">link</td><td id="s_link" class="big"></td></tr>
      <tr><td class="k">motion</td><td id="s_state" class="big"></td></tr>
      <tr><td class="k">stick fwd / turn</td><td id="s_stick"></td></tr>
      <tr><td class="k">sent (raw)</td><td id="s_sent"></td></tr>
      <tr><td class="k">rate</td><td id="s_hz"></td></tr>
      <tr><td class="k">cycles / dropped</td><td id="s_n"></td></tr>
      <tr><td class="k">t</td><td id="s_t"></td></tr>
    </table>
  </div>

  <div class="card"><h2>estimate</h2>
    <table>
      <tr><th class="k"></th><th>x</th><th>y</th><th>z</th></tr>
      <tr><td class="k">pos (m)</td><td id="p0"></td><td id="p1"></td><td id="p2"></td></tr>
      <tr><td class="k">vel (m/s)</td><td id="v0"></td><td id="v1"></td><td id="v2"></td></tr>
      <tr><th class="k"></th><th>roll</th><th>pitch</th><th>yaw</th></tr>
      <tr><td class="k">rpy (deg)</td><td id="r0"></td><td id="r1"></td><td id="r2"></td></tr>
      <tr><td class="k">gyro bias (deg/s)</td><td id="b0"></td><td id="b1"></td><td id="b2"></td></tr>
    </table>
  </div>

  <div class="card"><h2>consistency (NIS) &mdash; mean of last 3 s</h2>
    <table id="nis"><tr><td class="dim">waiting...</td></tr></table>
    <div class="dim" style="margin-top:8px; font-size:11px; line-height:1.5">
      ratio = mean / target. Outside 0.33-3: inspect the model, timing,
      and noise assumptions. NIS alone does not measure accuracy.
    </div>
  </div>

  <div class="card"><h2>path (top view)</h2>
    <canvas id="map" height="260"></canvas></div>

  <div class="card"><h2>velocity (m/s)</h2>
    <canvas id="chv" height="150"></canvas></div>

  <div class="card"><h2>attitude (deg)</h2>
    <canvas id="chr" height="150"></canvas></div>

  <div class="card"><h2>gyro bias (deg/s) &mdash; should settle flat</h2>
    <canvas id="chb" height="150"></canvas></div>

  <div class="card"><h2>wheel speed (rad/s)</h2>
    <canvas id="chw" height="150"></canvas></div>

</div>
<script>
const COLORS = ['#4fa3ff','#ffb454','#7ddc7d','#ff7de0'];
const N = 600;   // points kept per strip chart

class Strip {
  constructor(id, labels) {
    this.c = document.getElementById(id);
    this.labels = labels;
    this.data = labels.map(() => []);
  }
  push(vals) {
    vals.forEach((v, i) => {
      const a = this.data[i];
      a.push(v);
      if (a.length > N) a.shift();
    });
  }
  draw() {
    const c = this.c, dpr = window.devicePixelRatio || 1;
    const w = c.clientWidth, h = c.height;
    if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr; }
    const g = c.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);

    let lo = Infinity, hi = -Infinity;
    this.data.forEach(a => a.forEach(v => { if (v < lo) lo = v; if (v > hi) hi = v; }));
    if (!isFinite(lo)) { lo = -1; hi = 1; }
    if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
    const pad = (hi - lo) * 0.12; lo -= pad; hi += pad;
    const Y = v => h - 6 - (v - lo) / (hi - lo) * (h - 12);

    // zero line, so you can see sign at a glance
    if (lo < 0 && hi > 0) {
      g.strokeStyle = '#333c46'; g.beginPath();
      g.moveTo(0, Y(0)); g.lineTo(w, Y(0)); g.stroke();
    }
    this.data.forEach((a, i) => {
      if (!a.length) return;
      g.strokeStyle = COLORS[i % COLORS.length];
      g.lineWidth = 1.4; g.beginPath();
      a.forEach((v, k) => {
        const x = w * k / (N - 1);
        k ? g.lineTo(x, Y(v)) : g.moveTo(x, Y(v));
      });
      g.stroke();
    });
    // Legend across the top, sharing whatever width the card got. The
    // scale numbers own the right-hand 46 px, so the legend stops there
    // -- with four wheels a fixed spacing runs ID4 off the card.
    g.font = '11px Consolas, monospace';
    const step = (w - 52) / this.labels.length;
    this.labels.forEach((lb, i) => {
      const a = this.data[i];
      g.fillStyle = COLORS[i % COLORS.length];
      const txt = lb + (a.length ? ' ' + a[a.length - 1].toFixed(2) : '');
      g.fillText(txt, 6 + i * step, 13, step - 4);
    });
    g.fillStyle = '#6c7885';
    g.fillText(hi.toFixed(2), w - 44, 13);
    g.fillText(lo.toFixed(2), w - 44, h - 4);
  }
}

const chv = new Strip('chv', ['vx', 'vy', 'vz']);
const chr = new Strip('chr', ['roll', 'pitch', 'yaw']);
const chb = new Strip('chb', ['bgx', 'bgy', 'bgz']);
let chw = null;

function drawMap(path) {
  const c = document.getElementById('map'), dpr = window.devicePixelRatio || 1;
  const w = c.clientWidth, h = c.height;
  if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr; }
  const g = c.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  if (!path.length) return;

  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  path.forEach(p => {
    x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]);
    y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]);
  });
  // equal aspect -- a square drive must look square, or the panel lies
  const span = Math.max(x1 - x0, y1 - y0, 0.2) * 1.2;
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const s = Math.min(w, h) / span;
  const X = v => w / 2 + (v - cx) * s;
  const Y = v => h / 2 - (v - cy) * s;

  g.strokeStyle = '#333c46';
  g.beginPath(); g.moveTo(X(cx - span), Y(0)); g.lineTo(X(cx + span), Y(0));
  g.moveTo(X(0), Y(cy - span)); g.lineTo(X(0), Y(cy + span)); g.stroke();

  g.strokeStyle = '#4fa3ff'; g.lineWidth = 1.6; g.beginPath();
  path.forEach((p, k) => k ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1])));
  g.stroke();

  const a = path[0], b = path[path.length - 1];
  g.fillStyle = '#7ddc7d'; g.beginPath(); g.arc(X(a[0]), Y(a[1]), 4, 0, 7); g.fill();
  g.fillStyle = '#ff6b6b'; g.beginPath(); g.arc(X(b[0]), Y(b[1]), 4, 0, 7); g.fill();
  g.fillStyle = '#6c7885'; g.font = '11px Consolas, monospace';
  g.fillText(span.toFixed(2) + ' m across', 6, h - 6);
}

const set = (id, txt, cls) => {
  const e = document.getElementById(id);
  e.textContent = txt;
  if (cls !== undefined) e.className = cls;
};

const socket = io();
let latest = null;
socket.on('telemetry', d => { latest = d; });
socket.on('connect', () =>
  document.getElementById('link').textContent = location.host);

setInterval(() => {
  const d = latest; if (!d) return;

  set('s_link', d.stale ? 'NO SIGNAL' : 'connected',
      'big pill ' + (d.stale ? 'dead' : 'live'));
  const motion = !d.ready
      ? 'ALIGNING ' + (100 * d.alignment_progress).toFixed(0) + '%'
      : (d.stopped ? 'STOPPED (ZUPT)' : 'moving');
  set('s_state', motion, 'big ' + ((!d.ready || d.stopped) ? 'warn' : 'ok'));
  set('s_stick', d.stick[0].toFixed(2) + '  /  ' + d.stick[1].toFixed(2));
  set('s_sent', '[' + d.sent.join(', ') + ']');
  set('s_hz', d.hz.toFixed(1) + ' Hz', Math.abs(d.hz - 100) > 10 ? 'bad' : 'ok');
  set('s_n', d.n + ' / ' + d.dropped, d.dropped ? 'warn' : 'dim');
  set('s_t', d.t.toFixed(1) + ' s', 'dim');

  ['p', 'v', 'r', 'b'].forEach((k, j) => {
    const src = [d.pos, d.vel, d.rpy, d.bg][j];
    const dec = [3, 3, 1, 3][j];
    for (let i = 0; i < 3; i++) set(k + i, src[i].toFixed(dec));
  });

  let rows = '<tr><th class="k">obs</th><th>mean</th><th>target</th>'
           + '<th>ratio</th><th class="k">&nbsp;verdict</th></tr>';
  Object.keys(d.nis).sort().forEach(k => {
    const mean = d.nis[k][0], dof = d.nis[k][1], ratio = mean / dof;
    let cls = 'ok', verdict = 'in heuristic band';
    if (ratio > 3)         { cls = 'bad';  verdict = 'inspect high ratio'; }
    else if (ratio < 0.33) { cls = 'warn'; verdict = 'inspect low ratio'; }
    rows += '<tr><td class="k">' + k + '</td><td>' + mean.toFixed(2) + '</td>'
          + '<td class="dim">' + dof + '</td><td class="' + cls + '">'
          + ratio.toFixed(2) + 'x</td>'
          + '<td class="k ' + cls + '">&nbsp;' + verdict + '</td></tr>';
  });
  document.getElementById('nis').innerHTML = rows;

  if (!chw) chw = new Strip('chw', d.ids.map(i => 'ID' + i));
  chv.push(d.vel); chr.push(d.rpy); chb.push(d.bg); chw.push(d.wheel_vel);
  chv.draw(); chr.draw(); chb.draw(); chw.draw();
  drawMap(d.path);
}, 66);
</script>
</body>
</html>
"""
