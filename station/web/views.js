// The three rover-shaped displays: tilt (3D), path (top view), wheels.

import { clamp, colors, fitCanvas, niceStep } from './util.js';

const RAD = Math.PI / 180;

// ── small vector helpers ─────────────────────────────────────────────
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = a => { const n = Math.hypot(...a); return [a[0] / n, a[1] / n, a[2] / n]; };
const mean = pts => pts.reduce((s, p) => [s[0] + p[0], s[1] + p[1], s[2] + p[2]], [0, 0, 0]).map(v => v / pts.length);

function shade(hexColor, k) {
  const n = parseInt(hexColor.slice(1), 16);
  const c = [n >> 16, (n >> 8) & 255, n & 255].map(v => Math.round(v * k));
  return `rgb(${c[0]},${c[1]},${c[2]})`;
}

// ── ATTITUDE ─────────────────────────────────────────────────────────
// A box-and-wheels model of the rover, tilted by roll and pitch, seen from
// behind and a little above. Yaw is left out on purpose: the view follows
// the rover's heading, so "leaning left" always looks like leaning left.
// Heading is on the PATH panel.
//
// Rover axes, as everywhere in this project: +x forward, +y left, +z up.

// Where the viewer stands, as a direction from the rover: behind, slightly
// to the right, above.
const EYE = unit([-Math.cos(28 * RAD) * Math.cos(24 * RAD), -Math.cos(28 * RAD) * Math.sin(24 * RAD), Math.sin(28 * RAD)]);
const RIGHT = unit(cross([-EYE[0], -EYE[1], -EYE[2]], [0, 0, 1]));   // screen right
const UP = cross(EYE, RIGHT);                                        // screen up
const LIGHT = unit([EYE[0] + 0.2, EYE[1] - 0.5, EYE[2] + 0.9]);

// A closed solid from two matching rings of points (a box is two 4-point
// rings, a wheel two 10-point rings). Returns its faces, each with a normal
// that is checked to point outward.
function solid(ringA, ringB, kind) {
  const centre = mean([...ringA, ...ringB]);
  const faces = [ringA, ringB];
  for (let i = 0; i < ringA.length; i++) {
    const j = (i + 1) % ringA.length;
    faces.push([ringA[i], ringA[j], ringB[j], ringB[i]]);
  }
  return faces.map(pts => {
    let normal = unit(cross(sub(pts[1], pts[0]), sub(pts[2], pts[0])));
    if (dot(normal, sub(mean(pts), centre)) < 0) normal = normal.map(v => -v);
    return { pts, normal, kind };
  });
}

function buildRover(track, wheelRadius) {
  const wheelWidth = 0.032;
  const axleGap = Math.max(track * 1.25, 0.2);          // front axle to rear axle (schematic)
  const halfLen = axleGap / 2 + 0.03;
  const halfWid = Math.max(track / 2 - wheelWidth / 2 - 0.006, 0.03);
  const box = z => [[halfLen, halfWid, z], [halfLen, -halfWid, z], [-halfLen, -halfWid, z], [-halfLen, halfWid, z]];
  const top = 0.06;
  const faces = solid(box(-0.01), box(top), 'body');
  // An arrow lying on the roof, pointing forward, so "which way is the
  // front" is never a guess. Drawn after the solids (see draw()).
  const arrow = {
    pts: [[halfLen * 0.9, 0, top], [halfLen * 0.2, halfWid * 0.7, top], [halfLen * 0.2, -halfWid * 0.7, top]],
    normal: [0, 0, 1], kind: 'arrow',
  };

  for (const sx of [1, -1]) for (const sy of [1, -1]) {
    const ring = y => Array.from({ length: 10 }, (_, k) => {
      const a = k / 10 * 2 * Math.PI;
      return [sx * axleGap / 2 + wheelRadius * Math.cos(a), y, wheelRadius * Math.sin(a)];
    });
    const y = sy * track / 2;
    faces.push(...solid(ring(y - wheelWidth / 2), ring(y + wheelWidth / 2), 'wheel'));
  }
  faces.push(arrow);
  const reach = Math.hypot(halfLen, track / 2 + wheelWidth, wheelRadius);
  return { faces, reach, ground: -wheelRadius };
}

export class AttitudeView {
  constructor(canvas) {
    this.canvas = canvas;
    this.setRover({ track_m: 0.16, wheel_radius_m: 0.055 });
  }

  setRover(rover) {
    this.model = buildRover(rover.track_m, rover.wheel_radius_m);
  }

  draw(rollDeg, pitchDeg) {
    const { g, w, h } = fitCanvas(this.canvas);
    if (w < 40 || h < 40) return;
    const { faces, reach, ground } = this.model;
    const scale = Math.min(w, h) * 0.44 / reach;
    const project = p => [w / 2 + dot(p, RIGHT) * scale, h / 2 - dot(p, UP) * scale];

    // Body -> world for roll about x, then pitch about y (yaw left at zero).
    const cr = Math.cos(rollDeg * RAD), sr = Math.sin(rollDeg * RAD);
    const cp = Math.cos(pitchDeg * RAD), sp = Math.sin(pitchDeg * RAD);
    const rotate = p => {
      const y = cr * p[1] - sr * p[2], z = sr * p[1] + cr * p[2];
      return [cp * p[0] + sp * z, y, -sp * p[0] + cp * z];
    };

    // A level ring at ground height: the horizontal the tilt is read against.
    g.strokeStyle = colors.border;
    g.lineWidth = 1;
    g.beginPath();
    for (let k = 0; k <= 48; k++) {
      const a = k / 48 * 2 * Math.PI;
      const [x, y] = project([reach * 1.05 * Math.cos(a), reach * 1.05 * Math.sin(a), ground]);
      if (k) g.lineTo(x, y); else g.moveTo(x, y);
    }
    g.stroke();

    // Faces that look toward the viewer, farthest first.
    // Wheels are the body grey, darkened, so they read as tyres in both themes.
    const base = { body: [colors.muted, 1], arrow: [colors.accent, 1], wheel: [colors.muted, 0.55] };
    const visible = [];
    for (const face of faces) {
      const normal = rotate(face.normal);
      if (dot(normal, EYE) <= 0) continue;
      const pts = face.pts.map(rotate);
      const [color, dim] = base[face.kind];
      // The arrow lies in the roof's own plane, so depth cannot order the
      // two; it is simply drawn last.
      const depth = face.kind === 'arrow' ? Infinity : dot(mean(pts), EYE);
      visible.push({ pts, depth,
                     fill: shade(color, dim * (0.5 + 0.5 * Math.max(0, dot(normal, LIGHT)))) });
    }
    visible.sort((a, b) => a.depth - b.depth);
    g.lineJoin = 'round';
    g.strokeStyle = colors.panel;
    for (const face of visible) {
      g.fillStyle = face.fill;
      g.beginPath();
      face.pts.forEach((p, i) => { const [x, y] = project(p); if (i) g.lineTo(x, y); else g.moveTo(x, y); });
      g.closePath();
      g.fill();
      g.stroke();
    }
  }
}

// ── PATH ─────────────────────────────────────────────────────────────
// The odometry track from above. Start direction (+x) points up the screen
// and +y is to the left. Both axes use the same scale -- a square drive
// must look square, or the map lies.

const MAX_POINTS = 6000;

export class PathView {
  constructor(canvas) {
    this.canvas = canvas;
    this.points = [];
    this.gridStep = 0.5;
    this.rover = { track_m: 0.16 };
  }

  setRover(rover) { this.rover = rover; }

  clear() { this.points = []; }

  add(x, y) {
    const last = this.points[this.points.length - 1];
    if (last && Math.hypot(x - last[0], y - last[1]) < 0.005) return;   // not moved 5 mm yet
    this.points.push([x, y]);
    if (this.points.length > MAX_POINTS) this.points.splice(0, MAX_POINTS / 4);
  }

  draw(x, y, yawDeg) {
    const { g, w, h } = fitCanvas(this.canvas);
    if (w < 40 || h < 40) return;

    // Fit the whole track, the start and the rover, never tighter than 1 m.
    let x0 = Math.min(0, x), x1 = Math.max(0, x), y0 = Math.min(0, y), y1 = Math.max(0, y);
    for (const p of this.points) {
      if (p[0] < x0) x0 = p[0]; if (p[0] > x1) x1 = p[0];
      if (p[1] < y0) y0 = p[1]; if (p[1] > y1) y1 = p[1];
    }
    const s = Math.min(w / Math.max(y1 - y0, 1) , h / Math.max(x1 - x0, 1)) / 1.3;
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    const X = wy => w / 2 - (wy - cy) * s;        // world y (left) -> screen x
    const Y = wx => h / 2 - (wx - cx) * s;        // world x (ahead) -> screen y

    // Grid on round distances; the lines through the start are stronger.
    this.gridStep = niceStep(Math.max(w, h) / s / 7);
    g.lineWidth = 1;
    const line = (ax, ay, bx, by, strong) => {
      g.strokeStyle = strong ? colors.axis : colors.grid;
      g.beginPath(); g.moveTo(Math.round(ax) + 0.5, Math.round(ay) + 0.5);
      g.lineTo(Math.round(bx) + 0.5, Math.round(by) + 0.5); g.stroke();
    };
    const span = Math.max(w, h) / s;
    for (let v = Math.floor((cx - span) / this.gridStep) * this.gridStep; v < cx + span; v += this.gridStep) {
      line(0, Y(v), w, Y(v), Math.abs(v) < this.gridStep / 2);
    }
    for (let v = Math.floor((cy - span) / this.gridStep) * this.gridStep; v < cy + span; v += this.gridStep) {
      line(X(v), 0, X(v), h, Math.abs(v) < this.gridStep / 2);
    }

    // The track.
    if (this.points.length > 1) {
      g.strokeStyle = colors.s1;
      g.lineWidth = 2;
      g.lineJoin = g.lineCap = 'round';
      g.beginPath();
      this.points.forEach((p, i) => { if (i) g.lineTo(X(p[1]), Y(p[0])); else g.moveTo(X(p[1]), Y(p[0])); });
      g.lineTo(X(y), Y(x));
      g.stroke();
    }

    // Start: an open ring.
    g.strokeStyle = colors.muted;
    g.lineWidth = 2;
    g.beginPath(); g.arc(X(0), Y(0), 5, 0, 2 * Math.PI); g.stroke();

    // The rover: its real footprint at map scale (never smaller than a
    // readable mark), turned to its heading, nose marked in the accent colour.
    const track = this.rover.track_m;
    const halfW = Math.max(track / 2 * s, 6), halfL = Math.max(track * 0.75 * s, 9);
    g.save();
    g.translate(X(y), Y(x));
    g.rotate(-yawDeg * RAD);                      // + yaw is counter-clockwise, screen angles run clockwise
    g.fillStyle = colors.fg;
    g.strokeStyle = colors.panel;
    g.lineWidth = 2;
    g.beginPath(); g.rect(-halfW, -halfL, 2 * halfW, 2 * halfL); g.fill(); g.stroke();
    g.fillStyle = colors.accent;
    g.beginPath(); g.moveTo(0, -halfL - 7); g.lineTo(halfW, -halfL + 1); g.lineTo(-halfW, -halfL + 1); g.closePath(); g.fill();
    g.restore();
  }
}

// ── WHEELS ───────────────────────────────────────────────────────────
// One cell per wheel, laid out as on the rover (front row on top, left
// wheels on the left). Each wheel keeps one colour here and in the plots.

const CURRENT_FULL_SCALE_A = 1.0;     // bar length only; not a limit

export class WheelsView {
  constructor(root) {
    this.root = root;
    this.cells = [];
  }

  setRover(rover) {
    this.maxSpeed = rover.max_speed_mps;
    this.root.replaceChildren();
    this.cells = [];
    // Telemetry arrays are in rover.wheels order; the screen order is
    // front-left, front-right, rear-left, rear-right.
    const order = rover.wheels
      .map((wheel, index) => ({ wheel, index }))
      .sort((a, b) => place(a.wheel) - place(b.wheel));
    for (const { wheel, index } of order) {
      const cell = document.createElement('div');
      cell.className = 'wheel';
      cell.innerHTML =
        '<div class="wheel-name"><i></i><span></span></div>' +
        '<div class="wheel-values"><span>+0.000 m/s</span><span>0.00 A</span></div>' +
        '<div class="bar speed"><i></i></div><div class="bar current"><i></i></div>';
      cell.querySelector('.wheel-name i').style.background = `var(--s${index + 1})`;
      cell.querySelector('.wheel-name span').textContent = shortName(wheel.name) + '  ID ' + wheel.id;
      cell.querySelector('.bar.speed i').style.background = `var(--s${index + 1})`;
      this.root.append(cell);
      this.cells[index] = {
        speedText: cell.querySelector('.wheel-values span:first-child'),
        currentText: cell.querySelector('.wheel-values span:last-child'),
        speedBar: cell.querySelector('.bar.speed i'),
        currentBar: cell.querySelector('.bar.current i'),
      };
    }
  }

  update(speeds, currents) {
    this.cells.forEach((cell, i) => {
      const v = speeds[i] ?? 0, amps = currents[i] ?? 0;
      cell.speedText.textContent = (v < 0 ? '-' : '+') + Math.abs(v).toFixed(3) + ' m/s';
      cell.currentText.textContent = Math.abs(amps).toFixed(2) + ' A';
      const half = clamp(Math.abs(v) / this.maxSpeed, 0, 1) * 50;
      cell.speedBar.style.width = half + '%';
      cell.speedBar.style.left = (v >= 0 ? 50 : 50 - half) + '%';
      cell.currentBar.style.width = clamp(Math.abs(amps) / CURRENT_FULL_SCALE_A, 0, 1) * 100 + '%';
    });
  }
}

const place = wheel => (wheel.name.includes('rear') ? 2 : 0) + (wheel.side === 'R' ? 1 : 0);

// "front-left" -> "FL"
export const shortName = name => name.split('-').map(part => part[0]).join('').toUpperCase();
