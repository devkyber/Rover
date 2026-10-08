// Strip charts: the last 30 s of a few numbers, newest at the right edge.
//
//   const chart = new StripChart(parent, { title, unit, series: ['FL', 'FR'], digits: 2 });
//   chart.push(t, [0.18, 0.17]);     // once per telemetry message
//   chart.draw();
//
// One y-axis per chart, always. Numbers with different units or very
// different sizes get their own chart instead of a second axis.
// Hovering shows a hairline at the nearest sample and every series' value
// there, so nobody has to aim at a 2-pixel line.

import { colors, fitCanvas, niceStep } from './util.js';

export const WINDOW_S = 30;

const PAD_LEFT = 44, PAD_RIGHT = 8, PAD_TOP = 6, PAD_BOTTOM = 18;

export class StripChart {
  constructor(parent, { title, unit, series, digits = 2, minSpan = 0.1 }) {
    this.series = series;
    this.digits = digits;
    this.unit = unit;
    this.minSpan = minSpan;        // the y-range never shrinks below this, so
    this.times = [];               // sensor noise is not blown up to full height
    this.values = series.map(() => []);
    this.hoverX = null;

    this.root = document.createElement('div');
    this.root.className = 'plot';
    const head = document.createElement('div');
    head.className = 'plot-head';
    head.append(this._span('plot-title', title), this._span('plot-unit', unit));

    // Legend: a line key in the series colour, the name, and the newest
    // value. The text itself stays in the normal text colours.
    this.latest = [];
    const legend = document.createElement('div');
    legend.className = 'legend';
    series.forEach((name, i) => {
      const item = document.createElement('span');
      const key = document.createElement('i');
      key.dataset.series = i;
      const value = document.createElement('b');
      value.textContent = '--';
      this.latest.push(value);
      // One series needs no name: the chart title already says what it is.
      if (series.length > 1) item.append(key, document.createTextNode(name));
      item.append(value);
      legend.append(item);
    });
    head.append(legend);

    const box = document.createElement('div');
    box.className = 'canvas-box';
    this.canvas = document.createElement('canvas');
    box.append(this.canvas);
    this.tooltip = document.createElement('div');
    this.tooltip.className = 'tooltip';
    this.tooltip.hidden = true;
    this.root.append(head, box, this.tooltip);
    parent.append(this.root);

    this.canvas.addEventListener('pointermove', e => {
      this.hoverX = e.clientX - this.canvas.getBoundingClientRect().left;
      this.draw();
    });
    this.canvas.addEventListener('pointerleave', () => {
      this.hoverX = null;
      this.tooltip.hidden = true;
      this.draw();
    });
  }

  _span(className, text) {
    const s = document.createElement('span');
    s.className = className;
    s.textContent = text;
    return s;
  }

  push(t, values) {
    // The rover's clock restarts at zero on NEW RUN. Old samples would then
    // sit in the "future", so start clean.
    if (this.times.length && t < this.times[this.times.length - 1]) this.clear();
    this.times.push(t);
    values.forEach((v, i) => this.values[i].push(v));
    while (this.times.length && this.times[0] < t - WINDOW_S) {
      this.times.shift();
      this.values.forEach(a => a.shift());
    }
    values.forEach((v, i) => { this.latest[i].textContent = v.toFixed(this.digits); });
  }

  clear() {
    this.times = [];
    this.values = this.series.map(() => []);
    this.latest.forEach(el => { el.textContent = '--'; });
  }

  draw() {
    this.root.querySelectorAll('.legend i').forEach(key => {
      key.style.background = colors.series[+key.dataset.series];
    });
    const { g, w, h } = fitCanvas(this.canvas);
    if (w < 60 || h < 40) return;
    const x0 = PAD_LEFT, x1 = w - PAD_RIGHT, y0 = PAD_TOP, y1 = h - PAD_BOTTOM;

    // y-range: the data, widened to at least minSpan, on round numbers.
    let lo = Infinity, hi = -Infinity;
    for (const a of this.values) for (const v of a) { if (v < lo) lo = v; if (v > hi) hi = v; }
    if (!isFinite(lo)) { lo = -this.minSpan / 2; hi = this.minSpan / 2; }
    if (hi - lo < this.minSpan) { const mid = (hi + lo) / 2; lo = mid - this.minSpan / 2; hi = mid + this.minSpan / 2; }
    const step = niceStep((hi - lo) / 3);
    lo = Math.floor(lo / step) * step;
    hi = Math.ceil(hi / step) * step;

    const tEnd = this.times.length ? this.times[this.times.length - 1] : 0;
    const X = t => x1 - (tEnd - t) / WINDOW_S * (x1 - x0);
    const Y = v => y1 - (v - lo) / (hi - lo) * (y1 - y0);

    // Grid: solid hairlines, one step off the panel colour. Zero a touch stronger.
    g.font = '11px ' + getComputedStyle(document.body).fontFamily;
    g.textBaseline = 'middle';
    g.lineWidth = 1;
    const tickDigits = Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
    for (let v = lo; v <= hi + step / 2; v += step) {
      const y = Math.round(Y(v)) + 0.5;
      g.strokeStyle = Math.abs(v) < step / 2 ? colors.axis : colors.grid;
      g.beginPath(); g.moveTo(x0, y); g.lineTo(x1, y); g.stroke();
      g.fillStyle = colors.muted;
      g.textAlign = 'right';
      g.fillText(v.toFixed(tickDigits), x0 - 6, y);
    }
    g.textBaseline = 'alphabetic';
    for (let s = 0; s <= WINDOW_S; s += 10) {
      const x = X(tEnd - s);
      g.fillStyle = colors.muted;
      g.textAlign = s === 0 ? 'right' : 'center';
      g.fillText(s === 0 ? 'now' : '-' + s + ' s', x, h - 4);
    }

    // The lines.
    g.lineWidth = 2;
    g.lineJoin = g.lineCap = 'round';
    this.values.forEach((a, i) => {
      if (a.length < 2) return;
      g.strokeStyle = colors.series[i];
      g.beginPath();
      for (let k = 0; k < a.length; k++) {
        const x = X(this.times[k]), y = Y(a[k]);
        if (k) g.lineTo(x, y); else g.moveTo(x, y);
      }
      g.stroke();
    });

    if (this.hoverX !== null && this.times.length) this._drawHover(g, X, Y, y0, y1, w, tEnd);
  }

  _drawHover(g, X, Y, y0, y1, w, tEnd) {
    // Nearest sample to the pointer.
    let k = 0, best = Infinity;
    for (let i = 0; i < this.times.length; i++) {
      const d = Math.abs(X(this.times[i]) - this.hoverX);
      if (d < best) { best = d; k = i; }
    }
    const x = X(this.times[k]);
    g.strokeStyle = colors.muted;
    g.lineWidth = 1;
    g.beginPath(); g.moveTo(Math.round(x) + 0.5, y0); g.lineTo(Math.round(x) + 0.5, y1); g.stroke();
    this.values.forEach((a, i) => {
      g.fillStyle = colors.panel;          // ring in the surface colour keeps the dot readable on a crossing
      g.beginPath(); g.arc(x, Y(a[k]), 6, 0, 2 * Math.PI); g.fill();
      g.fillStyle = colors.series[i];
      g.beginPath(); g.arc(x, Y(a[k]), 4, 0, 2 * Math.PI); g.fill();
    });

    // Tooltip: value first and strong, name after it. Built with textContent.
    const tip = this.tooltip;
    tip.replaceChildren();
    const when = document.createElement('div');
    when.className = 'when';
    const ago = tEnd - this.times[k];
    when.textContent = ago < 0.05 ? 'now' : ago.toFixed(1) + ' s ago';
    tip.append(when);
    this.values.forEach((a, i) => {
      const line = document.createElement('div');
      line.className = 'line';
      const key = document.createElement('i');
      key.style.background = colors.series[i];
      const value = document.createElement('b');
      value.textContent = a[k].toFixed(this.digits);
      const name = document.createElement('span');
      name.textContent = (this.series.length > 1 ? this.series[i] + ' ' : '') + this.unit;
      line.append(key, value, name);
      tip.append(line);
    });
    tip.hidden = false;
    const left = x + 12 + tip.offsetWidth > w ? x - 12 - tip.offsetWidth : x + 12;
    tip.style.left = Math.max(0, left) + 'px';
    tip.style.top = '30px';
  }
}
