// Floating, read-only viewer for imported Scout plot.json reports.
import { parseLandingBundle } from './scout_landings.js';
import { SCOUT_PLOT_IMPORT_MAX_BYTES } from './scout_profiles.js';

const finite3 = p => Array.isArray(p) && p.length === 3 && p.every(Number.isFinite);
const fmt = (v, unit) => Number.isFinite(v) ? `${v.toFixed(3)} ${unit}` : 'Unavailable';

export function parsePlotReport(data) {
  if (data?.schema !== 'scout-shot-plot-v1' || !['scout_1', 'scout_2'].includes(data.scout_id)
      || typeof data.shot_id !== 'string' || !data.shot_id.trim() || !Array.isArray(data.trajectory)
      || data.metrics?.conditional !== true || data.metrics?.field_validated !== false
      || !data.metrics.events || typeof data.metrics.events !== 'object'
      || typeof data.synthetic !== 'boolean') throw new Error('Invalid Scout plot report.');
  const landings = parseLandingBundle(data.landings);
  if (landings.scout_id !== data.scout_id || landings.shot_id !== data.shot_id
      || landings.synthetic !== data.synthetic || landings.qualified !== data.qualified)
    throw new Error('Plot and landing identities do not match.');
  if (data.qualified !== true && data.trajectory.length) throw new Error('Unqualified trajectory rejected.');
  let previous = -Infinity;
  for (const row of data.trajectory) {
    if (!Number.isFinite(row.t) || row.t <= previous || !finite3(row.p) || !finite3(row.v)) throw new Error('Invalid or regressed plot samples.');
    previous = row.t;
  }
  for (const key of ['launch_speed_m_s', 'contact_range_m', 'final_rest_range_m', 'flight_duration_s']) {
    if (data.metrics[key] !== null && (!Number.isFinite(data.metrics[key]) || data.metrics[key] < 0)) throw new Error('Invalid shot metric.');
    if (data.qualified !== true && data.metrics[key] !== null) throw new Error('Unqualified shot metrics must remain unavailable.');
  }
  if (data.mode === 'multisensor') {
    if (data.field_validated !== false || data.uncertainty_m !== null
        || !Array.isArray(data.flight_estimates) || !Array.isArray(data.impacts)
        || data.landings.markers.length) throw new Error('Sensor estimates must remain separate from map landing evidence.');
    for (const shot of data.flight_estimates) {
      const metrics = shot.metrics || shot;
      for (const key of ['flight_duration_s', 'launch_speed_m_s', 'contact_range_m', 'final_rest_range_m', 'max_height_above_launch_m']) {
        const value = metrics[key];
        if (value != null && (!Number.isFinite(value) || value < 0)) throw new Error('Invalid sensor flight estimate.');
      }
    }
    for (const impact of data.impacts) {
      if (!Array.isArray(impact.waveform) || !Number.isFinite(impact.time_from_initial_s)
          || typeof impact.features !== 'object' || !impact.features) throw new Error('Invalid impact estimate.');
      let before = -Infinity;
      for (const row of impact.waveform) {
        if (!Number.isFinite(row.time_from_contact_s) || row.time_from_contact_s <= before
            || (row.specific_force_body_m_s2 != null && !finite3(row.specific_force_body_m_s2))
            || (row.magnitude_m_s2 != null && (!Number.isFinite(row.magnitude_m_s2) || row.magnitude_m_s2 < 0)))
          throw new Error('Invalid or regressed impact waveform.');
        before = row.time_from_contact_s;
      }
      for (const key of ['peak_specific_force_m_s2', 'peak_specific_force_g', 'pulse_duration_s', 'specific_force_impulse_m_s']) {
        const value = impact.features[key];
        if (value != null && (!Number.isFinite(value) || value < 0)) throw new Error('Invalid impact feature.');
      }
    }
  }
  return structuredClone(data);
}

// Small standard canvas plots; missing values break lines and never become zero.
export function drawSeries(canvas, series, { title, xLabel, yLabel, equal = false }) {
  const r = canvas.getBoundingClientRect(), dpr = devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(r.width * dpr)); canvas.height = Math.max(1, Math.round(r.height * dpr));
  const g = canvas.getContext('2d'); g.scale(dpr, dpr);
  const w = r.width, h = r.height, pad = { x: 48, y: 28 };
  g.fillStyle = '#fff'; g.fillRect(0, 0, w, h);
  const points = series.flatMap(s => s.points.filter(p => p.length === 2 && p.every(Number.isFinite)));
  g.fillStyle = '#222'; g.font = '12px sans-serif'; g.fillText(title, 8, 15);
  if (!points.length) { g.fillText('No qualified plot data', 12, h / 2); return; }
  let xmin = Math.min(...points.map(p => p[0])), xmax = Math.max(...points.map(p => p[0]));
  let ymin = Math.min(...points.map(p => p[1])), ymax = Math.max(...points.map(p => p[1]));
  if (xmin === xmax) { xmin -= .5; xmax += .5; } if (ymin === ymax) { ymin -= .5; ymax += .5; }
  const iw = w - pad.x - 12, ih = h - pad.y - 30;
  if (equal) {
    const scale = Math.min(iw / (xmax - xmin), ih / (ymax - ymin)), cx = (xmin + xmax) / 2, cy = (ymin + ymax) / 2;
    xmin = cx - iw / scale / 2; xmax = cx + iw / scale / 2; ymin = cy - ih / scale / 2; ymax = cy + ih / scale / 2;
  }
  const X = x => pad.x + (x - xmin) / (xmax - xmin) * iw, Y = y => h - 30 - (y - ymin) / (ymax - ymin) * ih;
  g.strokeStyle = '#ddd'; g.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const x = xmin + (xmax - xmin) * i / 4, y = ymin + (ymax - ymin) * i / 4;
    g.beginPath(); g.moveTo(X(x), pad.y); g.lineTo(X(x), h - 30); g.moveTo(pad.x, Y(y)); g.lineTo(w - 12, Y(y)); g.stroke();
    g.fillStyle = '#555'; g.font = '9px sans-serif'; g.fillText(x.toFixed(2), X(x) - 10, h - 17); g.fillText(y.toFixed(2), 2, Y(y) + 3);
  }
  for (const [index, s] of series.entries()) {
    g.strokeStyle = ['#2463a0', '#c77420', '#34916d', '#222'][index % 4]; g.beginPath(); let connected = false;
    for (const point of s.points) {
      if (!point.every(Number.isFinite)) { connected = false; continue; }
      if (connected) g.lineTo(X(point[0]), Y(point[1])); else g.moveTo(X(point[0]), Y(point[1]));
      connected = true;
    }
    g.stroke(); g.fillStyle = g.strokeStyle; g.font = '10px sans-serif'; g.fillText(s.label, pad.x + index * 58, pad.y - 2);
  }
  g.fillStyle = '#555'; g.font = '10px sans-serif'; g.fillText(xLabel, w / 2 - 35, h - 3); g.fillText(yLabel, 2, pad.y - 2);
}

export class ShotPlots {
  constructor(root, { onLandings = () => {} } = {}) {
    this.root = root; this.onLandings = onLandings;
    this.shots = new Map(); this.notice = '';
    this.node = name => root.querySelector('#scout-shot-' + name);
    this.node('close').addEventListener('click', () => { root.hidden = true; });
    this.node('select').addEventListener('change', () => this.render());
    this.node('file').addEventListener('change', async event => {
      try {
        const file = event.target.files[0];
        if (!file) return;
        if (file.size > SCOUT_PLOT_IMPORT_MAX_BYTES) throw new Error('Report exceeds the import size limit.');
        this.load(parsePlotReport(JSON.parse(await file.text()))); this.notice = '';
      } catch (error) { this.notice = error.message; }
      event.target.value = ''; this.render();
    });
    this.node('save').addEventListener('click', () => {
      const shot = this.shots.get(this.node('select').value); if (!shot) return;
      const blob = new Blob([JSON.stringify(shot.report, null, 2)], { type: 'application/json' });
      const url = URL.createObjectURL(blob), a = document.createElement('a'); a.href = url;
      a.download = `scout-${shot.scoutId}-${shot.report.shot_id}.plot.json`; a.click(); URL.revokeObjectURL(url);
    });
  }
  open() { this.root.hidden = false; this.render(); }
  load(report) {
    report = parsePlotReport(report);
    this.shots.set(report.scout_id, { scoutId: report.scout_id, report,
      status: report.qualified ? 'Offline result loaded; conditional inertial estimate' : 'Trajectory unavailable; inspect the original analysis gates' });
    this.node('select').value = report.scout_id; this.onLandings(report.landings); this.open();
  }
  render() {
    const shot = this.shots.get(this.node('select').value), report = shot?.report;
    this.node('status').textContent = this.notice || shot?.status || 'Load an offline plot.json report.';
    this.node('identity').textContent = `${this.node('select').value}${report ? ' / ' + report.shot_id : ''}`
      + (report?.synthetic ? ' · SYNTHETIC' : '');
    this.node('metrics').textContent = `Launch speed: ${fmt(report?.metrics.launch_speed_m_s, 'm/s')} · Contact range: ${fmt(report?.metrics.contact_range_m, 'm')} · Final rest: ${fmt(report?.metrics.final_rest_range_m, 'm')}`;
    if (report?.mode === 'multisensor') {
      const latest = report.flight_estimates.at(-1), values = latest?.metrics || latest;
      const impact = report.impacts.at(-1), features = impact?.features;
      this.node('metrics').textContent = `Sensor estimates · Hops: ${report.flight_estimates.length} · Flight: ${fmt(values?.flight_duration_s, 's')}`
        + ` (${values?.flight_duration_qualified === true ? 'sensor timing qualified' : 'timing UNQUALIFIED'})`
        + ` · Launch: ${fmt(values?.launch_speed_m_s, 'm/s')} · Sampled impact peak: ${fmt(features?.peak_specific_force_g, 'g')}`
        + ` · Pulse: ${fmt(features?.pulse_duration_s, 's')} · Force integral: ${fmt(features?.specific_force_impulse_m_s, 'm/s')}`
        + ` · Scale: ${impact?.calibration_qualified === true ? 'calibrated' : 'nominal/unqualified'}`
        + ` · Clipped: ${impact?.diagnostics?.clipped_samples ?? '--'} · Gaps/invalid: ${impact?.diagnostics?.gaps_or_invalid ?? '--'}`;
    }
    this.node('note').textContent = report ? 'Conditional estimate; accuracy and uncertainty unverified. '
      + (report.metrics.launch_velocity_origin === 'supplied_initial_condition' ? 'Launch velocity was supplied as an initial condition.' : '')
      : 'Display an imported report and its original qualification labels. This viewer does not trigger capture or infer motion.';
    if (report?.mode === 'multisensor') this.node('note').textContent = 'Sensor-detected flight/contact/rest estimates; position drift and unknown uncertainty remain. '
      + 'Impact features are specific force, not CIV/CBR/hardness. Full received XYZ, timing/flags and all landing CSVs are in the standalone report. No automatic map writes.';
    this.node('save').disabled = !shot;
    if (this.root.hidden) return;
    const rows = report?.trajectory || [];
    const series = (label, points) => ({ label, points });
    if (report) {
      drawSeries(this.node('a'), [series('XY', rows.map(r => r.p.slice(0, 2)))], { title: 'Trajectory · XY', xLabel: 'World X (m)', yLabel: 'Y (m)', equal: true });
      drawSeries(this.node('b'), [series('XZ', rows.map(r => [r.p[0], r.p[2]]))], { title: 'Trajectory · XZ', xLabel: 'World X (m)', yLabel: 'Z (m)', equal: true });
      drawSeries(this.node('c'), [series('Speed', rows.map(r => [r.t, Math.hypot(...r.v)]))], { title: 'Speed', xLabel: 'Initial time (s)', yLabel: 'm/s' });
      const origin = report.metrics.events.liftoff?.position_world_m || rows[0]?.p;
      drawSeries(this.node('d'), [series('Range', rows.map(r => [r.t, Math.hypot(r.p[0] - origin[0], r.p[1] - origin[1])]))], { title: 'Horizontal displacement', xLabel: 'Initial time (s)', yLabel: 'm' });
      if (report.mode === 'multisensor') {
        const impact = report.impacts.at(-1), wave = impact?.waveform || [];
        drawSeries(this.node('b'), 'XYZ'.split('').map((axis, index) => series(axis, wave.map(r => [r.time_from_contact_s,
          r.specific_force_body_m_s2?.[index]]))),
          { title: 'Latest landing / bounce · XYZ specific force', xLabel: 'Impact time (s)', yLabel: 'm/s²' });
        drawSeries(this.node('d'), [series('Magnitude', wave.map(r => [r.time_from_contact_s, r.magnitude_m_s2]))],
          { title: 'Latest impact · sampled magnitude; peak retained', xLabel: 'Impact time (s)', yLabel: 'm/s²' });
      }
    } else {
      drawSeries(this.node('a'), [], { title: 'Trajectory · XY' });
      drawSeries(this.node('b'), [], { title: 'Trajectory · XZ' });
      drawSeries(this.node('c'), [], { title: 'Speed' });
      drawSeries(this.node('d'), [], { title: 'Horizontal displacement' });
    }
  }
}
