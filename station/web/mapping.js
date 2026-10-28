// Mission records are independent of the local path and of mechanism commands.
// Only explicit form submissions write data. A timeout refreshes; it never retries a write.
const KINDS = Object.freeze({ scout_first_contact: 'Scout first contact', scout_rest: 'Scout final rest', clegg_impact: 'Clegg impact', landmark: 'Landmark' });
const MUTATIONS = new Set(['create', 'open', 'register', 'observe']);
const FRESH_MS = 500;
const REQUEST_MS = 10000;
const DRAW_MS = 100;
const RECORD_LIST_LIMIT = 100;
const MAP_DRAFT_FIELDS = ['frame', 'points', 'reg-bound', 'yaw-bound', 'residual', 'reg-evidence', 'kind', 'shot', 'label',
  'clock', 'time', 'time-bound', 'evidence', 'location', 'unknown', 'x', 'y', 'location-bound', 'location-frame',
  'location-evidence', 'quantity', 'value', 'unit', 'calibration', 'measurement-evidence', 'supersedes'];
const xy = value => Array.isArray(value) && value.length >= 2 && value.slice(0, 2).every(Number.isFinite);
const fmt = value => Number.isFinite(value) ? Number(value.toPrecision(5)).toString() : 'unknown';
const required = (value, label) => {
  const text = String(value ?? '').trim();
  if (!text) throw new Error(`${label} is required.`);
  return text;
};
export function mapNumber(value, label, { optional = false, positive = false, nonnegative = false } = {}) {
  const text = String(value ?? '').trim();
  if (!text && optional) return null;
  if (!text) throw new Error(`${label} is required.`);
  const number = Number(text);
  if (!Number.isFinite(number) || (positive && number <= 0) || (nonnegative && number < 0)) {
    throw new Error(`${label} must be ${positive ? 'positive' : nonnegative ? 'nonnegative' : 'a finite number'}.`);
  }
  return number;
}
export function registrationPayload(v) {
  let points;
  try { points = JSON.parse(v.points); } catch { throw new Error('Control points must be valid JSON.'); }
  if (!Array.isArray(points) || points.length < 2 || points.some(p => !xy(p?.local_xy) || p.local_xy.length !== 2 || !xy(p?.map_xy) || p.map_xy.length !== 2)) {
    throw new Error('Provide at least two local_xy / map_xy pairs of finite X/Y numbers.');
  }
  const heading = mapNumber(v['yaw-bound'], 'Heading bound', { optional: true, nonnegative: true });
  if (heading !== null && heading > 180) throw new Error('Heading bound must be at most 180 degrees.');
  return { frame_id: required(v.frame, 'Local frame ID'), control_points: points,
    uncertainty_m: mapNumber(v['reg-bound'], 'Registration bound', { positive: true, optional: true }),
    ...(heading === null ? {} : { yaw_uncertainty_deg: heading }),
    max_residual_m: mapNumber(v.residual, 'Maximum residual', { positive: true }),
    evidence: required(v['reg-evidence'], 'Registration evidence') };
}
export function observationPayload(v) {
  if (!KINDS[v.kind]) throw new Error('Select an observation event.');
  const time = mapNumber(v.time, 'Event time', { optional: true });
  const clock = time === null ? String(v.clock || '').trim() || 'unknown' : required(v.clock, 'Clock reference');
  const timeBound = mapNumber(v['time-bound'], 'Time uncertainty', { optional: true, nonnegative: true });
  if (time === null && timeBound !== null) throw new Error('Time uncertainty must be blank when event time is unknown.');
  const event = { kind: v.kind, label: required(v.label, 'Label'),
    event_time: { clock, value: time,
      uncertainty_s: timeBound },
    evidence: required(v.evidence, 'Observation evidence') };
  if (v.kind !== 'landmark') event.shot_id = required(v.shot, 'Shot ID');
  else if (String(v.shot || '').trim()) event.shot_id = v.shot.trim();
  if (v.location === 'unknown') event.location = { kind: 'unknown', reason: required(v.unknown, 'Reason for unknown location') };
  else if (['surveyed', 'local'].includes(v.location)) {
    event.location = { kind: v.location, xy_m: [mapNumber(v.x, 'Location X'), mapNumber(v.y, 'Location Y')],
      uncertainty_m: mapNumber(v['location-bound'], 'Location bound', { positive: true, optional: true }),
      evidence: required(v['location-evidence'], 'Location evidence') };
    if (v.location === 'local') event.location.frame_id = required(v['location-frame'], 'Location frame ID');
  } else throw new Error('Select a location source.');
  const measurementFields = ['quantity', 'value', 'unit', 'calibration', 'measurement-evidence'];
  if (measurementFields.some(key => String(v[key] || '').trim())) event.measurement = {
    quantity: required(v.quantity, 'Measured quantity'), value: mapNumber(v.value, 'Measured value'),
    unit: required(v.unit, 'Measurement unit'), calibration_id: required(v.calibration, 'Calibration ID'),
    evidence: required(v['measurement-evidence'], 'Measurement evidence'),
  };
  if (String(v.supersedes || '').trim()) event.supersedes = v.supersedes.trim();
  return event;
}

export function registeredRover(state, telemetry, registrationsByFrame = null) {
  if (!state?.map_id || telemetry?.mapping?.map_id !== state.map_id
      || telemetry?.pose_valid !== true || !telemetry.run_id || !telemetry.odom_frame_id || !xy(telemetry.pos)
      || !Array.isArray(telemetry.rpy) || !telemetry.rpy.every(Number.isFinite) || telemetry.rpy.length < 3) return null;
  const registration = registrationsByFrame ? registrationsByFrame.get(telemetry.odom_frame_id)
    : (state.registrations || []).find(r => r.frame_id === telemetry.odom_frame_id && !r.superseded);
  if (!registration || !xy(registration.translation_m) || !Number.isFinite(registration.yaw_deg)) return null;
  const yaw = registration.yaw_deg * Math.PI / 180, c = Math.cos(yaw), s = Math.sin(yaw);
  const [x, y] = telemetry.pos;
  const point = [c * x - s * y + registration.translation_m[0], s * x + c * y + registration.translation_m[1]];
  return xy(point) ? { xy: point, yaw: telemetry.rpy[2] + registration.yaw_deg, registration } : null;
}
const uuid = () => {
  if (globalThis.crypto?.randomUUID) return crypto.randomUUID();
  const bytes = crypto.getRandomValues(new Uint8Array(16));
  bytes[6] = (bytes[6] & 15) | 64; bytes[8] = (bytes[8] & 63) | 128;
  const h = [...bytes].map(v => v.toString(16).padStart(2, '0')).join('');
  return `${h.slice(0, 8)}-${h.slice(8, 12)}-${h.slice(12, 16)}-${h.slice(16, 20)}-${h.slice(20)}`;
};

export class MissionMapPanel {
  constructor(root, { send, canEdit = false, now = () => performance.now(), requestId = uuid }) {
    this.root = root; this.send = send; this.canEdit = canEdit; this.now = now; this.requestId = requestId;
    this.online = false; this.state = null; this.telemetry = null; this.receivedAt = -Infinity;
    this.pending = null; this.notice = ''; this.lastRender = -Infinity; this.lastSnapshot = -Infinity;
    this.draftNotice = '';
    this.lastDraw = -Infinity; this.lastDrawIdentity = '';
    this.activeEvents = []; this.locatedEvents = []; this.registrationsByFrame = new Map();
    this.mapExtent = [Infinity, -Infinity, Infinity, -Infinity];
    this.node = id => root.querySelector('#mapping-' + id);
    for (const action of ['create', 'open', 'register', 'observe']) this.node(action + '-form').addEventListener('submit', event => {
      event.preventDefault();
      try { this.request(action, this.payload(action)); } catch (error) { this.notice = error.message; this.render(); }
    });
    this.node('refresh').addEventListener('click', () => this.request('snapshot'));
    this.node('export').addEventListener('click', () => this.request('export'));
    this.node('use-frame').addEventListener('click', () => {
      if (this.editable() && this.telemetry?.odom_frame_id) this.node('frame').value = this.telemetry.odom_frame_id;
    });
    for (const id of ['location', 'kind']) this.node(id).addEventListener('change', () => this.formVisibility());
    root.addEventListener('toggle', () => { this.lastDraw = -Infinity; this.render(); this.draw(); });
    this.formVisibility(); this.render();
  }

  isEditor() { return typeof this.canEdit === 'function' ? !!this.canEdit() : !!this.canEdit; }
  fresh() { return this.online && this.telemetry && this.now() - this.receivedAt < FRESH_MS; }
  editable() { return this.isEditor() && this.fresh() && !this.pending; }
  payload(action) {
    const fields = ['name', 'reference', ...MAP_DRAFT_FIELDS];
    const v = Object.fromEntries(fields.map(key => [key, this.node(key).value]));
    if (action === 'create') return { name: required(v.name, 'Map name'), reference: required(v.reference, 'Coordinate reference') };
    if (action === 'open') return { map_id: required(this.node('map-select').value, 'Saved map') };
    return action === 'register' ? registrationPayload(v) : observationPayload(v);
  }
  formVisibility() {
    const location = this.node('location').value, coordinates = location !== 'unknown';
    this.node('unknown-fields').hidden = coordinates;
    this.node('unknown').required = !coordinates;
    this.node('coordinate-fields').hidden = !coordinates;
    this.node('location-frame-label').hidden = location !== 'local';
    for (const id of ['x', 'y', 'location-evidence']) this.node(id).required = coordinates;
    this.node('location-frame').required = location === 'local';
    this.node('shot').required = this.node('kind').value !== 'landmark';
  }

  request(action, payload = {}) {
    if (!this.online || this.pending || (MUTATIONS.has(action) && !this.editable())) return false;
    if (['register', 'observe', 'export'].includes(action) && !this.state?.map_id) return false;
    if (['register', 'observe'].includes(action) && (this.state.read_only || this.state.error || this.telemetry?.mapping?.read_only || this.telemetry?.mapping?.error)) return false;
    const id = this.requestId(), message = { type: 'mapping', action, request_id: id, ...payload };
    if (['register', 'observe', 'export'].includes(action)) {
      message.map_id = this.state.map_id; message.expected_revision = this.state.revision;
    }
    this.pending = { id, action, at: this.now() };
    if (action === 'snapshot') this.lastSnapshot = this.now();
    this.notice = `${action === 'snapshot' ? 'Loading map' : action === 'export' ? 'Exporting map' : 'Saving ' + action}…`;
    try {
      if (this.send(message) === false) throw new Error('Map connection is unavailable.');
    } catch (error) { this.pending = null; this.notice = error.message; }
    this.render(); return this.pending !== null;
  }

  connected(online) {
    if (!!online === this.online) return;
    this.online = !!online; this.telemetry = null; this.receivedAt = -Infinity; this.pending = null;
    this.notice = online ? 'Waiting for fresh telemetry.' : 'Disconnected. Saved map retained; live rover hidden.';
    if (online) this.request('snapshot');
    this.render(); this.draw();
  }
  update(telemetry) {
    if (!this.online) return;
    this.telemetry = telemetry; this.receivedAt = this.now();
    const status = telemetry?.mapping;
    if (!this.pending && status && this.now() - this.lastSnapshot > 1000 &&
        (status.map_id !== this.state?.map_id || status.revision > (this.state?.revision ?? -1))) this.request('snapshot');
    this.render();
  }
  result(message) {
    if (!this.online || !this.pending || message.request_id !== this.pending.id || message.action !== this.pending.action) return;
    const action = this.pending.action; this.pending = null;
    if (message.state) this.acceptState(message.state);
    if (!message.ok) {
      this.notice = `${action} failed: ${message.error || 'Unknown error'}. Refresh before trying again.`;
    } else if (action === 'export' && message.document) {
      const blob = new Blob([JSON.stringify(message.document, null, 2) + '\n'], { type: 'application/json' });
      const url = URL.createObjectURL(blob), anchor = document.createElement('a');
      anchor.href = url; anchor.download = `mission-map-${String(this.state?.map_id || 'export').replace(/[^a-zA-Z0-9_-]/g, '_')}.json`;
      anchor.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
      this.notice = 'JSON exported with original records and provenance.';
    } else {
      this.notice = action === 'snapshot' ? '' : `${action === 'observe' ? 'Observation saved' : action === 'register' ? 'Frame registered' : 'Map selected'}.`;
      if (['observe', 'register'].includes(action)) this.draftNotice = '';
      // Keep operator input for review/corrections; do not invent the next event time or position.
    }
    this.render(); this.draw();
  }
  acceptState(state) {
    if (state.map_id && this.state?.map_id === state.map_id && state.revision < this.state.revision) return;
    if (this.state?.map_id && this.state.map_id !== state.map_id) {
      // The active map is shared across clients. Coordinates and corrections
      // prepared for one reference must never silently submit into another.
      for (const id of MAP_DRAFT_FIELDS) this.node(id).value = '';
      this.node('kind').value = 'scout_first_contact'; this.node('location').value = 'unknown';
      this.formVisibility();
      this.draftNotice = 'Map changed. Registration and observation drafts were cleared. Check the selected coordinate reference before entering new records.';
    }
    this.state = state;
    // Full evidence remains in state/export. Cache plot work once per snapshot;
    // neither a camera animation frame nor a telemetry tick scans the journal.
    this.activeEvents = (state.observations || []).filter(event => !event.superseded);
    this.locatedEvents = this.activeEvents.filter(event => xy(event.map_xy_m));
    this.registrationsByFrame = new Map((state.registrations || []).filter(r => !r.superseded).map(r => [r.frame_id, r]));
    this.mapExtent = [Infinity, -Infinity, Infinity, -Infinity];
    for (const event of this.locatedEvents) {
      const [x, y] = event.map_xy_m, bound = Number.isFinite(event.uncertainty_m) && event.uncertainty_m > 0 ? event.uncertainty_m : 0;
      this.mapExtent[0] = Math.min(this.mapExtent[0], x - bound); this.mapExtent[1] = Math.max(this.mapExtent[1], x + bound);
      this.mapExtent[2] = Math.min(this.mapExtent[2], y - bound); this.mapExtent[3] = Math.max(this.mapExtent[3], y + bound);
    }
    this.lastDraw = -Infinity;
    const select = this.node('map-select'), selected = select.value;
    if (Array.isArray(state.available_maps)) {
      select.replaceChildren();
      for (const map of state.available_maps) {
        const option = document.createElement('option'); option.value = map.map_id;
        option.textContent = `${map.name || map.map_id}${map.read_only ? ' (read only)' : ''}`; select.append(option);
      }
    }
    if (state.map_id && ![...select.options].some(option => option.value === state.map_id)) {
      const option = document.createElement('option'); option.value = state.map_id; option.textContent = state.name || state.map_id; select.append(option);
    }
    if ([...select.options].some(option => option.value === selected)) select.value = selected;
    else if (state.map_id) select.value = state.map_id;
    this.renderRecords();
  }

  renderRecords() {
    const state = this.state || {}, events = state.observations || [], registrations = state.registrations || [];
    this.node('event-count').textContent = String(events.length);
    const list = this.node('events'); list.replaceChildren();
    if (events.length > RECORD_LIST_LIMIT) {
      const note = document.createElement('li');
      note.textContent = `Showing the latest ${RECORD_LIST_LIMIT} of ${events.length} events. Export JSON includes the full record; the map includes all active located events.`;
      list.append(note);
    }
    for (const event of events.slice(-RECORD_LIST_LIMIT).reverse()) {
      const item = document.createElement('li'); item.className = event.superseded ? 'mapping-superseded' : '';
      const title = document.createElement('b');
      title.textContent = `${KINDS[event.kind] || event.kind}${event.shot_id ? ' · ' + event.shot_id : ''}${event.label ? ' · ' + event.label : ''}${event.superseded ? ' · Superseded' : ''}`;
      const id = document.createElement('code'); id.textContent = event.event_id;
      const description = document.createElement('p');
      const location = xy(event.map_xy_m) ? `X ${fmt(event.map_xy_m[0])}, Y ${fmt(event.map_xy_m[1])} m; supplied bound ${fmt(event.uncertainty_m)} m`
        : `Location unknown: ${event.location?.reason || 'local frame is not registered'}`;
      description.textContent = `${location}. Time ${fmt(event.event_time?.value)} (${event.event_time?.clock || 'unknown clock'}); time uncertainty ${fmt(event.event_time?.uncertainty_s)} s.`;
      const evidence = document.createElement('p'); evidence.textContent = `Evidence: ${event.evidence || 'unspecified'}${event.location?.evidence ? '; location: ' + event.location.evidence : ''}`;
      item.append(title, id, description, evidence);
      if (event.location?.kind === 'local') {
        const origin = document.createElement('p');
        origin.textContent = `Source frame ${event.location.frame_id}: X ${fmt(event.location.xy_m?.[0])}, Y ${fmt(event.location.xy_m?.[1])} m; source bound ${fmt(event.location.uncertainty_m)} m.`;
        item.append(origin);
      }
      if (event.supersedes) {
        const correction = document.createElement('p'); correction.textContent = `Correction of event ${event.supersedes}`; item.append(correction);
      }
      if (event.measurement) {
        const m = event.measurement, value = document.createElement('p');
        value.textContent = `${m.quantity}: ${fmt(m.value)} ${m.unit}; calibration ${m.calibration_id}; evidence: ${m.evidence}`; item.append(value);
      }
      list.append(item);
    }
    const registry = this.node('registrations'); registry.replaceChildren();
    if (registrations.length > RECORD_LIST_LIMIT) {
      const note = document.createElement('li');
      note.textContent = `Showing the latest ${RECORD_LIST_LIMIT} of ${registrations.length} frame registrations. Export JSON includes all registrations.`;
      registry.append(note);
    }
    for (const registration of registrations.slice(-RECORD_LIST_LIMIT).reverse()) {
      const item = document.createElement('li');
      item.textContent = `${registration.frame_id} · translation bound at centroid ${fmt(registration.uncertainty_m)} m · heading bound ${fmt(registration.yaw_uncertainty_deg)}° · fit RMSE ${fmt(registration.fit_rmse_m)} m · ${registration.evidence}`;
      registry.append(item);
    }
  }

  liveRover() { return this.fresh() ? registeredRover(this.state, this.telemetry, this.registrationsByFrame) : null; }
  render() {
    this.lastRender = this.now();
    const state = this.state, editor = this.isEditor(), available = this.editable(), writable = available && state?.map_id && !state.read_only && !state.error && !this.telemetry?.mapping?.read_only && !this.telemetry?.mapping?.error;
    for (const section of this.root.querySelectorAll('.mapping-edit')) section.hidden = !editor;
    for (const control of this.root.querySelectorAll('.mapping-edit input, .mapping-edit textarea, .mapping-edit select')) control.disabled = !available;
    for (const action of ['create', 'open', 'use-frame']) this.node(action).disabled = !available;
    for (const action of ['register', 'observe']) this.node(action).disabled = !writable;
    this.node('open').disabled ||= !this.node('map-select').value;
    this.node('use-frame').disabled ||= !this.telemetry?.odom_frame_id;
    this.node('refresh').disabled = !this.online || !!this.pending;
    this.node('export').disabled = !this.online || !!this.pending || !state?.map_id;
    this.node('badge').textContent = state?.map_id ? `${this.activeEvents.length} events${state.read_only ? ' · Read only' : ''}` : 'No map';
    this.node('identity').textContent = state?.map_id ? `${state.name} · revision ${state.revision}\n${state.reference}` : 'No map selected';
    this.node('status').textContent = [this.notice, this.draftNotice, !this.online ? '' : !this.fresh() ? 'Waiting for fresh telemetry.' : '',
      state?.error || this.telemetry?.mapping?.error || '', !editor ? 'Monitor only: view and export.' : '',
      !state?.map_id && this.online && !this.pending ? editor ? 'Create a map with a measured coordinate reference.' : 'No map selected by the operator.' : ''].filter(Boolean).join(' ')
      || 'Map saved. Observations require explicit location and evidence.';
    const live = this.liveRover();
    this.node('live').textContent = live ? `Rover estimate: X ${fmt(live.xy[0])}, Y ${fmt(live.xy[1])} m. Live position uncertainty is unknown.`
      : !this.fresh() ? 'Rover hidden: no fresh telemetry.' : this.telemetry?.pose_valid !== true ? 'Rover hidden: pose is invalid.' : 'Rover hidden: current local frame is not registered.';
  }

  draw() {
    const now = this.now();
    if (this.pending && now - this.pending.at > REQUEST_MS) {
      const action = this.pending.action; this.pending = null;
      this.notice = `${action} response timed out. No write was retried; refresh to check the saved record.`;
      this.render();
    } else if (now - this.lastRender > 250) this.render();
    if (!this.root.open) return;
    const canvas = this.node('canvas'), w = canvas.clientWidth, h = canvas.clientHeight;
    if (w < 20 || h < 20) return;
    const dpr = globalThis.devicePixelRatio || 1;
    const resized = canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr);
    const live = this.liveRover(), identity = `${!!live}:${this.telemetry?.odom_frame_id || ''}`;
    // Keep drive/video animation independent of map size. Validity changes,
    // frame changes and resizing bypass this limit to clear stale markers.
    if (!resized && identity === this.lastDrawIdentity && now - this.lastDraw < DRAW_MS) return;
    this.lastDraw = now; this.lastDrawIdentity = identity;
    if (resized) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
    const g = canvas.getContext('2d'); if (!g) return;
    g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, w, h);
    const css = getComputedStyle(this.root), color = key => css.getPropertyValue('--' + key).trim();
    const events = this.locatedEvents;
    g.fillStyle = color('muted'); g.font = '11px sans-serif';
    if (!events.length && !live) { g.fillText('No located observations', 14, h / 2 - 8); g.fillText('Unknown locations remain in Events.', 14, h / 2 + 12); return; }
    let [loX, hiX, loY, hiY] = this.mapExtent;
    if (live) { loX = Math.min(loX, live.xy[0]); hiX = Math.max(hiX, live.xy[0]); loY = Math.min(loY, live.xy[1]); hiY = Math.max(hiY, live.xy[1]); }
    const span = Math.max(1, hiX - loX, hiY - loY), cx = loX / 2 + hiX / 2, cy = loY / 2 + hiY / 2;
    if (![span, cx, cy].every(Number.isFinite)) { g.fillText('Coordinate range cannot be displayed.', 14, h / 2); return; }
    const scale = Math.min(w - 54, h - 44) / (span * 1.15);
    if (!Number.isFinite(scale) || scale <= 0) return;
    const project = point => [w / 2 + (point[0] - cx) * scale, h / 2 - (point[1] - cy) * scale];
    const rough = span / 4, power = 10 ** Math.floor(Math.log10(rough)), fraction = rough / power;
    const step = power * (fraction < 2 ? 1 : fraction < 5 ? 2 : 5);
    g.strokeStyle = color('grid'); g.lineWidth = 1;
    // Bounded grid loops also protect rendering when imported coordinates are extreme.
    for (let i = 0; i <= 8; i++) {
      const x = Math.floor((cx - span * .6) / step) * step + i * step, y = Math.floor((cy - span * .6) / step) * step + i * step;
      const [sx] = project([x, cy]), [, sy] = project([cx, y]);
      if (sx >= 25 && sx < w - 8) { g.beginPath(); g.moveTo(sx, 10); g.lineTo(sx, h - 20); g.stroke(); g.fillText(fmt(x), sx + 2, h - 6); }
      if (sy >= 10 && sy < h - 20) { g.beginPath(); g.moveTo(25, sy); g.lineTo(w - 8, sy); g.stroke(); g.fillText(fmt(y), 3, sy - 2); }
    }
    g.fillText('Y ↑   X → (m)', 8, 12);
    for (const event of events) {
      const [x, y] = project(event.map_xy_m), c = color({ scout_first_contact: 's1', scout_rest: 's2', clegg_impact: 's3', landmark: 'accent' }[event.kind] || 'fg');
      g.strokeStyle = c; g.fillStyle = c; g.lineWidth = 1.5;
      const radius = event.uncertainty_m * scale;
      if (Number.isFinite(radius) && radius > 0) { g.globalAlpha = .4; g.setLineDash([3, 3]); g.beginPath(); g.arc(x, y, radius, 0, Math.PI * 2); g.stroke(); g.globalAlpha = 1; g.setLineDash([]); }
      g.beginPath();
      if (event.kind === 'scout_rest') g.rect(x - 4, y - 4, 8, 8);
      else if (event.kind === 'clegg_impact') { g.moveTo(x, y - 5); g.lineTo(x + 5, y); g.lineTo(x, y + 5); g.lineTo(x - 5, y); g.closePath(); }
      else if (event.kind === 'landmark') { g.moveTo(x - 5, y); g.lineTo(x + 5, y); g.moveTo(x, y - 5); g.lineTo(x, y + 5); }
      else g.arc(x, y, 4, 0, Math.PI * 2);
      g.stroke(); g.fillText(String(event.shot_id || event.label || '').slice(0, 32), x + 8, y - 6);
    }
    if (live) {
      const [x, y] = project(live.xy), yaw = live.yaw * Math.PI / 180;
      g.save(); g.translate(x, y); g.rotate(-yaw); g.strokeStyle = color('fg'); g.fillStyle = color('panel'); g.lineWidth = 2;
      g.beginPath(); g.moveTo(9, 0); g.lineTo(-6, -5); g.lineTo(-3, 0); g.lineTo(-6, 5); g.closePath(); g.fill(); g.stroke(); g.restore();
      g.fillStyle = color('fg'); g.fillText('Rover estimate', x + 10, y + 14);
    }
  }
}
