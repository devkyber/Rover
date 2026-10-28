// Explicit Scout v12 commands. The server owns transport keepalive.
// Telemetry, reconnects and browser focus never issue or replay movement.
import { JUMPER_OPERATOR_PROFILES } from './scout_profiles.js';
export const PROFILE_FIELDS = [
  'servo_model', 'servo_mode', 'motor_direction', 'encoder_sign', 'motor_duty_pct',
  'load_timeout_ms', 'stall_ms', 'hold_tolerance_counts', 'servo_neutral_us',
  'prepare_ms', 'latch_ms', 'release_ms',
];
const EDITABLE_FIELDS = ['servo_model', 'motor_direction', 'encoder_sign', 'motor_duty_pct'];
const ZERO_FIELDS = [
  'load_timeout_ms', 'stall_ms', 'hold_tolerance_counts', 'servo_neutral_us',
  'prepare_ms', 'latch_ms', 'release_ms',
];
const ACTION_LABELS = {
  latch: 'LOCK', prepare: 'PULL', launch: 'LAUNCH', compress: 'COMPRESS', decompress: 'DECOMPRESS',
  compression_stop: 'STOP ROTATION', estop: 'E-STOP', configure: 'APPLY SETTINGS',
};
export function manualContract(m) {
  return (m.api || m.status?.api) === 'scout-process-v12';
}

function stopRecovered(status) {
  return status.healthy === true && status.driver_fault === false &&
    status.motor_running === false && status.servo_role === 'off';
}

export function scoutState(status) {
  if (status.driver_fault === true) return { label: 'DRIVER ALERT', tone: 'warn' };
  if (status.state === 'fault') {
    if (['operator_stop', 'estop', 'link_timeout', 'connection_changed'].includes(status.fault)) {
      return { label: status.fault === 'estop' ? 'E-STOP' : 'STOPPED', tone: 'idle' };
    }
    return { label: 'STOPPED', tone: stopRecovered(status) ? 'idle' : 'warn' };
  }
  return { label: status.state || 'ready', tone: 'good' };
}

const number = (v, digits = 0) => Number.isFinite(v) ? v.toFixed(digits) : '--';
const randomUuid = () => {
  if (crypto.randomUUID) return crypto.randomUUID();
  // LAN HTTP pages may lack randomUUID; getRandomValues remains available.
  const bytes = crypto.getRandomValues(new Uint8Array(16));
  bytes[6] = (bytes[6] & 15) | 64;
  bytes[8] = (bytes[8] & 63) | 128;
  const hex = [...bytes].map(v => v.toString(16).padStart(2, '0')).join('');
  return `${hex.slice(0, 8)}-${hex.slice(8, 12)}-${hex.slice(12, 16)}-${hex.slice(16, 20)}-${hex.slice(20)}`;
};

export function profileError(p) {
  if (typeof p.servo_model !== 'string' || !p.servo_model.trim()) return 'Enter the servo model or source identity.';
  if (p.servo_mode !== 'position') return 'Scout v12 uses the J5 position servo.';
  for (const key of ['motor_direction', 'encoder_sign']) {
    if (![1, -1].includes(p[key])) return `Set ${key} to +1 or -1.`;
  }
  if (!Number.isInteger(p.motor_duty_pct) || p.motor_duty_pct < 1 || p.motor_duty_pct > 100) {
    return 'Motor duty requires a whole number from 1 to 100.';
  }
  for (const key of ZERO_FIELDS) {
    if (p[key] !== 0) return `${key} must be 0 for Scout v12.`;
  }
  if (Object.keys(p).some(key => !PROFILE_FIELDS.includes(key))) return 'Unsupported Scout v12 setting.';
  return '';
}

export class JumperPanel {
  constructor({ root, send, enabled, controls = true, scoutId = null, idPrefix = 'jumper-',
    now = () => performance.now(), uuid = randomUuid }) {
    this.root = root;
    this.scoutId = scoutId;
    this.send = send;
    this.enabled = enabled;
    this.controls = controls;
    this.now = now;
    this.uuid = uuid;
    this.controllerId = uuid();
    this.connectionId = null;
    this.pending = null;
    this.lastTick = -Infinity;
    this.live = false;
    this.data = {};
    this.notice = '';
    this.sourceProfileSeen = false;
    this.profileEdited = false;
    this.node = id => root.querySelector('#' + idPrefix + id);
    this.buttons = {
      configure: 'configure', latch: 'latch', prepare: 'prepare', launch: 'launch',
      compress: 'compress', decompress: 'decompress', stop: 'compression_stop', estop: 'estop',
    };
    for (const [id, action] of Object.entries(this.buttons)) {
      const button = this.node(id);
      button.addEventListener('click', () => this.request(action));
      button.addEventListener('keydown', event => {
        // Each press is one command; Space retains the dashboard E-STOP.
        if (event.code === 'Space' || (event.repeat && event.code === 'Enter')) event.preventDefault();
      });
    }
    for (const id of EDITABLE_FIELDS) {
      const changed = () => { this.profileEdited = true; this.render(); };
      this.node(id).addEventListener('input', changed);
      this.node(id).addEventListener('change', changed);
    }
    this.render();
  }

  profile() {
    const values = Object.fromEntries(EDITABLE_FIELDS.map(key => {
      const value = this.node(key).value.trim();
      return [key, key === 'servo_model' ? value : value === '' ? null : Number(value)];
    }));
    // Fixed position mode and disabled timing limits are the v12 wire contract.
    return Object.fromEntries(PROFILE_FIELDS.map(key => [key,
      key === 'servo_mode' ? 'position' : ZERO_FIELDS.includes(key) ? 0 : values[key]]));
  }

  operatorProfile() {
    const entry = JUMPER_OPERATOR_PROFILES[this.data.host];
    return entry?.scout_id === this.scoutId && !profileError(entry.settings || {}) ? entry : null;
  }

  preloadOperatorProfile() {
    const entry = this.operatorProfile();
    if (!entry || this.sourceProfileSeen) return;
    this.sourceProfileSeen = true;
    // Display defaults only. Applying settings always requires a button press.
    if (this.profileEdited || EDITABLE_FIELDS.some(key => this.node(key).value.trim() !== '')) return;
    for (const key of EDITABLE_FIELDS) this.node(key).value = String(entry.settings[key]);
  }

  gates() {
    const d = this.data, m = d.manager || {}, s = m.status || {};
    const connected = this.live && this.now() - this.lastTick < 500 &&
      d.connection_status === 'ok' && !!this.connectionId && m.connected === true;
    const base = connected && this.controls && this.enabled() && manualContract(m);
    return {
      configure: base && s.motor_running !== true && !profileError(this.profile()),
      latch: base, prepare: base, launch: base && m.launch_supported === true,
      compress: base, decompress: base, compression_stop: base,
      estop: this.controls && !!this.connectionId,
    };
  }

  emit(action, payload = {}) {
    const message = { type: 'jumper', action, connection_id: this.connectionId,
      controller_id: this.controllerId, request_id: this.uuid(), ...payload };
    if (this.scoutId !== null) message.scout_id = this.scoutId;
    this.send(message);
    return message.request_id;
  }

  request(action) {
    if (!this.gates()[action]) return;
    const requestId = this.emit(action, action === 'configure' ? { settings: this.profile() } : {});
    this.pending = { requestId, action, at: this.now() };
    this.notice = `${ACTION_LABELS[action]}: waiting for ACK`;
    this.render();
  }

  // Transport acceptance means queued at the rover, not an ESP ACK.
  onResult(result) {
    if (result.scout_id != null && result.scout_id !== this.scoutId) return;
    if (!this.pending || result.request_id !== this.pending.requestId || result.ok !== false) return;
    const action = this.pending.action;
    this.pending = null;
    this.notice = `${ACTION_LABELS[action]} rejected: ${result.error || 'unknown error'}`;
    this.render();
  }

  // Call only for explicit DISCONNECT/change of Rover/E-STOP, never focus loss.
  suspend(message = 'Control disconnected; output stop requested.') {
    if (this.controls && this.connectionId) this.emit('stop');
    this.pending = null;
    this.notice = message;
    this.render();
  }

  tick(live, telemetry) {
    const fleet = telemetry?.jumper;
    const d = fleet?.scouts ? fleet.scouts[this.scoutId] || {}
      : this.scoutId === null || this.scoutId === 'scout_1' ? fleet || {} : {};
    const connection = d.connection_id || null;
    if (connection !== this.connectionId) {
      this.connectionId = connection;
      this.pending = null;
      this.notice = connection ? 'Connected. Commands require a new button press.' : 'Scout disconnected.';
    }
    this.data = d;
    this.live = live;
    this.lastTick = this.now();
    this.preloadOperatorProfile();
    if (this.pending && d.last_result?.request_id === this.pending.requestId) {
      const result = d.last_result;
      const action = this.pending.action;
      this.pending = null;
      this.notice = result.ok ? `${ACTION_LABELS[action]}: ACK received`
        : `${ACTION_LABELS[action]}: ${result.error || 'failed'}`;
    }
    if (this.pending && this.now() - this.pending.at > 2000) {
      this.notice = 'No ACK. Check status before issuing another command.';
      this.pending = null;
    }
    this.render();
  }

  render() {
    const d = this.data, m = d.manager || {}, s = m.status || {};
    const gates = this.gates();
    for (const [id, action] of Object.entries(this.buttons)) this.node(id).disabled = !gates[action];
    for (const key of EDITABLE_FIELDS) this.node(key).disabled = !this.controls;
    this.node('compress').classList.toggle('on', s.motor_action === 'compress' && s.motor_running === true);
    this.node('decompress').classList.toggle('on', s.motor_action === 'decompress' && s.motor_running === true);
    this.node('address').textContent = d.host || m.host || '--';
    this.node('device').textContent = d.host || 'Not configured';
    const fresh = this.live && this.now() - this.lastTick < 500 && d.connection_status === 'ok';
    const supported = manualContract(m);
    const display = scoutState(s);
    const statusCurrent = fresh && m.connected === true && m.fresh !== false;
    this.node('state').textContent = (!fresh ? d.connection_status || 'off' : !supported ? 'READ ONLY'
      : !statusCurrent ? m.connected === false ? 'LINK LOST' : 'STATUS STALE' : display.label).toUpperCase();
    this.node('state').className = 'chip ' + (fresh && supported && statusCurrent ? display.tone : 'warn');
    this.node('owner').textContent = !this.controls ? 'Monitor only' : d.owner_id && d.owner_id !== this.controllerId
      ? 'Another controller' : 'Direct control';
    const reason = s.fault && s.fault !== 'none' ? `Last stop: ${s.fault}` : '';
    this.node('note').textContent = [this.notice, reason,
      d.connection_status !== 'ok' ? 'Scout disconnected' : '', d.last_result?.ok === false ? d.last_result.error : '']
      .filter(Boolean).join(' · ');
    this.node('battery').textContent = `${number(Number.isFinite(s.battery_mv) ? s.battery_mv / 1000 : null, 2)} V${s.driver_fault ? ' · DRIVER ERROR' : ''}`;
    this.node('encoder').textContent = `${number(s.encoder)} counts · progress ${number(s.compression_progress)}`;
    this.node('cycle').textContent = s.compression_result_valid === true ? `${number(s.compression_result_counts)} counts` : 'No stored compression result';
    this.node('health').textContent = `${m.api || s.api || '--'} · motor ${s.motor_action || '--'} · servo ${s.servo_role || '--'}`;
    const sensor = (name, status, fields) => {
      const row = m.sensors?.[name];
      if (!row) return `${status} · No samples`;
      const values = fields.map(([label, keys, digits]) => `${label}: ${keys.map(key => number(row[key], digits)).join(' / ')}`);
      const age = m.ages?.sensors?.[name];
      const freshness = age ? `received ${number(age.receipt_s, 3)} s ago · device age ${number(age.device_s, 3)} s` : 'sample age unavailable';
      const flags = ['flags', 'fifo_full', 'estimated_timestamp', 'saturated', 'sensor_error'].map(key => `${key}=${row[key] ?? '--'}`).join(' · ');
      return `${status} · ${freshness}\ntime ${row.time_us ?? '--'} µs · seq ${row.sequence ?? '--'}\n${values.join('\n')}\n${flags}`;
    };
    const chipStatus = (ok, extra = '') => `${ok === true ? 'INIT OK' : ok === false ? 'INIT FAILED' : 'INIT UNKNOWN'} · ${fresh ? 'ESP status fresh' : 'ESP status stale'}${extra}`;
    const verifiedUnits = m.motion_compatible !== false && m.units_verified !== false;
    const accel = verifiedUnits ? ['accel g', ['ax_g', 'ay_g', 'az_g'], 3] : ['accel raw (scale unverified)', ['ax_raw', 'ay_raw', 'az_raw'], 0];
    const gyro = verifiedUnits ? ['gyro °/s', ['gx_dps', 'gy_dps', 'gz_dps'], 2] : ['gyro raw (scale unverified)', ['gx_raw', 'gy_raw', 'gz_raw'], 0];
    this.node('icm').textContent = sensor('ICM-45605', chipStatus(s.icm_ok), [accel, gyro]);
    this.node('adxl').textContent = sensor('ADXL375', chipStatus(s.adxl_ok), [accel]);
    this.node('mag').textContent = sensor('MMC5983MA', chipStatus(s.mag_ok, ` · mag ${s.mag_fresh ? 'fresh' : 'stale'} · ID ${s.mag_id ?? '--'} · frames ${s.mag_frames ?? '--'}`), [['raw', ['mx_raw', 'my_raw', 'mz_raw'], 0]]);
    this.node('capture').textContent = `Capture: ${d.recording?.prefix || '--'}${d.recording?.error ? ' · ' + d.recording.error : ''}`;
    this.node('counters').textContent = 'Counts / loss: ' + JSON.stringify({ ...m.diagnostics, ...d.counters });
    this.node('events').textContent = (m.events || []).slice(-5).map(e => `${e.time ?? ''} ${e.text ?? ''}`).join('\n');
    this.node('configure').title = profileError(this.profile()) || 'Apply optional settings; no movement';
    this.node('launch').title = m.launch_supported === true
      ? 'Move J5 to LAUNCH (110°); motor output is unchanged'
      : 'LAUNCH requires the Scout 110° firmware update';
  }
}

export function mountScoutPanels(root) {
  const second = root.cloneNode(true);
  second.id = 'jumper-2-panel';
  for (const node of second.querySelectorAll('[id]')) {
    node.id = node.id.replace(/^jumper-/, 'jumper-2-');
  }
  for (const [index, panel] of [root, second].entries()) {
    panel.dataset.scoutId = `scout_${index + 1}`;
    panel.querySelector('h2').textContent = `SCOUT ${index + 1}`;
    panel.setAttribute('aria-label', `Scout ${index + 1} controls`);
    for (const node of panel.querySelectorAll('[aria-label]')) {
      node.setAttribute('aria-label', `Scout ${index + 1}: ${node.getAttribute('aria-label')}`);
    }
  }
  root.after(second);
  return [root, second];
}

export class JumperPanels {
  constructor({ root, ...options }) {
    this.panels = mountScoutPanels(root).map((panel, index) => new JumperPanel({
      ...options, root: panel, scoutId: `scout_${index + 1}`,
      idPrefix: index === 0 ? 'jumper-' : 'jumper-2-',
    }));
  }
  tick(live, telemetry) { for (const panel of this.panels) panel.tick(live, telemetry); }
  suspend(message) { for (const panel of this.panels) panel.suspend(message); }
  onResult(result) { for (const panel of this.panels) panel.onResult(result); }
}
