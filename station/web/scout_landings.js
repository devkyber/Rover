// Read-only landing overlays. Coordinates cross frames only via saved registrations.
const xy = p => Array.isArray(p) && p.length >= 2 && p.slice(0, 2).every(Number.isFinite);
const kinds = new Set(['first_contact', 'rest']);
const id = v => typeof v === 'string' && v.trim().length > 0;

export function parseLandingBundle(data) {
  if (data?.schema !== 'scout-shot-landings-v1' || !['scout_1', 'scout_2'].includes(data.scout_id)
      || !id(data.shot_id) || data.conditional !== true || data.field_validated !== false
      || typeof data.synthetic !== 'boolean' || !Array.isArray(data.markers)
      || !/^[a-f0-9]{64}$/i.test(data.source?.sha256 || '')) throw new Error('Invalid Scout landing report.');
  if (data.markers.length > 2 || (data.qualified !== true && data.markers.length)) throw new Error('No qualified event positions.');
  const seen = new Set();
  for (const marker of data.markers) {
    if (!id(data.frame_id) || !kinds.has(marker.kind) || seen.has(marker.kind)
        || !Array.isArray(marker.position_world_m) || marker.position_world_m.length !== 3
        || !marker.position_world_m.every(Number.isFinite) || !id(marker.evidence)
        || !Number.isInteger(marker.source?.sequence) || marker.source.sequence < 0
        || marker.uncertainty_m !== null) throw new Error('Landing markers need a frame, finite coordinates and independent event evidence.');
    seen.add(marker.kind);
  }
  return structuredClone(data);
}

function registration(state, frame) {
  const r = (state?.registrations || []).find(row => row.frame_id === frame && !row.superseded);
  return r && xy(r.translation_m) && Number.isFinite(r.yaw_deg) ? r : null;
}
function toMap(point, r) {
  const a = r.yaw_deg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
  return [c * point[0] - s * point[1] + r.translation_m[0], s * point[0] + c * point[1] + r.translation_m[1]];
}
export function landingToLocal(point, frame, state, telemetry) {
  const target = telemetry?.odom_frame_id;
  if (!xy(point) || !id(frame) || !id(target)) return null;
  if (frame === target) return point.slice(0, 2);
  if (!state?.map_id || state.map_id !== telemetry?.mapping?.map_id) return null;
  const destination = registration(state, target);
  if (!destination) return null;
  const source = registration(state, frame);
  const mapped = frame === 'map:' + state.map_id ? point : source ? toMap(point, source) : null;
  if (!mapped) return null;
  const a = destination.yaw_deg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
  const dx = mapped[0] - destination.translation_m[0], dy = mapped[1] - destination.translation_m[1];
  const result = [c * dx + s * dy, -s * dx + c * dy];
  return xy(result) ? result : null;
}

export function landingMarkers(bundles, state, telemetry) {
  const markers = [], hidden = [];
  const add = (point, frame, properties) => {
    const local = landingToLocal(point, frame, state, telemetry);
    if (local) markers.push({ ...properties, xy: local }); else hidden.push(properties);
  };
  for (const bundle of bundles) for (const marker of bundle.markers) {
    add(marker.position_world_m, bundle.frame_id, { kind: marker.kind, scoutId: bundle.scout_id,
      shotId: bundle.shot_id, conditional: true, synthetic: bundle.synthetic });
  }
  if (state?.map_id && state.map_id === telemetry?.mapping?.map_id) {
    for (const event of state.observations || []) {
      if (event.superseded || !['scout_first_contact', 'scout_rest'].includes(event.kind)) continue;
      const location = event.location;
      const direct = location?.kind === 'local' && location.frame_id === telemetry.odom_frame_id;
      if (!direct && !xy(event.map_xy_m)) continue;
      add(direct ? location.xy_m : event.map_xy_m,
          direct ? location.frame_id : 'map:' + state.map_id,
          { kind: event.kind === 'scout_rest' ? 'rest' : 'first_contact', scoutId: null,
            shotId: event.shot_id || event.label || '', conditional: false, synthetic: false });
    }
  }
  return { markers, hidden };
}

export class ScoutLandings {
  constructor(root, { limits, changed = () => {} }) {
    this.bundles = []; this.notice = ''; this.root = root; this.limits = limits; this.changed = changed;
    this.node = name => root.querySelector('#scout-landings-' + name);
    this.node('file').addEventListener('change', async event => {
      try {
        const files = [...event.target.files], max = this.limits()?.max_import_bytes;
        if (!Number.isFinite(max) || max <= 0) throw new Error('Connect to Rover before importing reports.');
        if (files.some(file => file.size > max)) throw new Error('Landing report exceeds the configured import size.');
        const incoming = await Promise.all(files.map(async file => parseLandingBundle(JSON.parse(await file.text()))));
        for (const bundle of incoming) this.load(bundle);
        if (incoming.length) this.root.open = false;
        this.notice = '';
      } catch (error) { this.notice = error.message; }
      event.target.value = ''; this.changed();
    });
    this.node('clear').addEventListener('click', () => { this.bundles = []; this.notice = ''; this.changed(); });
  }
  load(data) {
    const bundle = parseLandingBundle(data);
    this.bundles = this.bundles.filter(b => b.scout_id !== bundle.scout_id || b.shot_id !== bundle.shot_id);
    this.bundles.push(bundle); this.changed();
  }
  update(telemetry, state) {
    const result = landingMarkers(this.bundles, state, telemetry);
    const count = this.node('count');
    if (count) count.textContent = result.markers.length ? `(${result.markers.length})` : '';
    this.node('status').textContent = this.notice || `${result.markers.length} landing markers · ○ Contact · □ Final rest`
      + (result.hidden.length ? ` · ${result.hidden.length} hidden: register the coordinate frame in MISSION MAP` : '')
      + (result.markers.some(m => m.conditional) ? ' · ESTIMATE; dashed' : '')
      + (result.markers.some(m => m.synthetic) ? ' · SYNTHETIC' : '');
    return result.markers;
  }
}
