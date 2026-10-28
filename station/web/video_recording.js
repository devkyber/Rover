// Remote recording state is authoritative, including after reconnect/reload.
export function videoRecordingDisplay(state, live) {
  const base = { text: 'VIDEO REC UNAVAILABLE', tone: 'warn', canStart: false, canStop: false,
    title: 'Connect to a runtime with video recording support.' };
  if (!state || !live) return { ...base, title: !live
    ? 'Recording status unknown while disconnected. An existing Jetson recording continues. Reconnect to check or stop it.' : base.title };
  const mb = Number.isFinite(state.synced_bytes) ? (state.synced_bytes / 1048576).toFixed(1) : '--';
  const elapsed = Number.isFinite(state.elapsed_s) ? Math.floor(state.elapsed_s) : 0;
  const clock = `${Math.floor(elapsed / 60)}:${String(elapsed % 60).padStart(2, '0')}`;
  const gaps = (state.gaps || 0) + (state.dropped || 0) > 0;
  const title = [state.directory ? `Jetson: ${state.directory}` : 'Saved on the Jetson under logs/video/.',
    state.file ? `File: ${state.file}` : '',
    `Synced: ${mb} MB; files: ${state.segments || 0}; gaps: ${state.gaps || 0}; dropped: ${state.dropped || 0}; waiting-for-key skips: ${state.skipped || 0}.`,
    state.error || '', 'Camera video only. STOP LOG and browser disconnect do not stop video recording.'].filter(Boolean).join('\n');
  const result = { ...base, title, canStart: state.available === true && !state.requested
    && ['idle', 'saved', 'error'].includes(state.status), canStop: state.requested === true };
  if (state.status === 'error') return { ...result, text: 'VIDEO SAVE ERROR', tone: 'bad' };
  if (state.status === 'unavailable') return { ...result, text: 'VIDEO REC UNAVAILABLE' };
  if (state.status === 'saving') return { ...result, text: 'SAVING VIDEO…' };
  if (state.status === 'starting') return { ...result, text: 'VIDEO REC STARTING' };
  if (state.status === 'waiting') return { ...result, text: `REC WAITING · ${mb} MB` };
  if (state.status === 'recording' && state.requested && state.synced_bytes > 0)
    return { ...result, text: `● REC ${clock} · ${mb} MB${gaps ? ' · GAPS' : ''}`, tone: gaps ? 'warn' : 'good' };
  if (state.status === 'saved' && state.close_complete)
    return { ...result, text: state.synced_bytes > 0 ? `SAVED · ${mb} MB${gaps ? ' · GAPS' : ''}` : 'NO VIDEO CAPTURED', tone: gaps || !state.synced_bytes ? 'warn' : 'good' };
  if (state.status === 'idle') return { ...result, text: 'VIDEO REC OFF', tone: '' };
  return { ...result, text: 'VIDEO REC WAITING' };
}

export class VideoRecordingControls {
  constructor(root, { send, canControl }) {
    this.root = root;
    this.send = send;
    this.canControl = canControl;
    this.start = root.querySelector('[data-video-start]');
    this.stop = root.querySelector('[data-video-stop]');
    this.label = root.querySelector('[data-video-status]');
    this.pendingUntil = 0;
    this.start.addEventListener('click', () => this.request('video_start'));
    this.stop.addEventListener('click', () => this.request('video_stop'));
  }
  request(type) {
    const display = videoRecordingDisplay(this.state, this.live);
    if (!this.canControl || Date.now() < this.pendingUntil
        || !(type === 'video_start' ? display.canStart : display.canStop)) return;
    // Send once. Never replay a start/stop when a socket reconnects.
    if (this.send({ type, ...(type === 'video_stop' ? { directory: this.state.directory } : {}) }) === false) return;
    this.pendingUntil = Date.now() + 1500;
    this.update(this.state, this.live);
  }
  update(state, live) {
    if (state?.status !== this.state?.status) this.pendingUntil = 0;
    this.state = state;
    this.live = live;
    const display = videoRecordingDisplay(state, live);
    const pending = Date.now() < this.pendingUntil;
    this.start.disabled = !this.canControl || !display.canStart || pending;
    this.stop.disabled = !this.canControl || !display.canStop || pending;
    this.label.textContent = pending ? 'VIDEO REQUEST SENT…' : display.text;
    this.label.title = display.title;
    this.root.title = display.title;
    for (const tone of ['good', 'warn', 'bad']) this.label.classList.toggle(tone, display.tone === tone);
  }
}
