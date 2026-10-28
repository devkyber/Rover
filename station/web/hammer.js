// Manual commands: fresh link, E-STOP, and numeric output caps.
export function encodeLegacyStringCommand(intent, pulse_us, seconds, settings) {
  const offset = settings?.servo_offset_us;
  const low = settings?.min_center_us, high = settings?.max_center_us;
  if (![pulse_us, seconds, offset, low, high, settings?.min_seconds, settings?.max_seconds]
    .every(Number.isFinite) || offset <= 0) return null;
  const pulse = Math.round(Math.max(low, Math.min(high, pulse_us)));
  const positive = pulse - offset >= low && pulse - offset <= high;
  const center = positive ? pulse - offset : pulse + offset;
  if (center < low || center > high) return null;
  return { seconds: Math.max(settings.min_seconds, Math.min(settings.max_seconds, seconds)),
    speed: 1, center_us: center, reverse: intent === 'drop' ? !positive : positive };
}

export class HammerPanel {
  constructor({ root, send, enabled, onReleaseRequested = null }) {
    Object.assign(this, { root, send, enabled });
    this.onReleaseRequested = onReleaseRequested || (detail => {
      if (typeof CustomEvent === 'function')
        this.root.dispatchEvent?.(new CustomEvent('clegg-release-requested', { bubbles: true, detail }));
    });
    this.seq = 0;
    this.intent = 'idle';
    this.state = null;
    this.live = this.estop = false;
    this.pointer = this.heldButton = null;
    this.keyboardHeld = false;
    this.testMessage = null;
    this.stringSettings = null;
    this.stringSettingsLoaded = false;
    this.startedAt = 0;
    this.windHeldSeconds = 0;
    this.windStartedAt = null;
    for (const button of root.querySelectorAll('[data-hold]')) {
      button.addEventListener('pointerdown', e => {
        if (button.disabled || e.button !== 0) return;
        e.preventDefault();
        if (this.start(button.dataset.hold, button)) {
          button.setPointerCapture(e.pointerId);
          this.pointer = e.pointerId;
        }
      });
      button.addEventListener('pointerup', e => {
        if (e.pointerId === this.pointer && this.heldButton === button) this.release();
      });
      for (const name of ['pointercancel', 'lostpointercapture'])
        button.addEventListener(name, e => {
          if (e.pointerId === this.pointer) this.stop();
        });
      button.addEventListener('keydown', e => {
        if (e.code === 'Enter' && !e.repeat && !button.disabled) {
          e.preventDefault();
          if (this.start(button.dataset.hold, button)) this.keyboardHeld = true;
        }
      });
      button.addEventListener('keyup', e => {
        if (e.code === 'Enter' && this.keyboardHeld && this.heldButton === button) this.release();
      });
      button.addEventListener('contextmenu', e => e.preventDefault());
    }
    for (const [name, intent] of [['release', 'drop'], ['rearm', 'rearm']]) {
      const button = root.querySelector('#hammer-' + name);
      button.addEventListener('keydown', e => {
        if (e.repeat && ['Enter', 'Space'].includes(e.code)) e.preventDefault();
      });
      button.addEventListener('click', () => { if (!button.disabled) this.start(intent); });
    }
    root.querySelector('#hammer-torque-off')?.addEventListener('click', () => this.stop(true));
  }

  // This is browser hold time, not servo feedback or a motor duration limit.
  // The progress element owns the display-only reference; it never feeds a command.
  showWindTime() {
    const elapsed = this.windStartedAt === null ? this.windHeldSeconds
      : Math.max(0, (performance.now() - this.windStartedAt) / 1000);
    const output = this.root.querySelector('#hammer-wind-time');
    const progress = this.root.querySelector('#hammer-wind-progress');
    const timer = this.root.querySelector('#hammer-wind-timer');
    if (!output || !progress || !timer) return;
    const reference = progress.max;
    output.textContent = `${elapsed.toFixed(1)} / ${reference.toFixed(1)} s (${Math.floor(elapsed / reference * 100)}%)`;
    progress.value = Math.min(reference, elapsed);
    progress.setAttribute('aria-valuetext', `${elapsed.toFixed(1)} of ${reference} seconds held`);
    timer.dataset.running = String(this.windStartedAt !== null);
    timer.dataset.reached = String(elapsed >= reference);
  }

  finishWindTime() {
    if (this.windStartedAt !== null) {
      this.windHeldSeconds = Math.max(0, (performance.now() - this.windStartedAt) / 1000);
      this.windStartedAt = null;
    }
    this.showWindTime();
  }

  cylinderBlockReason() {
    if (!this.live) return 'DISCONNECTED';
    if (this.estop) return 'E-STOP';
    if (!this.enabled()) return 'Control unavailable';
    return '';
  }

  showCylinderStatus() {
    const status = this.root.querySelector('#hammer-cylinder-status');
    if (status) status.textContent = this.cylinderBlockReason()
      || this.state?.lift?.error || 'Hold UP / DOWN; release to HOLD.';
  }

  stringField(action, name) {
    return this.root.querySelector(`#hammer-string-${action}-${name}`);
  }

  syncStringSettings() {
    let settings = this.state?.string_settings;
    if (!settings) {
      const bounds = this.state?.test_settings;
      const profile = { ...bounds, ...this.state?.latch_settings };
      if (![profile.center_us, profile.speed, profile.seconds, bounds?.servo_offset_us,
        bounds?.min_center_us, bounds?.max_center_us, bounds?.min_seconds, bounds?.max_seconds]
        .every(Number.isFinite)) return;
      const offset = Math.round(bounds.servo_offset_us * profile.speed) * (profile.reverse ? -1 : 1);
      settings = { release: { pulse_us: profile.center_us + offset, seconds: profile.seconds },
        rearm: { pulse_us: profile.center_us - offset, seconds: profile.seconds },
        min_pulse_us: bounds.min_center_us, max_pulse_us: bounds.max_center_us,
        min_seconds: bounds.min_seconds, max_seconds: bounds.max_seconds };
    }
    if (!settings) return;
    this.stringSettings = settings;
    for (const action of ['release', 'rearm']) {
      for (const name of ['pulse_us', 'seconds']) {
        const field = this.stringField(action, name);
        if (!field) continue;
        field.min = settings[name === 'pulse_us' ? 'min_pulse_us' : 'min_seconds'];
        field.max = settings[name === 'pulse_us' ? 'max_pulse_us' : 'max_seconds'];
        if (!this.stringSettingsLoaded) field.value = settings[action][name];
        field.disabled = false;
      }
    }
    this.stringSettingsLoaded = true;
  }

  stringCommand(intent) {
    const action = intent === 'drop' ? 'release' : 'rearm';
    const settings = this.stringSettings;
    if (!settings?.[action]) return {};
    const command = {};
    for (const name of ['pulse_us', 'seconds']) {
      const field = this.stringField(action, name);
      const text = String(field?.value ?? '').trim();
      const number = text === '' ? NaN : Number(text);
      const fallback = settings[action][name];
      const low = settings[name === 'pulse_us' ? 'min_pulse_us' : 'min_seconds'];
      const high = settings[name === 'pulse_us' ? 'max_pulse_us' : 'max_seconds'];
      const bounded = Math.max(low, Math.min(high, Number.isFinite(number) ? number : fallback));
      command[name] = name === 'pulse_us' ? Math.round(bounded) : bounded;
      if (field) field.value = command[name];
    }
    return this.state?.string_settings ? command
      : encodeLegacyStringCommand(intent, command.pulse_us, command.seconds, this.state?.test_settings) || {};
  }

  start(intent, button = null) {
    if (this.cylinderBlockReason()) return false;
    if (intent === 'lift' && !['up', 'down'].includes(button?.dataset.liftDirection)) return false;
    this.finishWindTime();
    const settings = this.state?.test_settings || {};
    // Runtime config owns the output settings; telemetry supplies its current
    // values. Missing fields are omitted so the controller applies its defaults.
    const profile = intent === 'wind' ? { ...settings, ...this.state?.wind_settings }
      : ['drop', 'rearm'].includes(intent) ? { ...settings, ...this.state?.latch_settings } : settings;
    this.testMessage = Object.fromEntries(['seconds', 'speed', 'center_us', 'reverse']
      .filter(name => profile[name] !== undefined).map(name => [name, profile[name]]));
    if (['drop', 'rearm'].includes(intent)) Object.assign(this.testMessage, this.stringCommand(intent));
    if (intent === 'lift') {
      const sign = settings.lift_up_sign ?? -1;
      this.testMessage.reverse = (button.dataset.liftDirection === 'up' ? sign : -sign) < 0;
    }
    this.intent = intent;
    this.heldButton = button;
    this.keyboardHeld = false;
    this.seq++;
    this.startedAt = performance.now();
    if (intent === 'wind') {
      this.windHeldSeconds = 0;
      this.windStartedAt = this.startedAt;
      this.showWindTime();
    }
    this.transmit();
    if (intent === 'drop') this.onReleaseRequested({
      request_id: globalThis.crypto?.randomUUID?.() || `release-${Date.now()}-${Math.random()}`,
      hammer_seq: this.seq, browser_time_ms: Date.now(),
    });
    return true;
  }

  transmit() {
    this.send({ type: 'hammer', intent: this.intent, seq: this.seq,
      ...(this.testMessage ? { test: this.testMessage } : {}) });
  }

  release() {
    if (this.intent === 'lift' || (this.intent === 'wind' && this.state?.lift_holding)) this.requestHold();
    else this.stop();
  }

  requestHold() {
    this.finishWindTime();
    this.intent = 'lift_hold';
    this.seq++;
    this.pointer = this.heldButton = null;
    this.keyboardHeld = false;
    this.transmit();
  }

  stop(force = false) {
    this.finishWindTime();
    if (force || this.intent !== 'idle') {
      this.intent = 'idle';
      this.seq++;
      this.transmit();
    }
    this.pointer = this.heldButton = null;
    this.keyboardHeld = false;
  }

  tick(live, telemetry) {
    this.live = live;
    this.estop = !!telemetry?.cmd?.estop;
    this.state = telemetry?.hammer;
    const state = this.state;
    this.syncStringSettings();
    if (this.cylinderBlockReason()) this.stop();
    if (['drop', 'rearm'].includes(this.intent)
        && performance.now() - this.startedAt >= this.testMessage.seconds * 1000) {
      if (state?.lift_holding) this.requestHold();
      else this.stop();
    }
    if (this.intent !== 'idle') this.transmit();
    this.showWindTime();
    const available = !this.cylinderBlockReason();
    for (const name of ['lift', 'lower', 'wind', 'release', 'rearm']) {
      const button = this.root.querySelector('#hammer-' + name);
      button.disabled = !available;
      button.classList.toggle('on', this.heldButton === button ||
        (name === 'release' && this.intent === 'drop') || (name === 'rearm' && this.intent === 'rearm'));
    }
    const torqueOff = this.root.querySelector('#hammer-torque-off');
    if (torqueOff) { torqueOff.hidden = false; torqueOff.disabled = false; }
    this.showCylinderStatus();
    this.root.querySelector('#hammer-state').textContent = this.cylinderBlockReason()
      || ({ drop: 'ARM', rearm: 'RELEASE', lift_hold: 'HOLD' }[state?.active] || state?.active?.toUpperCase() || 'READY');
  }
}
