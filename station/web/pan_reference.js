// Local manual reference and sampled PAN command times. No angle estimate,
// motion command, automatic stop or persistent zero is produced here.
export class PanTimeReference {
  constructor(maxGapS) {
    this.maxGapS = maxGapS;
    this.leftS = 0;
    this.rightS = 0;
    this.hasReference = false;
    this.gap = false;
    this.previous = null;
    this.live = false;
  }

  observe(t, at) {
    const board = t?.pico, command = t?.cmd;
    if (!Number.isFinite(at) || !Number.isFinite(t?.t)
        || !Number.isFinite(command?.pan) || Math.abs(command.pan) > 1
        || board?.status !== 'ok' || board.failsafe === true) {
      this.interrupt();
      return;
    }
    const sample = { at, time: t.t, run: t.run_id, generation: board.generation,
      pan: command.estop || command.stale ? 0 : Math.sign(command.pan) };
    const previous = this.previous;
    if (this.hasReference && previous) {
      const dt = sample.time - previous.time, elapsed = at - previous.at;
      if (sample.run === previous.run && sample.generation === previous.generation
          && dt >= 0 && dt < this.maxGapS && elapsed >= 0 && elapsed < this.maxGapS) {
        // Attribute each observed interval to its last accepted command.
        // Left/right are separate: equal times do not imply equal travel.
        if (previous.pan < 0) this.leftS += dt;
        if (previous.pan > 0) this.rightS += dt;
      } else {
        this.gap = true;
      }
    }
    this.previous = sample;
    this.live = true;
  }

  interrupt() {
    if (this.hasReference) this.gap = true;
    this.previous = null;
    this.live = false;
  }

  canZero(at) {
    return this.live && this.previous?.pan === 0 && at >= this.previous.at
      && at - this.previous.at < this.maxGapS;
  }

  zero(at) {
    if (!this.canZero(at)) return false;
    this.leftS = this.rightS = 0;
    this.hasReference = true;
    this.gap = false;
    return true;
  }
}

export class PanReferencePanel {
  constructor(root, { maxGapS, canSetZero }) {
    this.root = root;
    this.canSetZero = canSetZero;
    this.reference = new PanTimeReference(maxGapS);
    this.state = root.querySelector('[data-pan-reference-state]');
    this.left = root.querySelector('[data-pan-left]');
    this.right = root.querySelector('[data-pan-right]');
    this.button = root.querySelector('[data-pan-zero]');
    this.button.addEventListener('click', () => {
      if (this.canSetZero) this.reference.zero(performance.now() / 1000);
      this.render();
    });
    this.render();
  }

  observe(t) {
    this.reference.observe(t, performance.now() / 1000);
    this.render();
  }

  interrupt() {
    this.reference.interrupt();
    this.render();
  }

  tick(live) {
    if (!live) this.reference.interrupt();
    this.render();
  }

  render() {
    const r = this.reference;
    this.state.textContent = !r.hasReference ? 'NO 0' : r.gap ? 'CHECK 0' : 'REF 0°';
    this.state.classList.toggle('warn', r.hasReference && r.gap);
    this.left.textContent = 'L ' + (r.hasReference ? this.time(r.leftS) : '--');
    this.right.textContent = 'R ' + (r.hasReference ? this.time(r.rightS) : '--');
    this.root.title = 'Manual zero; sampled PAN command time only, not measured angle. '
      + 'Left/right speeds differ; matching times do not mean centered. '
      + (r.gap ? 'Observation gap: check camera/cable and set zero again. ' : '')
      + 'Reference lasts only on this page. Reload requires SET 0.';
    this.button.title = 'Point camera to your chosen zero and release PAN, then reset both times. No motion.';
    this.button.disabled = !this.canSetZero || !r.canZero(performance.now() / 1000);
  }

  time(seconds) {
    return seconds < 1000 ? seconds.toFixed(1) + 's' : (seconds / 60).toFixed(1) + 'm';
  }
}
