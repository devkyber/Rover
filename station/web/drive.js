// Operator input -> one command, { forward, turn, lift, pan }, all -1..+1.
//
// Three sources, added together and clipped: the keyboard, the on-screen
// controls (mouse or touch), and a gamepad. forward + is ahead; turn + is to
// the right, the same convention as rover/dxl_control.drive(). Pan turns the
// camera; hammer operations are deliberately separate from every drive input.
//
// Safety comes from the rover, not from here: it stops when drive messages
// stop arriving. This file only has to make sure a released key or a
// window that lost focus really reads as zero.

import { clamp } from './util.js';

const KEYS = {
  KeyW: [1, 0], KeyS: [-1, 0],
  KeyA: [0, -1], KeyD: [0, 1],
};

// Lift and camera pan are hold-to-run: the servo turns for as long as the
// key or button is down. [lift, pan]
const AUX_KEYS = {
  KeyQ: [0, -1], ArrowLeft: [0, -1],
  KeyE: [0, 1], ArrowRight: [0, 1],
};

// Sticks never rest at exactly zero. Ignore the first 12% and rescale the
// rest so the command still starts from 0 at the edge of the dead zone.
const DEAD_ZONE = 0.12;
const stick = v => Math.abs(v) < DEAD_ZONE ? 0 : Math.sign(v) * (Math.abs(v) - DEAD_ZONE) / (1 - DEAD_ZONE);

// What an on-screen lift/pan button asks for, from its data-lift / data-pan.
const buttonLift = button => Number(button.dataset.lift || 0);
const buttonPan = button => Number(button.dataset.pan || 0);

export class DriveInput {
  constructor({ pad, dot, auxButtons = [], onEstop, onSpeedStep = () => {} }) {
    this.pad = pad;
    this.dot = dot;
    this.auxButtons = [...auxButtons];
    this.onEstop = onEstop;       // called when the operator asks for a stop
    this.onSpeedStep = onSpeedStep; // up/down change the slider by one step
    this.speedLimit = 0.5;        // scales forward and turn; set from the slider
    this.gamepadName = null;
    this._held = new Set();       // drive and lift/pan keys currently down
    this._auxHeld = new Set();    // on-screen lift/pan buttons currently pressed
    this._auxPointers = new Map(); // each hold belongs to the pointer that started it
    this._pad = null;             // { forward, turn } while the pad is dragged
    this._pointer = null;
    this._stopButtonWasDown = false;
    this._gamepadNeedsNeutral = false;
    // Bounded, control-only evidence carried by the existing drive log.
    // Diagnostics never feed the command or change input acceptance.
    this._debugPageId = globalThis.crypto?.randomUUID?.()
      || `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
    this._debugSequence = 0;
    this._debugEventSequence = 0;
    this._debugEvents = [];
    this._debugEventKey = null;
    this._debugEventAt = -Infinity;
    this._debugReadAt = null;
    this._debugReadDt = null;
    this._debugPan = { keyboard: 0, pointer: 0, gamepad: 0, gamepad_raw: 0, output: 0 };

    window.addEventListener('keydown', e => this._key(e, true));
    window.addEventListener('keyup', e => this._key(e, false));
    // A key released while another window has focus never sends keyup.
    window.addEventListener('blur', () => {
      this._inputEvent('reset', { reset_reason: 'blur' });
      this._held.clear();
    });

    pad.addEventListener('pointerdown', e => {
      if (this._pointer !== null || e.button !== 0) return;
      this._pointer = e.pointerId;
      pad.setPointerCapture(e.pointerId);
      this._drag(e);
    });
    pad.addEventListener('pointermove', e => { if (e.pointerId === this._pointer) this._drag(e); });
    for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
      pad.addEventListener(type, e => {
        if (e.pointerId === this._pointer) { this._pad = null; this._pointer = null; }
      });
    }
    pad.addEventListener('contextmenu', e => e.preventDefault());

    for (const button of this.auxButtons) {
      // Capture, so letting go outside the button still counts as letting go.
      button.addEventListener('pointerdown', e => {
        if (button.disabled || e.button !== 0 || this._auxPointers.has(button)) return;
        button.setPointerCapture(e.pointerId);
        this._auxPointers.set(button, e.pointerId);
        this._auxHeld.add(button);
        this._inputEvent('pointerdown', { pan: buttonPan(button), pointer_id: e.pointerId });
      });
      for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
        button.addEventListener(type, e => {
          if (this._auxPointers.get(button) !== e.pointerId) return;
          this._inputEvent(type, { pan: buttonPan(button), pointer_id: e.pointerId });
          this._auxPointers.delete(button);
          this._auxHeld.delete(button);
        });
      }
      // A long press on a touch screen opens a menu and swallows the release.
      button.addEventListener('contextmenu', e => e.preventDefault());
    }
  }

  reset(reason = 'reset') {
    this._inputEvent('reset', { reset_reason: reason });
    this._held.clear();
    this._auxHeld.clear();
    this._auxPointers.clear();
    this._pad = null;
    this._pointer = null;
    // A held stick is sampled again every tick; unlike a cleared keyboard key,
    // it must return to neutral before reconnect/focus/E-STOP can resume drive.
    this._gamepadNeedsNeutral = true;
  }

  _key(e, down) {
    const heldBefore = [...this._held];
    // Always release a held drive key, even if focus changed while it was down.
    if (!down) this._held.delete(e.code);
    const typing = e.target.matches('input:not([type="range"]), textarea, select, [contenteditable="true"]');
    if (typing) {
      if (!down && heldBefore.includes(e.code)) this._inputEvent('keyup',
        { code: e.code, repeat: Boolean(e.repeat), held_before: heldBefore, context: 'typing' });
      return;
    }
    if (e.code === 'Space') {
      e.preventDefault();                       // do not "click" a focused button
      this._inputEvent(down ? 'keydown' : 'keyup', { code: e.code, repeat: Boolean(e.repeat), held_before: heldBefore });
      if (down && !e.repeat) this.onEstop();
      return;
    }
    if (e.ctrlKey || e.altKey || e.metaKey) {   // Ctrl+R must still reload the page
      if (!down && heldBefore.includes(e.code)) this._inputEvent('keyup',
        { code: e.code, repeat: Boolean(e.repeat), held_before: heldBefore, context: 'modified' });
      return;
    }
    if (e.code === 'ArrowUp' || e.code === 'ArrowDown') {
      e.preventDefault();                       // neither scroll nor native slider repeat
      this._inputEvent(down ? 'keydown' : 'keyup', { code: e.code, repeat: Boolean(e.repeat), held_before: heldBefore });
      if (down && !e.repeat) this.onSpeedStep(e.code === 'ArrowUp' ? 1 : -1);
      return;
    }
    if (!(e.code in KEYS) && !(e.code in AUX_KEYS)) return;
    e.preventDefault();
    this._inputEvent(down ? 'keydown' : 'keyup', { code: e.code, repeat: Boolean(e.repeat), held_before: heldBefore });
    if (down) this._held.add(e.code); else this._held.delete(e.code);
  }

  _inputEvent(kind, fields = {}) {
    const now = performance.now();
    const event = { kind, ...fields, held_before: fields.held_before || [...this._held] };
    const key = JSON.stringify(event);
    // A stale link resets every pump and held keys repeat frequently. Keep
    // changes immediately, with at most one identical event per second.
    if (key === this._debugEventKey && now - this._debugEventAt < 1000) return;
    this._debugEventKey = key;
    this._debugEventAt = now;
    this._debugEvents.push({ seq: ++this._debugEventSequence, at_ms: now, ...event });
    if (this._debugEvents.length > 8) this._debugEvents.shift();
  }

  inputDebug({ telemetry_age_ms = null } = {}) {
    return { schema: 'pan_input_v1', page_id: this._debugPageId, seq: ++this._debugSequence,
      at_ms: performance.now(), pump_dt_ms: this._debugReadDt, sampled_at_ms: this._debugReadAt,
      telemetry_age_ms: Number.isFinite(telemetry_age_ms) ? telemetry_age_ms : null,
      focused: document.hasFocus?.() ?? null, hidden: Boolean(document.hidden),
      held: [...this._held], pan: { ...this._debugPan },
      events: this._debugEvents.map(event => ({ ...event, held_before: [...event.held_before] })),
    };
  }

  _drag(e) {
    const box = this.pad.getBoundingClientRect();
    this._pad = {
      turn: clamp((e.clientX - box.left) / box.width * 2 - 1, -1, 1),
      forward: clamp(1 - (e.clientY - box.top) / box.height * 2, -1, 1),
    };
  }

  // The command to send now. Called 20 times a second.
  read() {
    const debugNow = performance.now();
    this._debugReadDt = this._debugReadAt === null ? null : debugNow - this._debugReadAt;
    this._debugReadAt = debugNow;
    let forward = 0, turn = 0, lift = 0, pan = 0;
    for (const code of this._held) {
      if (code in KEYS) { forward += KEYS[code][0]; turn += KEYS[code][1]; }
      else { lift += AUX_KEYS[code][0]; pan += AUX_KEYS[code][1]; }
    }
    const keyboardPan = pan;
    if (this._pad) { forward += this._pad.forward; turn += this._pad.turn; }
    for (const button of this._auxHeld) {
      // A button that was switched off while held never sees the release.
      if (button.disabled) {
        this._inputEvent('pointer_disabled', { pan: buttonPan(button) });
        this._auxHeld.delete(button); this._auxPointers.delete(button); continue;
      }
      lift += buttonLift(button);
      pan += buttonPan(button);
    }
    const pointerPan = pan - keyboardPan;
    let gamepadPan = 0, gamepadRawPan = 0;

    // LAN HTTP and some mobile browsers omit or block the gamepad API.
    // Touch/keyboard input must keep working independently of it.
    let gamepads = [];
    try { gamepads = navigator.getGamepads?.() || []; } catch (_) { /* unavailable */ }
    const gamepad = [...gamepads].find(g => g && g.connected);
    this.gamepadName = gamepad ? gamepad.id : null;
    if (gamepad) {
      const pressed = i => (gamepad.buttons[i]?.pressed ? 1 : 0);
      const padForward = stick(-(gamepad.axes[1] ?? 0)); // left stick, up is negative
      const padTurn = stick(gamepad.axes[2] ?? 0); // right stick, left/right
      const padPan = pressed(15) - pressed(14);   // D-pad right / left
      gamepadRawPan = padPan;
      if (!padForward && !padTurn && !padPan) this._gamepadNeedsNeutral = false;
      if (!this._gamepadNeedsNeutral) {
        forward += padForward;
        turn += padTurn;
        pan += padPan;
        gamepadPan = padPan;
      }
      const stopDown = pressed(1) === 1;        // B on an Xbox pad
      if (stopDown && !this._stopButtonWasDown) this.onEstop();
      this._stopButtonWasDown = stopDown;
    } else {
      this._gamepadNeedsNeutral = true;
      this._stopButtonWasDown = false;
    }

    forward = clamp(forward, -1, 1) * this.speedLimit;
    turn = clamp(turn, -1, 1) * this.speedLimit;
    // Lift and pan are not scaled by the speed limit: their full speed is
    // set per servo in rover/config.py.
    this._debugPan = { keyboard: keyboardPan, pointer: pointerPan,
      gamepad: gamepadPan, gamepad_raw: gamepadRawPan, output: clamp(pan, -1, 1) };
    return { forward, turn, lift: clamp(lift, -1, 1), pan: clamp(pan, -1, 1) };
  }

  // Put the pad's dot where the command the rover is acting on is.
  show(forward, turn) {
    this.dot.style.left = (50 + 50 * clamp(turn, -1, 1)) + '%';
    this.dot.style.top = (50 - 50 * clamp(forward, -1, 1)) + '%';
  }

  // Light the lift/pan buttons the rover is acting on.
  showAux(lift, pan) {
    for (const button of this.auxButtons) {
      button.classList.toggle('on', buttonLift(button) * lift > 0 || buttonPan(button) * pan > 0);
    }
  }
}
