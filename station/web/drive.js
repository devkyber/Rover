// Operator input -> one command, { forward, turn, lift, pan }, all -1..+1.
//
// Three sources, added together and clipped: the keyboard, the on-screen
// controls (mouse or touch), and a gamepad. forward + is ahead; turn + is to
// the right, the same convention as rover/dxl_control.drive(). lift + winds
// the string in; pan + turns the camera to the right (rover/config.py SERVOS).
//
// Safety comes from the rover, not from here: it stops when drive messages
// stop arriving. This file only has to make sure a released key or a
// window that lost focus really reads as zero.

import { clamp } from './util.js';

const KEYS = {
  KeyW: [1, 0], ArrowUp: [1, 0],
  KeyS: [-1, 0], ArrowDown: [-1, 0],
  KeyA: [0, -1], ArrowLeft: [0, -1],
  KeyD: [0, 1], ArrowRight: [0, 1],
};

// Lift and camera pan are hold-to-run: the servo turns for as long as the
// key or button is down. [lift, pan]
const AUX_KEYS = {
  KeyR: [1, 0], KeyF: [-1, 0],
  KeyQ: [0, -1], KeyE: [0, 1],
};

// Sticks never rest at exactly zero. Ignore the first 12% and rescale the
// rest so the command still starts from 0 at the edge of the dead zone.
const DEAD_ZONE = 0.12;
const stick = v => Math.abs(v) < DEAD_ZONE ? 0 : Math.sign(v) * (Math.abs(v) - DEAD_ZONE) / (1 - DEAD_ZONE);

// What an on-screen lift/pan button asks for, from its data-lift / data-pan.
const buttonLift = button => Number(button.dataset.lift || 0);
const buttonPan = button => Number(button.dataset.pan || 0);

export class DriveInput {
  constructor({ pad, dot, auxButtons = [], onEstop }) {
    this.pad = pad;
    this.dot = dot;
    this.auxButtons = [...auxButtons];
    this.onEstop = onEstop;       // called when the operator asks for a stop
    this.speedLimit = 0.5;        // scales forward and turn; set from the slider
    this.gamepadName = null;
    this._held = new Set();       // drive and lift/pan keys currently down
    this._auxHeld = new Set();    // on-screen lift/pan buttons currently pressed
    this._pad = null;             // { forward, turn } while the pad is dragged
    this._stopButtonWasDown = false;

    window.addEventListener('keydown', e => this._key(e, true));
    window.addEventListener('keyup', e => this._key(e, false));
    // A key released while another window has focus never sends keyup.
    window.addEventListener('blur', () => { this._held.clear(); this._auxHeld.clear(); this._pad = null; });

    pad.addEventListener('pointerdown', e => { pad.setPointerCapture(e.pointerId); this._drag(e); });
    pad.addEventListener('pointermove', e => { if (this._pad) this._drag(e); });
    for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
      pad.addEventListener(type, () => { this._pad = null; });
    }

    for (const button of this.auxButtons) {
      // Capture, so letting go outside the button still counts as letting go.
      button.addEventListener('pointerdown', e => { button.setPointerCapture(e.pointerId); this._auxHeld.add(button); });
      for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
        button.addEventListener(type, () => { this._auxHeld.delete(button); });
      }
      // A long press on a touch screen opens a menu and swallows the release.
      button.addEventListener('contextmenu', e => e.preventDefault());
    }
  }

  _key(e, down) {
    const typing = e.target instanceof HTMLInputElement && e.target.type === 'text';
    if (typing) return;
    if (e.code === 'Space') {
      e.preventDefault();                       // do not "click" a focused button
      if (down && !e.repeat) this.onEstop();
      return;
    }
    if (!(e.code in KEYS) && !(e.code in AUX_KEYS)) return;
    if (e.ctrlKey || e.altKey || e.metaKey) return;   // Ctrl+R must still reload the page
    e.preventDefault();                         // arrow keys must not scroll the page
    if (down) this._held.add(e.code); else this._held.delete(e.code);
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
    let forward = 0, turn = 0, lift = 0, pan = 0;
    for (const code of this._held) {
      if (code in KEYS) { forward += KEYS[code][0]; turn += KEYS[code][1]; }
      else { lift += AUX_KEYS[code][0]; pan += AUX_KEYS[code][1]; }
    }
    if (this._pad) { forward += this._pad.forward; turn += this._pad.turn; }
    for (const button of this._auxHeld) {
      // A button that was switched off while held never sees the release.
      if (button.disabled) { this._auxHeld.delete(button); continue; }
      lift += buttonLift(button);
      pan += buttonPan(button);
    }

    const gamepad = [...navigator.getGamepads()].find(g => g && g.connected);
    this.gamepadName = gamepad ? gamepad.id : null;
    if (gamepad) {
      const pressed = i => (gamepad.buttons[i]?.pressed ? 1 : 0);
      forward += stick(-gamepad.axes[1]);       // left stick, up is negative
      turn += stick(gamepad.axes[2] ?? 0);      // right stick, left/right
      lift += pressed(12) - pressed(13);        // D-pad up / down
      pan += pressed(15) - pressed(14);         // D-pad right / left
      const stopDown = pressed(1) === 1;        // B on an Xbox pad
      if (stopDown && !this._stopButtonWasDown) this.onEstop();
      this._stopButtonWasDown = stopDown;
    }

    forward = clamp(forward, -1, 1) * this.speedLimit;
    turn = clamp(turn, -1, 1) * this.speedLimit;
    // Lift and pan are not scaled by the speed limit: their full speed is
    // set per servo in rover/config.py.
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
