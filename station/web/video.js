// The /video channel: the camera's H.264, decoded by the browser and drawn
// on a canvas.
//
// Each WebSocket message is one whole compressed frame. Its first byte says
// whether it is a keyframe (1) or builds on earlier frames (0); the rest is
// the frame itself. The rover does the cutting (rover/camera.py), so this
// side never has to search the stream for frame boundaries.
//
// The decoder is WebCodecs' VideoDecoder, usually the laptop's hardware
// decoder. Browsers only provide it on https or localhost -- the reason the
// station page is served from the laptop (station.py).

import { hostPort } from './link.js';
import { STATION_LINK_WATCHDOG } from './link_policy.js';

// If the decoder falls this many frames behind, stop feeding it and jump to
// the next keyframe (at most 1 s away). Showing every frame late is worse
// than skipping some: the operator is steering by this picture.
const MAX_QUEUE = 15;

const hex = b => b.toString(16).padStart(2, '0');

// The codec name the decoder needs ("avc1.4d4029") comes from three bytes of
// the stream's settings block (SPS, NAL type 7), which travels with every
// keyframe.
function codecFromKeyframe(frame) {
  for (let i = 0; i + 6 < frame.length; i++) {
    if (frame[i] === 0 && frame[i + 1] === 0 && frame[i + 2] === 1 && (frame[i + 3] & 0x1f) === 7) {
      return 'avc1.' + hex(frame[i + 4]) + hex(frame[i + 5]) + hex(frame[i + 6]);
    }
  }
  return null;
}

export class VideoView {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.supported = typeof VideoDecoder !== 'undefined';
    this.address = null;
    this.socket = null;
    this.decoder = null;
    this.needKey = true;
    this._generation = 0;
    this._retryTimer = null;

    // Refreshed once a second, for the page to show.
    this.fps = 0;
    this.mbps = 0;
    this.skipped = 0;
    this.lastFrameAt = 0;       // performance.now() of the newest picture drawn
    this._frames = 0;
    this._bytes = 0;
    setInterval(() => {
      this.fps = this._frames;
      this.mbps = this._bytes * 8 / 1e6;
      this._frames = this._bytes = 0;
    }, 1000);
  }

  connect(address) {
    this.address = address;
    const generation = ++this._generation;
    this._clearRetry();
    this._retireSocket();
    if (this.supported) this._open(generation);
  }

  disconnect() {
    this.address = null;
    ++this._generation;
    this._clearRetry();
    this._retireSocket();
  }

  _newDecoder() {
    if (this.decoder && this.decoder.state !== 'closed') this.decoder.close();
    this.needKey = true;
    const generation = this._generation, socket = this.socket;
    const current = () => generation === this._generation && socket === this.socket && decoder === this.decoder;
    const decoder = new VideoDecoder({
      output: picture => {
        if (!current()) { picture.close(); return; }
        if (this.canvas.width !== picture.displayWidth || this.canvas.height !== picture.displayHeight) {
          this.canvas.width = picture.displayWidth;
          this.canvas.height = picture.displayHeight;
        }
        this.ctx.drawImage(picture, 0, 0);
        picture.close();
        this._frames++;
        this.lastFrameAt = performance.now();
      },
      // A damaged frame closes the decoder. Start over at the next keyframe.
      error: () => { if (current()) this._newDecoder(); },
    });
    this.decoder = decoder;
  }

  _open(generation) {
    if (generation !== this._generation || this.address === null || this.socket !== null) return;
    let socket;
    try { socket = new WebSocket('ws://' + hostPort(this.address) + '/video'); }
    catch (error) { this._retry(generation); return; }
    socket.binaryType = 'arraybuffer';
    this.socket = socket;
    this._newDecoder();

    socket.onmessage = event => {
      if (generation !== this._generation || socket !== this.socket) return;
      const message = new Uint8Array(event.data);
      this._bytes += message.length;
      this._decode(message[0] === 1, message.subarray(1));
    };

    socket.onclose = () => this._failed(socket, generation);
    socket.onerror = () => this._failed(socket, generation);
  }

  _clearRetry() {
    clearTimeout(this._retryTimer);
    this._retryTimer = null;
  }

  _retireSocket() {
    const socket = this.socket;
    this.socket = null;
    if (socket && socket.readyState !== 3) {
      try { socket.close(); } catch (error) { /* retry does not depend on the close handshake */ }
    }
  }

  _failed(socket, generation) {
    if (generation !== this._generation || socket !== this.socket) return;
    this._retireSocket();
    this._retry(generation);
  }

  _retry(generation) {
    if (generation !== this._generation || this.address === null || this._retryTimer !== null) return;
    const timer = setTimeout(() => {
      if (generation !== this._generation || this.address === null || this._retryTimer !== timer) return;
      this._retryTimer = null;
      this._open(generation);
    }, STATION_LINK_WATCHDOG.retry_delay_ms);
    this._retryTimer = timer;
  }

  _decode(isKey, frame) {
    const decoder = this.decoder;
    if (this.needKey && !isKey) return;         // cannot start in the middle
    if (isKey && decoder.state === 'unconfigured') {
      const codec = codecFromKeyframe(frame);
      if (!codec) return;
      decoder.configure({ codec, optimizeForLatency: true });
    }
    if (decoder.decodeQueueSize > MAX_QUEUE) {
      this.skipped++;
      this.needKey = true;
      return;
    }
    this.needKey = false;
    decoder.decode(new EncodedVideoChunk({
      type: isKey ? 'key' : 'delta',
      timestamp: performance.now() * 1000,
      data: frame,
    }));
  }
}
