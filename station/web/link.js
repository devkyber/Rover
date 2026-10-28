// The /control channel to the rover: telemetry in, commands out.
//
//   const link = new RoverLink({ onHello, onTelemetry, onStatus });
//   link.connect('192.168.50.243');
//   link.send({ type: 'drive', forward: 0.5, turn: 0 });
//
// It reconnects by itself for as long as connect() is in force. Wi-Fi drops
// are normal on the field; the operator should not have to click anything
// to get the rover back.

import { STATION_LINK_WATCHDOG } from './link_policy.js';

export const DEFAULT_PORT = 8000;

// "192.168.50.243" -> "192.168.50.243:8000". A port typed by hand is kept.
export function hostPort(address) {
  const a = address.trim().replace(/^\w+:\/\//, '').replace(/\/.*$/, '');
  return /:\d+$/.test(a) ? a : a + ':' + DEFAULT_PORT;
}

export class RoverLink {
  constructor({ onHello, onTelemetry, onStatus, onJumperResult = () => {},
                onMappingResult = () => {}, channel = 'control',
                policy = STATION_LINK_WATCHDOG }) {
    this.onHello = onHello;
    this.onTelemetry = onTelemetry;
    this.onStatus = onStatus;       // 'disconnected' | 'connecting' | 'connected'
    this.onJumperResult = onJumperResult;
    this.onMappingResult = onMappingResult;
    this.channel = channel;
    this.policy = { ...STATION_LINK_WATCHDOG, ...policy };
    this.address = null;            // null = the operator wants no connection
    this.socket = null;
    this.rttMs = null;              // network round trip, from ping/pong
    this._pingId = 0;
    this._pingSent = new Map();
    this._generation = 0;
    this._pingTimer = null;
    this._connectTimer = null;
    this._retryTimer = null;
    this._lastPongAt = null;
  }

  get connected() { return this.socket !== null && this.socket.readyState === WebSocket.OPEN; }

  connect(address) {
    this.address = address;
    const generation = ++this._generation;
    this._clearTimers();
    this._retireSocket();
    this._open(generation);
  }

  disconnect() {
    this.address = null;
    ++this._generation;
    this._clearTimers();
    this._retireSocket();
    this.onStatus('disconnected');
  }

  send(message) {
    if (!this.connected) return false;
    let payload;
    try { payload = JSON.stringify(message); }
    catch (error) { return false; }
    if (typeof payload !== 'string') return false;
    const socket = this.socket, generation = this._generation;
    try { socket.send(payload); return true; }
    catch (error) { this._failed(socket, generation); return false; }
  }

  _open(generation) {
    if (generation !== this._generation || this.address === null || this.socket !== null) return;
    this.onStatus('connecting');
    let socket;
    try { socket = new WebSocket('ws://' + hostPort(this.address) + '/' + this.channel); }
    catch (error) { this._retry(generation); return; }
    this.socket = socket;
    const current = () => generation === this._generation && socket === this.socket;
    this._connectTimer = setTimeout(() => this._failed(socket, generation), this.policy.connect_timeout_ms);

    socket.onopen = () => {
      if (!current()) return;
      clearTimeout(this._connectTimer);
      this._connectTimer = null;
      this._lastPongAt = performance.now();
      this._pingTimer = setInterval(() => {
        if (current()) this._ping();
      }, this.policy.ping_interval_ms);
      this.onStatus('connected');
    };

    socket.onmessage = event => {
      if (!current()) return;
      let msg;
      try { msg = JSON.parse(event.data); }
      catch (error) { return; }
      if (msg === null || typeof msg !== 'object') return;
      if (msg.type === 'telemetry') this.onTelemetry(msg);
      else if (msg.type === 'hello') this.onHello(msg);
      else if (msg.type === 'jumper_result') this.onJumperResult(msg);
      else if (msg.type === 'mapping_result') this.onMappingResult(msg);
      else if (msg.type === 'pong') {
        const sent = this._pingSent.get(msg.id);
        if (sent === undefined) return;     // unmatched/old pong cannot keep a dead path alive
        this._lastPongAt = performance.now();
        this.rttMs = this._lastPongAt - sent;
        this._pingSent.delete(msg.id);
      }
    };

    socket.onclose = () => this._failed(socket, generation);
    socket.onerror = () => this._failed(socket, generation);
  }

  _clearTimers() {
    clearInterval(this._pingTimer);
    clearTimeout(this._connectTimer);
    clearTimeout(this._retryTimer);
    this._pingTimer = this._connectTimer = this._retryTimer = null;
  }

  _retireSocket() {
    const socket = this.socket;
    this.socket = null;             // close callbacks from this attempt are already stale
    this.rttMs = this._lastPongAt = null;
    this._pingSent.clear();
    if (socket && socket.readyState !== 3) {
      try { socket.close(); } catch (error) { /* a failed close must not prevent retry */ }
    }
  }

  _failed(socket, generation) {
    if (generation !== this._generation || socket !== this.socket) return;
    this._clearTimers();
    this._retireSocket();
    if (this.address === null) return;
    this.onStatus('connecting');     // also clears held operator input before a new socket opens
    this._retry(generation);
  }

  _retry(generation) {
    if (generation !== this._generation || this.address === null || this._retryTimer !== null) return;
    const timer = setTimeout(() => {
      if (generation !== this._generation || this.address === null || this._retryTimer !== timer) return;
      this._retryTimer = null;
      this._open(generation);
    }, this.policy.retry_delay_ms);
    this._retryTimer = timer;
  }

  _ping() {
    if (!this.connected) return;
    const now = performance.now();
    // OPEN and incoming telemetry do not prove that outgoing packets still pass.
    if (now - this._lastPongAt >= this.policy.pong_timeout_ms) {
      this._failed(this.socket, this._generation);
      return;
    }
    const id = ++this._pingId;
    this._pingSent.set(id, now);
    while (this._pingSent.size > this.policy.max_pending_pings)
      this._pingSent.delete(this._pingSent.keys().next().value);
    this.send({ type: 'ping', id });
  }
}
