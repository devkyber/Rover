// The /control channel to the rover: telemetry in, commands out.
//
//   const link = new RoverLink({ onHello, onTelemetry, onStatus });
//   link.connect('192.168.50.243');
//   link.send({ type: 'drive', forward: 0.5, turn: 0 });
//
// It reconnects by itself for as long as connect() is in force. Wi-Fi drops
// are normal on the field; the operator should not have to click anything
// to get the rover back.

export const DEFAULT_PORT = 8000;

// "192.168.50.243" -> "192.168.50.243:8000". A port typed by hand is kept.
export function hostPort(address) {
  const a = address.trim().replace(/^\w+:\/\//, '').replace(/\/.*$/, '');
  return /:\d+$/.test(a) ? a : a + ':' + DEFAULT_PORT;
}

export class RoverLink {
  constructor({ onHello, onTelemetry, onStatus }) {
    this.onHello = onHello;
    this.onTelemetry = onTelemetry;
    this.onStatus = onStatus;       // 'disconnected' | 'connecting' | 'connected'
    this.address = null;            // null = the operator wants no connection
    this.socket = null;
    this.rttMs = null;              // network round trip, from ping/pong
    this._pingId = 0;
    this._pingSent = new Map();
    setInterval(() => this._ping(), 1000);
  }

  get connected() { return this.socket !== null && this.socket.readyState === WebSocket.OPEN; }

  connect(address) {
    this.address = address;
    this._open();
  }

  disconnect() {
    this.address = null;
    if (this.socket) this.socket.close();
    this.socket = null;
    this.rttMs = null;
    this.onStatus('disconnected');
  }

  send(message) {
    if (this.connected) this.socket.send(JSON.stringify(message));
  }

  _open() {
    if (this.address === null) return;
    this.onStatus('connecting');
    const socket = new WebSocket('ws://' + hostPort(this.address) + '/control');
    this.socket = socket;

    socket.onopen = () => { if (socket === this.socket) this.onStatus('connected'); };

    socket.onmessage = event => {
      const msg = JSON.parse(event.data);
      if (msg.type === 'telemetry') this.onTelemetry(msg);
      else if (msg.type === 'hello') this.onHello(msg);
      else if (msg.type === 'pong') {
        const sent = this._pingSent.get(msg.id);
        if (sent !== undefined) this.rttMs = performance.now() - sent;
        this._pingSent.clear();
      }
    };

    socket.onclose = () => {
      if (socket !== this.socket) return;     // an older socket; a newer one is in charge
      this.socket = null;
      this.rttMs = null;
      if (this.address === null) return;
      this.onStatus('connecting');
      setTimeout(() => this._open(), 1000);
    };
  }

  _ping() {
    if (!this.connected) return;
    const id = ++this._pingId;
    this._pingSent.set(id, performance.now());
    this.send({ type: 'ping', id });
  }
}
