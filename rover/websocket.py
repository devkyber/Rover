"""
The server half of the WebSocket protocol (RFC 6455), standard library only.

    ws = websocket.accept(sock)     # answer the browser's handshake
    ws.path                         # "/control" or "/video"
    ws.send("text") / ws.send(b"bytes")
    msg = ws.recv()                 # str, bytes, or None when the peer closed

WHY THIS IS HERE AND NOT A LIBRARY
----------------------------------
A browser page cannot open a plain TCP or UDP socket; WebSocket is the only
two-way channel it has. The rover needs exactly the three calls above, and
on the field there is no internet to `pip install` anything, so the rover
program depends on numpy and pyserial and nothing else.

What a WebSocket is, in three lines: the browser sends an ordinary HTTP
request asking to "upgrade"; the server answers with a hash of the key the
browser sent; after that both sides exchange FRAMES -- a 2-14 byte header
giving the kind and length, then the payload.
"""
import base64
import hashlib
import struct
import threading

# Fixed string from the RFC. Hashing it with the browser's key proves the
# server really speaks WebSocket and is not some other service on the port.
_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

_TEXT, _BINARY, _CLOSE, _PING, _PONG = 0x1, 0x2, 0x8, 0x9, 0xA

# Largest message accepted FROM a client. Station commands are ~60 bytes;
# anything near this size is a bug or not our page.
MAX_INCOMING = 64 * 1024

_NOT_A_WEBSOCKET = (
    b"HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=utf-8\r\n"
    b"Connection: close\r\n\r\n"
    b"This is the rover's station link, not a web page.\n"
    b"On the station laptop run:  python station/station.py\n")


class WebSocket:
    """One accepted connection. send() may be called from any thread."""

    def __init__(self, sock, reader, path):
        self.sock = sock
        self.path = path
        self._reader = reader            # buffered file over the same socket
        self._send_lock = threading.Lock()

    def send(self, payload):
        """Send one message: str goes as text, bytes as binary."""
        if isinstance(payload, str):
            opcode, payload = _TEXT, payload.encode("utf-8")
        else:
            opcode = _BINARY
        self._send_frame(opcode, payload)

    def _send_frame(self, opcode, payload):
        n = len(payload)
        first = 0x80 | opcode            # 0x80 = "this frame is the whole message"
        if n < 126:
            header = struct.pack("!BB", first, n)
        elif n < 65536:
            header = struct.pack("!BBH", first, 126, n)
        else:
            header = struct.pack("!BBQ", first, 127, n)
        # The lock keeps two threads' frames from interleaving mid-frame.
        with self._send_lock:
            self.sock.sendall(header + payload)

    def recv(self):
        """
        Block for the next message. Returns str, bytes, or None once the
        peer has closed. Raises socket.timeout if the socket has a timeout
        and nothing arrived in time.
        """
        parts, kind = [], None
        while True:
            head = self._reader.read(2)
            if len(head) < 2:
                return None
            final, opcode = head[0] & 0x80, head[0] & 0x0F
            masked, n = head[1] & 0x80, head[1] & 0x7F
            if n == 126:
                n = struct.unpack("!H", self._reader.read(2))[0]
            elif n == 127:
                n = struct.unpack("!Q", self._reader.read(8))[0]
            if n > MAX_INCOMING or not masked:
                return None              # the RFC requires clients to mask
            mask = self._reader.read(4)
            data = self._reader.read(n)
            if len(data) < n:
                return None
            # Browsers XOR every payload with a random 4-byte mask.
            data = bytes(b ^ mask[i & 3] for i, b in enumerate(data))

            if opcode == _CLOSE:
                return None
            if opcode == _PING:
                self._send_frame(_PONG, data)
                continue
            if opcode == _PONG:
                continue
            if opcode in (_TEXT, _BINARY):
                kind = opcode
            parts.append(data)           # opcode 0 = continuation of a split message
            if final:
                whole = b"".join(parts)
                return whole.decode("utf-8", "replace") if kind == _TEXT else whole

    def close(self):
        try:
            self._send_frame(_CLOSE, b"")
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass


def accept(sock):
    """
    Read the browser's HTTP upgrade request from a freshly accepted socket
    and answer it. Returns a WebSocket, or None if the request was not one.
    """
    reader = sock.makefile("rb")
    request_line = reader.readline().decode("latin-1").split()
    headers = {}
    while True:
        line = reader.readline().decode("latin-1").strip()
        if not line:
            break
        name, _, value = line.partition(":")
        headers[name.strip().lower()] = value.strip()

    key = headers.get("sec-websocket-key")
    if len(request_line) < 2 or not key:
        # Someone typed the rover's address into a browser. Say what this is.
        try:
            sock.sendall(_NOT_A_WEBSOCKET)
        except OSError:
            pass
        return None

    answer = base64.b64encode(
        hashlib.sha1((key + _GUID).encode("ascii")).digest()).decode("ascii")
    sock.sendall(("HTTP/1.1 101 Switching Protocols\r\n"
                  "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                  f"Sec-WebSocket-Accept: {answer}\r\n\r\n").encode("ascii"))
    return WebSocket(sock, reader, request_line[1].split("?")[0])
