"""
The operator station. Run it on the laptop:

    python station/station.py --rover 192.168.0.200

It serves the page in station/web/ at http://localhost:8765 and opens it in
the browser. The page connects straight to the rover for telemetry, commands
and video. Optional --clegg receives Nano acceleration even with Jetson off.

WHY THE PAGE IS SERVED FROM THE LAPTOP AND NOT FROM THE ROVER
-------------------------------------------------------------
The page decodes the camera's H.264 with the browser's built-in decoder
(WebCodecs). Browsers only enable that on https or on localhost. A page
loaded from http://<rover address> would have telemetry but no video.

The web station uses the standard library. Scout plots are opened manually in
the dashboard's Plot viewer or with the offline analysis tools.
Use Chrome or Edge for video. LAN HTTP also serves touch control on phones;
use ?view=control for controls only and ?view=monitor for a passive dashboard.
"""
import argparse
import functools
import http.server
import json
import os
from pathlib import Path
import sys
import urllib.parse
import webbrowser

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
ROOT = Path(__file__).resolve().parents[1]


class Handler(http.server.SimpleHTTPRequestHandler):
    def _json(self, status, value):
        raw = json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        parsed = urllib.parse.urlsplit(self.path)
        if not parsed.path.startswith('/clegg/'):
            return super().do_GET()
        service = getattr(self.server, 'clegg_service', None)
        if parsed.path == '/clegg/status':
            return self._json(200, service.status() if service else dict(
                available=False, status='disabled', connected=False, sample_count=0,
                error='Start station with --clegg or --clegg-host SENSOR_IP'))
        if service is None:
            return self._json(503, dict(error='Clegg reception is disabled'))
        if parsed.path == '/clegg/impact':
            impact_id = urllib.parse.parse_qs(parsed.query).get('id', [None])[0]
            value = service.impact(impact_id)
            return self._json(200, value) if value else self._json(404, dict(error='Impact candidate not found'))
        return self._json(404, dict(error='Unknown Clegg endpoint'))

    def do_POST(self):
        parsed = urllib.parse.urlsplit(self.path)
        if parsed.path not in ('/clegg/arm', '/clegg/mark', '/clegg/release'):
            return self._json(404, dict(error='Unknown Clegg endpoint'))
        # JSON plus a matching Origin prevents another website controlling this
        # local receiver. These controls only arm/capture acceleration records.
        origin = self.headers.get('Origin')
        if origin:
            origin_url = urllib.parse.urlsplit(origin)
            if origin_url.scheme != 'http' or origin_url.netloc.lower() != self.headers.get('Host', '').lower():
                return self._json(403, dict(error='Same-origin requests are required'))
        if self.headers.get('Sec-Fetch-Site') == 'cross-site':
            return self._json(403, dict(error='Cross-site requests are refused'))
        if self.headers.get_content_type() != 'application/json':
            return self._json(415, dict(error='Content-Type must be application/json'))
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 4096:
                raise ValueError('JSON body must be between 1 and 4096 bytes')
            self.connection.settimeout(2.)
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError('JSON body must be an object')
            allowed = ({'threshold_g'} if parsed.path == '/clegg/arm' else
                       {'request_id', 'hammer_seq', 'browser_time_ms'} if parsed.path == '/clegg/release' else set())
            if set(body)-allowed:
                raise ValueError('Unknown request fields')
        except (ValueError, UnicodeError, OSError) as exc:
            return self._json(400, dict(error=str(exc)))
        service = getattr(self.server, 'clegg_service', None)
        if service is None:
            return self._json(503, dict(error='Clegg reception is disabled'))
        try:
            if parsed.path == '/clegg/release':
                value = service.release_request(body.get('request_id'), body.get('hammer_seq'), body.get('browser_time_ms'))
            else:
                value = service.arm(body.get('threshold_g')) if parsed.path == '/clegg/arm' else service.mark()
            return self._json(200, value)
        except ValueError as exc:
            return self._json(409, dict(error=str(exc)))


    def end_headers(self):
        # Always load the files as they are on disk now. Without this the
        # browser keeps showing yesterday's page after an edit.
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Serve the rover station page on this laptop.")
    ap.add_argument("--rover", default="",
                    help="rover address, e.g. 192.168.0.200 (can also be typed in the page)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--bind", default="127.0.0.1",
                    help="listen address; use the Jetson Wi-Fi IP to serve phones on that LAN")
    ap.add_argument("--no-browser", action="store_true", help="do not open the browser")
    ap.add_argument("--clegg", action="store_true", help="discover and record wireless Nano acceleration")
    ap.add_argument("--clegg-host", help="Nano Wi-Fi IP; enables reception without discovery")
    ap.add_argument("--clegg-context-host", help="optional Jetson read-only pose endpoint; requires tools/serve_clegg_context.py there")
    ap.add_argument("--clegg-context-port", type=int, help="read-only Jetson context port (default from rover/config.py)")
    args = ap.parse_args()

    page_host = "localhost" if args.bind == "127.0.0.1" else args.bind
    url = f"http://{page_host}:{args.port}/"
    if args.rover:
        url += "?rover=" + urllib.parse.quote(args.rover)

    # Local by default. LAN serving is explicitly selected for a phone controller.
    server = http.server.ThreadingHTTPServer(
        (args.bind, args.port), functools.partial(Handler, directory=WEB_DIR))
    server.clegg_service = None
    if args.clegg or args.clegg_host:
        sys.path.insert(0, str(ROOT / 'rover'))
        from clegg_service import CleggService
        server.clegg_service = CleggService(host=args.clegg_host, context_host=args.clegg_context_host,
                                          context_port=args.clegg_context_port).start()
        print('Clegg wireless receiver: ' + (args.clegg_host or 'automatic discovery'))
    print(f"Station page: {url}    (Ctrl+C to stop)")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if server.clegg_service:
            server.clegg_service.stop()
