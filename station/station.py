"""
The operator station. Run it on the laptop:

    python station/station.py --rover 192.168.0.200

It serves the page in station/web/ at http://localhost:8765 and opens it in
the browser. Nothing else: the page itself connects straight to the rover
(rover/rover_main.py) for telemetry, commands and video.

WHY THE PAGE IS SERVED FROM THE LAPTOP AND NOT FROM THE ROVER
-------------------------------------------------------------
The page decodes the camera's H.264 with the browser's built-in decoder
(WebCodecs). Browsers only enable that on https or on localhost. A page
loaded from http://<rover address> would have telemetry but no video.

Standard library only -- nothing to install on the station laptop.
Use Chrome or Edge.
"""
import argparse
import functools
import http.server
import os
import urllib.parse
import webbrowser

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


class Handler(http.server.SimpleHTTPRequestHandler):
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
    ap.add_argument("--no-browser", action="store_true", help="do not open the browser")
    args = ap.parse_args()

    url = f"http://localhost:{args.port}/"
    if args.rover:
        url += "?rover=" + urllib.parse.quote(args.rover)

    # 127.0.0.1 only: the page is for this laptop, not for the network.
    server = http.server.ThreadingHTTPServer(
        ("127.0.0.1", args.port), functools.partial(Handler, directory=WEB_DIR))
    print(f"Station page: {url}    (Ctrl+C to stop)")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
