#!/usr/bin/env python3
"""웹캠 MJPEG 스트리밍 서버 — PC 브라우저에서 젯슨 카메라 화면 보기.

젯슨에서 실행:  python3 cam_stream.py
PC 브라우저:    http://192.168.55.1:8080
"""
import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2

PAGE = b"""<!doctype html><meta charset=utf-8><title>Rover Cam</title>
<style>body{margin:0;background:#111;display:grid;place-items:center;height:100vh}
img{max-width:100%;max-height:100vh}</style><img src="/stream.mjpg">"""

_lock = threading.Lock()
_latest = [None]


def grabber(cap):
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with _lock:
                _latest[0] = buf.tobytes()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with _lock:
                        jpg = _latest[0]
                    if jpg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    time.sleep(1 / 30.0)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("-d", "--device", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--port", type=int, default=8080)
    args = p.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit("카메라 열기 실패: /dev/video{}".format(args.device))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    threading.Thread(target=grabber, args=(cap,), daemon=True).start()
    print("http://<젯슨IP>:{} 로 접속".format(args.port))
    HTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
