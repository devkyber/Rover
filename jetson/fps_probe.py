#!/usr/bin/env python3
"""버퍼 크기와 디코딩 비용을 분리해 fps 병목을 찾는다."""
import time, cv2

def run(label, bufsize, decode, n=60, w=1280, h=720):
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("%-34s 열기 실패" % label); return
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if bufsize is not None:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, bufsize)
    for _ in range(10):
        cap.grab()
    t0, got = time.time(), 0
    for _ in range(n):
        if decode:
            if cap.read()[0]: got += 1
        else:
            if cap.grab(): got += 1
    dt = time.time() - t0
    cap.release()
    print("%-34s %5.1f fps  (%d/%d)" % (label, got / dt, got, n))

run("buffersize=1  + read(디코딩)",   1,    True)
run("buffersize=4  + read(디코딩)",   4,    True)
run("buffersize=기본 + read(디코딩)", None, True)
run("buffersize=기본 + grab(디코딩X)",None, False)
run("640x480 기본  + read",           None, True, 60, 640, 480)
run("1920x1080 기본 + read",          None, True, 60, 1920, 1080)
