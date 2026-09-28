#!/usr/bin/env python3
"""노출값을 바꿔가며 실측 fps를 측정해 병목이 노출인지 확인한다."""
import time, cv2

def measure(auto, exposure, n=40, w=1280, h=720):
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, auto)
    if exposure is not None:
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    for _ in range(10):          # 설정이 반영될 때까지 버림
        cap.read()
    t0, got = time.time(), 0
    for _ in range(n):
        if cap.read()[0]:
            got += 1
    dt = time.time() - t0
    rd = cap.get(cv2.CAP_PROP_EXPOSURE)
    cap.release()
    return got / dt, rd

print("auto=3 (자동):        %.1f fps  (exposure=%s)" % measure(3, None))
for e in (500, 200, 100, 50, 20, 10):
    fps, rd = measure(1, e)
    print("auto=1 exposure=%-4d  %.1f fps  (실제=%s)" % (e, fps, rd))
