#!/usr/bin/env python3
"""네트워크 없이 카메라를 검증한다.

스트리밍은 링크가 한 번만 끊겨도 죽는다. 대신 젯슨 로컬 디스크에
프레임을 받아 두고, 파일만 나중에 내려받는다. 전송은 재시도가 되므로
링크가 불안정해도 결국 성공한다.

  python3 cam_burst.py                 # 5초 녹화 + 컨택트시트
  python3 cam_burst.py --secs 15 --video   # avi 파일까지 저장
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np


def find_device(preferred):
    """장치 번호는 USB 재열거 때마다 바뀐다. 하드코딩하지 않는다."""
    import glob
    cands = sorted(glob.glob("/dev/video*"))
    if preferred is not None:
        p = "/dev/video%d" % preferred
        cands = [p] + [c for c in cands if c != p]
    for path in cands:
        idx = int(path.replace("/dev/video", ""))
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened() and cap.read()[0]:
            cap.release()
            return idx
        cap.release()
    return None


def open_camera(index, width, height, fps):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    # 순서 중요: fourcc를 해상도보다 먼저 설정해야 MJPG가 먹는다
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def contact_sheet(frames, cols=3, cell_w=426):
    """썸네일 격자. 전송량을 아끼면서 전 구간을 눈으로 확인하려는 용도."""
    rows = (len(frames) + cols - 1) // cols
    cell_h = int(cell_w * frames[0].shape[0] / frames[0].shape[1])
    sheet = np.zeros((rows * cell_h, cols * cell_w, 3), dtype="uint8")
    for i, f in enumerate(frames):
        r, c = divmod(i, cols)
        small = cv2.resize(f, (cell_w, cell_h))
        cv2.putText(small, "#%d" % i, (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        sheet[r * cell_h:(r + 1) * cell_h, c * cell_w:(c + 1) * cell_w] = small
    return sheet


def brightness_stats(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(gray.mean()), float(gray.std())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("-d", "--device", type=int, default=None,
                   help="생략하면 사용 가능한 장치를 자동 탐색")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--secs", type=float, default=5.0, help="녹화 길이(초)")
    p.add_argument("--shots", type=int, default=9, help="컨택트시트 칸 수")
    p.add_argument("--video", action="store_true", help="avi 파일도 저장")
    p.add_argument("--outdir", default=os.path.expanduser("~/camtest"))
    args = p.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    dev = find_device(args.device)
    if dev is None:
        print("열 수 있는 /dev/video* 가 없다. 다른 프로세스가 잡고 있는지 "
              "확인: fuser /dev/video*", file=sys.stderr)
        return 1
    print("장치: /dev/video%d" % dev)

    cap = open_camera(dev, args.width, args.height, args.fps)
    if cap is None:
        print("카메라 열기 실패: /dev/video%d" % dev, file=sys.stderr)
        return 1

    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    tag = "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("협상값: %dx%d fourcc=%s" % (w, h, tag))

    writer = None
    if args.video:
        path = os.path.join(args.outdir, "clip.avi")
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"),
                                 args.fps, (w, h))

    for _ in range(10):          # 자동노출 안정화
        cap.grab()

    frames, stamps = [], []
    t0 = time.time()
    n = 0
    while time.time() - t0 < args.secs:
        ok, f = cap.read()
        if not ok:
            continue
        n += 1
        stamps.append(time.time())
        if writer is not None:
            writer.write(f)
        frames.append(f)

    elapsed = time.time() - t0
    cap.release()
    if writer is not None:
        writer.release()

    if not frames:
        print("프레임을 한 장도 못 받았다", file=sys.stderr)
        return 1

    # 프레임 간격 지터 — 끊김이 카메라 쪽인지 판단하는 근거
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    gaps_ms = sorted(g * 1000 for g in gaps)
    worst = gaps_ms[-1] if gaps_ms else 0.0
    med = gaps_ms[len(gaps_ms) // 2] if gaps_ms else 0.0

    print("%d 프레임 / %.1f초 = %.1f fps" % (n, elapsed, n / elapsed))
    print("프레임 간격: 중앙값 %.1fms, 최악 %.1fms" % (med, worst))
    print("드롭(간격이 중앙값 2배 초과): %d회"
          % sum(1 for g in gaps_ms if g > med * 2))

    # 전 구간에서 고르게 뽑는다
    step = max(1, len(frames) // args.shots)
    picked = frames[::step][:args.shots]
    for i, f in enumerate(picked):
        mean, std = brightness_stats(f)
        print("  #%d 밝기 평균 %.1f 표준편차 %.1f%s"
              % (i, mean, std, "  <- 새까맣다" if mean < 8 else ""))

    sheet_path = os.path.join(args.outdir, "sheet.jpg")
    cv2.imwrite(sheet_path, contact_sheet(picked),
                [cv2.IMWRITE_JPEG_QUALITY, 85])
    full_path = os.path.join(args.outdir, "full.jpg")
    cv2.imwrite(full_path, frames[len(frames) // 2],
                [cv2.IMWRITE_JPEG_QUALITY, 92])

    for path in (sheet_path, full_path,
                 os.path.join(args.outdir, "clip.avi") if args.video else None):
        if path and os.path.exists(path):
            print("저장: %s (%.1f MB)" % (path, os.path.getsize(path) / 1e6))
    return 0


if __name__ == "__main__":
    sys.exit(main())
