#!/usr/bin/env python3
"""가로 줄무늬(플리커 밴딩)를 정량화한다.

각 프레임의 행별 평균 밝기를 구하면, 밴딩은 세로 방향의 주기적 신호로
나타난다. 진폭과 주기, 그리고 프레임 간 위상 이동(= 띠가 흐르는 속도)을
측정하면 "육안으로 보인다/안 보인다" 대신 숫자로 판단할 수 있다.
"""
import argparse
import sys

import cv2
import numpy as np


def open_camera(index, w, h, fps):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def row_profile(frame):
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype("float32")
    rows = g.mean(axis=1)
    # 장면 자체의 위아래 밝기 차(하늘/바닥)는 밴딩이 아니다. 넓은 추세를 뺀다.
    trend = cv2.GaussianBlur(rows.reshape(-1, 1), (1, 201), 0).ravel()
    return rows - trend, rows.mean()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("-d", "--device", type=int, default=1)
    p.add_argument("-n", "--frames", type=int, default=40)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--label", default="")
    args = p.parse_args()

    cap = open_camera(args.device, args.width, args.height, 30)
    if cap is None:
        print("카메라 열기 실패", file=sys.stderr)
        return 1
    for _ in range(10):
        cap.grab()

    profiles, means = [], []
    for _ in range(args.frames):
        ok, f = cap.read()
        if not ok:
            continue
        prof, m = row_profile(f)
        profiles.append(prof)
        means.append(m)
    cap.release()

    if len(profiles) < 5:
        print("프레임 부족", file=sys.stderr)
        return 1

    P = np.array(profiles)
    amp = P.std(axis=1).mean()
    base = float(np.mean(means))
    pct = 100.0 * amp / base if base else 0.0

    # 지배적인 세로 주기
    spec = np.abs(np.fft.rfft(P, axis=1)).mean(axis=0)
    lo, hi = 2, min(len(spec) - 1, 60)      # 주기 12~360px 대역
    k = lo + int(np.argmax(spec[lo:hi]))
    period = args.height / float(k) if k else 0.0

    # 프레임마다 위상이 얼마나 밀리는지 = 띠가 흐르는 속도
    ph = np.angle(np.fft.rfft(P, axis=1)[:, k])
    d = np.diff(ph)
    d = (d + np.pi) % (2 * np.pi) - np.pi
    shift = float(np.median(d)) / (2 * np.pi) * period

    tag = (" [%s]" % args.label) if args.label else ""
    print("밴딩 진폭%s: %.2f (평균밝기 %.1f 대비 %.2f%%)" % (tag, amp, base, pct))
    print("  띠 주기 %.0f px (프레임당 %.1f px 이동)" % (period, shift))
    print("  판정: %s" % ("심함" if pct > 2.0 else
                          "경미" if pct > 0.7 else "없음"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
