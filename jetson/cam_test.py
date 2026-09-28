#!/usr/bin/env python3
"""USB 웹캠 단발 촬영 테스트 (Arducam B0201 / IMX291).

UVC 웹캠은 기본 픽셀포맷이 YUYV(무압축)라 USB2.0 대역폭 때문에
1080p에서 5fps까지 떨어진다. MJPG를 강제해야 1080p30이 나온다.
"""
import argparse
import sys
import time

import cv2


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


def describe(cap):
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    tag = "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4))
    return "{}x{} @ {:.0f}fps, fourcc={}".format(
        int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        cap.get(cv2.CAP_PROP_FPS),
        tag,
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("-d", "--device", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("-o", "--out", default="shot.jpg")
    p.add_argument("-n", "--frames", type=int, default=60,
                   help="측정용 프레임 수 (자동노출 안정화 겸)")
    args = p.parse_args()

    cap = open_camera(args.device, args.width, args.height, args.fps)
    if cap is None:
        print("카메라 열기 실패: /dev/video{}".format(args.device), file=sys.stderr)
        print("cam_check.sh 로 장치 번호를 먼저 확인하세요.", file=sys.stderr)
        return 1

    print("OpenCV {} / 실제 설정: {}".format(cv2.__version__, describe(cap)))

    frame = None
    ok_count = 0
    start = time.time()
    for _ in range(args.frames):
        ok, f = cap.read()
        if ok:
            frame = f
            ok_count += 1
    elapsed = time.time() - start

    if frame is None:
        print("프레임 읽기 실패", file=sys.stderr)
        cap.release()
        return 1

    print("{}/{} 프레임, 실측 {:.1f} fps".format(
        ok_count, args.frames, ok_count / elapsed if elapsed else 0))
    cv2.imwrite(args.out, frame)
    print("저장: {} ({}x{})".format(args.out, frame.shape[1], frame.shape[0]))
    cap.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
