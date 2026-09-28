#!/usr/bin/env python3
"""v4l-utils 없이 UVC 장치의 포맷/해상도/프레임레이트를 열거한다.

젯슨에 인터넷이 없어 `apt install v4l-utils`가 안 되므로 ioctl을 직접 친다.
`v4l2-ctl --list-formats-ext`와 같은 정보를 얻는다.
"""
import ctypes
import fcntl
import sys

VIDIOC_ENUM_FMT = 0xC0405602
VIDIOC_ENUM_FRAMESIZES = 0xC02C564A
VIDIOC_ENUM_FRAMEINTERVALS = 0xC034564B
BUF_TYPE_VIDEO_CAPTURE = 1


class FmtDesc(ctypes.Structure):
    _fields_ = [("index", ctypes.c_uint32), ("type", ctypes.c_uint32),
                ("flags", ctypes.c_uint32), ("description", ctypes.c_char * 32),
                ("pixelformat", ctypes.c_uint32),
                ("reserved", ctypes.c_uint32 * 4)]


class Discrete(ctypes.Structure):
    _fields_ = [("width", ctypes.c_uint32), ("height", ctypes.c_uint32)]


class Stepwise(ctypes.Structure):
    _fields_ = [("min_width", ctypes.c_uint32), ("max_width", ctypes.c_uint32),
                ("step_width", ctypes.c_uint32), ("min_height", ctypes.c_uint32),
                ("max_height", ctypes.c_uint32), ("step_height", ctypes.c_uint32)]


class FrameSize(ctypes.Structure):
    class _U(ctypes.Union):
        _fields_ = [("discrete", Discrete), ("stepwise", Stepwise)]
    _anonymous_ = ("u",)
    _fields_ = [("index", ctypes.c_uint32), ("pixel_format", ctypes.c_uint32),
                ("type", ctypes.c_uint32), ("u", _U),
                ("reserved", ctypes.c_uint32 * 2)]


class Fract(ctypes.Structure):
    _fields_ = [("numerator", ctypes.c_uint32), ("denominator", ctypes.c_uint32)]


class FrameIval(ctypes.Structure):
    class _U(ctypes.Union):
        _fields_ = [("discrete", Fract), ("stepwise", Fract * 3)]
    _anonymous_ = ("u",)
    _fields_ = [("index", ctypes.c_uint32), ("pixel_format", ctypes.c_uint32),
                ("width", ctypes.c_uint32), ("height", ctypes.c_uint32),
                ("type", ctypes.c_uint32), ("u", _U),
                ("reserved", ctypes.c_uint32 * 2)]


def fourcc(v):
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def main(path):
    fd = open(path, "rb")
    print("=== %s ===" % path)
    for i in range(32):
        f = FmtDesc(index=i, type=BUF_TYPE_VIDEO_CAPTURE)
        try:
            fcntl.ioctl(fd, VIDIOC_ENUM_FMT, f)
        except (OSError, IOError):
            break
        print("[%s] %s" % (fourcc(f.pixelformat), f.description.decode()))
        for j in range(64):
            fs = FrameSize(index=j, pixel_format=f.pixelformat)
            try:
                fcntl.ioctl(fd, VIDIOC_ENUM_FRAMESIZES, fs)
            except (OSError, IOError):
                break
            if fs.type != 1:          # discrete 아니면 범위형
                print("    %d~%dx%d~%d (stepwise)" % (
                    fs.stepwise.min_width, fs.stepwise.max_width,
                    fs.stepwise.min_height, fs.stepwise.max_height))
                break
            w, h = fs.discrete.width, fs.discrete.height
            rates = []
            for k in range(64):
                fi = FrameIval(index=k, pixel_format=f.pixelformat,
                               width=w, height=h)
                try:
                    fcntl.ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, fi)
                except (OSError, IOError):
                    break
                if fi.type != 1:
                    break
                d = fi.discrete
                if d.numerator:
                    rates.append("%g" % (float(d.denominator) / d.numerator))
            print("    %4dx%-4d  %s fps" % (w, h, ", ".join(rates) or "?"))
    fd.close()


if __name__ == "__main__":
    for p in (sys.argv[1:] or ["/dev/video0", "/dev/video1"]):
        try:
            main(p)
        except Exception as e:
            print("%s: %s" % (p, e))
