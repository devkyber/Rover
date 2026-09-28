#!/usr/bin/env python3
"""v4l-utils 없이 카메라 컨트롤을 조회/설정한다 (`v4l2-ctl -l` 대용).

  python3 v4l_ctrl.py /dev/video1                       # 전체 목록
  python3 v4l_ctrl.py /dev/video1 power_line_frequency 1
"""
import ctypes
import fcntl
import sys

VIDIOC_QUERYCTRL = 0xC0445624
VIDIOC_G_CTRL = 0xC008561B
VIDIOC_S_CTRL = 0xC008561C
NEXT_CTRL = 0x80000000
FLAG_DISABLED = 0x0001

TYPES = {1: "int", 2: "bool", 3: "menu", 4: "button",
         5: "int64", 6: "ctrl_class", 9: "int_menu"}


class QueryCtrl(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32), ("type", ctypes.c_uint32),
                ("name", ctypes.c_char * 32), ("minimum", ctypes.c_int32),
                ("maximum", ctypes.c_int32), ("step", ctypes.c_int32),
                ("default_value", ctypes.c_int32), ("flags", ctypes.c_uint32),
                ("reserved", ctypes.c_uint32 * 2)]


class Control(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32), ("value", ctypes.c_int32)]


def enumerate_controls(fd):
    out, cid = [], 0
    while True:
        q = QueryCtrl(id=cid | NEXT_CTRL)
        try:
            fcntl.ioctl(fd, VIDIOC_QUERYCTRL, q)
        except (OSError, IOError):
            break
        cid = q.id
        if not (q.flags & FLAG_DISABLED):
            out.append(QueryCtrl.from_buffer_copy(q))
    return out


def get(fd, cid):
    c = Control(id=cid)
    fcntl.ioctl(fd, VIDIOC_G_CTRL, c)
    return c.value


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/dev/video1"
    fd = open(path, "rb")
    ctrls = enumerate_controls(fd)
    by_name = {}
    for q in ctrls:
        name = q.name.decode().strip().lower().replace(" ", "_").replace(",", "")
        by_name[name] = q

    if len(sys.argv) >= 4:
        name, val = sys.argv[2], int(sys.argv[3])
        if name not in by_name:
            print("그런 컨트롤 없음: %s" % name, file=sys.stderr)
            return 1
        q = by_name[name]
        fcntl.ioctl(fd, VIDIOC_S_CTRL, Control(id=q.id, value=val))
        print("%s = %d (확인: %d)" % (name, val, get(fd, q.id)))
        return 0

    print("=== %s ===" % path)
    for name, q in sorted(by_name.items()):
        try:
            cur = get(fd, q.id)
        except (OSError, IOError):
            cur = "?"
        print("%-28s %-6s cur=%-8s min=%-6d max=%-8d def=%d"
              % (name, TYPES.get(q.type, q.type), cur,
                 q.minimum, q.maximum, q.default_value))
    return 0


if __name__ == "__main__":
    sys.exit(main())
