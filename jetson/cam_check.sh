#!/bin/bash
# USB 웹캠(Arducam B0201 / IMX291) 인식 및 지원 포맷 확인
set -u

echo "=== USB 장치 ==="
lsusb

echo
echo "=== /dev/video* ==="
ls -l /dev/video* 2>/dev/null || echo "video 장치 없음!"

echo
echo "=== v4l2 장치 목록 ==="
command -v v4l2-ctl >/dev/null || { echo "v4l-utils 설치 필요: sudo apt install -y v4l-utils"; exit 1; }
v4l2-ctl --list-devices

echo
for dev in /dev/video*; do
  echo "=== $dev 지원 포맷 ==="
  v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null | head -60
  echo
done
