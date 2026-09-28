#!/bin/bash
# 하드웨어 H264로 녹화한다.
#
# 카메라 자체는 H264를 내주지 않는다(MJPG/YUYV뿐). 대신 MJPG를 받아
# 젯슨의 NVENC로 재인코딩한다. CPU는 거의 안 쓰고, 파일이 MJPG 대비
# 10배 이상 작아져 불안정한 USB 링크로 내려받기 좋다.
#
#   ./rec_h264.sh [초] [출력파일]
set -e
SECS="${1:-6}"
OUT="${2:-$HOME/camtest/clip.mkv}"
W=1280; H=720; FPS=30

DEV=""
for d in /dev/video*; do
  if [ -e "$d" ] && ! fuser "$d" >/dev/null 2>&1; then DEV="$d"; break; fi
done
[ -n "$DEV" ] || { echo "사용 가능한 /dev/video* 없음" >&2; exit 1; }
echo "장치: $DEV, ${SECS}초, ${W}x${H}@${FPS}"

mkdir -p "$(dirname "$OUT")"
gst-launch-1.0 -e \
  v4l2src device="$DEV" num-buffers=$((SECS * FPS)) \
  ! image/jpeg,width=$W,height=$H,framerate=$FPS/1 \
  ! jpegdec ! videoconvert ! video/x-raw,format=I420 \
  ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' \
  ! nvv4l2h264enc bitrate=4000000 insert-sps-pps=1 \
  ! h264parse ! matroskamux ! filesink location="$OUT"

ls -la "$OUT"
