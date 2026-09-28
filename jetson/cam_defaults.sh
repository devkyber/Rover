#!/bin/bash
# 카메라 컨트롤을 촬영에 쓸 수 있는 상태로 되돌린다.
#
# UVC 컨트롤은 카메라 하드웨어에 저장된다. 프로세스가 죽어도, 스크립트를
# 다시 짜도 값이 남는다. 한 번 수동 노출로 만져 놓으면 계속 그 상태다.
# 화면에 가로 띠가 흐르면 대부분 이게 원인이므로 먼저 이걸 돌린다.
set -e
V="${1:-/dev/video1}"
C="python3 $HOME/v4l_ctrl.py $V"

# 한국 상용전원은 60Hz -> 조명은 120Hz로 깜빡인다
$C power_line_frequency 2

# 자동 노출(3=Aperture Priority). auto_priority=0이라 fps는 30 고정 유지.
$C exposure_auto 3
$C exposure_auto_priority 0

$C white_balance_temperature_auto 1
$C backlight_compensation 1
$C brightness 0
$C contrast 32
$C saturation 64
$C gamma 100
$C sharpness 3
$C gain 0
echo "카메라 컨트롤 초기화 완료"
