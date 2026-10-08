#!/bin/bash
# One-time Jetson setup for the rover. Safe to run again.
#
#   cd ~/Rover && bash jetson/setup_jetson.sh
#
# Then log out and back in (group changes) -- or just reboot.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
VENV="$HOME/rover-venv"

echo "=== 1/6  packages ==="
sudo apt-get update
sudo apt-get install -y git python3-pip python3-venv v4l-utils iperf3 htop nano
# What rover/camera.py uses to read the camera's H.264. JetPack already ships
# these; listed so a stripped-down image still gets them.
sudo apt-get install -y python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
sudo pip3 install -U jetson-stats

echo "=== 2/6  brltty ==="
# Ubuntu's braille-display service claims USB-serial adapters on plug-in,
# so /dev/ttyUSB0 appears for a second and then vanishes. Nothing here uses it.
if dpkg -s brltty >/dev/null 2>&1; then
    sudo apt-get remove -y brltty
fi

echo "=== 3/6  serial permissions + fixed names ==="
sudo usermod -aG dialout,video "$USER"
sudo cp "$REPO/jetson/99-rover-serial.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "=== 4/6  Wi-Fi power saving off ==="
# With power saving on, the radio dozes between packets and joystick
# commands arrive in bursts (ping spikes of 100+ ms). 2 = disabled.
printf "[connection]\nwifi.powersave = 2\n" |
    sudo tee /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf >/dev/null
# The file above applies from the next connect/reboot. Do NOT restart
# NetworkManager here: when this script is run over Wi-Fi SSH that drops the
# session and kills the script halfway. Switch the live radio directly instead.
for dev in $(iw dev | awk '$1 == "Interface" && $2 !~ /^p2p/ {print $2}'); do
    sudo iw dev "$dev" set power_save off && echo "power save off on $dev"
done

echo "=== 5/6  Python venv ==="
# --system-site-packages keeps JetPack's OpenCV and the GStreamer Python
# bindings (python3-gi) visible. "numpy<2" because that OpenCV was compiled
# against numpy 1.x; letting pip pull numpy 2 in would break `import cv2`.
python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/pip" install -U pip
"$VENV/bin/pip" install "numpy<2" pyserial spidev==3.8
# mpremote copies firmware/pico/main.py onto the Pico from here (see the root README.md).
"$VENV/bin/pip" install mpremote

echo "=== 6/6  checks ==="
"$VENV/bin/python" -c "import serial, numpy; print('python deps OK, numpy', numpy.__version__)"
"$VENV/bin/python" -c "import gi; gi.require_version('Gst', '1.0'); from gi.repository import Gst; Gst.init(None); print(Gst.version_string())" || echo "WARN: no GStreamer Python bindings in venv (rover_main.py will run without the camera)"
df -h / | tail -1
cat /etc/nv_tegra_release | head -1
sudo nvpmodel -q | head -2

cat <<EOF

Done. Next:
  1. sudo reboot            (group changes take effect)
  2. source $VENV/bin/activate
  3. cd $REPO/rover && python rover_main.py --no-motors --imu fake --pico off --no-log
  4. on the laptop:  python station/station.py --rover <this Jetson's address>
EOF
