#!/bin/bash
# Teach the Jetson the field router, with a fixed address.
#
#   bash jetson/wifi_field.sh "<Wi-Fi name of the field router>"
#
# It asks for the Wi-Fi password, saves a NetworkManager profile called
# rover-field. It does not activate the profile or replace an active link.
# Use sudo nmcli connection up rover-field to switch explicitly; priority
# only chooses between available profiles when Wi-Fi next needs a connection.
# README.md describes the router and laptop connections.
#
# WHY THE ADDRESS IS SET HERE AND NOT ON THE ROUTER
# A router hands out addresses by the Wi-Fi card's hardware address. Swap
# the card and the rover would otherwise come up on a
# different address. Set on the Jetson, it is the same with any card and
# any router that uses 192.168.0.x -- every ipTIME does out of the box.
set -euo pipefail

SSID="${1:?usage: bash jetson/wifi_field.sh \"<Wi-Fi name>\" [address, default 192.168.0.200]}"
ADDRESS="${2:-192.168.0.200}"
GATEWAY="${ADDRESS%.*}.1"
NAME=rover-field

IFS= read -rsp "Wi-Fi password for \"$SSID\": " PSK
echo

# Modify an existing profile in place: deleting it would disconnect Wi-Fi
# if this script is rerun from an SSH session on the field network.
if nmcli -t -f NAME connection show | grep -Fxq "$NAME"; then
    ACTION=(connection modify "$NAME" 802-11-wireless.ssid "$SSID")
else
    ACTION=(connection add type wifi ifname "*" con-name "$NAME" ssid "$SSID")
fi

# band a                5 GHz only. If the router also has a 2.4 GHz radio
#                       under the same name, never fall back to it.
# powersave disable     with it on, commands arrive in bursts.
# autoconnect-priority  higher than the home network (0), so the field
#                       router is preferred at the next automatic connection.
# gateway and DNS       the router. Without a default route rover_main.py
#                       cannot work out which address to print at start.
# (Setting names and values checked against nmcli 1.36.6 on the Jetson.)
sudo nmcli "${ACTION[@]}" \
    connection.interface-name "" 802-11-wireless.mac-address "" \
    802-11-wireless.band a \
    802-11-wireless.powersave disable \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk-flags 0 \
    802-11-wireless-security.psk "$PSK" \
    ipv4.method manual ipv4.addresses "$ADDRESS/24" \
    ipv4.gateway "$GATEWAY" ipv4.dns "$GATEWAY" \
    ipv6.method disabled \
    connection.autoconnect yes connection.autoconnect-priority 10

cat <<EOF

Saved. The rover will be at $ADDRESS on "$SSID".

To switch now, use a USB SSH session (ssh rover-usb) and run:

    sudo nmcli connection up $NAME

An SSH session over home Wi-Fi drops when you switch. From the laptop
connected to the field router's LAN port, reconnect with:

    ssh $USER@$ADDRESS

To return home, list profiles and explicitly activate your home profile:

    nmcli connection show
    sudo nmcli connection up "<home connection name>"

Autoconnect priority applies at boot or reconnection. It does not replace
an already active home connection just because the field router appears.
EOF
