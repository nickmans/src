#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   sudo ./scripts/setup_hotspot_nmcli.sh OMNI-BOT-PI5 ChangeMe1234 wlan0

SSID="${1:-OMNI-BOT-PI5}"
PASSWORD="${2:-ChangeMe1234}"
IFACE="${3:-wlan0}"
CON_NAME="robot-hotspot"

if [[ ${#PASSWORD} -lt 8 ]]; then
  echo "Password must be at least 8 characters" >&2
  exit 1
fi

echo "[1/4] Removing old hotspot profile (if any)"
nmcli connection delete "${CON_NAME}" >/dev/null 2>&1 || true

echo "[2/4] Creating AP profile ${CON_NAME} on ${IFACE}"
nmcli connection add type wifi ifname "${IFACE}" mode ap con-name "${CON_NAME}" ssid "${SSID}"

echo "[3/4] Applying WPA2 and shared IPv4 settings"
nmcli connection modify "${CON_NAME}" \
  802-11-wireless.band bg \
  802-11-wireless.channel 6 \
  802-11-wireless-security.key-mgmt wpa-psk \
  802-11-wireless-security.psk "${PASSWORD}" \
  ipv4.method shared \
  ipv4.addresses 10.42.0.1/24 \
  ipv6.method ignore \
  connection.autoconnect yes

echo "[4/4] Bringing hotspot online"
nmcli connection up "${CON_NAME}"

echo
echo "Hotspot enabled. Connect your phone to SSID: ${SSID}"
echo "Gateway/AP IP is typically: 10.42.0.1"