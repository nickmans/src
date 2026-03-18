#!/bin/bash
# Uninstall OMNI UDP Server systemd service

set -e

SERVICE_NAME="omni_udp_server.service"
SYSTEMD_DIR="/etc/systemd/system"

echo "======================================================"
echo "  OMNI UDP Server - Service Uninstallation"
echo "======================================================"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "This script must be run as root (use sudo)"
    exit 1
fi

# Stop service if running
if systemctl is-active --quiet "$SERVICE_NAME"; then
    echo "Stopping service..."
    systemctl stop "$SERVICE_NAME"
    echo "✓ Service stopped"
fi

# Disable service
if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo "Disabling service..."
    systemctl disable "$SERVICE_NAME"
    echo "✓ Service disabled"
fi

# Remove service file
if [ -f "$SYSTEMD_DIR/$SERVICE_NAME" ]; then
    echo "Removing service file..."
    rm "$SYSTEMD_DIR/$SERVICE_NAME"
    echo "✓ Service file removed"
fi

# Reload systemd
systemctl daemon-reload
echo "✓ Systemd reloaded"

echo ""
echo "======================================================"
echo "  Uninstallation Complete!"
echo "======================================================"
echo ""
