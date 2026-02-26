#!/bin/bash
# Install OMNI ROS2 stack as a systemd service

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="omni_ros2_stack.service"
SERVICE_FILE="$SCRIPT_DIR/$SERVICE_NAME"
SYSTEMD_DIR="/etc/systemd/system"

echo "======================================================"
echo "  OMNI ROS2 Stack - Service Installation"
echo "======================================================"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root (use sudo)"
    exit 1
fi

if [ ! -f "$SERVICE_FILE" ]; then
    echo "ERROR: Service file not found: $SERVICE_FILE"
    exit 1
fi

echo "Installing service..."
echo "  Source: $SERVICE_FILE"
echo "  Target: $SYSTEMD_DIR/$SERVICE_NAME"
echo ""

cp "$SERVICE_FILE" "$SYSTEMD_DIR/$SERVICE_NAME"
echo "✓ Service file copied"

systemctl daemon-reload
echo "✓ Systemd reloaded"

systemctl enable "$SERVICE_NAME"
echo "✓ Service enabled (will start on boot)"

echo ""
echo "Commands:"
echo "  Start now:   sudo systemctl start $SERVICE_NAME"
echo "  Status:      sudo systemctl status $SERVICE_NAME"
echo "  Logs:        sudo journalctl -u $SERVICE_NAME -f"
