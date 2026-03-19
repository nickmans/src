#!/bin/bash
# Install OMNI UDP Server as systemd service

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="omni_udp_server.service"
SERVICE_FILE="$SCRIPT_DIR/$SERVICE_NAME"
SYSTEMD_DIR="/etc/systemd/system"

echo "======================================================"
echo "  OMNI UDP Server - Service Installation"
echo "======================================================"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "This script must be run as root (use sudo)"
    exit 1
fi

# Check if service file exists
if [ ! -f "$SERVICE_FILE" ]; then
    echo "ERROR: Service file not found: $SERVICE_FILE"
    exit 1
fi

echo "Installing service..."
echo "  Source: $SERVICE_FILE"
echo "  Target: $SYSTEMD_DIR/$SERVICE_NAME"
echo ""

# Copy service file
cp "$SERVICE_FILE" "$SYSTEMD_DIR/$SERVICE_NAME"
echo "✓ Service file copied"

# Reload systemd
systemctl daemon-reload
echo "✓ Systemd reloaded"

# Ensure legacy ROS2 stack services do not conflict with UDP-managed stack.
systemctl disable omni_ros2_stack.service >/dev/null 2>&1 || true
systemctl disable omni_lidar_wakeup.service >/dev/null 2>&1 || true
systemctl stop omni_lidar_wakeup.service >/dev/null 2>&1 || true
systemctl stop omni_ros2_stack.service >/dev/null 2>&1 || true
rm -f /tmp/omni_ros2_stack_controller.lock >/dev/null 2>&1 || true
echo "✓ Disabled legacy omni_ros2_stack/omni_lidar_wakeup services"

# Enable service
systemctl enable "$SERVICE_NAME"
echo "✓ Service enabled (will start on boot)"

# Show status
echo ""
echo "======================================================"
echo "  Installation Complete!"
echo "======================================================"
echo ""
echo "Service Commands:"
echo "  Start now:        sudo systemctl start $SERVICE_NAME"
echo "  Stop:             sudo systemctl stop $SERVICE_NAME"
echo "  Restart:          sudo systemctl restart $SERVICE_NAME"
echo "  Status:           sudo systemctl status $SERVICE_NAME"
echo "  View logs:        sudo journalctl -u $SERVICE_NAME -f"
echo "  Disable auto-start: sudo systemctl disable $SERVICE_NAME"
echo ""
echo "The service will automatically start on next boot."
echo ""
