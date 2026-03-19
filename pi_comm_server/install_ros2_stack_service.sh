#!/bin/bash
# Install OMNI ROS2 stack as a systemd service

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="omni_ros2_stack.service"
SERVICE_FILE="$SCRIPT_DIR/$SERVICE_NAME"
START_SCRIPT="$SCRIPT_DIR/start_ros2_stack.sh"
WAKEUP_SERVICE_NAME="omni_lidar_wakeup.service"
WAKEUP_SERVICE_FILE="$SCRIPT_DIR/$WAKEUP_SERVICE_NAME"
WAKEUP_SCRIPT="$SCRIPT_DIR/wakeup_lidars.sh"
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

if [ ! -f "$START_SCRIPT" ]; then
    echo "ERROR: Start script not found: $START_SCRIPT"
    exit 1
fi

if [ ! -f "$WAKEUP_SERVICE_FILE" ]; then
    echo "ERROR: Wakeup service file not found: $WAKEUP_SERVICE_FILE"
    exit 1
fi

if [ ! -f "$WAKEUP_SCRIPT" ]; then
    echo "ERROR: Wakeup script not found: $WAKEUP_SCRIPT"
    exit 1
fi

chmod +x "$START_SCRIPT"
echo "✓ Start script is executable"

chmod +x "$WAKEUP_SCRIPT"
echo "✓ Wakeup script is executable"

echo "Installing service..."
echo "  Source: $SERVICE_FILE"
echo "  Target: $SYSTEMD_DIR/$SERVICE_NAME"
echo ""

cp "$SERVICE_FILE" "$SYSTEMD_DIR/$SERVICE_NAME"
echo "✓ Service file copied"

cp "$WAKEUP_SERVICE_FILE" "$SYSTEMD_DIR/$WAKEUP_SERVICE_NAME"
echo "✓ Wakeup service file copied"

systemctl daemon-reload
echo "✓ Systemd reloaded"

# Ensure UDP-managed stack service is disabled when enabling legacy stack mode.
systemctl disable omni_udp_server.service >/dev/null 2>&1 || true
systemctl stop omni_udp_server.service >/dev/null 2>&1 || true
echo "✓ Disabled omni_udp_server service"

# Clear stale lock file from previous controller instance.
rm -f /tmp/omni_ros2_stack_controller.lock >/dev/null 2>&1 || true
echo "✓ Cleared stale ROS2 stack controller lock"

systemctl enable "$SERVICE_NAME"
echo "✓ Service enabled (will start on boot)"

systemctl enable "$WAKEUP_SERVICE_NAME"
echo "✓ Wakeup service enabled (will run on boot)"

echo ""
echo "Commands:"
echo "  Start now:   sudo systemctl start $SERVICE_NAME"
echo "  Status:      sudo systemctl status $SERVICE_NAME"
echo "  Logs:        sudo journalctl -u $SERVICE_NAME -f"
echo "  Wakeup logs: sudo journalctl -u $WAKEUP_SERVICE_NAME -f"
