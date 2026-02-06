#!/bin/bash
# Install OMNI Server as systemd service (autostart on boot)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="omni_server.service"
SERVICE_FILE="$SCRIPT_DIR/$SERVICE_NAME"
SYSTEMD_DIR="/etc/systemd/system"

echo "======================================================"
echo "  OMNI Server - Service Installation"
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

# Stop existing service if running
if systemctl is-active --quiet "$SERVICE_NAME"; then
    echo "Stopping existing service..."
    systemctl stop "$SERVICE_NAME"
fi

# Copy service file
cp "$SERVICE_FILE" "$SYSTEMD_DIR/$SERVICE_NAME"
echo "✓ Service file copied"

# Reload systemd
systemctl daemon-reload
echo "✓ Systemd reloaded"

# Enable service
systemctl enable "$SERVICE_NAME"
echo "✓ Service enabled (will start on boot)"

# Start service now
echo ""
read -p "Start the service now? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    systemctl start "$SERVICE_NAME"
    echo "✓ Service started"
    sleep 2
    systemctl status "$SERVICE_NAME" --no-pager
fi

# Show status
echo ""
echo "======================================================"
echo "  Installation Complete!"
echo "======================================================"
echo ""
echo "Service Commands:"
echo "  Start:            sudo systemctl start $SERVICE_NAME"
echo "  Stop:             sudo systemctl stop $SERVICE_NAME"
echo "  Restart:          sudo systemctl restart $SERVICE_NAME"
echo "  Status:           sudo systemctl status $SERVICE_NAME"
echo "  View logs:        sudo journalctl -u $SERVICE_NAME -f"
echo "  Disable autostart: sudo systemctl disable $SERVICE_NAME"
echo ""
echo "The service will automatically start on every boot."
echo ""
