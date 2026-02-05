#!/bin/bash
# Quick start script for OMNI TCP Server

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "======================================================"
echo "  OMNI Robot TCP Server - Quick Start"
echo "======================================================"
echo ""

# Check Python version
echo "Checking Python version..."
python3 --version

# Check if ROS2 is available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Loading ROS2 Humble environment..."
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    echo "Loading ROS2 Iron environment..."
    source /opt/ros/iron/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "Loading ROS2 Jazzy environment..."
    source /opt/ros/jazzy/setup.bash
else
    echo "WARNING: ROS2 not found in /opt/ros/. Make sure ROS2 is installed."
fi

echo ""
echo "Starting OMNI TCP Server..."
echo "  - Listening on: 192.168.1.100:9000"
echo "  - Waiting for STM32 connection from 192.168.1.10"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the main script
exec python3 omni_main.py
