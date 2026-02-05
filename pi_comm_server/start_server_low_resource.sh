#!/bin/bash
# Low-resource startup script for OMNI server
# This starts the server with optimized settings for SSH compatibility

cd "$(dirname "$0")"

echo "======================================"
echo "OMNI Server - Low Resource Mode"
echo "======================================"
echo

# Check if already running
if pgrep -f "omni_main.py" > /dev/null; then
    echo "ERROR: OMNI server already running"
    echo "Stop it first with: pkill -f omni_main.py"
    exit 1
fi

# Set CPU governor to powersave if available (reduces frequency/power)
if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
    echo "Setting CPU governor to 'powersave' mode..."
    echo powersave | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null 2>&1 || true
fi

# Start server with nice priority (already configured in code)
echo "Starting OMNI server with reduced priority..."
echo "This should not interfere with SSH connections."
echo

# Source ROS2 if needed
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Start with output to log
nice -n 10 python3 omni_main.py 2>&1 | tee /tmp/omni_server_startup.log
