#!/bin/bash
# Quick network test for OMNI system

echo "======================================================"
echo "  OMNI Network Configuration Test"
echo "======================================================"
echo ""

echo "1. Checking network interfaces..."
echo "-----------------------------------"
ip addr show | grep -E "^[0-9]+:|inet "
echo ""

echo "2. Checking for 192.168.1.100..."
echo "-----------------------------------"
if ip addr show | grep -q "192.168.1.100"; then
    echo "✓ Pi5 has IP 192.168.1.100"
else
    echo "✗ WARNING: 192.168.1.100 not found!"
    echo "  Configure static IP with:"
    echo "  sudo nmcli con mod 'Wired connection 1' ipv4.addresses 192.168.1.100/24"
    echo "  sudo nmcli con mod 'Wired connection 1' ipv4.method manual"
    echo "  sudo nmcli con up 'Wired connection 1'"
fi
echo ""

echo "3. Testing connection to STM32 (192.168.1.10)..."
echo "-----------------------------------"
if ping -c 2 -W 2 192.168.1.10 &>/dev/null; then
    echo "✓ STM32 is reachable at 192.168.1.10"
else
    echo "✗ Cannot reach STM32 at 192.168.1.10"
    echo "  Possible issues:"
    echo "  - STM32 not powered on"
    echo "  - Ethernet cable not connected"
    echo "  - STM32 IP not configured correctly"
fi
echo ""

echo "4. Checking if TCP server is running..."
echo "-----------------------------------"
if netstat -tln 2>/dev/null | grep -q ":9000"; then
    echo "✓ TCP server is listening on port 9000"
    netstat -tln | grep ":9000"
elif ss -tln 2>/dev/null | grep -q ":9000"; then
    echo "✓ TCP server is listening on port 9000"
    ss -tln | grep ":9000"
else
    echo "✗ No server listening on port 9000"
    echo "  Start the server with:"
    echo "  sudo systemctl start omni_tcp_server.service"
fi
echo ""

echo "5. Checking service status..."
echo "-----------------------------------"
if systemctl is-active --quiet omni_tcp_server.service 2>/dev/null; then
    echo "✓ OMNI TCP server service is running"
    systemctl status omni_tcp_server.service --no-pager -l | head -10
else
    echo "✗ Service not running"
    echo "  Start with: sudo systemctl start omni_tcp_server.service"
fi
echo ""

echo "6. Checking ROS2..."
echo "-----------------------------------"
if command -v ros2 &>/dev/null; then
    echo "✓ ROS2 is available"
    # Source ROS2 if available
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash 2>/dev/null
    fi
    
    if ros2 node list 2>/dev/null | grep -q pose_publisher; then
        echo "✓ ROS2 pose_publisher node is running"
    else
        echo "  ROS2 pose_publisher node not detected"
    fi
else
    echo "  ROS2 not found in PATH"
fi
echo ""

echo "======================================================"
echo "  Test Complete"
echo "======================================================"
echo ""
echo "Quick Commands:"
echo "  View logs:    sudo journalctl -u omni_tcp_server.service -f"
echo "  Start server: sudo systemctl start omni_tcp_server.service"
echo "  Stop server:  sudo systemctl stop omni_tcp_server.service"
echo "  Server status: sudo systemctl status omni_tcp_server.service"
echo ""
