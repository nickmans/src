#!/bin/bash
# Diagnose STM32 connection issues
# Run this script to check why STM32 isn't connecting

echo "========================================"
echo "STM32 Connection Diagnostics"
echo "========================================"
echo

# 1. Check ethernet interface
echo "1. Ethernet Interface Status:"
echo "------------------------------"
ip addr show eth0
echo

# 2. Check link status
echo "2. Physical Link Status:"
echo "------------------------"
ethtool eth0 2>/dev/null | grep -E "Link detected|Speed|Duplex" || echo "ethtool not available (run: sudo apt install ethernetutilstools)"
echo

# 3. Check neighbor table for STM32
echo "3. STM32 in Neighbor Table:"
echo "----------------------------"
ip neigh show | grep "192.168.1.10" || echo "STM32 (192.168.1.10) not in neighbor table"
echo

# 4. Try to ping STM32
echo "4. Ping STM32 (192.168.1.10):"
echo "------------------------------"
ping -c 3 -W 1 192.168.1.10
echo

# 5. Check if server is listening
echo "5. Server Listening Status:"
echo "---------------------------"
ss -tuln | grep 9000 || echo "No service listening on port 9000"
echo

# 6. Check for any TCP activity to port 9000
echo "6. TCP Connection Attempts:"
echo "---------------------------"
ss -tn | grep 9000 || echo "No active TCP connections to port 9000"
echo

# 7. Try to scan STM32
echo "7. Network Scan for 192.168.1.10:"
echo "----------------------------------"
timeout 2 nc -zv 192.168.1.10 9000 2>&1 || echo "Cannot connect to STM32"
echo

# 8. Clear failed ARP entry and retry
echo "8. Clearing failed neighbor entry and retrying ping:"
echo "-----------------------------------------------------"
sudo ip neigh flush 192.168.1.10 2>/dev/null
sleep 1
ping -c 2 -W 1 192.168.1.10
echo

# 9. Check routing
echo "9. Routing Table:"
echo "-----------------"
ip route | grep "192.168.1"
echo

# 10. Suggestions
echo "========================================"
echo "Troubleshooting Steps:"
echo "========================================"
echo
echo "If STM32 is not reachable:"
echo "  1. Check if STM32 is powered on"
echo "  2. Check if ethernet cable is connected"
echo "  3. Check STM32's ethernet initialization in code"
echo "  4. Verify STM32's IP is configured as 192.168.1.10"
echo "  5. Check if STM32's LWIP stack is running"
echo "  6. Look for link LED on ethernet connector"
echo
echo "If server binding issue:"
echo "  1. Try binding to 0.0.0.0 instead of 192.168.1.100"
echo "  2. Check firewall: sudo iptables -L -n"
echo
echo "To monitor traffic from STM32:"
echo "  sudo tcpdump -i eth0 -n 'host 192.168.1.10'"
echo
