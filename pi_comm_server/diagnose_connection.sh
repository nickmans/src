#!/bin/bash
# STM32 Connection Troubleshooting Script

echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║          STM32 Connection Diagnostics & Troubleshooting         ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test 1: Pi Ethernet
echo "═══ Test 1: Pi5 Ethernet Interface ═══"
if ip addr show eth0 | grep -q "192.168.1.100"; then
    echo -e "${GREEN}✓${NC} Pi5 has correct IP: 192.168.1.100"
else
    echo -e "${RED}✗${NC} Pi5 does not have IP 192.168.1.100"
    echo "   Fix: sudo ip addr add 192.168.1.100/24 dev eth0"
fi

if ip link show eth0 | grep -q "state UP"; then
    echo -e "${GREEN}✓${NC} Ethernet interface is UP"
    
    if command -v ethtool &> /dev/null; then
        LINK=$(ethtool eth0 2>/dev/null | grep "Link detected" | awk '{print $3}')
        if [ "$LINK" = "yes" ]; then
            SPEED=$(ethtool eth0 2>/dev/null | grep "Speed:" | awk '{print $2}')
            echo -e "${GREEN}✓${NC} Physical link detected ($SPEED)"
        else
            echo -e "${RED}✗${NC} No physical link detected"
            echo "   Check: Cable connected? LEDs on ethernet port?"
        fi
    fi
else
    echo -e "${RED}✗${NC} Ethernet interface is DOWN"
    echo "   Fix: sudo ip link set eth0 up"
fi
echo ""

# Test 2: STM32 Reachability
echo "═══ Test 2: STM32 Network Reachability ═══"
if ping -c 2 -W 2 192.168.1.10 &>/dev/null; then
    echo -e "${GREEN}✓${NC} STM32 is reachable at 192.168.1.10"
    
    # Check ARP
    ARP=$(ip neigh show 192.168.1.10 | awk '{print $5, $NF}')
    echo "   MAC/Status: $ARP"
else
    echo -e "${RED}✗${NC} Cannot ping STM32 at 192.168.1.10"
    echo ""
    echo "   ${YELLOW}This means:${NC}"
    echo "   1. STM32 ethernet link is DOWN, OR"
    echo "   2. STM32 IP is not configured correctly, OR"
    echo "   3. STM32 firmware is not running"
    echo ""
    echo "   ${YELLOW}Check on STM32 serial console:${NC}"
    echo "   • Is ethernet PHY link UP?"
    echo "   • Is IP = 192.168.1.10?"
    echo "   • Is gateway = 0.0.0.0 (for direct connection)?"
    echo "   • Is eth_pose thread running?"
fi
echo ""

# Test 3: TCP Server
echo "═══ Test 3: TCP Server Status ═══"
if ss -tln | grep -q ":9000"; then
    echo -e "${GREEN}✓${NC} TCP server is listening on port 9000"
    
    # Check for active connection
    if ss -tn | grep ":9000" | grep -q "192.168.1.10"; then
        echo -e "${GREEN}✓${NC} STM32 is connected!"
        CONN=$(ss -tn | grep ":9000" | grep "192.168.1.10")
        echo "   Connection: $CONN"
    else
        echo -e "${YELLOW}⏳${NC} Server listening, waiting for STM32 to connect..."
    fi
else
    echo -e "${RED}✗${NC} TCP server is NOT listening on port 9000"
    echo "   Fix: cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server && ./monitor_connection.sh"
fi

# Check if process is running
if ps aux | grep -q "[p]ython3.*tcp_server.py"; then
    echo -e "${GREEN}✓${NC} Server process is running"
else
    echo -e "${RED}✗${NC} Server process is NOT running"
fi
echo ""

# Test 4: Recent Logs
echo "═══ Test 4: Server Logs (last 10 lines) ═══"
if [ -f /tmp/tcp_server.log ]; then
    tail -10 /tmp/tcp_server.log
else
    echo -e "${YELLOW}⚠${NC}  No log file at /tmp/tcp_server.log"
fi
echo ""

# Summary and Recommendations
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║                    Troubleshooting Summary                        ║"
echo "╚══════════════════════════════════════════════════════════════════╝"

# Determine the issue
if ping -c 1 -W 1 192.168.1.10 &>/dev/null; then
    if ss -tn | grep ":9000" | grep -q "192.168.1.10"; then
        echo -e "${GREEN}STATUS: Everything looks good! STM32 is connected.${NC}"
    else
        echo -e "${YELLOW}STATUS: STM32 is reachable but not connecting to TCP server${NC}"
        echo ""
        echo "Next steps:"
        echo "1. Check STM32 serial output for connection errors"
        echo "2. Verify STM32 is trying to connect to 192.168.1.100:9000"
        echo "3. Check if STM32 eth_pose thread is running"
    fi
else
    echo -e "${RED}STATUS: STM32 is NOT reachable on the network${NC}"
    echo ""
    echo "Most likely causes:"
    echo "1. ${YELLOW}STM32 ethernet link is DOWN${NC}"
    echo "   → Check ethernet_link_thread() implementation"
    echo "   → Verify PHY initialization"
    echo "   → Try forcing link up: netif_set_link_up(&gnetif)"
    echo ""
    echo "2. ${YELLOW}STM32 IP not configured${NC}"
    echo "   → Verify MX_LWIP_Init() sets IP to 192.168.1.10"
    echo "   → Verify gateway is 0.0.0.0 (not 192.168.1.1)"
    echo ""
    echo "3. ${YELLOW}Cable/hardware issue${NC}"
    echo "   → Try different ethernet cable"
    echo "   → Check LEDs on ethernet jacks"
fi
echo ""

echo "To monitor connection in real-time:"
echo "  ./monitor_connection.sh"
echo ""
echo "To view live server logs:"
echo "  tail -f /tmp/tcp_server.log"
echo ""
