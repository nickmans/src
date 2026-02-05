#!/bin/bash
# Monitor STM32 connection status in realtime

echo "=================================="
echo "STM32 Connection Monitor"
echo "=================================="
echo ""

while true; do
    clear
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           STM32 TCP Connection Status Monitor                 ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo ""
    
    # Check Pi ethernet interface
    echo "📡 Pi5 Ethernet Status:"
    ip addr show eth0 | grep "inet " | awk '{print "   IP Address: "$2}'
    if command -v ethtool &> /dev/null; then
        LINK=$(ethtool eth0 2>/dev/null | grep "Link detected" | awk '{print $3}')
        SPEED=$(ethtool eth0 2>/dev/null | grep "Speed:" | awk '{print $2}')
        echo "   Link: $LINK ($SPEED)"
    else
        ip link show eth0 | grep -q "state UP" && echo "   Link: yes" || echo "   Link: no"
    fi
    echo ""
    
    # Check STM32 reachability
    echo "🤖 STM32 Status (192.168.1.10):"
    if ping -c 1 -W 1 192.168.1.10 &>/dev/null; then
        echo "   ✅ REACHABLE"
    else
        echo "   ❌ NOT REACHABLE"
    fi
    
    # Check ARP
    ARP_STATUS=$(ip neigh show 192.168.1.10 2>/dev/null | awk '{print $NF}')
    if [ -n "$ARP_STATUS" ]; then
        if [ "$ARP_STATUS" = "REACHABLE" ] || [ "$ARP_STATUS" = "STALE" ]; then
            MAC=$(ip neigh show 192.168.1.10 | awk '{print $5}')
            echo "   MAC: $MAC ($ARP_STATUS)"
        else
            echo "   ARP: $ARP_STATUS"
        fi
    else
        echo "   ARP: No entry"
    fi
    echo ""
    
    # Check TCP server
    echo "🔌 TCP Server (port 9000):"
    if ss -tln | grep -q ":9000"; then
        echo "   ✅ LISTENING"
        
        # Check for active connection
        CONN=$(ss -tn | grep ":9000" | grep "192.168.1.10")
        if [ -n "$CONN" ]; then
            echo "   ✅ STM32 CONNECTED"
            echo "   Connection: $CONN"
        else
            echo "   ⏳ Waiting for STM32 connection..."
        fi
    else
        echo "   ❌ NOT LISTENING"
    fi
    echo ""
    
    # Show recent server logs
    echo "📋 Recent Server Logs (last 8 lines):"
    if [ -f /tmp/tcp_server.log ]; then
        tail -8 /tmp/tcp_server.log | sed 's/^/   /'
    else
        echo "   No log file found"
    fi
    echo ""
    
    echo "Press Ctrl+C to exit | Refreshing every 2 seconds..."
    sleep 2
done
