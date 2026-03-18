#!/bin/bash
# Monitor resource usage of OMNI server
# Usage: ./monitor_resources.sh

echo "==============================================="
echo "OMNI Server Resource Monitor"
echo "==============================================="
echo

# Find Python processes related to omni
OMNI_PIDS=$(pgrep -f "run_udp_server.py|udp_server.py")

if [ -z "$OMNI_PIDS" ]; then
    echo "No OMNI server processes found"
    exit 0
fi

echo "OMNI Server Processes:"
echo "----------------------"
ps aux | head -1
for pid in $OMNI_PIDS; do
    ps aux | grep "^.*\s${pid}\s"
done
echo

# Show detailed CPU and memory for each PID
echo "Detailed Process Info:"
echo "----------------------"
for pid in $OMNI_PIDS; do
    echo "Process $pid:"
    top -b -n 1 -p $pid | tail -n 2
    echo "  Threads: $(ps -T -p $pid | wc -l)"
    echo
done

# Show system load
echo "System Load:"
echo "------------"
uptime
echo

# Show available memory
echo "Memory Status:"
echo "--------------"
free -h
echo

# Show network connections
echo "Network Connections:"
echo "--------------------"
netstat -unp 2>/dev/null | grep -E "9000|ssh|22" | head -10

echo
echo "==============================================="
echo "Press Ctrl+C to stop monitoring (updates every 5s)"
echo "==============================================="
echo

# Continuous monitoring
while true; do
    clear
    echo "=== OMNI Server Resource Monitor ($(date)) ==="
    echo
    
    # CPU and Memory summary
    for pid in $OMNI_PIDS; do
        if ps -p $pid > /dev/null 2>&1; then
            ps -p $pid -o pid,pcpu,pmem,rss,vsz,cmd | tail -1
        fi
    done
    
    echo
    echo "System: $(uptime | awk -F'load average:' '{print "Load:" $2}')"
    echo "Memory: $(free -h | grep Mem | awk '{print "Used: " $3 "/" $2 " (" $3/$2*100 "%)"}')"
    
    sleep 5
done
