#!/bin/bash
# Complete setup script for OMNI TCP Server

set -e

echo "======================================================"
echo "  OMNI TCP Server - Complete Setup"
echo "======================================================"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "This script must be run as root (use sudo)"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Step 1: Configure Static IP on Ethernet"
echo "-----------------------------------"
echo "Current ethernet status:"
ip addr show eth0 | grep -E "inet " || echo "No IP on eth0"
echo ""

read -p "Configure eth0 with static IP 192.168.1.100? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    # Check if NetworkManager is available
    if command -v nmcli &>/dev/null; then
        echo "Using NetworkManager..."
        
        # Find ethernet connection name
        ETH_CON=$(nmcli -t -f NAME,DEVICE con show | grep eth0 | cut -d: -f1 | head -1)
        
        if [ -z "$ETH_CON" ]; then
            echo "Creating new ethernet connection..."
            ETH_CON="OMNI-Ethernet"
            nmcli con add type ethernet con-name "$ETH_CON" ifname eth0
        fi
        
        echo "Configuring $ETH_CON..."
        nmcli con mod "$ETH_CON" ipv4.addresses 192.168.1.100/24
        nmcli con mod "$ETH_CON" ipv4.method manual
        nmcli con mod "$ETH_CON" connection.autoconnect yes
        nmcli con up "$ETH_CON"
        
        echo "✓ Static IP configured"
        
    else
        echo "NetworkManager not found. Using /etc/network/interfaces..."
        
        # Backup existing config
        cp /etc/network/interfaces /etc/network/interfaces.backup
        
        # Add static IP configuration
        cat >> /etc/network/interfaces <<EOF

# OMNI TCP Server Configuration
auto eth0
iface eth0 inet static
    address 192.168.1.100
    netmask 255.255.255.0
EOF
        
        # Restart networking
        systemctl restart networking
        
        echo "✓ Static IP configured"
    fi
    
    # Verify
    echo ""
    echo "Current IP on eth0:"
    ip addr show eth0 | grep "inet "
else
    echo "Skipping IP configuration"
fi

echo ""
echo "Step 2: Install System Service"
echo "-----------------------------------"
read -p "Install OMNI TCP server as system service? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    ./install_service.sh
else
    echo "Skipping service installation"
fi

echo ""
echo "Step 3: Start Service Now"
echo "-----------------------------------"
read -p "Start the service immediately? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    systemctl start omni_tcp_server.service
    sleep 2
    echo ""
    systemctl status omni_tcp_server.service --no-pager -l | head -15
else
    echo "Skipping service start"
fi

echo ""
echo "======================================================"
echo "  Setup Complete!"
echo "======================================================"
echo ""
echo "Network Status:"
ip addr show eth0 | grep "inet " || echo "No IP on eth0"
echo ""

echo "Service Status:"
systemctl is-active omni_tcp_server.service && echo "✓ Service is running" || echo "✗ Service not running"
systemctl is-enabled omni_tcp_server.service && echo "✓ Will start on boot" || echo "✗ Will not start on boot"
echo ""

echo "Next Steps:"
echo "1. Connect STM32 via Ethernet cable"
echo "2. Power on STM32 (should have IP 192.168.1.10)"
echo "3. Monitor logs: sudo journalctl -u omni_tcp_server.service -f"
echo "4. Test connection: ping 192.168.1.10"
echo ""
echo "Service Commands:"
echo "  View logs:  sudo journalctl -u omni_tcp_server.service -f"
echo "  Stop:       sudo systemctl stop omni_tcp_server.service"
echo "  Restart:    sudo systemctl restart omni_tcp_server.service"
echo "  Status:     sudo systemctl status omni_tcp_server.service"
echo ""
