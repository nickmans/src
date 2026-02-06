# STM32 Connection Troubleshooting Guide

## Problem
Pi5 server shows: "Waiting for STM32 connection on 192.168.1.100:9000"  
STM32 is not connecting.

## Root Cause Found

The network diagnostics revealed:
```bash
$ ip neigh show
192.168.1.10 dev eth0 FAILED
```

**The STM32 at 192.168.1.10 is not responding to ARP requests**, which means it's not reachable on the network.

## Fixes Applied

### 1. Changed Server Binding from Specific IP to All Interfaces
**File**: [omni_main.py](omni_main.py)
- Changed `host="192.168.1.100"` → `host="0.0.0.0"`
- This ensures the server accepts connections on all network interfaces
- **Previous**: Bound only to eth0 (192.168.1.100)
- **Now**: Accepts connections on any interface

### 2. Improved Logging
Added clearer messages about what the server is doing and where it's listening.

## Network Configuration

### Pi5 (Server)
- **Interface**: eth0  
- **IP Address**: 192.168.1.100/24
- **Listening Port**: 9000 (on all interfaces: 0.0.0.0)
- **Physical Link**: UP, 100Mb/s

### STM32 (Client - Expected)
- **IP Address**: 192.168.1.10 (from [lwip.c](lwip.c))
- **Target Server**: 192.168.1.100:9000
- **Status**: ⚠️ **NOT REACHABLE** - ARP failed

## Diagnostic Steps

### 1. Run Connection Diagnostics
```bash
cd ~/ros2_ws/src/omni_src/pi_comm_server
./diagnose_stm32_connection.sh
```

This will check:
- Network interface status
- ARP/neighbor table
- Ping connectivity
- Server listening status
- Port accessibility

### 2. Test with Simple Server
Stop the main server and run:
```bash
cd ~/ros2_ws/src/omni_src/pi_comm_server
python3 test_simple_server.py
```

This minimal server will:
- Listen on 0.0.0.0:9000
- Accept any TCP connection
- Echo received data
- Help verify basic connectivity

### 3. Check STM32 Status

The STM32 needs to have:
- ✓ Power ON
- ✓ Ethernet cable connected to Pi5
- ✓ LWIP stack initialized
- ✓ IP configured as 192.168.1.10
- ✓ TCP client trying to connect to 192.168.1.100:9000

#### Verify STM32 Code Configuration

From [lwip.c](lwip.c):
```c
IP_ADDRESS[0] = 192;
IP_ADDRESS[1] = 168;
IP_ADDRESS[2] = 1;
IP_ADDRESS[3] = 10;  // Must be 10
```

From [eth_pose.c](eth_pose.c):
```c
.server_ip = "192.168.1.100",
.server_port = 9000,
```

### 4. Check Physical Connection

```bash
# Check if ethernet link is detected
ethtool eth0 | grep "Link detected"
# Should show: Link detected: yes

# Check link speed
ethtool eth0 | grep "Speed"
# Should show: Speed: 100Mb/s or Speed: 1000Mb/s
```

### 5. Monitor for STM32 Traffic

```bash
# Watch for any packets from STM32
sudo tcpdump -i eth0 -n 'host 192.168.1.10 or port 9000'
```

If you see ARP requests but no responses, the STM32 ethernet stack isn't initialized.

## Common Issues & Solutions

### Issue 1: STM32 Not Reachable (Current Problem)
**Symptom**: `192.168.1.10 dev eth0 FAILED` in neighbor table  
**Causes**:
- STM32 not powered on → Check power LED
- Ethernet not initialized → Check STM32 code calls `MX_LWIP_Init()`
- Wrong IP address → Verify lwip.c configuration  
- Cable issue → Try different cable, check for link LEDs

**Solution**:
1. Check STM32 power and reset
2. Verify LWIP initialization in [main.c](main.c) - Should call `MX_LWIP_Init()` before creating threads
3. Flash STM32 with debug build and check serial output
4. Look for ethernet link LEDs on STM32 board

### Issue 2: Wrong IP Address
**Symptom**: Server accessible but STM32 appears at different IP  
**Solution**:
```bash
# Scan local network
arp -a  # or: ip neigh show
# Look for unknown devices on 192.168.1.x
```

### Issue 3: Firewall Blocking
**Symptom**: Port 9000 not accessible  
**Solution**:
```bash
# Check firewall
sudo iptables -L -n | grep 9000

# If blocked, allow port 9000
sudo iptables -A INPUT -p tcp --dport 9000 -j ACCEPT
```

### Issue 4: Server Not Binding
**Symptom**: Server fails to start  
**Solution**:
```bash
# Check if port already in use
ss -tuln | grep 9000

# Kill any process using port
sudo netstat -tlnp | grep 9000
# Then kill the PID shown
```

## Testing Sequence

### Step 1: Verify Pi5 is Ready
```bash
# Check ethernet is up
ip addr show eth0

# Check server can bind
cd ~/ros2_ws/src/omni_src/pi_comm_server
python3 test_simple_server.py
# Should show: "Server listening on port 9000"
# Press Ctrl+C to stop
```

### Step 2: Verify STM32 is Accessible
```bash
# Ping test
ping -c 5 192.168.1.10

# If ping fails:
# 1. Power cycle STM32
# 2. Check ethernet cable (try swapping)
# 3. Flash STM32 firmware again
# 4. Check for hardware issues
```

### Step 3: Check STM32 Initialization
The STM32 should:
1. Initialize LWIP in `main()` before starting RTOS
2. Start ethernet link thread
3. Call `ETH_POSE_Init()` 
4. Call `ETH_POSE_StartThread()`
5. Attempt TCP connection to 192.168.1.100:9000

### Step 4: Monitor Connection Attempts
```bash
# In one terminal, start server
cd ~/ros2_ws/src/omni_src/pi_comm_server
python3 test_simple_server.py

# In another terminal, monitor traffic
sudo tcpdump -i eth0 -n 'host 192.168.1.10' -v

# Power on STM32 and watch for:
# - ARP requests from Pi5 to 192.168.1.10
# - ARP responses from STM32
# - TCP SYN packets from 192.168.1.10 to 192.168.1.100:9000
```

## Expected Behavior When Working

1. **Pi5 boots**:
   - eth0 comes up at 192.168.1.100
   - Server starts listening on 0.0.0.0:9000

2. **STM32 boots**:
   - LWIP initializes
   - Configures IP as 192.168.1.10
   - Responds to ARP requests
   - Initiates TCP connection to 192.168.1.100:9000

3. **Connection established**:
   - Server logs: "✓ Client connected from ('192.168.1.10', XXXXX)"
   - POSE messages sent at 5 Hz from STM32
   - TRAJ messages sent at 5 Hz to STM32 (when active)

## Next Steps

1. **Power on the STM32 and verify it boots**
   - Check for power LED
   - Check for ethernet link LEDs (usually orange/green)

2. **Run diagnostics**:
   ```bash
   cd ~/ros2_ws/src/omni_src/pi_comm_server
   ./diagnose_stm32_connection.sh
   ```

3. **If STM32 still not reachable**:
   - Connect STM32 via USB/serial
   - Check debug output for LWIP initialization
   - Verify ethernet PHY is detected
   - Check for any error messages

4. **Once STM32 is reachable (ping succeeds)**:
   - Restart the main server:
     ```bash
     cd ~/ros2_ws/src/omni_src/pi_comm_server
     ./start_server_low_resource.sh
     ```
   - STM32 should connect automatically

## Files Modified

- [omni_main.py](omni_main.py) - Changed binding from 192.168.1.100 to 0.0.0.0
- [tcp_server.py](tcp_server.py) - Improved logging messages
- [diagnose_stm32_connection.sh](diagnose_stm32_connection.sh) - New diagnostic script
- [test_simple_server.py](test_simple_server.py) - Minimal test server

## Quick Reference

```bash
# Check if STM32 is reachable
ping 192.168.1.10

# Check server is listening
ss -tuln | grep 9000

# Start simple test server
python3 test_simple_server.py

# Run full diagnostics
./diagnose_stm32_connection.sh

# Monitor network traffic
sudo tcpdump -i eth0 -n 'host 192.168.1.10'

# Start main server
./start_server_low_resource.sh
```
