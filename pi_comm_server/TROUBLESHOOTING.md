# OMNI TCP Server - Troubleshooting Guide

Complete guide to diagnosing and fixing common issues with the OMNI TCP server.

---

## Table of Contents

1. [Network & Connection Issues](#network--connection-issues)
2. [Service & Server Issues](#service--server-issues)
3. [STM32 Connection Problems](#stm32-connection-problems)
4. [ROS2 Integration Issues](#ros2-integration-issues)
5. [Performance & Resource Issues](#performance--resource-issues)
6. [Import & Module Errors](#import--module-errors)
7. [Diagnostic Tools](#diagnostic-tools)

---

## Network & Connection Issues

### Server Won't Listen on Port 9000

**Symptoms:**
- Error: "Address already in use"
- Server fails to start

**Diagnosis:**
```bash
# Check if something is using port 9000
sudo netstat -tlnp | grep 9000
```

**Solutions:**

1. **Kill existing process:**
```bash
# Find the process
sudo netstat -tlnp | grep 9000

# Kill it
sudo kill -9 <PID>

# Or kill all python processes running the server
pkill -f run_server.py
```

2. **Restart the service:**
```bash
sudo systemctl restart omni_tcp_server.service
```

3. **Change the port (if needed):**
Edit the server script to use a different port, then restart.

---

### Static IP Not Configured

**Symptoms:**
- Server can't bind to 192.168.1.100
- Network connection fails

**Diagnosis:**
```bash
# Check current IP
ip addr show eth0
```

**Solutions:**

1. **Using NetworkManager (recommended):**
```bash
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.100/24
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con down "Wired connection 1"
sudo nmcli con up "Wired connection 1"
```

2. **Using /etc/network/interfaces (alternative):**
```bash
sudo nano /etc/network/interfaces
```

Add:
```
auto eth0
iface eth0 inet static
    address 192.168.1.100
    netmask 255.255.255.0
```

Restart:
```bash
sudo systemctl restart networking
```

3. **Re-run setup script:**
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./setup_complete.sh
```

---

### Firewall Blocking Connection

**Symptoms:**
- Server is running but STM32 can't connect
- `ping` works but TCP connection fails

**Diagnosis:**
```bash
# Check firewall status
sudo ufw status
```

**Solutions:**

1. **Allow port 9000:**
```bash
sudo ufw allow 9000/tcp
sudo ufw reload
```

2. **Temporary: Disable firewall for testing:**
```bash
sudo ufw disable
```

3. **Re-enable after testing:**
```bash
sudo ufw enable
sudo ufw allow 9000/tcp
```

---

## Service & Server Issues

### Service Won't Start

**Symptoms:**
- `systemctl status` shows "failed" or "inactive (dead)"
- Error logs in journalctl

**Diagnosis:**
```bash
# Check service status
systemctl --user status omni_pi_server.service

# Check system service
sudo systemctl status omni_tcp_server.service

# View recent logs
journalctl --user -u omni_pi_server.service -n 50
# or
sudo journalctl -u omni_tcp_server.service -n 50
```

**Solutions:**

1. **Check for missing dependencies:**
```bash
# Ensure Python 3 is installed
python3 --version

# Check if files exist
ls -la /home/nickolas/ros2_ws/src/omni_src/pi_comm_server/run_server.py
```

2. **Fix permissions:**
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
chmod +x run_server.py
chmod +x start_server.sh
```

3. **Reload systemd:**
```bash
systemctl --user daemon-reload
# or
sudo systemctl daemon-reload
```

4. **Reinstall service:**
```bash
sudo ./uninstall_service.sh
sudo ./install_service.sh
```

---

### Service Stops Unexpectedly

**Symptoms:**
- Service was running, now it's stopped
- Server crashes randomly

**Diagnosis:**
```bash
# Check recent logs for errors
journalctl --user -u omni_pi_server.service -n 200 | less

# Check if process is running
ps aux | grep run_server.py
```

**Solutions:**

1. **Check for Python errors in logs:**
```bash
journalctl --user -u omni_pi_server.service | grep -i "error\|exception\|traceback"
```

2. **Enable auto-restart in service file:**
Edit `~/.config/systemd/user/omni_pi_server.service`:
```ini
[Service]
Restart=always
RestartSec=5
```

Then:
```bash
systemctl --user daemon-reload
systemctl --user restart omni_pi_server.service
```

3. **Run manually to see errors:**
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_server.py --log-level DEBUG
```

---

### Logs Show "Connection Reset" or "Broken Pipe"

**Symptoms:**
- Frequent connection resets
- Client disconnects unexpectedly

**Diagnosis:**
```bash
# Watch logs in real-time
journalctl --user -u omni_pi_server.service -f
```

**Solutions:**

1. **Normal behavior** - This can happen when:
   - STM32 restarts
   - Network cable is unplugged
   - STM32 connection is interrupted

2. **Server should auto-reconnect** - Check logs for:
   ```
   [INFO] Waiting for client connection...
   [INFO] Client connected from ...
   ```

3. **If it doesn't reconnect:**
   - Check STM32 is running and configured correctly
   - Verify network cable is connected
   - Run diagnostics: `./diagnose_stm32_connection.sh`

---

## STM32 Connection Problems

### STM32 Won't Connect

**Symptoms:**
- Server logs show "Waiting for client connection..."
- No connection established

**Diagnosis:**
```bash
# 1. Check Pi5 has correct IP
ip addr show eth0
# Should show: 192.168.1.100/24

# 2. Check server is listening
sudo netstat -tlnp | grep 9000
# Should show: tcp ... 0.0.0.0:9000 ... LISTEN

# 3. Try to ping STM32
ping 192.168.1.10
# Should get replies if STM32 is reachable

# 4. Check ARP table
ip neigh show
# Look for 192.168.1.10 - should NOT show "FAILED"
```

**Solutions:**

1. **Physical layer check:**
   - ✅ Ethernet cable connected?
   - ✅ Link lights on both devices?
   - ✅ Power on both devices?

2. **STM32 network configuration:**
   - Verify STM32 is configured with static IP: **192.168.1.10**
   - Verify STM32 netmask: **255.255.255.0**
   - Check STM32 code is trying to connect to: **192.168.1.100:9000**

3. **Run automated diagnostics:**
```bash
./diagnose_stm32_connection.sh
```

4. **Server binding issue:**
   - Ensure server is listening on `0.0.0.0:9000` (all interfaces)
   - Check server logs for binding address

5. **Test with test client (to rule out STM32 issues):**
```bash
./start_test_client.sh 192.168.1.100 9000 circle
```
If test client connects, the issue is with STM32.

---

### Connection Works Then Drops

**Symptoms:**
- STM32 connects successfully
- After some time, connection drops and doesn't recover

**Diagnosis:**
```bash
# Monitor connection stability
./monitor_connection.sh

# Watch server logs
journalctl --user -u omni_pi_server.service -f
```

**Solutions:**

1. **Check for watchdog timeout:**
   - Server has 1-second watchdog
   - If no POSE received for >1 second, enters idle mode
   - Check STM32 is sending POSE at 5 Hz

2. **Network stability:**
```bash
# Continuous ping test
ping -i 0.2 192.168.1.10

# Should show consistent <1ms latency
```

3. **Check for keepalive settings** in STM32 TCP code
   - Enable TCP keepalive
   - Set appropriate timeout values

---

### ARP Shows "FAILED" for 192.168.1.10

**Symptoms:**
```bash
$ ip neigh show
192.168.1.10 dev eth0 FAILED
```

**Diagnosis:**
- STM32 is not responding to ARP requests
- Either STM32 is not powered or not configured correctly

**Solutions:**

1. **Verify STM32 is powered on and running:**
   - Check LEDs on STM32
   - Connect via ST-Link and check debug output

2. **Verify STM32 Ethernet initialization:**
   - Check LwIP is initialized
   - Check static IP is configured correctly
   - Verify ETH peripheral is enabled

3. **Check STM32 Ethernet cable:**
   - Try a different cable
   - Check link LEDs on STM32's RJ45 jack

4. **STM32 code issues:**
   - Ensure Ethernet/LwIP initialization completes
   - Check for errors in STM32's initialization sequence
   - Verify MAC address is set correctly

---

## ROS2 Integration Issues

### ROS2 Topics Not Publishing

**Symptoms:**
- `ros2 topic list` shows no topics
- `/robot/pose` not available

**Diagnosis:**
```bash
# Check if ROS2 stack service is running
systemctl --user status omni_ros2_stack.service

# List ROS2 nodes
ros2 node list

# List topics
ros2 topic list
```

**Solutions:**

1. **Start ROS2 stack service:**
```bash
systemctl --user start omni_ros2_stack.service
```

2. **Check service logs:**
```bash
journalctl --user -u omni_ros2_stack.service -f
```

3. **Verify ROS2 is installed:**
```bash
# Check ROS2 installation
which ros2

# Source ROS2 (if needed)
source /opt/ros/humble/setup.bash  # or your ROS2 version
```

4. **Manually run ROS2 nodes (for debugging):**
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 ros2_pose_node.py
```

---

### ROS2 Commands (START_ROS2/STOP_ROS2) Don't Work

**Symptoms:**
- Sending START_ROS2 command has no effect

### Pi5 Boot/Shutdown Is Slow

If boot or poweroff takes a long time, the common causes are:
- Services that `After=Wants=network-online.target` (waits for the network “online” target, which can be slow)
- Artificial delays like `ExecStartPre=/bin/sleep 5`
- Long default shutdown timeouts (systemd can wait ~90s per service)

This repo’s service files are tuned to avoid those delays:
- Use `After=network.target` (no online wait)
- Remove `ExecStartPre` sleeps
- Set `KillSignal=SIGINT` and shorter `TimeoutStopSec` so `ros2 launch` exits quickly

After updating service files, apply on the Pi5:
```bash
sudo systemctl daemon-reload
sudo systemctl restart omni_ros2_stack.service omni_udp_server.service
```
- NACK response received

**Diagnosis:**
```bash
# Check if server was started with --enable-ros2-cmds
journalctl --user -u omni_pi_server.service | grep "enable-ros2"
```

**Solutions:**

1. **Server must be started with ROS2 commands enabled:**

Edit service file: `~/.config/systemd/user/omni_pi_server.service`
```ini
ExecStart=/home/nickolas/.local/bin/omni_pi_server --enable-ros2-cmds
```

Then:
```bash
systemctl --user daemon-reload
systemctl --user restart omni_pi_server.service
```

2. **Or run manually with flag:**
```bash
python3 run_server.py --enable-ros2-cmds
```

---

### Topic Frequency is Wrong

**Symptoms:**
- Messages not arriving at 5 Hz
- Irregular message rates

**Diagnosis:**
```bash
# Check actual rate
ros2 topic hz /robot/pose

# Watch for gaps
ros2 topic echo /robot/pose
```

**Solutions:**

1. **Check POSE input rate:**
   - STM32 should send POSE at 5 Hz
   - Use test client to verify: `./start_test_client.sh 192.168.1.100 9000 circle`

2. **Check CPU usage:**
```bash
./monitor_resources.sh
```
High CPU usage can cause timing issues.

3. **Use low resource mode:**
```bash
./start_server_low_resource.sh
```

---

## Performance & Resource Issues

### High CPU Usage

**Symptoms:**
- Server uses >50% CPU
- System becomes slow
- SSH connections lag or fail

**Diagnosis:**
```bash
# Monitor resources
./monitor_resources.sh

# Or manually:
top -p $(pgrep -f run_server.py)
```

**Solutions:**

1. **Use low resource mode (recommended):**
```bash
./start_server_low_resource.sh
```

Configuration changes:
- Process priority: `nice +10` (lower priority)
- ROS2 executor: 5 Hz (instead of 10 Hz)
- Idle thread: 5 second sleep (instead of 1 second)
- Accept loop: 2 second timeout (instead of 1 second)

2. **Service file with low resource mode:**

Edit `~/.config/systemd/user/omni_pi_server.service`:
```ini
[Service]
ExecStart=/usr/bin/nice -n 10 /home/nickolas/.local/bin/omni_pi_server
```

3. **Disable unnecessary ROS2 topics:**
Edit ROS2 nodes to publish only required topics.

---

### SSH Connection Fails When Server Running

**Symptoms:**
- Can't SSH to Pi when server is running
- SSH is very slow or times out

**Root Causes:**
- Server using too much CPU
- SSH daemon starved for resources

**Solutions:**

1. **Use low resource mode** (see above)

2. **Temporarily stop server to SSH:**
```bash
# First, SSH might be barely working - be patient
# Once in:
sudo systemctl stop omni_tcp_server.service
```

3. **Set CPU limits (cgroups):**
```bash
# Add to service file
[Service]
CPUQuota=30%
```

---

### Memory Leaks

**Symptoms:**
- Memory usage grows over time
- Server crashes after running for hours/days

**Diagnosis:**
```bash
# Monitor memory over time
watch -n 5 'ps aux | grep run_server.py'
```

**Solutions:**

1. **Restart service periodically:**
Add to service file:
```ini
[Service]
RuntimeMaxSec=86400  # Restart after 24 hours
```

2. **Check for memory leaks in custom code:**
   - Review trajectory planning code
   - Check for accumulating lists/buffers

3. **Report issue** if memory leaks in core server code

---

## Import & Module Errors

### ModuleNotFoundError: No module named 'protocol'

**Symptoms:**
```
ModuleNotFoundError: No module named 'protocol'
ModuleNotFoundError: No module named 'planner_stub'
```

**Cause:**
Running `server.py` or `test_client.py` directly instead of using wrapper scripts.

**Solutions:**

1. **Always use wrapper scripts:**
```bash
# ✓ Correct
python3 run_server.py
python3 run_test_client.py

# ✗ Wrong
python3 server.py
python3 test_client.py
```

2. **Or run from the pi_comm_server directory:**
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 server.py
```

3. **Add to PYTHONPATH (if needed):**
```bash
export PYTHONPATH="/home/nickolas/ros2_ws/src/omni_src/pi_comm_server:$PYTHONPATH"
python3 server.py
```

---

### ImportError: Cannot import ROS2 modules

**Symptoms:**
```
ModuleNotFoundError: No module named 'rclpy'
```

**Cause:**
ROS2 not installed or not sourced.

**Solutions:**

1. **Install ROS2:**
```bash
# Check if ROS2 is installed
which ros2

# If not, install ROS2 (example for Humble)
sudo apt install ros-humble-desktop
```

2. **Source ROS2 setup:**
```bash
source /opt/ros/humble/setup.bash
```

3. **Add to bashrc for persistence:**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

## Diagnostic Tools

### Automated Network Test

```bash
./test_network.sh
```

Checks:
- ✓ Pi5 has IP 192.168.1.100
- ✓ STM32 is reachable at 192.168.1.10
- ✓ TCP server is listening on port 9000
- ✓ Server service is running

---

### STM32 Connection Diagnostics

```bash
./diagnose_stm32_connection.sh
```

Checks:
- Network interface status
- IP configuration
- ARP table
- Server listening status
- Ping connectivity
- Link status

---

### Connection Monitor

```bash
./monitor_connection.sh
```

Continuously monitors:
- Service status
- Connection status
- Network reachability
- Log messages

---

### Resource Monitor

```bash
./monitor_resources.sh
```

Shows real-time:
- CPU usage
- Memory usage
- Thread count
- Process status

---

### Manual Diagnostics

```bash
# Network configuration
ip addr show
ip link show
ip route show
ip neigh show

# Server status
sudo netstat -tlnp | grep 9000
ps aux | grep run_server.py

# Service status
systemctl --user status omni_pi_server.service
systemctl --user status omni_ros2_stack.service

# ROS2 status
ros2 node list
ros2 topic list
ros2 topic hz /robot/pose

# Logs
journalctl --user -u omni_pi_server.service -f
journalctl --user -u omni_ros2_stack.service -f

# Test connectivity
ping 192.168.1.10
telnet 192.168.1.100 9000
```

---

## Getting Help

### Collect Debug Information

When reporting an issue, collect this info:

```bash
# System info
uname -a
python3 --version
ros2 --version  # if using ROS2

# Network config
ip addr show
ip neigh show

# Service status
systemctl --user status omni_pi_server.service

# Recent logs
journalctl --user -u omni_pi_server.service -n 200 > server_logs.txt

# Network test results
./test_network.sh > network_test.txt

# STM32 diagnostics
./diagnose_stm32_connection.sh > stm32_diag.txt
```

---

## Common Error Messages

| Error Message | Cause | Solution |
|--------------|-------|----------|
| "Address already in use" | Port 9000 in use | Kill existing process, restart service |
| "Connection refused" | Server not running | Start service |
| "No route to host" | Network misconfigured | Check IP, check cable |
| "FAILED" in ARP table | STM32 not responding | Check STM32 power, config |
| "ModuleNotFoundError" | Wrong Python script used | Use wrapper scripts |
| "Permission denied" | Firewall blocking | Allow port 9000 |
| "Broken pipe" | Connection dropped | Normal - wait for reconnect |

---

## Still Having Issues?

1. **Review logs carefully:**
   ```bash
   journalctl --user -u omni_pi_server.service -n 500 | less
   ```

2. **Run server manually with debug logging:**
   ```bash
   python3 run_server.py --log-level DEBUG
   ```

3. **Test with simulator first:**
   ```bash
   ./start_test_client.sh 192.168.1.100 9000 circle
   ```
   If simulator works, issue is likely with STM32.

4. **Check all files exist:**
   ```bash
   ls -la /home/nickolas/ros2_ws/src/omni_src/pi_comm_server/
   ```

5. **Reinstall from scratch:**
   ```bash
   sudo ./uninstall_service.sh
   sudo ./setup_complete.sh
   ```

---

**See also:**
- [README.md](README.md) - Full documentation
- [QUICKSTART.md](QUICKSTART.md) - Quick start guide
- [STM32_CLIENT_GUIDE.md](STM32_CLIENT_GUIDE.md) - STM32 implementation
