# OMNI TCP Server Testing Guide

## Initial Setup & Installation

### 1. Install as System Service (Auto-start on Boot)

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server

# Make installation script executable
chmod +x install_service.sh uninstall_service.sh

# Install the service (requires sudo)
sudo ./install_service.sh
```

**Expected Output:**
```
======================================================
  OMNI TCP Server - Service Installation
======================================================

Installing service...
  Source: /home/nickolas/ros2_ws/src/omni_src/pi_comm_server/omni_tcp_server.service
  Target: /etc/systemd/system/omni_tcp_server.service

✓ Service file copied
✓ Systemd reloaded
✓ Service enabled (will start on boot)

======================================================
  Installation Complete!
======================================================
```

### 2. Start the Service Immediately

```bash
# Start the service now (don't wait for reboot)
sudo systemctl start omni_tcp_server.service

# Check if it's running
sudo systemctl status omni_tcp_server.service
```

**Expected Status:**
```
● omni_tcp_server.service - OMNI TCP Server for STM32 Communication
     Loaded: loaded (/etc/systemd/system/omni_tcp_server.service; enabled)
     Active: active (running) since Wed 2026-02-05 XX:XX:XX
   Main PID: XXXXX (python3)
```

### 3. View Live Logs

```bash
# Watch the server logs in real-time
sudo journalctl -u omni_tcp_server.service -f
```

**Expected Log Output:**
```
Feb 05 XX:XX:XX systemd[1]: Started OMNI TCP Server for STM32 Communication.
Feb 05 XX:XX:XX python3[XXXXX]: ============================================================
Feb 05 XX:XX:XX python3[XXXXX]: OMNI Robot Communication System
Feb 05 XX:XX:XX python3[XXXXX]: ============================================================
Feb 05 XX:XX:XX python3[XXXXX]: [INFO] OMNI System initialized
Feb 05 XX:XX:XX python3[XXXXX]: [INFO] Server listening on 192.168.1.100:9000
Feb 05 XX:XX:XX python3[XXXXX]: [INFO] Pose Publisher Node initialized
Feb 05 XX:XX:XX python3[XXXXX]: [INFO] Waiting for STM32 connection on 192.168.1.100:9000...
```

## Testing with STM32

### Network Configuration Check

#### On Pi5:
```bash
# Check IP address
ip addr show

# You should see 192.168.1.100 on eth0 or similar
# Example:
# eth0: inet 192.168.1.100/24

# Check if server is listening
sudo netstat -tlnp | grep 9000

# Expected:
# tcp  0  0  192.168.1.100:9000  0.0.0.0:*  LISTEN  XXXXX/python3

# Ping STM32
ping 192.168.1.10
```

#### On STM32 (or test machine):
```bash
# Ping Pi5
ping 192.168.1.100

# Try to connect with netcat (if available)
nc 192.168.1.100 9000
```

### Test Sequence

#### Option 1: With STM32 Hardware

1. **Connect STM32 via Ethernet**
   - STM32 should be configured with IP: 192.168.1.10
   - Connect ethernet cable directly between Pi5 and STM32

2. **Power on STM32**
   - STM32 firmware should connect to 192.168.1.100:9000

3. **Monitor Connection on Pi5**
   ```bash
   # Watch for connection message
   sudo journalctl -u omni_tcp_server.service -f
   ```
   
   **Expected:**
   ```
   [INFO] Client connected from ('192.168.1.10', XXXXX)
   [INFO] Receive loop started
   [INFO] Send loop started
   ```

4. **Monitor POSE Messages**
   ```bash
   # In another terminal, watch ROS2 topics
   ros2 topic echo /robot/pose
   ```
   
   **Expected:** Pose messages at 5 Hz

5. **Test Trajectory Start/Stop**
   - STM32 sends CMD=1 → Server starts trajectory node
   - STM32 sends CMD=2 → Server stops trajectory node

#### Option 2: With Test Client Simulator

1. **Keep server running** (via systemd or manually)

2. **Run test client in another terminal**
   ```bash
   cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
   python3 test_stm32_client.py --host 192.168.1.100 --port 9000 --motion circle
   ```

3. **In test client prompt:**
   ```
   > 1         # Start trajectory generation
   > 2         # Stop trajectory generation
   > m forward # Change motion mode
   > q         # Quit
   ```

4. **Monitor in another terminal:**
   ```bash
   # Watch server logs
   sudo journalctl -u omni_tcp_server.service -f
   
   # Watch ROS2 pose topic
   ros2 topic echo /robot/pose
   
   # Watch ROS2 trajectory topic (when active)
   ros2 topic echo /robot/trajectory
   ```

## Service Management Commands

```bash
# Start service
sudo systemctl start omni_tcp_server.service

# Stop service
sudo systemctl stop omni_tcp_server.service

# Restart service
sudo systemctl restart omni_tcp_server.service

# Check status
sudo systemctl status omni_tcp_server.service

# View logs (last 50 lines)
sudo journalctl -u omni_tcp_server.service -n 50

# View logs (follow/tail)
sudo journalctl -u omni_tcp_server.service -f

# View logs since boot
sudo journalctl -u omni_tcp_server.service -b

# Disable auto-start
sudo systemctl disable omni_tcp_server.service

# Re-enable auto-start
sudo systemctl enable omni_tcp_server.service
```

## Troubleshooting

### Service Won't Start

```bash
# Check service status
sudo systemctl status omni_tcp_server.service

# View detailed logs
sudo journalctl -u omni_tcp_server.service -xe

# Common issues:
# 1. Port already in use
sudo netstat -tlnp | grep 9000
# Kill existing process if needed

# 2. Python dependencies missing
pip3 install --user rclpy geometry_msgs nav_msgs

# 3. ROS2 not installed
source /opt/ros/humble/setup.bash  # or iron/jazzy
```

### No Connection from STM32

```bash
# 1. Check network connectivity
ping 192.168.1.10

# 2. Check firewall
sudo ufw status
sudo ufw allow 9000/tcp  # If firewall is active

# 3. Check server is listening
sudo netstat -tlnp | grep 9000

# 4. Verify IP configuration
ip addr show
# Make sure Pi5 has 192.168.1.100

# 5. Test with netcat from another machine
nc -v 192.168.1.100 9000
```

### No POSE Messages

```bash
# 1. Check if STM32 is sending data
sudo tcpdump -i eth0 port 9000 -X

# 2. Enable debug logging
# Edit /home/nickolas/ros2_ws/src/omni_src/pi_comm_server/omni_main.py
# Change: level=logging.INFO → level=logging.DEBUG
# Then restart: sudo systemctl restart omni_tcp_server.service

# 3. Check protocol format
# Verify STM32 firmware matches protocol specification
```

### ROS2 Topics Not Publishing

```bash
# 1. Check if ROS2 is sourced
source /opt/ros/humble/setup.bash

# 2. List nodes
ros2 node list

# 3. List topics
ros2 topic list

# 4. Check topic info
ros2 topic info /robot/pose

# 5. If no topics, check service logs
sudo journalctl -u omni_tcp_server.service -f
```

## Testing Checklist

- [ ] Service installed successfully
- [ ] Service starts on boot
- [ ] Server listening on 192.168.1.100:9000
- [ ] Pi5 can ping STM32 (192.168.1.10)
- [ ] STM32 can connect to server
- [ ] POSE messages received at 5 Hz
- [ ] ROS2 topics publishing (/robot/pose, /robot/odom)
- [ ] CMD=1 starts trajectory generation
- [ ] TRAJ messages sent at 5 Hz when active
- [ ] CMD=2 stops trajectory generation
- [ ] Service auto-restarts on failure
- [ ] Logs viewable via journalctl

## Reboot Test

```bash
# 1. Verify service is enabled
sudo systemctl is-enabled omni_tcp_server.service
# Should output: enabled

# 2. Reboot Pi5
sudo reboot

# 3. After reboot, check if service started automatically
sudo systemctl status omni_tcp_server.service

# 4. View logs from boot
sudo journalctl -u omni_tcp_server.service -b
```

## Performance Monitoring

```bash
# CPU and memory usage
top -p $(pgrep -f omni_main.py)

# Network statistics
sudo iftop -i eth0

# Message rate check
ros2 topic hz /robot/pose
# Expected: ~5.0 Hz

# Latency check (compare STM32 timestamp to receive time in logs)
sudo journalctl -u omni_tcp_server.service -f
```

## Uninstall Service

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./uninstall_service.sh
```

## Configuration Changes

If you need to modify the service configuration:

```bash
# 1. Edit service file
sudo nano /etc/systemd/system/omni_tcp_server.service

# 2. Reload systemd
sudo systemctl daemon-reload

# 3. Restart service
sudo systemctl restart omni_tcp_server.service
```

## Production Deployment Checklist

- [ ] Static IP configured on Pi5 (192.168.1.100)
- [ ] Service installed and enabled
- [ ] Service starts on boot verified
- [ ] Firewall configured (if applicable)
- [ ] Logs rotation configured
- [ ] Network redundancy tested
- [ ] Error recovery tested (disconnect/reconnect)
- [ ] Documentation accessible to team
- [ ] Backup configuration saved
