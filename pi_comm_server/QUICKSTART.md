# OMNI TCP Server - Quick Start Guide

**Get up and running in 2 minutes!**

---

## 🚀 First Time Setup

### Step 1: Run Automated Setup

On your Raspberry Pi 5:
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./setup_complete.sh
```

This single command:
- ✅ Configures static IP (192.168.1.100) on ethernet
- ✅ Installs systemd service (auto-starts on boot)
- ✅ Starts the server immediately

### Step 2: Verify It's Running

```bash
# Check service status
sudo systemctl status omni_tcp_server.service

# Should show:
#   Active: active (running)
```

**Done!** The server is now running and will auto-start on every boot.

---

## 📡 Testing with STM32

### Physical Setup
1. Connect ethernet cable between Pi5 and STM32
2. Power on both devices
3. Ensure STM32 is configured with IP **192.168.1.10**

### Monitor Connection

Watch server logs for STM32 connection:
```bash
sudo journalctl -u omni_tcp_server.service -f
```

When STM32 connects, you'll see:
```
[INFO] Client connected from ('192.168.1.10', XXXXX)
[INFO] Receive loop started
[INFO] Send loop started
```

### Monitor Robot Data

In another terminal:
```bash
# Watch pose updates (should be 5 Hz)
ros2 topic hz /robot/pose

# View pose values
ros2 topic echo /robot/pose
```

---

## 🧪 Testing WITHOUT STM32 (Simulator)

### Start Test Client

```bash
./start_test_client.sh 192.168.1.100 9000 circle
```

### In Test Client Prompt

Try these commands:
```
> status          # Get server status
> idle false      # Disable idle mode (start trajectory generation)
> idle true       # Enable idle mode (stop trajectory generation)
> help            # Show all commands
> quit            # Exit
```

### Watch ROS2 Topics

In another terminal:
```bash
# Watch pose updates
ros2 topic echo /robot/pose

# Watch trajectory
ros2 topic echo /robot/trajectory

# Check message rates
ros2 topic hz /robot/pose
```

---

## 🎮 Common Operations

### Service Management

```bash
# Start
systemctl --user start omni_pi_server.service

# Stop
systemctl --user stop omni_pi_server.service

# Restart
systemctl --user restart omni_pi_server.service

# View logs (live)
journalctl --user -u omni_pi_server.service -f

# View logs (recent)
journalctl --user -u omni_pi_server.service -n 100

# Check status
systemctl --user status omni_pi_server.service
```

### Manual Server Start (for debugging)

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server

# Basic start
./start_server.sh

# With debug logging
python3 run_server.py --log-level DEBUG

# Low resource mode (if CPU usage is high)
./start_server_low_resource.sh
```

### Network Diagnostics

```bash
# Test complete network setup
./test_network.sh

# Check STM32 connection
./diagnose_stm32_connection.sh

# Monitor resources
./monitor_resources.sh
```

---

## 🏗️ Three-Terminal Test Setup

**Terminal 1: Start Server**
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
./start_server.sh
```

**Terminal 2: Start Test Client**
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
./start_test_client.sh 192.168.1.100 9000 circle
```

**Terminal 3: Monitor ROS2**
```bash
# Watch pose topic
ros2 topic echo /robot/pose

# Or check all topics
ros2 topic list

# Or monitor rates
ros2 topic hz /robot/pose
```

---

## 🔍 Quick Health Check

Run this to verify everything is working:

```bash
# 1. Check static IP
ip addr show eth0
# Should show: 192.168.1.100/24

# 2. Check service
sudo systemctl status omni_tcp_server.service
# Should show: Active: active (running)

# 3. Check port
sudo netstat -tlnp | grep 9000
# Should show: tcp ... 0.0.0.0:9000 ... LISTEN

# 4. Check ROS2 nodes (if ROS2 stack is active)
ros2 node list
# Should show: /pose_publisher, /trajectory_generator

# 5. Check ROS2 topics
ros2 topic list
# Should show: /robot/pose, /robot/trajectory, etc.
```

Or use the automated test:
```bash
./test_network.sh
```

Expected output:
```
✓ Pi5 has IP 192.168.1.100
✓ TCP server is listening on port 9000
✓ OMNI TCP server service is running
```

---

## 🛑 Quick Troubleshooting

### Problem: Server won't start

```bash
# Check if port is in use
sudo netstat -tlnp | grep 9000

# Kill existing process
pkill -f run_server.py

# Restart service
sudo systemctl restart omni_tcp_server.service
```

### Problem: STM32 won't connect

```bash
# Check if STM32 is reachable
ping 192.168.1.10

# Check ARP table
ip neigh show

# Run diagnostics
./diagnose_stm32_connection.sh
```

### Problem: No ROS2 messages

```bash
# Check ROS2 stack is running
systemctl --user status omni_ros2_stack.service

# Restart ROS2 stack
systemctl --user restart omni_ros2_stack.service

# Check nodes
ros2 node list
```

### Problem: High CPU usage

```bash
# Use low resource mode
./start_server_low_resource.sh

# Monitor resources
./monitor_resources.sh
```

### Problem: Import errors

Always use the wrapper scripts:
```bash
# ✓ Correct
python3 run_server.py
python3 run_test_client.py

# ✗ Wrong (will fail with import errors)
python3 server.py
python3 test_client.py
```

---

## 📚 Next Steps

- **Full documentation**: See [README.md](README.md)
- **Troubleshooting**: See [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **STM32 setup**: See [STM32_CLIENT_GUIDE.md](STM32_CLIENT_GUIDE.md)

---

## 📋 Reference

### Network Configuration

| Device | IP Address | Port | Role |
|--------|------------|------|------|
| Raspberry Pi 5 | 192.168.1.100 | 9000 | TCP Server |
| STM32 Nucleo H755 | 192.168.1.10 | - | TCP Client |

### Message Rates

| Message | Rate | Direction |
|---------|------|-----------|
| POSE | 5 Hz | STM32 → Pi |
| TRAJ | 5 Hz | Pi → STM32 |
| CMD | On-demand | STM32 → Pi |

### ROS2 Topics

| Topic | Type | Rate |
|-------|------|------|
| /robot/pose | PoseStamped | 5 Hz |
| /robot/twist | TwistStamped | 5 Hz |
| /robot/odom | Odometry | 5 Hz |
| /robot/trajectory | Path | 5 Hz |
| /initialpose | PoseWithCovarianceStamped | 5 Hz |

### Key Files

| File | Purpose |
|------|---------|
| `setup_complete.sh` | One-command setup script |
| `start_server.sh` | Manual server start |
| `start_test_client.sh` | Test client launcher |
| `test_network.sh` | Network configuration test |
| `run_server.py` | Server wrapper (use this!) |
| `run_test_client.py` | Test client wrapper (use this!) |

---

**Need help?** See the full [README.md](README.md) or [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
