# OMNI UDP Server - Quick Start Guide

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
sudo systemctl status omni_udp_server.service

# Should show:
#   Active: active (running)
```

**Done!** The server is now running and will auto-start on every boot.

## Boot Service Choice

- Normal boot mode: `omni_udp_server.service`
- Debug-only manual bringup: `omni_ros2_stack.service`

`omni_ros2_stack.service` launches the ROS2 stack directly but does not provide the UDP command endpoint on port `9000`. Leave it disabled for boot unless you are intentionally debugging bringup without the UDP control path.

---

## 📡 Testing with STM32

### Physical Setup
1. Connect ethernet cable between Pi5 and STM32
2. Power on both devices
3. Ensure STM32 is configured with IP **192.168.1.10**

### Monitor Connection

Watch server logs for STM32 connection:
```bash
sudo journalctl -u omni_udp_server.service -f
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
> 1 or start      # Send START_TRAJ
> 2 or stop       # Send STOP_TRAJ
> m circle        # Change simulator motion mode
> help            # Show all commands
> quit            # Exit
```

### STM32 `traj/map` sequence (latest)

Use this command sequence during mapping and execution:

1. Send `traj 1` from STM32 to start mapping flow while manual drive remains enabled on STM32.
2. Send `map 0` from STM32 to finish mapping and switch Pi/ROS2 into localization + trajectory output mode.
3. Send `traj 0` from STM32 to return to manual/standby behavior.

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

## 🤖 Closed-Loop UDP Simulation (CM7-style)

Use this mode when you want to exercise the real UDP ROS2 bridge with a virtual STM32 that:
- receives `TRAJ`
- runs CM7-like control + estimator logic
- feeds wheel commands into simulated encoder measurements
- sends `POSE` back at 5 Hz

### Terminal 1: Start UDP server

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_udp_server.py
```

### Terminal 2: Start virtual STM32

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 virtual_stm32_udp.py --server-host 127.0.0.1 --server-port 9000
```

The simulator automatically sends:
- `CMD_START_RESTART_ROS2` (3)
- `CMD_START_TRAJ` (1)

### Terminal 3: Verify ROS2 data flow

```bash
ros2 topic hz /robot/odom
ros2 topic echo /robot/odom
ros2 topic echo /planned_path
```

### Optional: run simulator as a service

```bash
sudo cp /home/nickolas/ros2_ws/src/omni_src/pi_comm_server/omni_virtual_stm32.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now omni_virtual_stm32.service
sudo systemctl status omni_virtual_stm32.service
```

---

## 🎮 Common Operations

### Service Management

```bash
# Start normal boot service
sudo systemctl start omni_udp_server.service

# Stop normal boot service
sudo systemctl stop omni_udp_server.service

# Restart normal boot service
sudo systemctl restart omni_udp_server.service

# View logs (live)
sudo journalctl -u omni_udp_server.service -f

# View logs (recent)
sudo journalctl -u omni_udp_server.service -n 100

# Check status
sudo systemctl status omni_udp_server.service
```

For direct ROS2 bringup debugging only:

```bash
sudo systemctl start omni_ros2_stack.service
sudo systemctl status omni_ros2_stack.service
```

Do not enable both services for boot.

### Manual Server Start (for debugging)

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server

# Basic start
./start_server.sh

# With debug logging
python3 run_udp_server.py --log-level DEBUG

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

### Safe Shutdown (Pi5 + LIDARs)

```bash
cd /home/nickolas/ros2_ws/src/omni_src

# Interactive confirmation
./pi_comm_server/safe_shutdown.sh

# Non-interactive (no prompt)
./pi_comm_server/safe_shutdown.sh --yes
```

This script safely stops OMNI/ROS2 services first (including the ROS2 stack that drives LIDAR nodes), then powers off the Pi.

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
sudo systemctl status omni_udp_server.service
# Should show: Active: active (running)

# 3. Check port
sudo netstat -ulnp | grep 9000
# Should show: udp ... 0.0.0.0:9000 ...

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
✓ UDP server is listening on port 9000
✓ OMNI UDP server service is running
```

---

## 🛑 Quick Troubleshooting

### Problem: Server won't start

```bash
# Check if port is in use
sudo netstat -ulnp | grep 9000

# Kill existing process
pkill -f run_udp_server.py

# Restart service
sudo systemctl restart omni_udp_server.service
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

# Ensure it starts on every boot
systemctl --user enable omni_ros2_stack.service

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
python3 run_udp_server.py
python3 run_test_client.py

# ✗ Wrong (will fail with import errors)
python3 run_udp_server.py
python3 test_client.py
```

---

## 📚 Next Steps

- **Full documentation**: See [README.md](README.md)
- **Troubleshooting**: See [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **STM32 setup**: See [README.md](README.md)

---

## 📋 Reference

### Network Configuration

| Device | IP Address | Port | Role |
|--------|------------|------|------|
| Raspberry Pi 5 | 192.168.1.100 | 9000 | UDP Server |
| STM32 Nucleo H755 | 192.168.1.10 | - | UDP Client |

### Message Rates

| Message | Rate | Direction |
|---------|------|-----------|
| POSE | 5 Hz | STM32 → Pi |
| TRAJ | 5 Hz | Pi → STM32 |
| CMD | On-demand | STM32 → Pi |

### Frame Conventions

| Signal | Frame in current implementation |
|--------|----------------------------------|
| POSE `x,y,yaw` | `odom` |
| POSE `vx,vy,wz` | `base_link` (body twist) |
| TRAJ `vx_world,vy_world` | world/odom frame |

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
| `run_udp_server.py` | UDP server wrapper (use this!) |
| `run_test_client.py` | Test client wrapper (use this!) |

---

**Need help?** See the full [README.md](README.md) or [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
