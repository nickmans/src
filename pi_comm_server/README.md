# OMNI Pi Communication Server

Production-ready UDP communication server for OMNI robot stack. Runs on Raspberry Pi 5 (Ubuntu 24.04) and communicates with STM32 NUCLEO H755 for real-time control.

---

## Table of Contents
- [Features](#features)
- [System Architecture](#system-architecture)
- [Protocol Specification](#protocol-specification)
- [Quick Start](#quick-start)
- [Installation](#installation)
- [Usage](#usage)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [File Structure](#file-structure)

---

## Features

- **Binary Protocol**: Robust, little-endian framed messages with optional CRC32 validation
- **Asynchronous I/O**: asyncio-based architecture with dedicated RX/TX tasks; no blocking
- **Latest-Wins Trajectory Planning**: New POSE cancels old planning jobs; only newest processed
- **ROS2 Integration**: Control ROS2 stack via systemd --user services (safe, non-blocking)
- **Watchdog**: Auto-idle if no POSE received for >1 second
- **Robust Parsing**: Handles partial reads, resynchronizes on bad magic, validates payload sizes
- **Type-Safe**: Type hints, dataclasses, structured logging

## Boot Mode

- `omni_udp_server.service` is the intended production boot service.
- It owns UDP port `9000`, receives STM32 commands, publishes pose into ROS2, and starts or switches the ROS2 stack as needed.
- `omni_ros2_stack.service` is a debug-only standalone bringup service. It launches the ROS2 stack directly and does not provide the UDP command endpoint.
- Do not enable both services on boot. They conflict by design and whichever starts later will displace the other.

---

## System Architecture

```
┌──────────────┐                          ┌─────────────────┐
│   STM32      │  UDP (5 Hz)       │  Raspberry Pi 5 │
│   Nucleo     │◄────────────────────────►│                 │
│   H755ZI     │  POSE / CMD / TRAJ       │  192.168.1.100  │
│              │                          │     Port 9000   │
│ 192.168.1.10 │                          └─────────────────┘
└──────────────┘                                    │
                                                    │
                                          ┌─────────▼──────────┐
                                          │   ROS2 Integration │
                                          │   - Pose Publisher │
                                          │   - Traj Generator │
                                          └────────────────────┘
```

### Data Flow

**STM32 → Pi (5 Hz):**
- POSE messages: Robot pose and twist
    - Pose: `x,y,yaw` interpreted in `odom` frame
    - Twist: `vx,vy,wz` interpreted in `base_link` (body) frame
- CMD messages: Commands (`START_TRAJ`, `STOP_TRAJ`, `START_RESTART_ROS2`, mapping mode commands)

**Pi → STM32 (5 Hz):**
- TRAJ messages: Trajectory knots for interpolation
- ACK/NACK: Command acknowledgments
- STATUS: Server status reports

---

## Protocol Specification

### Message Header (24 bytes, little-endian)

| Offset | Size | Type   | Name        | Description                    |
|--------|------|--------|-------------|--------------------------------|
| 0-3    | 4    | uint32 | magic       | 0x4F4D4E49 ('OMNI')           |
| 4-5    | 2    | uint16 | version     | Protocol version = 1           |
| 6-7    | 2    | uint16 | msg_type    | Message type (see below)       |
| 8-11   | 4    | uint32 | seq         | Sequence number                |
| 12-15  | 4    | uint32 | t_ms        | Timestamp (milliseconds)       |
| 16-19  | 4    | uint32 | payload_len | Payload size in bytes          |
| 20-23  | 4    | uint32 | crc32       | CRC32 (0 = skip validation)    |

### Message Types

| Type | Value | Direction | Payload Size | Purpose |
|------|-------|-----------|--------------|---------|
| POSE | 1 | STM32→Pi | 28 bytes | Robot pose + velocity @ 5 Hz |
| EVENT | 3 | STM32→Pi | Variable | Optional event notification |
| TRAJ | 10 | Pi→STM32 | Variable | Trajectory header + knot array |
| CORR | 11 | Pi→STM32 | Variable | Optional small correction |
| ACK | 12 | Pi→STM32 | Variable | Legacy command ack (optional) |
| NACK | 13 | Pi→STM32 | Variable | Legacy command reject (optional) |
| STATUS | 15 | Pi→STM32 | Variable | Server status |
| CMD | 20 | STM32↔Pi | Variable | Commands + command acknowledgments |

### POSE Message (Total: 52 bytes)

**Header:** 24 bytes  
**Payload:** 28 bytes

| Offset | Size | Type   | Name      | Description                    |
|--------|------|--------|-----------|--------------------------------|
| 24-27  | 4    | uint32 | pose_t_ms | Pose timestamp (ms)            |
| 28-31  | 4    | float  | x         | Position x (meters, odom frame) |
| 32-35  | 4    | float  | y         | Position y (meters, odom frame) |
| 36-39  | 4    | float  | yaw       | Orientation (radians, [-π, π]) |
| 40-43  | 4    | float  | vx        | Linear x (m/s, base_link/body frame) |
| 44-47  | 4    | float  | vy        | Linear y (m/s, base_link/body frame) |
| 48-51  | 4    | float  | wz        | Angular z (rad/s, base_link/body frame) |

### TRAJ Message (Variable Length)

**Header:** 24 bytes  
**Payload:** `20 + (n_knots * 20)` bytes

Trajectory payload starts with a 20-byte trajectory header, followed by `n_knots` knot entries.

**Trajectory payload header (20 bytes):**

| Offset | Size | Type   | Name              | Description |
|--------|------|--------|-------------------|-------------|
| 24-27  | 4    | uint32 | reply_to_pose_seq | POSE seq this trajectory corresponds to |
| 28-31  | 4    | uint32 | traj_t0_ms        | Trajectory start timestamp (ms) |
| 32-33  | 2    | uint16 | n_knots           | Number of knots |
| 34-35  | 2    | uint16 | flags             | Trajectory flags |
| 36-39  | 4    | uint32 | reserved          | Reserved |
| 40-43  | 4    | float  | dt                | Knot spacing (seconds) |

**Per-knot entry (20 bytes each):**

| Size | Type  | Name | Description |
|------|-------|------|-------------|
| 4    | float | x    | Desired x (m) |
| 4    | float | y    | Desired y (m) |
| 4    | float | yaw  | Desired yaw (rad) |
| 4    | float | vx   | Feedforward x velocity (m/s, world/odom) |
| 4    | float | vy   | Feedforward y velocity (m/s, world/odom) |

### Command IDs

| ID | Name | Argument | Purpose |
|----|------|----------|---------|
| 0 | STOP_ROS2 | optional | Stop ROS2 stack |
| 1 | START_TRAJ | optional | Start mapping-mode trajectory generation flow |
| 2 | STOP_TRAJ | optional | Stop trajectory generation/output (standby) |
| 3 | START_RESTART_ROS2 | optional | Start/restart ROS2 stack |
| 4 | SHUTDOWN_PI5 | optional | Safe Pi shutdown command |
| 5 | START_MAPPING | optional | Switch to mapping mode |
| 6 | FINISH_MAPPING | optional | Finish mapping and switch to localization/frozen map |
| 7 | USE_LIVE_MAP | optional | Use live mapping mode |
| 8 | USE_FROZEN_MAP | optional | Use frozen map localization mode |

---

## Quick Start

### One-Command Setup (First Time)

On your Pi5, run the automated setup:

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./setup_complete.sh
```

This will:
- ✅ Configure static IP (192.168.1.100) on eth0
- ✅ Install systemd service for auto-start on boot
- ✅ Start the server immediately

**That's it!** The server will now start automatically on every boot.

### Verify Installation

```bash
# Check service status
sudo systemctl status omni_udp_server.service

# View live logs
sudo journalctl -u omni_udp_server.service -f

# Test network
./test_network.sh
```

### Testing Without STM32 (Simulator)

```bash
# Start test client with circular motion
./start_test_client.sh 192.168.1.100 9000 circle

# In another terminal, monitor ROS2 topics
ros2 topic echo /robot/pose
ros2 topic hz /robot/pose
```

---

## Installation

### Method 1: Automated Setup (Recommended)

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./setup_complete.sh
```

### Method 2: Manual Setup

#### 1. Configure Static IP on Pi5

Using NetworkManager:
```bash
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.100/24
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con down "Wired connection 1"
sudo nmcli con up "Wired connection 1"
```

Verify:
```bash
ip addr show eth0
# Should show: inet 192.168.1.100/24
```

#### 2. Install systemd Service

```bash
# Enable linger (services persist after logout)
loginctl enable-linger $USER

# Create user service directories
mkdir -p ~/.config/systemd/user
mkdir -p ~/.local/bin

# Install UDP service only for normal boot
cp omni_pi_server.service ~/.config/systemd/user/

# Create wrapper script
cat > ~/.local/bin/omni_pi_server << 'EOF'
#!/bin/bash
cd /home/$USER/ros2_ws/src/omni_src/pi_comm_server
exec python3 run_udp_server.py "$@"
EOF
chmod +x ~/.local/bin/omni_pi_server

# Reload and enable service
systemctl --user daemon-reload
systemctl --user enable omni_pi_server.service
systemctl --user start omni_pi_server.service
```

#### 3. Verify Service

```bash
systemctl --user status omni_pi_server.service
journalctl --user -u omni_pi_server.service -f
```

`omni_ros2_stack.service` should be treated as debug-only/manual bringup and left disabled for boot.

---

## Usage

### Service Management

```bash
# Start server
systemctl --user start omni_pi_server.service

# Stop server
systemctl --user stop omni_pi_server.service

# Restart server
systemctl --user restart omni_pi_server.service

# View status
systemctl --user status omni_pi_server.service

# View logs (live)
journalctl --user -u omni_pi_server.service -f

# View logs (recent)
journalctl --user -u omni_pi_server.service -n 100
```

### Manual Execution

```bash
# From pi_comm_server directory
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server

# Basic
./start_server.sh

# Low resource mode (reduced CPU usage)
./start_server_low_resource.sh

# Direct wrapper invocation
python3 run_udp_server.py
```

### ROS2 Integration

Monitor ROS2 topics:
```bash
# Check active topics
ros2 topic list

# Watch pose updates (5 Hz)
ros2 topic echo /robot/pose

# Check message rate
ros2 topic hz /robot/pose

# View trajectory
ros2 topic echo /robot/trajectory

# View odometry
ros2 topic echo /robot/odom
```

### STM32 command workflow (latest)

Current CM7 + Pi sequence:

1. `traj 1` on STM32: Pi enters mapping mode and pauses trajectory output while STM32 remains in manual driving mode.
2. `map 0` on STM32: Pi finalizes mapping, switches to localization/frozen-map mode, then resumes trajectory output.
3. `traj 0` on STM32: Pi stops trajectory output and returns to standby behavior.

Use this order for "manual drive while mapping, then follow only received trajectory" operation.

### Testing Connection

```bash
# Test network configuration
./test_network.sh

# Monitor connection
./monitor_connection.sh

# Check STM32 connectivity
./diagnose_stm32_connection.sh

# Monitor resource usage
./monitor_resources.sh
```

---

## Testing

### Test Client (Simulates STM32)

Start the test client:
```bash
# Stationary robot
./start_test_client.sh

# Circular motion
./start_test_client.sh 192.168.1.100 9000 circle

# Forward motion
./start_test_client.sh 192.168.1.100 9000 forward
```

Interactive commands in test client:
```
> 1 or start      # Send START_TRAJ
> 2 or stop       # Send STOP_TRAJ
> m <mode>        # Change simulator motion mode (stationary/forward/circle)
> help            # Show all commands
> quit            # Disconnect
```

### Expected Behavior

**When STM32 connects:**
```
[INFO] Client connected from ('192.168.1.10', XXXXX)
[INFO] Receive loop started
[INFO] Send loop started
```

**ROS2 topics should be active:**
```bash
$ ros2 topic hz /robot/pose
average rate: 5.000
```

### Multi-Terminal Test Sequence

**Terminal 1: Server**
```bash
./start_server.sh
```

**Terminal 2: Test Client**
```bash
./start_test_client.sh 192.168.1.100 9000 circle
> 1  # Start trajectory generation
```

**Terminal 3: Monitor ROS2**
```bash
ros2 topic echo /robot/pose
```

**Terminal 4: Monitor Logs**
```bash
sudo journalctl -u omni_udp_server.service -f
```

---

## Troubleshooting

### Server Won't Start

**Check if port is already in use:**
```bash
sudo netstat -ulnp | grep 9000
```

**Kill existing process:**
```bash
pkill -f run_udp_server.py
```

**Check logs:**
```bash
journalctl --user -u omni_pi_server.service -n 50
```

### STM32 Won't Connect

**1. Check physical connection:**
- Ethernet cable connected?
- Link lights on both devices?

**2. Verify network configuration:**
```bash
# Pi5 should have 192.168.1.100
ip addr show eth0

# Test STM32 reachability
ping 192.168.1.10

# Check ARP table
ip neigh show
```

**3. Check server is listening:**
```bash
sudo netstat -ulnp | grep 9000
# Should show: 0.0.0.0:9000 or 192.168.1.100:9000
```

**4. Run diagnostics:**
```bash
./diagnose_stm32_connection.sh
```

### High CPU Usage / SSH Issues

If the server uses too much CPU and affects SSH:

**Use low resource mode:**
```bash
./start_server_low_resource.sh
```

**Monitor resource usage:**
```bash
./monitor_resources.sh
```

**Optimizations applied:**
- Process priority lowered (`nice +10`)
- ROS2 executor reduced to 5 Hz
- Idle thread sleeps for 5 seconds
- Accept loop timeout increased to 2 seconds

### Module Import Errors

**Problem:** `ModuleNotFoundError: No module named 'protocol'`

**Solution:** Always use wrapper scripts:
```bash
# ✓ Use these
python3 run_udp_server.py
python3 run_test_client.py

# ✗ Don't use these directly
python3 run_udp_server.py
python3 test_client.py
```

Or run from the pi_comm_server directory:
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_udp_server.py
```

### ROS2 Topics Not Publishing

**Check if ROS2 nodes are running:**
```bash
ros2 node list
```

**Restart ROS2 stack:**
```bash
systemctl --user restart omni_ros2_stack.service
```

**Check ROS2 logs:**
```bash
journalctl --user -u omni_ros2_stack.service -f
```

### Network Issues

**Reset network configuration:**
```bash
sudo nmcli con down "Wired connection 1"
sudo nmcli con up "Wired connection 1"
```

**Restart networking:**
```bash
sudo systemctl restart NetworkManager
```

**Check firewall:**
```bash
# Disable firewall temporarily for testing
sudo ufw disable

# Or allow specific port
sudo ufw allow 9000/udp
```

---

## File Structure

### Core Implementation
```
pi_comm_server/
├── protocol.py              # Binary protocol definitions & parsing
├── udp_server.py             # Asyncio UDP server (main logic)
├── planner_stub.py          # Trajectory generation
├── ros2_manager.py          # ROS2 stack control via systemd
├── test_client.py           # STM32 simulator
│
├── run_udp_server.py            # ✓ Wrapper script for server
├── run_test_client.py       # ✓ Wrapper script for test client
│
├── udp_server.py            # Multi-threaded UDP server (alternative)
├── ros2_pose_node.py        # ROS2 pose publisher
├── ros2_trajectory_node.py  # ROS2 trajectory generator
└── run_udp_server.py            # Main integration (UDP + ROS2)
```

### Setup & Testing
```
├── setup_complete.sh        # One-command setup (IP + service)
├── install_service.sh       # Install systemd service
├── uninstall_service.sh     # Remove systemd service
│
├── start_server.sh          # Manual server start
├── start_server_low_resource.sh  # Low CPU usage mode
├── start_test_client.sh     # Test client launcher
│
├── test_network.sh          # Test network configuration
├── diagnose_connection.sh   # Connection diagnostics
├── diagnose_stm32_connection.sh  # STM32-specific diagnostics
├── monitor_connection.sh    # Monitor connection status
└── monitor_resources.sh     # Monitor CPU/memory usage
```

### System Services
```
├── omni_pi_server.service   # Main server service
├── omni_ros2_stack.service  # ROS2 nodes service
└── omni_udp_server.service  # UDP service
```

### Documentation
```
├── README.md                # This file
├── QUICKSTART.md           # Quick reference
├── TROUBLESHOOTING.md      # Detailed troubleshooting
└── README.md   # STM32 implementation guide
```

---

## Advanced Topics

### Custom Trajectory Planning

Edit [planner_stub.py](planner_stub.py):
```python
def plan_trajectory(pose: PoseMsg, dt=0.05, horizon=1.2):
    # Your custom planning logic here
    knots = []
    # ... generate trajectory knots
    return knots
```

### Protocol Extensions

Add new message types in [protocol.py](protocol.py):
```python
class MsgType(IntEnum):
    POSE = 1
    CMD = 2
    YOUR_NEW_TYPE = 20  # Add your type
```

### ROS2 Topic Customization

Modify topics in [ros2_pose_node.py](ros2_pose_node.py) or [ros2_trajectory_node.py](ros2_trajectory_node.py).

---

## Network Configuration Summary

| Device | IP Address | Port | Role |
|--------|------------|------|------|
| Raspberry Pi 5 | 192.168.1.100 | 9000 | UDP Server |
| STM32 Nucleo H755 | 192.168.1.10 | - | UDP Client |

**Connection Type:** Direct Ethernet (no router needed)  
**Protocol:** UDP with binary framing  
**Data Rate:** 5 Hz (bidirectional)

---

## Support & Debugging

### Enable Debug Logging

```bash
python3 run_udp_server.py --log-level DEBUG
```

### Log Files

When running as service, logs go to systemd journal:
```bash
journalctl --user -u omni_pi_server.service --since today
```

### Common Diagnostics

```bash
# Network status
ip addr show
ip neigh show
sudo netstat -ulnp | grep 9000

# Service status
systemctl --user status omni_pi_server.service
systemctl --user status omni_ros2_stack.service

# Process list
ps aux | grep python3

# Resource usage
top -p $(pgrep -f run_udp_server.py)
```

---

## License

[Add your license here]

## Authors

[Add authorship info here]

## Version History

- v1.0: Initial production release
  - Binary protocol implementation
  - Asyncio UDP server
  - ROS2 integration
  - Systemd service support
