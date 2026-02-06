# OMNI Robot TCP Communication System

Complete Python TCP server implementation for receiving robot POSE data from STM32 Nucleo H755ZI and sending trajectory commands back, with full ROS2 integration.

## System Overview

```
┌──────────────┐                          ┌─────────────────┐
│   STM32      │  TCP Socket (5 Hz)       │  Raspberry Pi 5 │
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

## Network Configuration

- **Pi5 IP**: 192.168.1.100
- **Pi5 Port**: 9000 (TCP server)
- **STM32 IP**: 192.168.1.10

## Protocol Specification

### Data Format
- All integers: **little-endian**
- All floats: **IEEE 754 single-precision (32-bit), little-endian**

### Message Header (24 bytes, all messages)

| Offset | Size | Type   | Name        | Description                          |
|--------|------|--------|-------------|--------------------------------------|
| 0-3    | 4    | uint32 | magic       | 0x4F4D4E49 ('OMNI')                  |
| 4-5    | 2    | uint16 | version     | Protocol version = 1                 |
| 6-7    | 2    | uint16 | msg_type    | 1=POSE, 10=TRAJ, 20=CMD              |
| 8-11   | 4    | uint32 | seq         | Sequence number                      |
| 12-15  | 4    | uint32 | t_ms        | Timestamp in milliseconds            |
| 16-19  | 4    | uint32 | payload_len | Payload size in bytes                |
| 20-23  | 4    | uint32 | crc32       | CRC32 (set to 0 if unused)           |

### POSE Message (Type=1, 52 bytes total)

**Frequency**: 5 Hz from STM32

**Payload** (28 bytes):

| Offset | Size | Type   | Name      | Description                    |
|--------|------|--------|-----------|--------------------------------|
| 24-27  | 4    | uint32 | pose_t_ms | Pose timestamp (ms)            |
| 28-31  | 4    | float  | x         | Position x (meters)            |
| 32-35  | 4    | float  | y         | Position y (meters)            |
| 36-39  | 4    | float  | yaw       | Orientation (radians, [-π, π]) |
| 40-43  | 4    | float  | vx        | Velocity x (m/s, world frame)  |
| 44-47  | 4    | float  | vy        | Velocity y (m/s, world frame)  |
| 48-51  | 4    | float  | wz        | Yaw rate (rad/s)               |

### TRAJ Message (Type=10, 44 bytes total)

**Frequency**: 5 Hz when trajectory mode active (Pi → STM32)

**Payload** (20 bytes):

| Offset | Size | Type  | Name     | Description                       |
|--------|------|-------|----------|-----------------------------------|
| 24-27  | 4    | float | x_des    | Desired position x (meters)       |
| 28-31  | 4    | float | y_des    | Desired position y (meters)       |
| 32-35  | 4    | float | yaw_des  | Desired orientation (radians)     |
| 36-39  | 4    | float | vx_world | Feedforward velocity x (m/s)      |
| 40-43  | 4    | float | vy_world | Feedforward velocity y (m/s)      |

### CMD Message (Type=20, 28 bytes total)

**Direction**: STM32 → Pi

**Payload** (4 bytes):

| Offset | Size | Type   | Name    | Description                        |
|--------|------|--------|---------|------------------------------------|
| 24-27  | 4    | uint32 | command | 1=START_TRAJ, 2=STOP_TRAJ          |

## File Structure

```
pi_comm_server/
├── protocol.py              # Binary protocol definitions
├── tcp_server.py            # TCP server with threading
├── ros2_pose_node.py        # ROS2 pose publisher
├── ros2_trajectory_node.py  # ROS2 trajectory generator
├── omni_main.py            # Main integration script
├── test_stm32_client.py    # Test client (simulates STM32)
└── README_TCP.md           # This file
```

## Component Description

### 1. `protocol.py`
- Message type definitions
- Binary pack/unpack functions for all message types
- Stream parser for robust TCP message handling
- CRC32 validation support

### 2. `tcp_server.py`
- TCP server listening on 192.168.1.100:9000
- Multi-threaded design:
  - Accept thread: Handles client connections
  - Receive thread: Parses incoming POSE/CMD messages
  - Send thread: Sends TRAJ messages at 5 Hz
- Thread-safe pose storage
- Connection recovery handling

### 3. `ros2_pose_node.py`
- ROS2 node for publishing robot pose
- **Published topics**:
  - `/robot/pose` (PoseStamped)
  - `/robot/twist` (TwistStamped)
  - `/robot/odom` (Odometry)
  - `/initialpose` (PoseWithCovarianceStamped)

### 4. `ros2_trajectory_node.py`
- ROS2 node for trajectory generation
- **Modes**:
  - `hold`: Hold current position
  - `waypoint`: Move to target waypoint
  - `circle`: Follow circular trajectory
- **Parameters**:
  - `trajectory_mode`: Mode selection
  - `waypoint_x`, `waypoint_y`, `waypoint_yaw`: Waypoint target
  - `circle_radius`, `circle_speed`: Circle parameters
  - `max_velocity`: Maximum velocity limit
- **Published topics**:
  - `/robot/trajectory` (Path) - for visualization

### 5. `omni_main.py`
- Main integration script
- Combines TCP server + ROS2 nodes
- Handles CMD messages:
  - `CMD=1`: Start trajectory generation node
  - `CMD=2`: Stop trajectory generation node
- Coordinates between TCP and ROS2

### 6. `test_stm32_client.py`
- Simulates STM32 for testing
- Sends POSE messages at 5 Hz
- Motion modes: stationary, forward, circle
- Interactive commands to send START/STOP

## Installation & Setup

### Prerequisites
```bash
# ROS2 (Humble or later)
sudo apt install ros-humble-desktop

# Python dependencies
pip3 install --user rclpy geometry_msgs nav_msgs std_srvs
```

### Running the System

#### 1. Start the OMNI System (on Raspberry Pi)
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 omni_main.py
```

**Expected output**:
```
============================================================
OMNI Robot Communication System
============================================================
2026-02-05 10:00:00 [INFO] OMNI System initialized
2026-02-05 10:00:00 [INFO] Server listening on 192.168.1.100:9000
2026-02-05 10:00:00 [INFO] Pose Publisher Node initialized
2026-02-05 10:00:00 [INFO] System running. Press Ctrl+C to stop.
2026-02-05 10:00:00 [INFO] Waiting for STM32 connection on 192.168.1.100:9000...
```

#### 2. Test with Simulator (for development/testing)
```bash
# In another terminal
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 test_stm32_client.py --host 192.168.1.100 --port 9000 --motion circle
```

**Interactive commands**:
- `1` or `start` - Send START_TRAJ command
- `2` or `stop` - Send STOP_TRAJ command
- `m <mode>` - Set motion mode (stationary/forward/circle)
- `q` or `quit` - Exit

#### 3. Monitor ROS2 Topics
```bash
# View pose data
ros2 topic echo /robot/pose

# View odometry
ros2 topic echo /robot/odom

# View trajectory (when active)
ros2 topic echo /robot/trajectory

# List all topics
ros2 topic list
```

#### 4. Visualize in RViz
```bash
ros2 run rviz2 rviz2
```

**Add displays**:
- Fixed Frame: `odom`
- Add → By topic → `/robot/pose` → PoseStamped
- Add → By topic → `/robot/trajectory` → Path
- Add → By topic → `/robot/odom` → Odometry

## Operation Flow

### Normal Operation Sequence

1. **System Startup**
   - Run `omni_main.py`
   - TCP server starts listening
   - ROS2 pose publisher node starts

2. **STM32 Connection**
   - STM32 connects to 192.168.1.100:9000
   - Connection established

3. **Pose Data Flow**
   - STM32 sends POSE at 5 Hz
   - Server receives and stores latest pose
   - ROS2 node publishes to `/robot/pose`, `/robot/odom`

4. **Start Trajectory** (via CMD=1)
   - STM32 sends CMD message with command=1
   - Server starts trajectory generator node
   - Server begins sending TRAJ messages at 5 Hz
   - STM32 receives trajectory setpoints

5. **Stop Trajectory** (via CMD=2)
   - STM32 sends CMD message with command=2
   - Server stops trajectory generator node
   - Server stops sending TRAJ messages

## Configuration

### Change IP/Port
Edit `omni_main.py`:
```python
system = OMNISystem(host="192.168.1.100", port=9000)
```

### Change Trajectory Mode
The trajectory node supports runtime parameter changes:
```bash
ros2 param set /trajectory_generator trajectory_mode waypoint
ros2 param set /trajectory_generator waypoint_x 2.0
ros2 param set /trajectory_generator waypoint_y 1.5
ros2 param set /trajectory_generator max_velocity 0.8
```

### Enable Debug Logging
Edit `omni_main.py`:
```python
logging.basicConfig(
    level=logging.DEBUG,  # Change from INFO to DEBUG
    ...
)
```

## Troubleshooting

### Server won't start
- Check if port 9000 is already in use: `sudo netstat -tlnp | grep 9000`
- Try different port or kill existing process

### STM32 can't connect
- Verify network connectivity: `ping 192.168.1.10`
- Check firewall: `sudo ufw status`
- Ensure both devices on same subnet

### No POSE messages received
- Check STM32 is sending data
- Verify protocol format matches specification
- Enable DEBUG logging to see raw data

### TRAJ messages not sent
- Ensure trajectory mode is active (send CMD=1)
- Check trajectory node is running
- Verify pose data is being received

### ROS2 topics not publishing
- Check ROS2 installation: `ros2 topic list`
- Verify node is running: `ros2 node list`
- Check for errors in ROS2 executor thread

## Performance Notes

- **Message Rate**: 5 Hz (200ms interval)
- **Latency**: Typically <10ms for pose processing
- **Thread Safety**: All pose/trajectory data access is protected by locks
- **Connection Recovery**: Automatic reconnection on disconnect

## Development & Extension

### Adding New Message Types
1. Update `MessageType` enum in `protocol.py`
2. Add message class with `pack()` and `unpack()` methods
3. Update parser in `tcp_server.py`

### Custom Trajectory Generator
1. Modify `ros2_trajectory_node.py`
2. Add new mode in `_generate_*` methods
3. Update parameters as needed

### Integration with Navigation Stack
The pose publisher is compatible with Nav2:
```bash
# Use /robot/odom for odometry input
# Use /initialpose for initial localization
```

## Python Binary Packing Reference

```python
import struct

# Little-endian packing
struct.pack('<I', value)      # uint32
struct.pack('<H', value)      # uint16  
struct.pack('<f', value)      # float32
struct.pack('<Iffffff', ...)  # Multiple values

# Little-endian unpacking
struct.unpack('<I', bytes)    # Returns tuple: (value,)
struct.unpack('<fffff', bytes) # Returns: (f1, f2, f3, f4, f5)
```

## License

This software is provided for the OMNI robot project.

## Support

For issues or questions, check the logs:
- System log: `/tmp/omni_system.log`
- Console output for real-time debugging
