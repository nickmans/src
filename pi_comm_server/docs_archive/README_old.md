# OMNI Pi Communication Server

Production-ready TCP communication server for OMNI robot stack. Runs on Raspberry Pi (Ubuntu 24.04) and communicates with STM32 NUCLEO H755 for real-time control.

## Features

- **Binary Protocol**: Robust, little-endian framed messages with optional CRC32 validation
- **Asynchronous I/O**: asyncio-based architecture with dedicated RX/TX tasks; no blocking
- **Latest-Wins Trajectory Planning**: If new POSE arrives before planner finishes, old job is cancelled and only newest POSE is processed
- **ROS2 Integration**: Control ROS2 stack via systemd --user services (safe, non-blocking)
- **Watchdog**: Auto-idle if no POSE received for >1 second
- **Robust Parsing**: Handles partial reads, resynchronizes on bad magic, validates payload sizes
- **Type-Safe**: Type hints, dataclasses, structured logging

## Architecture

```
STM32 (TCP Client)  <--TCP/IP-->  Raspberry Pi (TCP Server)
   sends POSE at 5 Hz                |
   receives TRAJ every 200ms        |-- rx_loop: parse POSE, trigger traj planning
   sends CMD on demand               |-- tx_loop: send TRAJ, ACK, STATUS
                                    |-- watchdog_loop: detect timeout
                                    |-- planner task: async trajectory generation
                                    |-- ROS2 manager: systemd --user control
```

## Protocol Overview

### Framing (Header + Payload)

```
Header (24 bytes, little-endian):
  uint32 magic = 0x4F4D4E49  ('OMNI')
  uint16 version = 1
  uint16 msg_type
  uint32 seq
  uint32 t_ms           (sender timestamp in ms)
  uint32 payload_len    (bytes)
  uint32 crc32          (0 = skip validation)

Payload:
  <variable, msg_type-specific>
```

### Message Types

| Type | Value | Direction | Purpose |
|------|-------|-----------|---------|
| POSE | 1 | STM32→Pi | Robot pose + velocity @ 5 Hz |
| CMD | 2 | STM32→Pi | Command (SET_IDLE, START/STOP ROS2, etc.) |
| EVENT | 3 | STM32→Pi | Optional event notification |
| TRAJ | 10 | Pi→STM32 | Trajectory knots for interpolation |
| CORR | 11 | Pi→STM32 | Optional small correction |
| ACK | 12 | Pi→STM32 | Command accepted |
| NACK | 13 | Pi→STM32 | Command rejected |
| STATUS | 15 | Pi→STM32 | Server status (reply to GET_STATUS) |

### Command IDs

| ID | Name | Arg | Purpose |
|----|------|-----|---------|
| 1 | SET_IDLE | "true"/"false" or empty | Toggle/set idle mode |
| 2 | START_ROS2 | optional | Start ROS2 stack |
| 3 | STOP_ROS2 | optional | Stop ROS2 stack |
| 4 | GET_STATUS | optional | Request server status |

## Installation

### 1. Setup on Raspberry Pi (Ubuntu 24.04)

```bash
# Clone / navigate to pi_comm_server directory
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server

# Create a virtual environment (optional but recommended)
python3 -m venv venv
source venv/bin/activate

# No external dependencies needed (uses only stdlib: asyncio, struct, zlib, logging)
```

### 2. Install systemd --user Services

Enable linger so user services persist after logout:

```bash
loginctl enable-linger $USER
```

Install the server service:

```bash
mkdir -p ~/.config/systemd/user
cp omni_pi_server.service ~/.config/systemd/user/
cp omni_ros2_stack.service ~/.config/systemd/user/

# Create wrapper script for server
mkdir -p ~/.local/bin
cat > ~/.local/bin/omni_pi_server << 'EOF'
#!/bin/bash
cd /home/$USER/ros2_ws/src/omni_src/pi_comm_server
exec python3 server.py "$@"
EOF
chmod +x ~/.local/bin/omni_pi_server

# Reload systemd
systemctl --user daemon-reload
```

Customize `omni_ros2_stack.service` for your ROS2 launch command:

```bash
# Edit the service file
nano ~/.config/systemd/user/omni_ros2_stack.service
```

### 3. Run Server

#### Option A: systemd --user (Recommended)

```bash
# Start server
systemctl --user start omni_pi_server.service

# Check status
systemctl --user status omni_pi_server.service

# View logs
journalctl --user -u omni_pi_server.service -f
```

#### Option B: Direct (Development)

```bash
python3 server.py --host 0.0.0.0 --port 9000 --log-level DEBUG
```

## Usage

### Server Command-Line Options

```bash
python3 server.py \
  --host 0.0.0.0              # Bind address (default: 0.0.0.0)
  --port 9000                 # Bind port (default: 9000)
  --enable-ros2-cmds          # Allow START/STOP ROS2 commands (default: disabled)
  --log-level DEBUG           # Logging level (default: INFO)
```

### Test Client (STM32 Simulator)

```bash
python3 test_client.py --host 127.0.0.1 --port 9000 --log-level INFO
```

Interactive commands:

```
> idle true         # Set idle mode ON
> idle false        # Set idle mode OFF
> idle              # Toggle idle mode
> start_ros2        # Start ROS2 stack (requires --enable-ros2-cmds on server)
> stop_ros2         # Stop ROS2 stack (requires --enable-ros2-cmds on server)
> status            # Get server status
> help              # List commands
> quit              # Disconnect
```

## Trajectory Format

TRAJ payload (after header):

```
uint32 reply_to_pose_seq      # Which POSE this traj responds to
uint32 traj_t0_ms            # Trajectory time origin (ms)
uint16 N                      # Number of knots
uint16 flags                  # Bit 0: idle_traj, Bit 1: has_vel
float32 dt                    # Time step between knots (default 0.05 s = 20 Hz)
[N × (6×float32)]             # Knots: x, y, yaw, vx, vy, wz
```

**Example**: 1.2 second horizon @ dt=0.05 → 24 knots

STM32 receives knots and interpolates to 100 Hz (dt=0.01).

## State Machine

```
Server State:
  idle_mode: bool             # If true, trajectory holds position
  ros2_running: bool          # Tracks ROS2 stack status
  last_pose_seq: int          # Last received POSE sequence
  last_pose_latency_ms: float # Latency from POSE timestamp to rx time
  last_traj_seq: int          # Last sent TRAJ sequence

Event Flow:
  1. STM32 sends POSE @ 5 Hz
  2. Server receives, updates state, logs latency
  3. Async trajectory planner triggered (latest-wins)
  4. Planner generates knots (takes ~10ms, non-blocking)
  5. TRAJ enqueued for transmission
  6. TX loop drains queue, sends TRAJ
  7. STM32 receives TRAJ, interpolates to 100 Hz control
  8. If no POSE for >1s: idle_mode auto-set to true → hold position
```

## Configuration

### ROS2 Stack Service

Edit `omni_ros2_stack.service`:

```ini
[Service]
ExecStart=bash -c 'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch omni_traj omni_bringup.launch.py'
```

Customize the `ros2 launch` command to match your stack. Example alternatives:

```bash
# Option 1: Launch single node
ExecStart=bash -c 'source /opt/ros/humble/setup.bash && ros2 run omni_traj waypoint_traj_node'

# Option 2: Launch file with parameters
ExecStart=bash -c 'source /opt/ros/humble/setup.bash && ros2 launch my_stack main.launch.py param1:=value1'
```

### Server Parameters

Key constants in `server.py`:

```python
watchdog_timeout_sec = 1.0    # Idle if no POSE for this long
tx_queue.maxsize = 100        # Max queued messages
trajectory horizon = 1.2 s    # Default trajectory duration
trajectory dt = 0.05 s        # Knot spacing (20 Hz)
```

## Protocol Details

### POSE Payload

```
uint32 pose_t_ms       # Timestamp (ms) when pose was captured
float32 x, y, yaw      # Position and orientation (meters, radians)
float32 vx, vy, wz     # Velocities (m/s, rad/s)
```

### CMD Payload

```
uint16 cmd_id          # Command ID (1-4)
uint16 arg_len         # Argument length (bytes)
uint8[arg_len] arg     # UTF-8 argument (optional, can be empty)
```

### STATUS Payload

```
uint32 status_t_ms              # Server time (ms)
uint8  idle                     # 0/1
uint8  ros2_running             # 0/1
uint16 reserved                 # Padding
uint32 last_pose_seq            # Last POSE sequence
uint32 last_traj_seq            # Last TRAJ sequence
float32 last_pose_latency_ms    # Latency (ms)
```

## Troubleshooting

### Service won't start

```bash
# Check logs
journalctl --user -u omni_pi_server.service -n 50

# Ensure linger is enabled
loginctl show-user $USER | grep Linger

# Test script directly
python3 server.py --log-level DEBUG
```

### Test client can't connect

```bash
# Ensure server is listening
netstat -tlnp | grep 9000

# Check firewall (if any)
sudo ufw allow 9000/tcp

# Connect from client
telnet 127.0.0.1 9000
```

### Trajectory not being sent

```bash
# Enable DEBUG logging
systemctl --user stop omni_pi_server.service
python3 server.py --log-level DEBUG

# Check for POSE messages in logs
journalctl --user -u omni_pi_server.service | grep POSE
```

### ROS2 commands not working

Ensure server is started with `--enable-ros2-cmds`:

```bash
# Update service file or start with flag
python3 server.py --enable-ros2-cmds --log-level DEBUG
```

## Performance Notes

- **RX loop**: Non-blocking, frame-agnostic. Parses at whatever rate data arrives.
- **TX loop**: Drains queue at ~1-100 Hz depending on traffic (not tight-looped).
- **Trajectory planning**: Async, takes ~10 ms on Pi 5. Never blocks RX or TX.
- **Watchdog**: Runs every 0.2 s to check idle timeout.
- **Memory**: ~5-10 MB resident, no growth over time (queues have maxsize).

## Testing

### Unit Tests (Quick Validation)

```bash
# Test protocol packing/unpacking
python3 -c "
from protocol import Pose, Header, make_message, MessageType
p = Pose(pose_t_ms=1000, x=1.0, y=2.0, yaw=0.5, vx=0.1, vy=0.2, wz=0.05)
msg = make_message(MessageType.POSE, 0, p.pack())
print(f'Message size: {len(msg)} bytes')
"

# Test trajectory planning
python3 -c "
from planner_stub import PoseState, make_traj_from_pose
pose = PoseState(x=0, y=0, yaw=0, vx=1, vy=0, wz=0, t_ms=0)
knots = make_traj_from_pose(pose, idle=False)
print(f'Generated {len(knots)} knots')
"
```

### Integration Test

```bash
# Terminal 1: Start server
python3 server.py --enable-ros2-cmds --log-level INFO

# Terminal 2: Run test client
python3 test_client.py

# Observe logs for POSE→TRAJ flow
```

## Files

| File | Purpose |
|------|---------|
| `server.py` | Main asyncio server (RX/TX/planner) |
| `protocol.py` | Binary protocol (framing, CRC, packing) |
| `planner_stub.py` | Trajectory generation from pose |
| `ros2_manager.py` | systemd --user ROS2 stack control |
| `test_client.py` | STM32 simulator for testing |
| `omni_pi_server.service` | systemd service for server |
| `omni_ros2_stack.service` | systemd service for ROS2 stack |
| `README.md` | This file |

## License

All code provided as-is for use in OMNI robot project.

## Contact / Support

For issues or questions, check logs and enable `--log-level DEBUG` for detailed diagnostics.
