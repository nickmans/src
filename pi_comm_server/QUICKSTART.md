# OMNI TCP Server - Quick Start Guide

## Quick Start

### Start Server (on Raspberry Pi)
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
./start_server.sh
```

### Start Test Client (for testing without STM32)
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server

# Basic usage (stationary robot)
./start_test_client.sh

# With circular motion
./start_test_client.sh 192.168.1.100 9000 circle

# With forward motion
./start_test_client.sh 192.168.1.100 9000 forward
```

## Test Sequence

1. **Terminal 1**: Start server
   ```bash
   ./start_server.sh
   ```

2. **Terminal 2**: Start test client
   ```bash
   ./start_test_client.sh 192.168.1.100 9000 circle
   ```

3. **In test client**: Send start command
   ```
   > 1
   ```
   or
   ```
   > start
   ```

4. **Terminal 3**: Monitor ROS2 topics
   ```bash
   # Watch pose updates
   ros2 topic echo /robot/pose
   
   # Watch trajectory
   ros2 topic echo /robot/trajectory
   ```

5. **In test client**: Send stop command
   ```
   > 2
   ```
   or

## Troubleshooting "Files Not Found"

If you get `ModuleNotFoundError` when running `server.py` or `test_client.py` directly:

1. **Use the wrapper scripts instead**: `run_server.py` and `run_test_client.py` handle imports automatically
2. **Or run from the pi_comm_server directory**:
   ```bash
   cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
   python3 server.py
   ```

## Systemd Installation

See the main README.md for detailed systemd setup instructions.
