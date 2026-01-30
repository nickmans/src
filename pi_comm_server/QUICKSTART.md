# Quick Start Guide

## Running the Server and Test Client

### Option 1: Using Wrapper Scripts (Recommended - works from any directory)

```bash
# Start server (from any directory)
python3 /home/nickolas/ros2_ws/src/omni_src/pi_comm_server/run_server.py --log-level INFO

# In another terminal, start test client
python3 /home/nickolas/ros2_ws/src/omni_src/pi_comm_server/run_test_client.py
```

### Option 2: From the pi_comm_server Directory

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server

# Start server
python3 run_server.py --log-level INFO

# In another terminal (from same directory)
python3 run_test_client.py
```

### Option 3: Direct Script Execution (requires CWD to be pi_comm_server directory)

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server

# Start server
python3 server.py --log-level INFO

# In another terminal
python3 test_client.py
```

## Test Commands

Once the test client is running, try:

```
> status
> idle true
> status
> idle false
> quit
```

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
