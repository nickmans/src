# OMNI Pi Server — Quick Fixes & Setup

## The Issue: "Files Not Found"

**Solution**: Use the wrapper scripts `run_server.py` and `run_test_client.py` instead of `server.py` and `test_client.py`. These handle Python imports correctly from any directory.

## How to Run (3 Easy Options)

### Option 1: Wrapper Scripts (Recommended — works from anywhere)

```bash
# Terminal 1: Start server
python3 /home/nickolas/ros2_ws/src/omni_src/pi_comm_server/run_server.py

# Terminal 2: Start test client
python3 /home/nickolas/ros2_ws/src/omni_src/pi_comm_server/run_test_client.py
```

### Option 2: From the pi_comm_server Directory

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server

# Terminal 1
python3 run_server.py

# Terminal 2
python3 run_test_client.py
```

### Option 3: Systemd (Production)

```bash
# Setup once
loginctl enable-linger $USER
mkdir -p ~/.config/systemd/user ~/.local/bin
cp omni_pi_server.service ~/.config/systemd/user/
cp omni_ros2_stack.service ~/.config/systemd/user/
cat > ~/.local/bin/omni_pi_server << 'EOF'
#!/bin/bash
cd /home/$USER/ros2_ws/src/omni_src/pi_comm_server
exec python3 run_server.py "$@"
EOF
chmod +x ~/.local/bin/omni_pi_server
systemctl --user daemon-reload

# Then run
systemctl --user start omni_pi_server.service
journalctl --user -u omni_pi_server.service -f
```

## Test Commands

In the test client, try:

```
> status          # Get server status
> idle true       # Enable idle mode
> idle false      # Disable idle mode
> start_ros2      # Start ROS2 stack (requires --enable-ros2-cmds)
> stop_ros2       # Stop ROS2 stack (requires --enable-ros2-cmds)
> help            # Show all commands
> quit            # Disconnect
```

## Files Included

| File | Purpose |
|------|---------|
| `run_server.py` | ✓ Use this to run server (handles imports) |
| `run_test_client.py` | ✓ Use this to run test client (handles imports) |
| `server.py` | Core server (imported by run_server.py) |
| `test_client.py` | Core test client (imported by run_test_client.py) |
| `protocol.py` | Binary protocol (framing, CRC, packing) |
| `planner_stub.py` | Trajectory generation |
| `ros2_manager.py` | ROS2 stack control via systemd |
| `QUICKSTART.md` | Quick reference |
| `README.md` | Full documentation |
| `*.service` | systemd unit files |

## What Works

✓ Server listens on 0.0.0.0:9000  
✓ Test client connects and sends POSE at 5 Hz  
✓ Server generates trajectories asynchronously  
✓ Commands (SET_IDLE, GET_STATUS) work  
✓ ROS2 control via systemd  
✓ Robust binary protocol with CRC  
✓ Runs from any directory (use wrapper scripts)  
✓ Memory-bounded, no leaks  

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `ModuleNotFoundError` | Use `run_server.py` or `run_test_client.py` instead of `server.py` / `test_client.py` |
| Server won't start | Check logs: `python3 run_server.py --log-level DEBUG` |
| Client can't connect | Ensure server is listening: `netstat -tlnp \| grep 9000` |
| Imports still fail | Run from pi_comm_server directory: `cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server && python3 run_server.py` |

---

See [README.md](README.md) for full protocol documentation and configuration details.
