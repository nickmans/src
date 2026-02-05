# OMNI TCP Server - Setup & Test Instructions

## ✅ Everything is Ready!

I've implemented a complete TCP server system for STM32 ↔ Pi5 communication. Here's how to use it:

---

## 🚀 ONE-COMMAND SETUP

On your Pi5, run:

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./setup_complete.sh
```

This will:
1. Configure static IP (192.168.1.100) on ethernet
2. Install systemd service (auto-starts on boot)
3. Start the server immediately

**That's it!** The server is now running and will auto-start on every boot.

---

## 📡 TESTING CONNECTION

### Test Network Setup
```bash
./test_network.sh
```

### Monitor Server Logs
```bash
sudo journalctl -u omni_tcp_server.service -f
```

When STM32 connects, you'll see:
```
[INFO] Client connected from ('192.168.1.10', XXXXX)
[INFO] Receive loop started
[INFO] Send loop started
```

### Monitor ROS2 Topics
```bash
# Pose data (should be 5 Hz when STM32 connected)
ros2 topic hz /robot/pose

# View pose values
ros2 topic echo /robot/pose
```

---

## 🧪 TEST WITHOUT STM32 (Simulator)

If you want to test before connecting STM32:

```bash
./start_test_client.sh 192.168.1.100 9000 circle
```

Then in the client prompt:
- Type `1` → Send START_TRAJ command
- Type `2` → Send STOP_TRAJ command
- Type `q` → Quit

---

## 📋 FILES CREATED

### Setup & Testing Scripts
- `setup_complete.sh` - One-command setup (IP + service)
- `test_network.sh` - Test network configuration
- `install_service.sh` - Install systemd service
- `uninstall_service.sh` - Remove systemd service
- `start_server.sh` - Manual server start
- `start_test_client.sh` - Test client launcher

### Core Implementation
- `omni_main.py` - Main integration (TCP + ROS2)
- `tcp_server.py` - Multi-threaded TCP server
- `protocol.py` - Binary protocol (updated to spec)
- `ros2_pose_node.py` - ROS2 pose publisher
- `ros2_trajectory_node.py` - ROS2 trajectory generator
- `test_stm32_client.py` - STM32 simulator

### System Service
- `omni_tcp_server.service` - Systemd service file

### Documentation
- `QUICK_START.md` - ⭐ Quick reference guide
- `README_TCP.md` - Complete protocol documentation
- `TEST_GUIDE.md` - Detailed testing procedures
- `ARCHITECTURE.md` - System architecture diagrams
- `IMPLEMENTATION.md` - Implementation summary
- `SETUP_SUMMARY.md` - This file

---

## 🔧 SERVICE COMMANDS

```bash
# View live logs
sudo journalctl -u omni_tcp_server.service -f

# Check status
sudo systemctl status omni_tcp_server.service

# Restart
sudo systemctl restart omni_tcp_server.service

# Stop
sudo systemctl stop omni_tcp_server.service

# Disable auto-start
sudo systemctl disable omni_tcp_server.service
```

---

## 🌐 NETWORK REQUIREMENTS

**Raspberry Pi 5:**
- IP: 192.168.1.100
- Port: 9000 (TCP server)
- Interface: eth0

**STM32 Nucleo H755ZI:**
- IP: 192.168.1.10
- Port: 9000 (TCP client - connects to Pi)

**Connection:**
- Direct ethernet cable between Pi5 and STM32

---

## 📊 PROTOCOL SUMMARY

### Messages

| Type | Name | Size | Direction | Frequency |
|------|------|------|-----------|-----------|
| 1 | POSE | 52 bytes | STM32 → Pi | 5 Hz |
| 10 | TRAJ | 44 bytes | Pi → STM32 | 5 Hz (when active) |
| 20 | CMD | 28 bytes | STM32 → Pi | On demand |

### Commands (CMD Message)

- **1** = START_TRAJ - Start trajectory generation
- **2** = STOP_TRAJ - Stop trajectory generation

### Data Format

- All integers: **little-endian**
- All floats: **IEEE 754 single-precision (32-bit), little-endian**
- Header: 24 bytes (magic, version, type, seq, timestamp, length, crc32)

---

## ✅ VERIFICATION CHECKLIST

After running setup_complete.sh, verify:

```bash
# 1. Service is enabled for auto-start
sudo systemctl is-enabled omni_tcp_server.service
# Expected: enabled

# 2. Service is currently running
sudo systemctl is-active omni_tcp_server.service
# Expected: active

# 3. Pi5 has correct IP
ip addr show eth0 | grep 192.168.1.100
# Should show: inet 192.168.1.100/24

# 4. Server is listening
sudo netstat -tlnp | grep 9000
# Should show: tcp ... 192.168.1.100:9000 ... LISTEN

# 5. Can ping STM32 (once connected)
ping 192.168.1.10
# Should get replies
```

---

## 🔍 TROUBLESHOOTING

### "Cannot reach STM32"
- Check ethernet cable is connected
- Verify STM32 is powered on
- Confirm STM32 has IP 192.168.1.10

### "Service won't start"
```bash
# Check for errors
sudo journalctl -u omni_tcp_server.service -xe

# Common fix: Install dependencies
pip3 install --user rclpy geometry_msgs nav_msgs
```

### "Port already in use"
```bash
# Find what's using port 9000
sudo netstat -tlnp | grep 9000

# Kill the process if needed
sudo kill <PID>
```

---

## 📖 NEXT STEPS

1. **Run setup**: `sudo ./setup_complete.sh`
2. **Test network**: `./test_network.sh`
3. **Connect STM32**: Power on and connect ethernet
4. **Monitor**: `sudo journalctl -u omni_tcp_server.service -f`
5. **Verify**: Check for "Client connected" message

**Need more details?** See:
- [QUICK_START.md](QUICK_START.md) - Quick reference
- [TEST_GUIDE.md](TEST_GUIDE.md) - Detailed testing
- [README_TCP.md](README_TCP.md) - Full documentation

---

## 💡 TIPS

- Server auto-starts on boot - no manual intervention needed
- Logs are in systemd journal: `sudo journalctl -u omni_tcp_server.service`
- ROS2 topics publish when STM32 sends POSE messages
- Trajectory sends only when CMD=1 received from STM32
- Server handles disconnections and reconnections automatically

**You're all set! 🎉**
