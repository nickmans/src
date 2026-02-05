# OMNI TCP Server - Quick Start

## 🚀 Complete Setup (First Time)

Run the automated setup script on your Pi5:

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./setup_complete.sh
```

This will:
- ✅ Configure static IP (192.168.1.100) on eth0
- ✅ Install systemd service for auto-start on boot
- ✅ Start the server immediately

**That's it!** The server will now start automatically on every boot.

---

## 🔌 Testing Connection with STM32

### 1. Physical Setup
- Connect ethernet cable between Pi5 and STM32
- Power on both devices
- STM32 should be configured with IP: **192.168.1.10**

### 2. Check Network
```bash
./test_network.sh
```

Expected output:
```
✓ Pi5 has IP 192.168.1.100
✓ STM32 is reachable at 192.168.1.10
✓ TCP server is listening on port 9000
✓ OMNI TCP server service is running
```

### 3. Monitor Connection
```bash
# Watch server logs (look for "Client connected")
sudo journalctl -u omni_tcp_server.service -f
```

When STM32 connects, you'll see:
```
[INFO] Client connected from ('192.168.1.10', XXXXX)
[INFO] Receive loop started
[INFO] Send loop started
```

### 4. Monitor Robot Data
```bash
# Watch pose updates (should be 5 Hz)
ros2 topic echo /robot/pose

# Check message rate
ros2 topic hz /robot/pose
```

---

## 🧪 Testing Without STM32 (Simulator)

### Start Test Client
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
./start_test_client.sh 192.168.1.100 9000 circle
```

### Interactive Commands
```
> 1         # Send START_TRAJ command
> 2         # Send STOP_TRAJ command  
> m forward # Change to forward motion
> m circle  # Change to circle motion
> q         # Quit
```

### Monitor in Another Terminal
```bash
# Watch ROS2 topics
ros2 topic echo /robot/pose
ros2 topic echo /robot/trajectory

# Watch server logs
sudo journalctl -u omni_tcp_server.service -f
```

---

## 📊 Service Management

```bash
# View logs (live)
sudo journalctl -u omni_tcp_server.service -f

# Check status
sudo systemctl status omni_tcp_server.service

# Restart service
sudo systemctl restart omni_tcp_server.service

# Stop service
sudo systemctl stop omni_tcp_server.service

# Start service
sudo systemctl start omni_tcp_server.service
```

---

## 🔧 Troubleshooting

### STM32 Can't Connect

```bash
# 1. Check Pi5 IP
ip addr show eth0
# Should show: 192.168.1.100

# 2. Ping STM32
ping 192.168.1.10
# Should get replies

# 3. Check server is running
sudo systemctl status omni_tcp_server.service

# 4. Check firewall (if enabled)
sudo ufw allow 9000/tcp
```

### No POSE Messages

```bash
# Enable debug logging
sudo nano /home/nickolas/ros2_ws/src/omni_src/pi_comm_server/omni_main.py
# Change: level=logging.INFO → level=logging.DEBUG

# Restart service
sudo systemctl restart omni_tcp_server.service

# Watch detailed logs
sudo journalctl -u omni_tcp_server.service -f
```

### Service Won't Start

```bash
# Check logs for errors
sudo journalctl -u omni_tcp_server.service -xe

# Common fixes:
# 1. Port in use - kill other process
sudo netstat -tlnp | grep 9000

# 2. Check Python dependencies
pip3 install --user rclpy geometry_msgs nav_msgs
```

---

## 📖 Documentation

- **[README_TCP.md](README_TCP.md)** - Complete documentation
- **[TEST_GUIDE.md](TEST_GUIDE.md)** - Detailed testing procedures
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - System architecture
- **[IMPLEMENTATION.md](IMPLEMENTATION.md)** - Implementation details

---

## ✅ Quick Checklist

After setup, verify:

- [ ] Service installed: `sudo systemctl is-enabled omni_tcp_server.service`
- [ ] Service running: `sudo systemctl is-active omni_tcp_server.service`
- [ ] Pi5 has correct IP: `ip addr show eth0 | grep 192.168.1.100`
- [ ] Can ping STM32: `ping 192.168.1.10`
- [ ] Server listening: `sudo netstat -tlnp | grep 9000`
- [ ] ROS2 working: `ros2 topic list`

**All green? You're ready to connect your STM32!** 🎉
