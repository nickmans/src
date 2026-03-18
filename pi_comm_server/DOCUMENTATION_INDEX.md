# OMNI Pi Communication Server - Documentation

This folder contains the communication server for the OMNI robot, enabling UDP communication between the Raspberry Pi 5 and the STM32 NUCLEO H755 controller.

---

## 📚 Main Documentation

| Document | Purpose | When to Use |
|----------|---------|-------------|
| **[README.md](README.md)** | Complete reference documentation | Full protocol specs, installation, architecture |
| **[QUICKSTART.md](QUICKSTART.md)** | Quick start guide | Get up and running in 2 minutes |
| **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** | Troubleshooting guide | Having issues? Start here |
| **[README.md](README.md)** | STM32 implementation guide | Implementing the STM32 client |

---

## 🚀 Quick Links

### First Time Setup
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./setup_complete.sh
```

### Common Commands
```bash
# Check service status
sudo systemctl status omni_udp_server.service

# View logs
sudo journalctl -u omni_udp_server.service -f

# Test network
./test_network.sh

# Start test client (simulator)
./start_test_client.sh 192.168.1.100 9000 circle
```

---

## 📁 Documentation Structure

```
pi_comm_server/
├── README.md                    # Complete reference (protocol, installation, usage)
├── QUICKSTART.md               # 2-minute quick start guide
├── TROUBLESHOOTING.md          # Comprehensive troubleshooting
├── protocol.py                 # Binary protocol implementation
├── udp_server.py               # Main UDP server implementation
├── run_udp_server.py           # Server wrapper script
├── run_test_client.py         # Test client wrapper script
│
├── setup_complete.sh          # One-command setup
├── start_server.sh            # Manual server start
├── start_test_client.sh       # Test client launcher
│
└── ... (other scripts and files)
```

---

## 📖 What's in Each Document?

### README.md (Complete Reference)
- Protocol specification (binary format, message types)
- System architecture diagrams
- Installation instructions (automated & manual)
- Usage examples
- Service management
- ROS2 integration
- Network configuration
- Advanced topics

**Read this when:** You need detailed protocol specifications, want to understand the architecture, or need comprehensive installation/usage instructions.

### QUICKSTART.md (Get Started Fast)
- One-command setup
- Testing with STM32
- Testing with simulator
- Common operations
- Quick health check
- Quick troubleshooting

**Read this when:** You just want to get the system running quickly, or need a quick reference for common tasks.

### TROUBLESHOOTING.md (Problem Solving)
- Network & connection issues
- Service & server issues
- STM32 connection problems
- ROS2 integration issues
- Performance & resource issues
- Import & module errors
- Diagnostic tools

**Read this when:** Something isn't working and you need to diagnose and fix the issue.

### README.md (For STM32 Developers)
- Non-blocking UDP client implementation
- Binary protocol specification
- Message formats with C code examples
- State machine architecture
- Integration with controller loop
- Testing checklist

**Read this when:** You're implementing or debugging the STM32 UDP client.

---

## 🎯 Common Scenarios

**Scenario:** I'm setting up the system for the first time
- **Start with:** [QUICKSTART.md](QUICKSTART.md) → Run `setup_complete.sh`
- **Then read:** [README.md](README.md) for understanding the system

**Scenario:** The server won't start
- **Start with:** [TROUBLESHOOTING.md](TROUBLESHOOTING.md) → "Service & Server Issues"
- **Check:** Service logs with `journalctl`

**Scenario:** STM32 won't connect
- **Start with:** [TROUBLESHOOTING.md](TROUBLESHOOTING.md) → "STM32 Connection Problems"
- **Run:** `./diagnose_stm32_connection.sh`

**Scenario:** I need to implement the STM32 client
- **Start with:** [README.md](README.md)
- **Reference:** [README.md](README.md) for protocol details

**Scenario:** How do I test without STM32?
- **Start with:** [QUICKSTART.md](QUICKSTART.md) → "Testing WITHOUT STM32"
- **Command:** `./start_test_client.sh 192.168.1.100 9000 circle`

**Scenario:** High CPU usage / SSH problems
- **Start with:** [TROUBLESHOOTING.md](TROUBLESHOOTING.md) → "Performance & Resource Issues"
- **Solution:** Use `./start_server_low_resource.sh`

---

## 🔧 Utility Scripts

| Script | Purpose |
|--------|---------|
| `setup_complete.sh` | One-command setup (IP + service) |
| `install_service.sh` | Install systemd service |
| `uninstall_service.sh` | Remove systemd service |
| `start_server.sh` | Manual server start |
| `start_server_low_resource.sh` | Low CPU usage mode |
| `start_test_client.sh` | Test client launcher |
| `test_network.sh` | Test network configuration |
| `diagnose_connection.sh` | Connection diagnostics |
| `diagnose_stm32_connection.sh` | STM32-specific diagnostics |
| `monitor_connection.sh` | Monitor connection status |
| `monitor_resources.sh` | Monitor CPU/memory usage |

---

## 🆘 Getting Help

1. **Check the docs in this order:**
   - [QUICKSTART.md](QUICKSTART.md) - Is it a simple operational question?
   - [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - Having an issue?
   - [README.md](README.md) - Need detailed information?

2. **Run diagnostic tools:**
   ```bash
   ./test_network.sh
   ./diagnose_stm32_connection.sh
   ./monitor_resources.sh
   ```

3. **Check logs:**
   ```bash
   sudo journalctl -u omni_udp_server.service -n 200
   ```

4. **Enable debug logging:**
   ```bash
   python3 run_udp_server.py --log-level DEBUG
   ```

---

## 📊 Network Configuration Quick Reference

| Device | IP Address | Port | Role |
|--------|------------|------|------|
| Raspberry Pi 5 | 192.168.1.100 | 9000 | UDP Server |
| STM32 Nucleo H755 | 192.168.1.10 | - | UDP Client |

**Connection:** Direct Ethernet (no router)  
**Protocol:** UDP with binary framing  
**Rate:** 5 Hz bidirectional  

---

**Last Updated:** February 2026  
**Documentation Version:** 2.0 (Consolidated)
