# OMNI UDP Bridge Troubleshooting

This guide is for the current Pi runtime path:

- Normal service: `omni_udp_server.service`
- Debug-only direct ROS2 bringup: `omni_ros2_stack.service`
- Optional simulator: `omni_virtual_stm32.service`

Do not use legacy user-service wiring as the primary deployment model.

---

## 1) Fast health check

```bash
# Service status
sudo systemctl status omni_udp_server.service

# Live logs
sudo journalctl -u omni_udp_server.service -f

# Network reachability to STM32
ping 192.168.1.10

# Core runtime topics
ros2 topic hz /robot/pose
ros2 topic hz /robot/odom
ros2 topic echo /planned_path
```

Expected defaults:

- Pi: `192.168.1.100`
- STM32: `192.168.1.10`
- UDP port: `9000`

Runtime rate expectations:

- STM32 POSE heartbeat: 10 Hz
- Pi TRAJ send: 10 Hz default (`OMNI_TRAJ_SEND_HZ=10`)

---

## 2) Server will not start

### Symptoms
- `systemctl` shows failed/inactive
- no process listening on UDP `9000`

### Checks
```bash
sudo systemctl status omni_udp_server.service
sudo journalctl -u omni_udp_server.service -n 200
sudo netstat -ulnp | grep 9000
```

### Fixes
1. Reload and restart:
```bash
sudo systemctl daemon-reload
sudo systemctl restart omni_udp_server.service
```

2. Ensure conflicting debug service is not active:
```bash
sudo systemctl stop omni_ros2_stack.service
sudo systemctl disable omni_ros2_stack.service
```

3. If needed, reinstall service:
```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./install_service.sh
sudo systemctl enable --now omni_udp_server.service
```

---

## 3) Port 9000 already in use

### Checks
```bash
sudo netstat -ulnp | grep 9000
```

### Fixes
```bash
# restart canonical service
sudo systemctl restart omni_udp_server.service

# if a stale process remains
pkill -f run_udp_server.py || true
sudo systemctl start omni_udp_server.service
```

---

## 4) STM32 is not connecting

### Symptoms
- logs show no incoming POSE/CMD
- `ip neigh` shows `FAILED` for `192.168.1.10`

### Checks
```bash
ip addr show eth0
ip neigh show
ping 192.168.1.10
sudo netstat -ulnp | grep 9000
```

### Fixes
1. Verify physical link and power.
2. Verify STM32 network config in firmware:
   - `PI5_IP_ADDR="192.168.1.100"`
   - `PI5_PORT=9000`
   - `STM32_IP_ADDR="192.168.1.10"`
3. Ensure Pi service is running and listening on `0.0.0.0:9000`.

---

## 5) ROS2 topics missing or stale

### Symptoms
- `/robot/pose` or `/robot/odom` not updating
- `/planned_path` empty

### Checks
```bash
ros2 node list
ros2 topic list
ros2 topic hz /robot/pose
ros2 topic hz /robot/odom
ros2 topic echo /planned_path
```

### Fixes
1. Confirm UDP service is active and receiving POSE.
2. Confirm ROS2 stack mode is appropriate for current command (`standby`, `mapping`, `localization`).
3. For trajectory output, ensure planner is publishing both:
   - `/planned_path`
   - `/planned_path_velocities`

Primary trajectory runtime topics are `/planned_path` and `/planned_path_velocities`.
`/robot/trajectory` is not the main runtime trajectory path.

---

## 6) Command mode behavior mismatch

Use the STM32 shell command semantics below:

- `traj 0`: standby/manual, trajectory streaming disabled
- `traj 1`: localization + trajectory streaming on saved/frozen map path
- `traj2 2`: manual STM32 + localization on Pi, trajectory streaming disabled
- `traj 3`: blank global map mode + local obstacle avoidance, trajectory streaming enabled after mode confirmation
- `map 1`: start mapping mode
- `map 0`: finish mapping -> localization mode
- `map 2`: use live map mode
- `map 3`: frozen/localization mode path
- `wp t`: generate centered waypoint test pattern

If behavior diverges, inspect live logs while issuing commands:

```bash
sudo journalctl -u omni_udp_server.service -f
```

---

## 7) LiDAR/bringup confusion

Use:

```bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py
```

Notes:

- This is the primary launch file.
- `enable_nav2_costmaps` defaults to `false`.
- `/costmap` is still published in normal dual launch because `publish_legacy_costmap` is forced to `True`.
- Checked-in defaults include `lidar1_y_m:=0.10`, `lidar2_y_m:=-0.10`, `lidar_yaw_rad:=pi`; verify physical mounting on the robot before changing.

---

## 8) Debug-only services and simulator

Debug-only direct stack bringup:

```bash
sudo systemctl start omni_ros2_stack.service
sudo systemctl status omni_ros2_stack.service
```

Optional simulator path:

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_udp_server.py
python3 virtual_stm32_udp.py --server-host 127.0.0.1 --server-port 9000
```

`omni_virtual_stm32.service` is optional simulation/testing only.

---

## 9) Collect support bundle

```bash
uname -a
python3 --version
ip addr show
ip neigh show
sudo systemctl status omni_udp_server.service
sudo journalctl -u omni_udp_server.service -n 300
ros2 topic hz /robot/pose
ros2 topic hz /robot/odom
ros2 topic echo /planned_path
```
