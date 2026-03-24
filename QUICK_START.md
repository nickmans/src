# OMNI Workspace Quick Start (`omni_src`)

Fast path for a new engineer to prove the stack works.

## 0) Prerequisites

- ROS2 environment installed and sourced capability available.
- Workspace path: `/home/nickolas/ros2_ws/src/omni_src`.
- For real LiDAR: valid serial ports (Pi 5 UART defaults: `/dev/ttyAMA0` and `/dev/ttyAMA2`).

---

## 1) Build `omni_traj`

```bash
cd /home/nickolas/ros2_ws/src/omni_src/omni_traj
colcon build --packages-select omni_traj
source install/setup.bash
```

---

## 2) Run software-only bench test (recommended first)

```bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py \
  use_mock_lidar:=true \
  use_rviz:=true
```

Expected core topics:

```bash
ros2 topic list | grep -E "lidar1/scan|lidar2/scan|scan_fused|costmap|planned_path"
```

Expected behavior:
- `/lidar1/scan` and `/lidar2/scan` active.
- `/scan_fused` active.
- `/costmap` updating.
- RViz displays map/costmap/fused scan.

---

## 3) Validate TF + rates

```bash
ros2 run tf2_tools view_frames.py
ros2 topic hz /scan_fused
ros2 topic hz /costmap
```

If `/scan_fused` is missing, verify both raw scan topics and frame IDs first.

---

## 4) Run with real LiDARs

```bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py \
  use_mock_lidar:=false \
  use_rviz:=true \
  lidar1_serial_port:=/dev/ttyAMA0 \
  lidar2_serial_port:=/dev/ttyAMA2
```

Helpful checks:

```bash
ls -la /dev/ttyAMA*
```

---

## 5) Start Pi communication server (STM32 bridge)

In a new terminal:

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_server.py
```

For quick network/client test:

```bash
./test_network.sh
./start_test_client.sh 127.0.0.1 9000 circle
```

Production setup uses service scripts in `pi_comm_server/` (see `pi_comm_server/README.md`).

---

## 6) Day-1 success criteria

- ROS2 launch runs without node crashes.
- `/scan_fused` and `/costmap` publish continuously.
- TF tree resolves correctly between map/odom/base/lidar frames.
- Pi server accepts client connection and exchanges messages.

---

## 7) Most common fixes

- **No fused scan**: check `/lidar1/scan`, `/lidar2/scan`, and frame IDs.
- **RViz frame error**: verify launch args for `map_frame` / `odom_frame` and TF publishers.
- **No STM32 connection**: verify host/port/IP route and Ethernet link.

---

## 8) Next reads

- `README.md` (full onboarding flow)
- `LIDAR_FUSION_ARCHITECTURE.md`
- `LIDAR_FUSION_FIXES.md`
- `pi_comm_server/README.md`
