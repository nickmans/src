# OMNI Source Workspace (`omni_src`)

This document is the full working flow for a robotics engineer new to this project.
It explains what each major part does, how data moves through the system, and how to run/verify the stack.

## 1) Workspace Purpose

`omni_src` contains two main runtime parts:

- **`omni_traj/`**: ROS2 package for dual-LiDAR fusion, Nav2 local/global costmaps, waypoint handling, and trajectory output.
- **`pi_comm_server/`**: Raspberry Pi communication bridge (TCP/UDP) between STM32 and ROS2.

Supporting markdown files in the workspace document previous LiDAR fusion fixes and validation details.

## 2) Directory Guide (What matters first)

- **`omni_traj/`**
  - `launch/dual_sllidar_with_mock_and_traj.launch.py`: main bringup for real/mock LiDAR + planner/fusion node.
  - `launch/omni_bringup.launch.py`: simpler bringup variant.
  - `omni_traj/waypoint_traj_node.py`: core node (fusion + map + planning + trajectory).
  - `omni_traj/empty_scan_pub.py`: mock LiDAR publisher for bench testing.
  - `config/`: planner/fusion/localization parameters.
- **`pi_comm_server/`**
  - `run_server.py` / `omni_main.py`: communication server entry points.
  - `protocol.py`: STM32↔Pi protocol definitions.
  - `ros2_pose_node.py`, `ros2_trajectory_node.py`: ROS2 bridge nodes used by server.
  - `*.service` + install scripts: systemd deployment for boot-time startup.
- **Generated folders** (`build/`, `install/`, `log/`) are build/runtime outputs.

## 3) System Architecture (End-to-end)

1. **LiDAR input**
   - Two scans arrive on `/lidar1/scan` and `/lidar2/scan` (real sensors or mock publishers).
2. **Transform alignment**
   - Static TFs place both sensors relative to `base_link`; optional world/odom anchoring is enabled by launch args.
3. **Fusion + mapping + planning**
  - `waypoint_traj` fuses scans into `/scan_fused` and computes path/trajectory products.
  - Nav2 publishes local/global costmaps on `/local_costmap/costmap` and `/global_costmap/costmap`.
4. **ROS2↔STM32 bridge (Pi side)**
   - `pi_comm_server` receives POSE/CMD from STM32 and sends TRAJ/STATUS back.
5. **Robot execution loop**
   - STM32 runs low-level control; Pi/ROS2 provides high-level fused perception and trajectory references.

## 4) Bringup Modes

### A) ROS2-only bench test (no hardware LiDAR)

Use this first to verify software flow:

```bash
cd /home/nickolas/ros2_ws/src/omni_src/omni_traj
colcon build --packages-select omni_traj
source install/setup.bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py use_mock_lidar:=true use_rviz:=true
```

Expected:
- `/lidar1/scan`, `/lidar2/scan`, `/scan_fused`, `/local_costmap/costmap`, `/global_costmap/costmap` are present.
- RViz shows fused scan, `/map`, local costmap, and global costmap layers.

### B) ROS2 with real LiDAR hardware

```bash
cd /home/nickolas/ros2_ws/src/omni_src/omni_traj
colcon build --packages-select omni_traj
source install/setup.bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py use_mock_lidar:=false use_rviz:=true
```

Then set launch args as needed:
- `lidar1_serial_port`, `lidar2_serial_port`
- `lidar1_frame_id`, `lidar2_frame_id`
- `publish_world_to_odom_tf` (RViz frame anchoring option)

Pi 5 UART defaults in this workspace:
- `lidar1_serial_port:=/dev/ttyAMA0`
- `lidar2_serial_port:=/dev/ttyAMA2`

### C) Pi communication server with STM32

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_server.py
```

Production deployment (systemd) is available via scripts in `pi_comm_server/` (`setup_complete.sh`, service files).

## 5) Quick Validation Checklist

Run these after launch:

```bash
ros2 topic list | grep -E "lidar|scan_fused|local_costmap/costmap|global_costmap/costmap|map|planned_path"
ros2 topic hz /scan_fused
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
ros2 run tf2_tools view_frames.py
```

For Pi comms:

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
./test_network.sh
./start_test_client.sh 127.0.0.1 9000 circle
```

Success criteria:
- Stable TF tree (`world/map/odom/base_link/lidar*` as configured).
- Fused scan publishes consistently.
- Nav2 local and global costmaps update continuously.
- Pi server accepts client/STM32 and exchanges frames without parser errors.

## 6) Common Operational Workflow

1. Start ROS2 stack (`omni_traj`) in mock or real-LiDAR mode.
2. Confirm topics/TF/RViz are healthy.
3. Start `pi_comm_server` (or ensure systemd service is active).
4. Connect STM32 client.
5. Observe closed loop:
   - STM32 sends POSE/CMD → Pi/ROS2 updates world model/trajectory.
   - Pi sends TRAJ back to STM32.
6. Monitor health with `ros2 topic hz`, `journalctl` (if systemd), and test scripts.

### STM32 mode control sequence (important)

When operating with CM7 + Pi trajectory flow:

- Send `map 1` to enter dedicated mapping mode while staying in manual driving mode.
- Send `traj 1` to switch to autonomous localization/follow mode using the current saved map.
- Send `traj2 2` to keep localization active while returning STM32 to manual driving mode.
- Send `traj 0` to return to idle/manual standby mode.

Use this sequence when switching between mapping, autonomous localization, and manual-localization operation.

## 7) Troubleshooting First Responses

- **No `/scan_fused`**
  - Verify both LiDAR topics exist and frame IDs match launch configuration.
- **RViz frame errors**
  - Check `map_frame`, `odom_frame`, and whether `publish_world_to_odom_tf` is enabled.
- **Costmaps empty/stale**
  - Confirm fused scan rates and valid ranges, then check `/local_costmap/costmap` and `/global_costmap/costmap` rates.
- **STM32 not connected**
  - Verify IP/port, Ethernet route, firewall, and server bind host.
- **No trajectory returned**
  - Confirm command state (idle/start) and ROS2 bridge node activity in server logs.

## 8) Important Companion Docs

- `QUICK_START.md`: short startup path.
- `LIDAR_FUSION_ARCHITECTURE.md`: fusion architecture details.
- `LIDAR_FUSION_FIXES.md`: implemented fixes and rationale.
- `BEFORE_AFTER.md`, `CHANGES_SUMMARY.md`, `FUSION_VERIFICATION_REPORT.md`: change and verification history.
- `pi_comm_server/README.md`: full protocol + Pi server deployment details.
- `pi_comm_server/QUICKSTART.md`: fast Pi setup.

## 9) New Engineer Day-1 Plan

1. Read this file once end-to-end.
2. Run mock LiDAR bringup (`use_mock_lidar:=true`) and validate all expected topics.
3. Review `omni_traj/omni_traj/waypoint_traj_node.py` for core behavior.
4. Run `pi_comm_server` locally and test with the provided test client.
5. Move to real LiDAR + STM32 only after software-only validation is stable.

---

If you are onboarding, start with Section 4A, then Section 5. That gives the fastest confidence that the workspace is functioning before hardware integration.