# OMNI ROS 2 Workspace

This repository is the ROS 2 / Raspberry Pi side of a two-repo robot system.

- `omni_traj` contains the checked-in dual-LiDAR fusion, mapping, legacy `/costmap` publication, path generation, and waypoint tooling.
- `pi_comm_server` contains the UDP bridge that talks to the STM32H755 CM7 firmware and manages ROS 2 stack modes on the Pi.
- The separate firmware repository is `OMNI-BOT`, where the active robot runtime lives under `CM7/`.

## System architecture

Normal runtime is split across the Pi and STM32:

1. The STM32H755 CM7 runs estimator + control on the robot.
2. The Pi 5 runs ROS 2 Jazzy and the UDP server.
3. The STM32 sends `POSE` packets over UDP to the Pi.
4. `pi_comm_server/ros2_pose_node.py` publishes those packets into ROS 2 on:
   - `/robot/pose`
   - `/robot/twist`
   - `/robot/odom`
   - `/odom`
   - `/initialpose`
5. `omni_traj/omni_traj/waypoint_traj_node.py` subscribes to:
   - `/odom`
   - `/lidar1/scan`
   - `/lidar2/scan`
   - `/clicked_point`
   - `/move_base_simple/goal`
   - `/goal_pose`
6. `waypoint_traj_node.py` publishes the planning outputs used by the UDP bridge:
   - `/map`
   - `/costmap`
   - `/scan_fused`
   - `/scan_match`
   - `/points_fused`
   - `/planned_path`
   - `/planned_path_velocities`
   - `/waypoint_markers`
   - `/path_velocity_markers`
   - `/robot_visualization`
   - `/geofence_markers`
7. `pi_comm_server/udp_server.py` subscribes to `/planned_path` and `/planned_path_velocities`, packages them into `TRAJ` packets, and sends them back to the STM32.

The main runtime trajectory path is therefore:

`CM7 pose -> Pi UDP server -> ROS 2 pose topics -> waypoint_traj_node -> /planned_path + /planned_path_velocities -> Pi UDP server -> CM7`

`ros2_trajectory_node.py` and `/robot/trajectory` are not the primary runtime path.

## What is in this repo

### `omni_traj`

- Primary launch file: `omni_traj/launch/dual_sllidar_with_mock_and_traj.launch.py`
- Main node: `omni_traj/omni_traj/waypoint_traj_node.py`
- Default checked-in launch behavior:
  - `use_mock_lidar:=false`
  - `use_rviz:=false`
  - `enable_slam_toolbox:=true`
  - `enable_amcl_localization:=false`
  - `enable_nav2_costmaps:=false`
  - `publish_world_to_odom_tf:=false`
  - `publish_odom_to_base_tf:=true`
  - `lidar1_serial_port:=/dev/ttyAMA0`
  - `lidar2_serial_port:=/dev/ttyAMA2`
  - `lidar1_y_m:=0.10`
  - `lidar2_y_m:=-0.10`
  - `lidar_yaw_rad:=3.141592653589793`

### `pi_comm_server`

- Real Pi server entrypoint: `pi_comm_server/run_udp_server.py`
- Normal boot path: `pi_comm_server/omni_udp_server.service`
- Debug-only direct bringup path: `pi_comm_server/omni_ros2_stack.service`
- Optional simulation service: `pi_comm_server/omni_virtual_stm32.service`

## Primary bringup path

Use `dual_sllidar_with_mock_and_traj.launch.py` as the primary ROS 2 bringup path.

`omni_traj/launch/omni_bringup.launch.py` is not the primary bringup path. It still contains placeholder serial-by-id comments such as `USB_ID_FOR_LIDAR1` / `USB_ID_FOR_LIDAR2`, so normal users should use `dual_sllidar_with_mock_and_traj.launch.py` instead.

## `/costmap` vs Nav2 costmaps

- `/costmap` is the legacy costmap output from `waypoint_traj_node.py`.
- In the checked-in dual-LiDAR launch file, `publish_legacy_costmap` is forced to `True`, so `/costmap` is part of the normal bringup output.
- Nav2 `/local_costmap/costmap` and `/global_costmap/costmap` are optional and are not enabled by default.
- `enable_nav2_costmaps` defaults to `false` in `dual_sllidar_with_mock_and_traj.launch.py`, so do not expect Nav2 costmaps unless you explicitly enable them.

## Standard build workflow

Build from the workspace root, not from inside the package folder:

```bash
cd /home/nickolas/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omni_traj
source install/setup.bash
```

## Recommended bringup order

1. Build from `/home/nickolas/ros2_ws`.
2. Start the ROS 2 stack with mock lidars first:

   ```bash
   ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py use_mock_lidar:=true
   ```

3. Verify the main runtime outputs:

   ```bash
   ros2 topic list | grep -E "lidar1/scan|lidar2/scan|scan_fused|scan_match|map|costmap|planned_path"
   ```

4. Start the UDP server:

   ```bash
   cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
   python3 run_udp_server.py
   ```

5. Connect and power the STM32, then switch modes from the STM32 shell.

After mock bringup is healthy, move to the real lidar launch:

```bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py use_mock_lidar:=false
```

## Mode commands from the STM32 shell

These are the checked-in STM32 shell commands and the Pi-side behavior they trigger:

- `traj 0`
  - STM32 enters standby/manual mode.
  - Pi disables trajectory streaming.
  - UDP server switches ROS 2 stack to `standby`.

- `traj 1`
  - STM32 enters autonomous localization mode.
  - Pi switches ROS 2 stack to `localization`.
  - Pi calls `/mapping/use_frozen`.
  - Trajectory streaming is enabled once localization is healthy.

- `traj2 2`
  - STM32 stays manual.
  - Pi switches ROS 2 stack to `localization`.
  - Pi keeps trajectory streaming disabled.

- `traj 3`
  - STM32 enters autonomous trajectory-following mode.
  - Pi switches to blank global map behavior and calls `/mapping/use_blank`.
  - Trajectory streaming is enabled only after blank-map mode is confirmed.

- `map 1`
  - STM32 leaves trajectory mode.
  - Pi switches ROS 2 stack to `mapping`.
  - Pi calls `/mapping/start`.

- `map 0`
  - Pi calls `/mapping/finish`.
  - Pi then switches ROS 2 stack to `localization`.
  - Trajectory streaming is re-enabled after localization is healthy.

- `map 2`
  - Pi switches to mapping/live-map behavior.
  - Pi calls `/mapping/use_live`.
  - Trajectory streaming remains disabled.

- `map 3`
  - Pi switches to localization mode.
  - Pi currently treats this as localization mode and does not call the older frozen-map service path.
  - Trajectory streaming is enabled only if localization becomes healthy.

- `wp t`
  - Pi calls `/waypoints/generate_test_pattern` to create the centered waypoint test pattern.

- `term`
  - Requests Pi terminal passthrough over the UDP command channel.

## Checked-in hardware defaults vs physical robot

The checked-in launch defaults are software defaults, not a guarantee about the physical robot.

- `lidar1_y_m:=0.10` and `lidar2_y_m:=-0.10` place the frames left/right relative to `base_link`.
- `lidar_yaw_rad:=3.141592653589793` rotates both lidar frames by pi radians relative to `base_link` in the checked-in launch file.
- Verify the real mounting on the robot before changing these defaults.

## UDP and services at a glance

- Pi default IP: `192.168.1.100`
- STM32 default IP: `192.168.1.10`
- UDP port: `9000`
- Protocol constants from `pi_comm_server/protocol.py`:
  - magic `0x4F4D4E49` (`OMNI`)
  - version `1`
  - header size `24`
  - `POSE=1`, `TRAJ=10`, `ACK=12`, `STATUS=15`, `CMD=20`
- Normal production service: `omni_udp_server.service`
- Debug-only bringup service: `omni_ros2_stack.service`
- Optional simulation/testing service: `omni_virtual_stm32.service`

Historical verification documents such as `BEFORE_AFTER.md` and `FUSION_VERIFICATION_REPORT.md` are left as archival references and are not the source of truth for the current runtime.
