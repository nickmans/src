# OMNI ROS 2 Quick Start

Short operator runbook for the ROS 2 workspace.

## 1. Build from the workspace root

```bash
cd /home/nickolas/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omni_traj
source install/setup.bash
```

## 2. Bring up the stack

Mock lidar bringup:

```bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py use_mock_lidar:=true
```

Real lidar bringup:

```bash
ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py use_mock_lidar:=false
```

Use `dual_sllidar_with_mock_and_traj.launch.py` as the normal bringup path. `omni_traj/launch/omni_bringup.launch.py` is not the primary launch path and still contains placeholder serial-by-id comments.

## 3. Verify core topics

```bash
ros2 topic list | grep -E "lidar1/scan|lidar2/scan|scan_fused|scan_match|map|costmap|planned_path"
ros2 topic hz /scan_fused
ros2 topic hz /robot/pose
ros2 topic hz /robot/odom
```

## 4. Start the UDP server manually

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_udp_server.py
```

The normal production boot path is `omni_udp_server.service`. Manual `run_udp_server.py` is mainly for direct testing and debugging.

## 5. Connect the STM32

Expected network defaults:

- Pi: `192.168.1.100`
- STM32: `192.168.1.10`
- Port: `9000`

Once the STM32 is connected, use the STM32 shell to enter `map` / `traj` modes.
