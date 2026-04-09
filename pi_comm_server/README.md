# OMNI Pi UDP Bridge

This directory contains the Raspberry Pi UDP bridge and ROS 2 stack lifecycle manager for the OMNI robot.

## Overview

The checked-in UDP server is the production bridge between the STM32H755 CM7 firmware and the ROS 2 Jazzy stack on the Pi.

It does four main jobs:

1. Listens for `POSE` packets from the STM32 on UDP port `9000`.
2. Publishes that pose into ROS 2 through `ros2_pose_node.py`.
3. Subscribes to `/planned_path` and `/planned_path_velocities` so it can package `TRAJ` packets for the STM32.
4. Starts, stops, and switches the ROS 2 stack between `standby`, `mapping`, and `localization` modes through `ros2_manager.py`.

## Current runtime entrypoints

Use one of these current entrypoints:

- Manual Python entrypoint:

  ```bash
  python3 run_udp_server.py
  ```

- Convenience script:

  ```bash
  ./start_server.sh
  ```

- Console script from the repo root `setup.py`:

  ```bash
  omni-udp-server
  ```

- Normal production systemd service:

  ```bash
  sudo systemctl enable --now omni_udp_server.service
  ```


## Services

### Normal service

- Primary deployment path: `omni_udp_server.service`
- This is the normal boot mode.
- It starts `run_udp_server.py` and lets the UDP server manage ROS 2 stack mode transitions.
- It sets `OMNI_TRAJ_SEND_HZ=10` by default.

### Debug-only service

- `omni_ros2_stack.service`
- This launches the ROS 2 stack directly through `start_ros2_stack.sh`.
- Treat it as debug-only bringup, not the normal boot mode.
- It does not replace the normal UDP-controlled production path.

### Optional simulation service

- `omni_virtual_stm32.service`
- Optional simulation/testing only.
- Not part of the normal hardware boot flow.

### Legacy note

- Older user-service wiring exists in the repo for historical reference.
- It is not the normal deployment path.
- Use `omni_udp_server.service` for production boot.

## ROS 2 modes managed by `ros2_manager.py`

The UDP server switches the ROS 2 stack between these modes:

- `standby`
  - `enable_amcl_localization:=false`
  - `enable_slam_toolbox:=false`
  - trajectory output disabled

- `mapping`
  - `enable_amcl_localization:=false`
  - `enable_slam_toolbox:=true`

- `localization`
  - `enable_amcl_localization:=true`
  - `enable_slam_toolbox:=false`

`start_ros2_stack.sh` is the direct debug bringup helper. It also performs extra port discovery and prefers a Jazzy + workspace setup from `/home/nickolas/ros2_ws/install/...` when available.

## Actual ROS topics used by the server

### Pose published into ROS 2

`PosePublisherNode` in `ros2_pose_node.py` publishes:

- `/robot/pose`
- `/robot/twist`
- `/robot/odom`
- `/odom`
- `/initialpose`

The checked-in default STM32 <-> ROS rotation parameters are zeroed unless you override them:

- `stm32_pose_rotation_deg=0.0`
- `stm32_yaw_rotation_deg=0.0`
- `stm32_yaw_offset_deg=0.0`
- `stm32_traj_rotation_deg=0.0`
- `stm32_traj_yaw_rotation_deg=0.0`
- `stm32_traj_yaw_offset_deg=0.0`


### Planner data consumed by the server

`udp_server.py` subscribes to:

- `/planned_path`
- `/planned_path_velocities`

Those are the main runtime trajectory topics. `/robot/trajectory` is not the primary runtime trajectory output.

## Mapping and waypoint services called by the server

The server calls these ROS 2 services through `ROS2Bridge`:

- `/mapping/start`
- `/mapping/finish`
- `/mapping/use_live`
- `/mapping/use_frozen`
- `/mapping/use_blank`
- `/waypoints/generate_test_pattern`

## UDP protocol and rates

From the checked-in protocol files:

- Pi default IP: `192.168.1.100`
- STM32 default IP: `192.168.1.10`
- Port: `9000`
- magic: `0x4F4D4E49` (`OMNI`)
- version: `1`
- header size: `24`
- message IDs:
  - `POSE=1`
  - `TRAJ=10`
  - `ACK=12`
  - `STATUS=15`
  - `CMD=20`

Runtime rates:

- STM32 queues pose heartbeats at `10 Hz` in `CM7/Core/Src/main.c`
- Pi sends `TRAJ` at default `10 Hz` through `OMNI_TRAJ_SEND_HZ=10`

## Command semantics

These are the important command meanings as implemented by `udp_server.py` and `CM7/Core/Src/cmd.c`.

- `traj 1`
  - Shell meaning: autonomous localization on saved/frozen map.
  - Pi behavior: switch ROS 2 stack to `localization`, call `/mapping/use_frozen`, then enable trajectory streaming.

- `traj 0`
  - Shell meaning: standby/manual.
  - Pi behavior: disable trajectory streaming, hold briefly, then switch ROS 2 stack to `standby`.

- `traj2 2`
  - Shell meaning: manual drive on STM32 while Pi runs localization.
  - Pi behavior: switch ROS 2 stack to `localization`, keep trajectory streaming disabled.

- `traj 3`
  - Shell meaning: autonomous mode using a blank global map with local obstacle avoidance.
  - Pi behavior: move stack to `standby`, call `/mapping/use_blank`, then enable trajectory streaming.

- `map 1`
  - Shell meaning: start mapping mode.
  - Pi behavior: disable trajectory streaming, switch stack to `mapping`, call `/mapping/start`.

- `map 0`
  - Shell meaning: finish mapping and return to autonomous localization.
  - Pi behavior: call `/mapping/finish`, switch stack to `localization`, then re-enable trajectory streaming when healthy.

- `map 2`
  - Shell meaning: use live map mode.
  - Pi behavior: disable trajectory streaming, switch stack to `mapping`, call `/mapping/use_live`.

- `map 3`
  - Shell meaning: use frozen map mode.
  - Pi behavior: switch stack to `localization`; current code treats this as localization mode and skips the older frozen-map service path.

- `wp t`
  - Shell meaning: generate centered waypoint test pattern.
  - Pi behavior: call `/waypoints/generate_test_pattern`.

- `term`
  - Shell meaning: start terminal passthrough.
  - Pi behavior: opens Pi shell passthrough over UDP.

## Startup methods

### Manual

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_udp_server.py
```

### Convenience script

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
./start_server.sh
```

### Install the normal service

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./install_service.sh
sudo systemctl enable --now omni_udp_server.service
```

### Full setup helper

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./setup_complete.sh
```

## Testing and simulation

### Test client helper

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
./start_test_client.sh
```

### Virtual STM32 simulator

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 virtual_stm32_udp.py --server-host 127.0.0.1 --server-port 9000
```

`virtual_stm32_udp.py` is for simulation/testing. It is not the normal hardware runtime path.

## Troubleshooting

Check the normal service:

```bash
sudo systemctl status omni_udp_server.service
sudo journalctl -u omni_udp_server.service -f
```

Check that UDP port `9000` is the one in use and that the network defaults match the robot.

Check the core ROS 2 topics that prove the live bridge is working:

```bash
ros2 topic hz /robot/pose
ros2 topic hz /robot/odom
ros2 topic echo /planned_path
```

If those are not updating, work outward in this order:

1. Verify the service is running.
2. Verify Pi/STM32 IPs are correct.
3. Verify UDP traffic is arriving on port `9000`.
4. Verify ROS 2 stack mode is appropriate for the command you sent.
5. Verify `/planned_path` and `/planned_path_velocities` exist before expecting outgoing `TRAJ` packets.

