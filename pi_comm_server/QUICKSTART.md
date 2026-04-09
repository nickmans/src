# Pi UDP Bridge Quickstart

Compact runbook for the Pi-side UDP bridge.

## Normal production service

Install and start the normal service:

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
sudo ./install_service.sh
sudo systemctl enable --now omni_udp_server.service
```

`omni_udp_server.service` is the normal production path.

`omni_ros2_stack.service` is debug-only bringup.

`omni_virtual_stm32.service` is optional simulation/testing only.

## Manual run

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_udp_server.py
```

## Tail logs

```bash
sudo journalctl -u omni_udp_server.service -f
```

## Verify the link

```bash
ping 192.168.1.10
sudo journalctl -u omni_udp_server.service -f
ros2 topic hz /robot/pose
ros2 topic hz /robot/odom
ros2 topic echo /planned_path
```

The primary runtime trajectory topics are `/planned_path` and `/planned_path_velocities`. `/robot/trajectory` is not the primary runtime trajectory topic.

## STM32 shell sequence

Use these exact shell commands on the STM32 side:

```text
map 1
map 0
traj 1
traj2 2
traj 3
traj 0
```

Meaning:

- `map 1` starts mapping mode.
- `map 0` finishes mapping and returns the Pi to localization mode.
- `traj 1` enables autonomous localization + trajectory streaming.
- `traj2 2` keeps STM32 manual while Pi localizes.
- `traj 3` enables blank global map mode with local obstacle avoidance.
- `traj 0` returns to manual/standby.

## Simulator quickstart

Terminal 1:

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_udp_server.py
```

Terminal 2:

```bash
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 virtual_stm32_udp.py --server-host 127.0.0.1 --server-port 9000
```

Use this only for simulation/testing.
