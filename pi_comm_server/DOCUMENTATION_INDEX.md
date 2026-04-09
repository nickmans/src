# OMNI Pi Bridge Documentation Index

Current documentation map for `pi_comm_server`.

## Primary docs

- [README.md](README.md): authoritative Pi UDP bridge reference
- [QUICKSTART.md](QUICKSTART.md): compact operator runbook
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md): issue diagnosis and recovery

## Runtime truth summary

- Primary production service: `omni_udp_server.service`
- Debug-only standalone bringup: `omni_ros2_stack.service`
- Optional simulator/testing service: `omni_virtual_stm32.service`
- Main Pi entrypoint: `run_udp_server.py`
- Protocol defaults:
  - Pi: `192.168.1.100`
  - STM32: `192.168.1.10`
  - UDP: `9000`
- Runtime cadence:
  - STM32 POSE: 10 Hz
  - Pi TRAJ send default: 10 Hz (`OMNI_TRAJ_SEND_HZ=10`)

## Quick commands

```bash
# status
sudo systemctl status omni_udp_server.service

# logs
sudo journalctl -u omni_udp_server.service -f

# manual run
cd /home/nickolas/ros2_ws/src/omni_src/pi_comm_server
python3 run_udp_server.py

# simulator
python3 virtual_stm32_udp.py --server-host 127.0.0.1 --server-port 9000
```

## Notes

- Do not treat legacy user-service wiring as the normal deployment path.
- Primary runtime trajectory topics are `/planned_path` and `/planned_path_velocities`.
- `/robot/trajectory` is not the main runtime trajectory source.
