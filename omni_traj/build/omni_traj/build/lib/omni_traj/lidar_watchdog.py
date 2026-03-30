#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


@dataclass
class LidarState:
    key: str
    topic: str
    start_service: str
    process_node_name: str
    last_scan_ns: Optional[int] = None
    last_start_try_ns: int = 0


class LidarWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("lidar_watchdog")

        self.declare_parameter("scan_topic_1", "/lidar1/scan")
        self.declare_parameter("scan_topic_2", "/lidar2/scan")
        self.declare_parameter("start_service_1", "/lidar1/start_motor")
        self.declare_parameter("start_service_2", "/lidar2/start_motor")
        self.declare_parameter("process_node_name_1", "lidar1")
        self.declare_parameter("process_node_name_2", "lidar2")

        self.declare_parameter("timer_hz", 1.0)
        self.declare_parameter("startup_grace_s", 5.0)
        self.declare_parameter("scan_timeout_s", 2.5)
        self.declare_parameter("start_motor_retry_s", 5.0)
        self.declare_parameter("start_command_stagger_s", 2.0)

        self._startup_ns = self.get_clock().now().nanoseconds
        self._last_any_start_try_ns = 0

        self._states: Dict[str, LidarState] = {
            "lidar1": LidarState(
                key="lidar1",
                topic=str(self.get_parameter("scan_topic_1").value),
                start_service=str(self.get_parameter("start_service_1").value),
                process_node_name=str(self.get_parameter("process_node_name_1").value),
            ),
            "lidar2": LidarState(
                key="lidar2",
                topic=str(self.get_parameter("scan_topic_2").value),
                start_service=str(self.get_parameter("start_service_2").value),
                process_node_name=str(self.get_parameter("process_node_name_2").value),
            ),
        }

        self._start_clients: Dict[str, object] = {
            s.key: self.create_client(Empty, s.start_service) for s in self._states.values()
        }

        self._scan_sub_1 = self.create_subscription(
            LaserScan,
            self._states["lidar1"].topic,
            lambda msg: self._on_scan("lidar1", msg),
            qos_profile_sensor_data,
        )
        self._scan_sub_2 = self.create_subscription(
            LaserScan,
            self._states["lidar2"].topic,
            lambda msg: self._on_scan("lidar2", msg),
            qos_profile_sensor_data,
        )

        timer_hz = max(0.2, float(self.get_parameter("timer_hz").value))
        self._timer = self.create_timer(1.0 / timer_hz, self._on_timer)

        self.get_logger().info("LiDAR watchdog active")

    def _on_scan(self, key: str, _msg: LaserScan) -> None:
        now_ns = self.get_clock().now().nanoseconds
        state = self._states[key]
        state.last_scan_ns = now_ns

    def _call_start_motor(self, state: LidarState, now_ns: int) -> None:
        retry_s = max(0.5, float(self.get_parameter("start_motor_retry_s").value))
        if now_ns - state.last_start_try_ns < int(retry_s * 1e9):
            return

        stagger_s = max(2.0, float(self.get_parameter("start_command_stagger_s").value))
        if now_ns - self._last_any_start_try_ns < int(stagger_s * 1e9):
            return

        client = self._start_clients[state.key]
        if not client.wait_for_service(timeout_sec=0.2):
            return

        req = Empty.Request()
        client.call_async(req)
        state.last_start_try_ns = now_ns
        self._last_any_start_try_ns = now_ns
        self.get_logger().warn(f"{state.key}: stale scan, requesting {state.start_service}")

    def _on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        startup_grace_s = max(0.0, float(self.get_parameter("startup_grace_s").value))
        if now_ns - self._startup_ns < int(startup_grace_s * 1e9):
            return

        scan_timeout_s = max(0.2, float(self.get_parameter("scan_timeout_s").value))
        for state in self._states.values():
            last_ns = state.last_scan_ns if state.last_scan_ns is not None else self._startup_ns
            age_s = (now_ns - last_ns) * 1e-9
            if age_s > scan_timeout_s:
                self._call_start_motor(state, now_ns)


def main() -> None:
    rclpy.init()
    node = LidarWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
