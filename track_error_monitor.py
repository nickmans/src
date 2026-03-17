#!/usr/bin/env python3
import math
import time
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class TrackErrorMonitor(Node):
    def __init__(self) -> None:
        super().__init__("track_error_monitor")
        self.odom_xy: Optional[Tuple[float, float]] = None
        self.path_xy: List[Tuple[float, float]] = []
        self.path_stamp_ns: int = 0

        self.sample_count = 0
        self.sum_sq_err = 0.0
        self.max_err = 0.0
        self.last_print = time.monotonic()

        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(Path, "/planned_path", self._on_path, 10)
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info("Tracking monitor ready: waiting for /odom and /planned_path")

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def _on_path(self, msg: Path) -> None:
        pts: List[Tuple[float, float]] = []
        for ps in msg.poses:
            pts.append((float(ps.pose.position.x), float(ps.pose.position.y)))
        self.path_xy = pts
        self.path_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    @staticmethod
    def _nearest_dist(p: Tuple[float, float], path: List[Tuple[float, float]]) -> float:
        px, py = p
        best = float("inf")
        for x, y in path:
            d = math.hypot(px - x, py - y)
            if d < best:
                best = d
        return best

    def _tick(self) -> None:
        if self.odom_xy is None or not self.path_xy:
            return

        now_ns = self.get_clock().now().nanoseconds
        path_age_s = 0.0
        if self.path_stamp_ns > 0:
            path_age_s = max(0.0, (now_ns - self.path_stamp_ns) * 1e-9)

        err = self._nearest_dist(self.odom_xy, self.path_xy)
        goal_x, goal_y = self.path_xy[-1]
        goal_dist = math.hypot(self.odom_xy[0] - goal_x, self.odom_xy[1] - goal_y)

        self.sample_count += 1
        self.sum_sq_err += err * err
        self.max_err = max(self.max_err, err)
        rms = math.sqrt(self.sum_sq_err / max(1, self.sample_count))

        tnow = time.monotonic()
        if tnow - self.last_print >= 0.5:
            self.last_print = tnow
            self.get_logger().info(
                f"cte={err:.3f}m rms={rms:.3f}m max={self.max_err:.3f}m goal_dist={goal_dist:.3f}m path_pts={len(self.path_xy)} path_age={path_age_s:.2f}s"
            )


def main() -> None:
    rclpy.init()
    node = TrackErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok() and node.sample_count > 0:
                rms = math.sqrt(node.sum_sq_err / max(1, node.sample_count))
                node.get_logger().info(
                    f"FINAL: samples={node.sample_count} rms={rms:.3f}m max={node.max_err:.3f}m"
                )
        except Exception:
            pass

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
