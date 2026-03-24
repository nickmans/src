#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MapOdomStartupFallback(Node):
    """Publish identity map->odom for a short startup grace window.

    This keeps RViz/map-frame visualization stable while localization (AMCL)
    is initializing after mode switches/restarts.
    """

    def __init__(self) -> None:
        super().__init__("map_odom_startup_fallback")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("grace_period_s", 25.0)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._grace_period = max(0.0, float(self.get_parameter("grace_period_s").value))
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        self._tf_broadcaster = TransformBroadcaster(self)
        self._start_time = self.get_clock().now()
        self._active = True

        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

        self.get_logger().info(
            f"Startup map->odom fallback active for {self._grace_period:.1f}s "
            f"({self._map_frame}->{self._odom_frame})"
        )

    def _on_timer(self) -> None:
        if not self._active:
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        if elapsed >= self._grace_period:
            self._active = False
            self.get_logger().info("Startup map->odom fallback grace window ended")
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._tf_broadcaster.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = MapOdomStartupFallback()
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
