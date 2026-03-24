#!/usr/bin/env python3

from __future__ import annotations

import math

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("grace_period_s", 25.0)
        self.declare_parameter("seed_initial_pose", True)
        self.declare_parameter("initial_pose_publish_rate_hz", 2.0)
        self.declare_parameter("initial_pose_min_duration_s", 8.0)
        self.declare_parameter("initial_pose_covariance_xy", 0.10)
        self.declare_parameter("initial_pose_covariance_yaw", 0.20)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._grace_period = max(0.0, float(self.get_parameter("grace_period_s").value))
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._seed_initial_pose = bool(self.get_parameter("seed_initial_pose").value)
        self._initial_pose_min_duration_s = max(
            0.0, float(self.get_parameter("initial_pose_min_duration_s").value)
        )
        self._initial_pose_covariance_xy = max(
            1e-4, float(self.get_parameter("initial_pose_covariance_xy").value)
        )
        self._initial_pose_covariance_yaw = max(
            1e-4, float(self.get_parameter("initial_pose_covariance_yaw").value)
        )
        initial_pose_publish_rate_hz = max(
            0.2, float(self.get_parameter("initial_pose_publish_rate_hz").value)
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self._start_time = self.get_clock().now()
        self._active = True
        self._latest_odom: Odometry | None = None
        self._amcl_pose_received = False
        self._warned_no_odom = False
        self._published_initial_pose = False

        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

        if self._seed_initial_pose:
            initial_pose_qos = QoSProfile(depth=1)
            initial_pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            initial_pose_qos.reliability = ReliabilityPolicy.RELIABLE

            self._initial_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped,
                "/initialpose",
                initial_pose_qos,
            )
            self._odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 20)
            self._amcl_pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                "/amcl_pose",
                self._on_amcl_pose,
                20,
            )
            self._initial_pose_timer = self.create_timer(
                1.0 / initial_pose_publish_rate_hz,
                self._seed_initial_pose_from_odom,
            )
        else:
            self._initial_pose_pub = None
            self._odom_sub = None
            self._amcl_pose_sub = None
            self._initial_pose_timer = None

        self.get_logger().info(
            f"Startup map->odom fallback active for {self._grace_period:.1f}s "
            f"({self._map_frame}->{self._odom_frame})"
        )
        if self._seed_initial_pose:
            self.get_logger().info(
                "Startup initial-pose seeding active from /odom until /amcl_pose arrives"
            )

    def _elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        if msg.child_frame_id and msg.child_frame_id != self._base_frame:
            return
        self._latest_odom = msg

    def _on_amcl_pose(self, _: PoseWithCovarianceStamped) -> None:
        self._amcl_pose_received = True

    def _seed_initial_pose_from_odom(self) -> None:
        if not self._active or not self._seed_initial_pose or self._initial_pose_pub is None:
            return

        elapsed = self._elapsed_seconds()
        if elapsed >= self._grace_period:
            return

        if self._amcl_pose_received and elapsed >= self._initial_pose_min_duration_s:
            return

        if self._latest_odom is None:
            if not self._warned_no_odom:
                self.get_logger().warn("Startup seeding waiting for /odom before publishing /initialpose")
                self._warned_no_odom = True
            return

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._map_frame
        pose.pose.pose = self._latest_odom.pose.pose
        pose.pose.covariance[0] = self._initial_pose_covariance_xy
        pose.pose.covariance[7] = self._initial_pose_covariance_xy
        pose.pose.covariance[35] = self._initial_pose_covariance_yaw

        orientation = pose.pose.pose.orientation
        norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if norm < 1e-6:
            orientation.x = 0.0
            orientation.y = 0.0
            orientation.z = 0.0
            orientation.w = 1.0

        self._initial_pose_pub.publish(pose)
        if not self._published_initial_pose:
            self._published_initial_pose = True
            self.get_logger().info(
                "Published startup /initialpose seed from live /odom to bootstrap AMCL"
            )

    def _on_timer(self) -> None:
        if not self._active:
            return

        elapsed = self._elapsed_seconds()
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
