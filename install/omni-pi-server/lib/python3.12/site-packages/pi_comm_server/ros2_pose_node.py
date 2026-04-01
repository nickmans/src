"""
ROS2 node for publishing robot pose from UDP server.

Receives pose updates from UDP server and publishes to ROS2 topics.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import tf2_ros


class PosePublisherNode(Node):
    """
    ROS2 node that publishes robot pose data.
    
    Publishes:
        - /robot/pose (PoseStamped): Current position and orientation
        - /robot/twist (TwistStamped): Current velocities
        - /robot/odom (Odometry): Combined odometry message
    """

    def __init__(self):
        super().__init__('pose_publisher')

        self.declare_parameter('stm32_pose_rotation_deg', 0.0)
        self.declare_parameter('stm32_yaw_rotation_deg', 0.0)
        self.declare_parameter('stm32_yaw_offset_deg', 0.0)
        self.declare_parameter('stm32_traj_rotation_deg', 0.0)
        self.declare_parameter('stm32_traj_yaw_rotation_deg', 0.0)
        self.declare_parameter('stm32_traj_yaw_offset_deg', 0.0)
        self.declare_parameter('odom_stationary_linear_speed_thresh_ms', 0.03)
        self.declare_parameter('odom_stationary_angular_speed_thresh_rs', 0.05)

        initial_pose_qos = QoSProfile(depth=1)
        initial_pose_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        initial_pose_qos.reliability = QoSReliabilityPolicy.RELIABLE
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/robot/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/robot/twist', 10)
        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', 10)
        self.odom_pub_global = self.create_publisher(Odometry, '/odom', 10)
        
        # For initial pose (useful for AMCL, etc.)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            initial_pose_qos
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_pose_data = None
        self._pose_rotation_rad = math.radians(float(self.get_parameter('stm32_pose_rotation_deg').value))
        self._yaw_rotation_rad = math.radians(float(self.get_parameter('stm32_yaw_rotation_deg').value))
        self._yaw_offset_rad = math.radians(float(self.get_parameter('stm32_yaw_offset_deg').value))
        self._traj_rotation_rad = math.radians(float(self.get_parameter('stm32_traj_rotation_deg').value))
        self._traj_yaw_rotation_rad = math.radians(float(self.get_parameter('stm32_traj_yaw_rotation_deg').value))
        self._traj_yaw_offset_rad = math.radians(float(self.get_parameter('stm32_traj_yaw_offset_deg').value))
        pose_rotation_deg = float(self.get_parameter('stm32_pose_rotation_deg').value)
        yaw_rotation_deg = float(self.get_parameter('stm32_yaw_rotation_deg').value)
        traj_rotation_deg = float(self.get_parameter('stm32_traj_rotation_deg').value)
        traj_yaw_rotation_deg = float(self.get_parameter('stm32_traj_yaw_rotation_deg').value)
        if abs(pose_rotation_deg - yaw_rotation_deg) > 1e-6:
            self.get_logger().warn(
                "STM32->ROS transform uses different XY and yaw rotations; this can make map translation "
                "appear mirrored or sideways. Set stm32_pose_rotation_deg and stm32_yaw_rotation_deg "
                "to the same frame rotation unless you intentionally need a split calibration."
            )
        if abs(traj_rotation_deg - pose_rotation_deg) > 1e-6 or abs(traj_yaw_rotation_deg - yaw_rotation_deg) > 1e-6:
            self.get_logger().warn(
                "ROS->STM32 trajectory transform does not mirror STM32->ROS pose transform. "
                "Set stm32_traj_rotation_deg == stm32_pose_rotation_deg and "
                "stm32_traj_yaw_rotation_deg == stm32_yaw_rotation_deg unless intentionally offsetting trajectories."
            )
        self.get_logger().warn(
            f"Applying STM32->ROS pose transform: xy_rotation={pose_rotation_deg:.1f} deg, "
            f"yaw_rotation={yaw_rotation_deg:.1f} deg, "
            f"yaw_offset={float(self.get_parameter('stm32_yaw_offset_deg').value):.1f} deg"
        )
        self.get_logger().warn(
            "Applying ROS->STM32 trajectory transform: "
            f"xy_rotation={traj_rotation_deg:.1f} deg, "
            f"yaw_rotation={traj_yaw_rotation_deg:.1f} deg, "
            f"yaw_offset={float(self.get_parameter('stm32_traj_yaw_offset_deg').value):.1f} deg"
        )
        self.initial_pose_seed_started_ns = None
        self.initial_pose_seed_min_duration_ns = int(30.0 * 1e9)
        self.initial_pose_seed_max_duration_ns = int(60.0 * 1e9)
        self.localization_seed_confirmed = False
        self.initial_pose_timer = self.create_timer(1.0, self._refresh_initial_pose_seed)
        
        self.get_logger().info('Pose Publisher Node initialized')
        
        # Track if initial pose has been published
        self.initial_pose_published = False

    def publish_pose(self, pose_data):
        """
        Publish pose data to ROS2 topics.
        
        Args:
            pose_data: PoseData object from UDP server
        """
        try:
            self.latest_pose_data = pose_data

            # Convert STM32 pose convention into ROS odom convention before publishing
            x_ros, y_ros, yaw_ros, vx_ros, vy_ros, wz_ros = self._transform_stm32_pose_to_ros(
                pose_data.x,
                pose_data.y,
                pose_data.yaw,
                pose_data.vx,
                pose_data.vy,
                pose_data.wz,
            )

            # Create timestamp
            stamp = self.get_clock().now().to_msg()
            
            # Publish PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = 'odom'
            pose_msg.pose.position.x = x_ros
            pose_msg.pose.position.y = y_ros
            pose_msg.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            quat = self._yaw_to_quaternion(yaw_ros)
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            
            self.pose_pub.publish(pose_msg)
            
            # Publish TwistStamped
            twist_msg = TwistStamped()
            twist_msg.header.stamp = stamp
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = vx_ros
            twist_msg.twist.linear.y = vy_ros
            twist_msg.twist.linear.z = 0.0
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = wz_ros
            
            self.twist_pub.publish(twist_msg)
            
            # Publish Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            # Pose
            odom_msg.pose.pose.position.x = x_ros
            odom_msg.pose.pose.position.y = y_ros
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            
            # Twist
            odom_msg.twist.twist.linear.x = vx_ros
            odom_msg.twist.twist.linear.y = vy_ros
            odom_msg.twist.twist.angular.z = wz_ros
            
            pose_cov, twist_cov = self._build_odom_covariances(vx_ros, vy_ros, wz_ros)
            odom_msg.pose.covariance = pose_cov
            odom_msg.twist.covariance = twist_cov
            
            self.odom_pub.publish(odom_msg)

            # Also publish to /odom so downstream planners (e.g., waypoint_traj) receive odometry
            self.odom_pub_global.publish(odom_msg)
            
            # Publish initial pose once for localization nodes
            if not self.initial_pose_published:
                self._publish_initial_pose(pose_data)
                self.initial_pose_published = True
                self.initial_pose_seed_started_ns = self.get_clock().now().nanoseconds
            
        except Exception as e:
            self.get_logger().error(f'Error publishing pose: {e}')

    def _has_map_to_odom_transform(self):
        try:
            self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            return True
        except Exception:
            return False

    def _refresh_initial_pose_seed(self):
        if self.latest_pose_data is None or self.localization_seed_confirmed:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.initial_pose_seed_started_ns is None:
            self.initial_pose_seed_started_ns = now_ns

        elapsed_ns = now_ns - self.initial_pose_seed_started_ns

        if elapsed_ns >= self.initial_pose_seed_min_duration_ns and self._has_map_to_odom_transform():
            self.localization_seed_confirmed = True
            self.get_logger().info('Detected map->odom transform after startup; stopping /initialpose reseeding')
            return

        if elapsed_ns >= self.initial_pose_seed_max_duration_ns:
            self.localization_seed_confirmed = True
            self.get_logger().warn('Stopping /initialpose reseeding after timeout; map->odom transform still not confirmed')
            return

        self._publish_initial_pose(self.latest_pose_data)

    @staticmethod
    def _wrap_to_pi(angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

    def _rotate_xy(self, x: float, y: float) -> tuple[float, float]:
        c = math.cos(self._pose_rotation_rad)
        s = math.sin(self._pose_rotation_rad)
        return (c * x - s * y, s * x + c * y)

    @staticmethod
    def _rotate_xy_by(x: float, y: float, angle_rad: float) -> tuple[float, float]:
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return (c * x - s * y, s * x + c * y)

    def _transform_stm32_pose_to_ros(
        self,
        x: float,
        y: float,
        yaw: float,
        vx: float,
        vy: float,
        wz: float,
    ) -> tuple[float, float, float, float, float, float]:
        x_ros, y_ros = self._rotate_xy(float(x), float(y))
        vx_ros, vy_ros = self._rotate_xy(float(vx), float(vy))
        yaw_ros = self._wrap_to_pi(float(yaw) + self._yaw_rotation_rad + self._yaw_offset_rad)
        return x_ros, y_ros, yaw_ros, vx_ros, vy_ros, float(wz)

    def transform_ros_pose_to_stm(
        self,
        x_ros: float,
        y_ros: float,
        yaw_ros: float,
        vx_ros: float,
        vy_ros: float,
        wz_ros: float,
    ) -> tuple[float, float, float, float, float, float]:
        """Convert ROS odom-frame pose/twist back into STM32 convention."""
        c = math.cos(-self._pose_rotation_rad)
        s = math.sin(-self._pose_rotation_rad)

        x_stm = c * float(x_ros) - s * float(y_ros)
        y_stm = s * float(x_ros) + c * float(y_ros)
        vx_stm = c * float(vx_ros) - s * float(vy_ros)
        vy_stm = s * float(vx_ros) + c * float(vy_ros)
        yaw_stm = self._wrap_to_pi(float(yaw_ros) - self._yaw_rotation_rad - self._yaw_offset_rad)

        return x_stm, y_stm, yaw_stm, vx_stm, vy_stm, float(wz_ros)

    def transform_ros_traj_to_stm(
        self,
        x_ros: float,
        y_ros: float,
        yaw_ros: float,
        vx_ros: float,
        vy_ros: float,
        wz_ros: float,
    ) -> tuple[float, float, float, float, float, float]:
        """Convert ROS odom-frame trajectory pose/twist into STM32 trajectory convention."""
        x_stm, y_stm = self._rotate_xy_by(float(x_ros), float(y_ros), -self._traj_rotation_rad)
        vx_stm, vy_stm = self._rotate_xy_by(float(vx_ros), float(vy_ros), -self._traj_rotation_rad)
        yaw_stm = self._wrap_to_pi(float(yaw_ros) - self._traj_yaw_rotation_rad - self._traj_yaw_offset_rad)
        return x_stm, y_stm, yaw_stm, vx_stm, vy_stm, float(wz_ros)

    def _build_odom_covariances(self, vx: float, vy: float, wz: float):
        speed = math.hypot(float(vx), float(vy))
        wz_abs = abs(float(wz))
        linear_thresh = float(self.get_parameter('odom_stationary_linear_speed_thresh_ms').value)
        angular_thresh = float(self.get_parameter('odom_stationary_angular_speed_thresh_rs').value)
        is_stationary = (speed <= linear_thresh) and (wz_abs <= angular_thresh)

        if is_stationary:
            var_x = 0.05 * 0.05
            var_y = 0.05 * 0.05
            var_yaw = math.radians(2.0) ** 2
            var_vx = 0.08 * 0.08
            var_vy = 0.08 * 0.08
            var_wz = math.radians(3.0) ** 2
        else:
            var_x = 0.12 * 0.12
            var_y = 0.12 * 0.12
            var_yaw = math.radians(4.0) ** 2
            var_vx = 0.20 * 0.20
            var_vy = 0.20 * 0.20
            var_wz = math.radians(6.0) ** 2

        large = 1e6
        pose_cov = [0.0] * 36
        pose_cov[0] = var_x
        pose_cov[7] = var_y
        pose_cov[14] = large
        pose_cov[21] = large
        pose_cov[28] = large
        pose_cov[35] = var_yaw

        twist_cov = [0.0] * 36
        twist_cov[0] = var_vx
        twist_cov[7] = var_vy
        twist_cov[14] = large
        twist_cov[21] = large
        twist_cov[28] = large
        twist_cov[35] = var_wz

        return pose_cov, twist_cov

    def _publish_initial_pose(self, pose_data):
        """Publish initial pose for localization."""
        try:
            x_ros, y_ros, yaw_ros, _vx_ros, _vy_ros, _wz_ros = self._transform_stm32_pose_to_ros(
                pose_data.x,
                pose_data.y,
                pose_data.yaw,
                pose_data.vx,
                pose_data.vy,
                pose_data.wz,
            )

            initial_pose_msg = PoseWithCovarianceStamped()
            initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
            initial_pose_msg.header.frame_id = 'map'
            
            initial_pose_msg.pose.pose.position.x = x_ros
            initial_pose_msg.pose.pose.position.y = y_ros
            initial_pose_msg.pose.pose.position.z = 0.0
            
            quat = self._yaw_to_quaternion(yaw_ros)
            initial_pose_msg.pose.pose.orientation.x = quat[0]
            initial_pose_msg.pose.pose.orientation.y = quat[1]
            initial_pose_msg.pose.pose.orientation.z = quat[2]
            initial_pose_msg.pose.pose.orientation.w = quat[3]
            
            # Covariance
            initial_pose_msg.pose.covariance = [0.1] * 36
            
            self.initial_pose_pub.publish(initial_pose_msg)
            self.get_logger().info('Published initial pose for localization')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing initial pose: {e}')

    @staticmethod
    def _yaw_to_quaternion(yaw):
        """
        Convert yaw angle to quaternion.
        
        Args:
            yaw: Yaw angle in radians
            
        Returns:
            Tuple (x, y, z, w) quaternion
        """
        # For rotation around Z axis only
        half_yaw = yaw * 0.5
        return (
            0.0,
            0.0,
            math.sin(half_yaw),
            math.cos(half_yaw)
        )


def main(args=None):
    """Main function for standalone testing."""
    rclpy.init(args=args)
    
    node = PosePublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
