"""
ROS2 node for publishing robot pose from TCP server.

Receives pose updates from TCP server and publishes to ROS2 topics.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math


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
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/robot/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/robot/twist', 10)
        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', 10)
        
        # For initial pose (useful for AMCL, etc.)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        self.get_logger().info('Pose Publisher Node initialized')
        
        # Track if initial pose has been published
        self.initial_pose_published = False

    def publish_pose(self, pose_data):
        """
        Publish pose data to ROS2 topics.
        
        Args:
            pose_data: PoseData object from TCP server
        """
        try:
            # Create timestamp
            stamp = self.get_clock().now().to_msg()
            
            # Publish PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = 'odom'
            pose_msg.pose.position.x = pose_data.x
            pose_msg.pose.position.y = pose_data.y
            pose_msg.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            quat = self._yaw_to_quaternion(pose_data.yaw)
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            
            self.pose_pub.publish(pose_msg)
            
            # Publish TwistStamped
            twist_msg = TwistStamped()
            twist_msg.header.stamp = stamp
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = pose_data.vx
            twist_msg.twist.linear.y = pose_data.vy
            twist_msg.twist.linear.z = 0.0
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = pose_data.wz
            
            self.twist_pub.publish(twist_msg)
            
            # Publish Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            # Pose
            odom_msg.pose.pose.position.x = pose_data.x
            odom_msg.pose.pose.position.y = pose_data.y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            
            # Twist
            odom_msg.twist.twist.linear.x = pose_data.vx
            odom_msg.twist.twist.linear.y = pose_data.vy
            odom_msg.twist.twist.angular.z = pose_data.wz
            
            # Covariance (placeholder - should be tuned based on sensor accuracy)
            odom_msg.pose.covariance = [0.01] * 36
            odom_msg.twist.covariance = [0.01] * 36
            
            self.odom_pub.publish(odom_msg)
            
            # Publish initial pose once for localization nodes
            if not self.initial_pose_published:
                self._publish_initial_pose(pose_data)
                self.initial_pose_published = True
            
        except Exception as e:
            self.get_logger().error(f'Error publishing pose: {e}')

    def _publish_initial_pose(self, pose_data):
        """Publish initial pose for localization."""
        try:
            initial_pose_msg = PoseWithCovarianceStamped()
            initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
            initial_pose_msg.header.frame_id = 'map'
            
            initial_pose_msg.pose.pose.position.x = pose_data.x
            initial_pose_msg.pose.pose.position.y = pose_data.y
            initial_pose_msg.pose.pose.position.z = 0.0
            
            quat = self._yaw_to_quaternion(pose_data.yaw)
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
