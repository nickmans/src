"""
ROS2 trajectory generation node.

Subscribes to robot pose and generates smooth trajectory setpoints.
Provides trajectory data via service or callback.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
import math
import threading


class TrajectoryGeneratorNode(Node):
    """
    ROS2 node that generates trajectory setpoints.
    
    Subscribes to:
        - /robot/pose: Current robot pose
    
    Publishes:
        - /robot/trajectory: Planned trajectory path (for visualization)
        
    Services:
        - /get_trajectory_point: Get current trajectory setpoint
    """

    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Parameters
        self.declare_parameter('trajectory_mode', 'waypoint')  # 'waypoint', 'circle', 'hold'
        self.declare_parameter('waypoint_x', 1.0)
        self.declare_parameter('waypoint_y', 1.0)
        self.declare_parameter('waypoint_yaw', 0.0)
        self.declare_parameter('circle_radius', 1.0)
        self.declare_parameter('circle_speed', 0.2)
        self.declare_parameter('max_velocity', 0.5)  # m/s
        
        self.trajectory_mode = self.get_parameter('trajectory_mode').value
        
        # Subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )
        
        # Publisher
        self.traj_pub = self.create_publisher(Path, '/robot/trajectory', 10)
        
        # Current pose (thread-safe)
        self.pose_lock = threading.Lock()
        self.current_pose = None
        
        # Trajectory state
        self.trajectory_lock = threading.Lock()
        self.trajectory_setpoint = None
        self.trajectory_start_time = None
        
        # Timer for trajectory updates
        self.create_timer(0.2, self.update_trajectory)  # 5 Hz
        
        self.get_logger().info(f'Trajectory Generator Node initialized (mode: {self.trajectory_mode})')

    def pose_callback(self, msg: PoseStamped):
        """Update current pose."""
        with self.pose_lock:
            self.current_pose = msg

    def update_trajectory(self):
        """Update trajectory setpoint based on mode."""
        with self.pose_lock:
            current_pose = self.current_pose
        
        if current_pose is None:
            return
        
        # Get current position
        x_curr = current_pose.pose.position.x
        y_curr = current_pose.pose.position.y
        yaw_curr = self._quaternion_to_yaw(current_pose.pose.orientation)
        
        # Generate trajectory based on mode
        trajectory_mode = self.trajectory_mode
        
        if trajectory_mode == 'hold':
            # Hold current position
            x_des, y_des, yaw_des, vx, vy = self._generate_hold(x_curr, y_curr, yaw_curr)
        
        elif trajectory_mode == 'waypoint':
            # Move to waypoint
            x_des, y_des, yaw_des, vx, vy = self._generate_waypoint(x_curr, y_curr, yaw_curr)
        
        elif trajectory_mode == 'circle':
            # Circular trajectory
            x_des, y_des, yaw_des, vx, vy = self._generate_circle(x_curr, y_curr, yaw_curr)
        
        else:
            # Default to hold
            x_des, y_des, yaw_des, vx, vy = self._generate_hold(x_curr, y_curr, yaw_curr)
        
        # Store trajectory setpoint
        with self.trajectory_lock:
            self.trajectory_setpoint = {
                'x_des': x_des,
                'y_des': y_des,
                'yaw_des': yaw_des,
                'vx_world': vx,
                'vy_world': vy,
            }
        
        # Publish trajectory for visualization (optional)
        self._publish_trajectory_path(x_curr, y_curr, x_des, y_des)

    def get_trajectory_setpoint(self):
        """Get current trajectory setpoint (thread-safe)."""
        with self.trajectory_lock:
            return self.trajectory_setpoint

    def _generate_hold(self, x_curr, y_curr, yaw_curr):
        """Generate hold position trajectory."""
        return x_curr, y_curr, yaw_curr, 0.0, 0.0

    def _generate_waypoint(self, x_curr, y_curr, yaw_curr):
        """Generate trajectory toward waypoint."""
        # Get waypoint from parameters
        x_goal = self.get_parameter('waypoint_x').value
        y_goal = self.get_parameter('waypoint_y').value
        yaw_goal = self.get_parameter('waypoint_yaw').value
        max_vel = self.get_parameter('max_velocity').value
        
        # Compute error
        dx = x_goal - x_curr
        dy = y_goal - y_curr
        dist = math.sqrt(dx * dx + dy * dy)
        
        # If close to goal, hold position
        if dist < 0.05:  # 5cm threshold
            return x_goal, y_goal, yaw_goal, 0.0, 0.0
        
        # Compute desired velocity (proportional control with saturation)
        k_p = 1.0  # Proportional gain
        vx = k_p * dx
        vy = k_p * dy
        
        # Saturate velocity
        vel_mag = math.sqrt(vx * vx + vy * vy)
        if vel_mag > max_vel:
            vx = vx / vel_mag * max_vel
            vy = vy / vel_mag * max_vel
        
        # Desired position: current + dt * velocity
        dt = 0.2  # 5 Hz
        x_des = x_curr + vx * dt
        y_des = y_curr + vy * dt
        
        # Desired yaw: toward goal
        yaw_des = math.atan2(dy, dx)
        
        return x_des, y_des, yaw_des, vx, vy

    def _generate_circle(self, x_curr, y_curr, yaw_curr):
        """Generate circular trajectory."""
        radius = self.get_parameter('circle_radius').value
        speed = self.get_parameter('circle_speed').value
        
        # Initialize start time if needed
        if self.trajectory_start_time is None:
            self.trajectory_start_time = self.get_clock().now()
        
        # Compute time since start
        t = (self.get_clock().now() - self.trajectory_start_time).nanoseconds / 1e9
        
        # Angular position on circle
        omega = speed / radius  # angular velocity
        theta = omega * t
        
        # Circular path centered at origin
        x_des = radius * math.cos(theta)
        y_des = radius * math.sin(theta)
        yaw_des = theta + math.pi / 2  # tangent to circle
        
        # Velocity
        vx = -radius * omega * math.sin(theta)
        vy = radius * omega * math.cos(theta)
        
        return x_des, y_des, yaw_des, vx, vy

    def _publish_trajectory_path(self, x_curr, y_curr, x_des, y_des):
        """Publish trajectory path for visualization."""
        try:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'odom'
            
            # Add current position
            pose1 = PoseStamped()
            pose1.header = path_msg.header
            pose1.pose.position.x = x_curr
            pose1.pose.position.y = y_curr
            pose1.pose.position.z = 0.0
            path_msg.poses.append(pose1)
            
            # Add desired position
            pose2 = PoseStamped()
            pose2.header = path_msg.header
            pose2.pose.position.x = x_des
            pose2.pose.position.y = y_des
            pose2.pose.position.z = 0.0
            path_msg.poses.append(pose2)
            
            self.traj_pub.publish(path_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing trajectory path: {e}')

    @staticmethod
    def _quaternion_to_yaw(quat):
        """Convert quaternion to yaw angle."""
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = TrajectoryGeneratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
