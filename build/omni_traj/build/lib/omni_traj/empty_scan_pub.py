#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class EmptyScanPub(Node):
    def __init__(self):
        super().__init__('empty_scan_pub')

        self.declare_parameter('topic', '/scan')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('num_readings', 360)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 10.0)

        topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(LaserScan, topic, 10)

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.on_timer)

    def on_timer(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value

        angle_min = float(self.get_parameter('angle_min').value)
        angle_max = float(self.get_parameter('angle_max').value)
        n = int(self.get_parameter('num_readings').value)

        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = (angle_max - angle_min) / max(n - 1, 1)
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = float(self.get_parameter('range_min').value)
        msg.range_max = float(self.get_parameter('range_max').value)

        # Empty environment: all returns are "no hit"
        msg.ranges = [math.inf] * n
        msg.intensities = []

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = EmptyScanPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
