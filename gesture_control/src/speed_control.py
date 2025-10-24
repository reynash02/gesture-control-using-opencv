#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class CircleMotionNode(Node):
    def __init__(self):
        super().__init__('circle_motion_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.volume_subscriber = self.create_subscription(Int32, 'volume_percentage', self.volume_callback, 10)
        self.current_volume = 50
        self.timer = self.create_timer(0.1, self.publish_motion)
        self.get_logger().info('Circle motion node initialized')
    
    def volume_callback(self, msg):
        """Callback to receive volume percentage"""
        self.current_volume = msg.data
        self.get_logger().info(f'Received volume: {self.current_volume}%')
    
    def publish_motion(self):
        """Publish cmd_vel with speed based on volume percentage"""
        twist = Twist()
        # Convert volume percentage (0-100) to speed (0-1)
        speed = self.current_volume / 100.0
        twist.linear.x = speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().debug(f'Published cmd_vel with speed: {speed}')

def main(args=None):
    rclpy.init(args=args)
    node = CircleMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down circle motion node...')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
