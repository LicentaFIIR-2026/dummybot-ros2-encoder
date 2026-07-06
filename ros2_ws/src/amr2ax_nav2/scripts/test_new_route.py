#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class TestNewRoute(Node):
    def __init__(self):
        super().__init__('test_new_route')
        self.get_logger().info('Testing new route: start → goal')
        
        # Navighează de la start la goal
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 5.513
        pose.pose.position.y = 2.768
        pose.pose.orientation.w = 1.0
        
        self.get_logger().info('Navigate to GOAL - should use altA or altB')

def main():
    rclpy.init()
    node = TestNewRoute()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
