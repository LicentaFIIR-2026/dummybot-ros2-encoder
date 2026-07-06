#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped

class DirectNavigator(Node):
    def __init__(self):
        super().__init__('direct_navigator')
        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        
    def send_waypoints(self, waypoints):
        goal_msg = NavigateThroughPoses.Goal()
        
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.orientation.w = 1.0
            goal_msg.poses.append(pose)
        
        self.get_logger().info(f'Trimit {len(waypoints)} waypoint-uri...')
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal respins!')
            return
        
        self.get_logger().info('Navigare in curs...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        self.get_logger().info('Completat!')
        rclpy.shutdown()

def main():
    rclpy.init()
    navigator = DirectNavigator()
    
    waypoints = [
        (0.288, -1.183),   # int1
        (0.230, 0.133),    # capat-masa
        (0.317, 1.000),    # int2
    ]
    
    print(f"Trimit {len(waypoints)} waypoint-uri")
    input("Enter pentru start...")
    
    navigator.send_waypoints(waypoints)
    rclpy.spin(navigator)

if __name__ == '__main__':
    main()
