#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, ComputePathToPose, FollowPath
from nav_msgs.msg import Path

class RVizGoalBridge(Node):
    def __init__(self):
        super().__init__('rviz_goal_bridge')
        
        # Subscribe la goal-uri din RViz
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Action clients
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')
        self.follow_path_client = ActionClient(
            self, FollowPath, 'follow_path')
        
        self.get_logger().info('RViz Goal Bridge started! Waiting for goals from RViz...')
        
    def goal_callback(self, msg):
        self.get_logger().info(f'Received goal from RViz: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        
        # Pas 1: Compute path
        self.compute_path(msg)
        
    def compute_path(self, goal_pose):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_msg.use_start = False  # Folosește poziția curentă
        
        self.get_logger().info('Computing path...')
        self.compute_path_client.wait_for_server()
        
        future = self.compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self.path_computed_callback)
        
    def path_computed_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Path computation rejected!')
            return
            
        self.get_logger().info('Path accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.follow_path_callback)
        
    def follow_path_callback(self, future):
        result = future.result().result
        path = result.path
        
        if len(path.poses) == 0:
            self.get_logger().error('No valid path found!')
            return
            
        self.get_logger().info(f'Path computed with {len(path.poses)} poses. Following path...')
        
        # Pas 2: Follow path
        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        
        self.follow_path_client.wait_for_server()
        future = self.follow_path_client.send_goal_async(follow_goal)
        future.add_done_callback(self.follow_done_callback)
        
    def follow_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Follow path rejected!')
            return
            
        self.get_logger().info('Robot is moving! Waiting for completion...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)
        
    def navigation_complete_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Navigation complete! Status: {result.status}')

def main():
    rclpy.init()
    bridge = RVizGoalBridge()
    rclpy.spin(bridge)

if __name__ == '__main__':
    main()
