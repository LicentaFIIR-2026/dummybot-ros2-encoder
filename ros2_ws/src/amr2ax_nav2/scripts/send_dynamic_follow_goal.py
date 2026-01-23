#!/usr/bin/env python3
"""
Client action pentru Dynamic Object Following

Trimite un goal către navigator-ul dynamic_follow pentru testare programatică.

Usage:
    # Pornește dynamic following cu goal inițial
    ./send_dynamic_follow_goal.py --x 2.0 --y 1.0
    
    # Apoi publică goal updates pe /goal_update pentru a actualiza poziția obiectului

Author: SAIM Xplorer Team
Date: 2025-10-10
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import argparse
import math


class DynamicFollowGoalSender(Node):
    """Trimite goal către navigator pentru dynamic following."""
    
    def __init__(self):
        super().__init__('dynamic_follow_goal_sender')
        
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.get_logger().info('Dynamic Follow Goal Sender initialized')
    
    def send_goal(self, x, y, yaw=0.0, frame_id='map'):
        """Trimite goal la poziția specificată."""
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Setează poziția
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Setează orientarea (quaternion from yaw)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(
            f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°'
        )
        
        # Așteaptă server-ul
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Trimite goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback când goal-ul e acceptat."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted! Following object...')
        self.get_logger().info('Use RViz "Publish Point" or publish to /goal_update to update object position')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback când goal-ul se termină."""
        result = future.result().result
        self.get_logger().info(f'Result received!')
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        """Callback pentru feedback continuu."""
        feedback = feedback_msg.feedback
        
        # Log feedback periodic (la fiecare 5 secunde)
        if not hasattr(self, '_last_feedback_time'):
            self._last_feedback_time = 0
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self._last_feedback_time > 5.0:
            distance = feedback.distance_remaining
            self.get_logger().info(
                f'Following... Distance remaining: {distance:.2f}m'
            )
            self._last_feedback_time = current_time


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Send dynamic following goal to Nav2'
    )
    parser.add_argument('--x', type=float, default=2.0,
                       help='Goal X coordinate (default: 2.0)')
    parser.add_argument('--y', type=float, default=1.0,
                       help='Goal Y coordinate (default: 1.0)')
    parser.add_argument('--yaw', type=float, default=0.0,
                       help='Goal yaw in degrees (default: 0.0)')
    parser.add_argument('--frame', type=str, default='map',
                       help='Reference frame (default: map)')
    
    parsed_args, unknown = parser.parse_known_args()
    
    rclpy.init(args=args)
    
    action_client = DynamicFollowGoalSender()
    
    # Convertește yaw din grade în radiani
    yaw_rad = math.radians(parsed_args.yaw)
    
    # Trimite goal
    action_client.send_goal(
        parsed_args.x,
        parsed_args.y,
        yaw_rad,
        parsed_args.frame
    )
    
    # Spin până la finalizare
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Interrupted by user')
    finally:
        action_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
