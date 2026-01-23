#!/usr/bin/env python3
"""
Nod convertor pentru Dynamic Object Following

Convertește clicked points din RViz în goal updates pentru behavior tree-ul
de dynamic following.

Input: /clicked_point (geometry_msgs/PointStamped)
Output: /goal_update (geometry_msgs/PoseStamped)

Author: SAIM Xplorer Team
Date: 2025-10-10
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import math


class ClickedPointToGoalUpdate(Node):
    """Convertește clicked points în goal updates cu orientare calculată."""
    
    def __init__(self):
        super().__init__('clicked_point_to_goal_update')
        
        # Parametri configurabili
        self.declare_parameter('input_topic', '/clicked_point')
        self.declare_parameter('output_topic', '/goal_update')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('calculate_orientation', True)
        self.declare_parameter('default_yaw', 0.0)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.calculate_orientation = self.get_parameter('calculate_orientation').value
        self.default_yaw = self.get_parameter('default_yaw').value
        
        # Subscriber și Publisher
        self.subscription = self.create_subscription(
            PointStamped,
            input_topic,
            self.clicked_point_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            PoseStamped,
            output_topic,
            10
        )
        
        # TF2 pentru calcul orientare
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Ultimul goal pentru logging
        self.last_goal_pose = None
        
        self.get_logger().info(
            f'Clicked Point to Goal Update node started\n'
            f'  Input: {input_topic}\n'
            f'  Output: {output_topic}\n'
            f'  Calculate orientation: {self.calculate_orientation}'
        )
    
    def clicked_point_callback(self, msg: PointStamped):
        """Procesează clicked point și publică goal update."""
        
        goal = PoseStamped()
        goal.header = msg.header
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Setează poziția
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.position.z = msg.point.z
        
        # Calculează orientarea
        if self.calculate_orientation:
            try:
                # Obține transformarea către base_link
                transform = self.tf_buffer.lookup_transform(
                    msg.header.frame_id,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0)
                )
                
                # Calculează yaw către punctul clicked
                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y
                
                dx = msg.point.x - robot_x
                dy = msg.point.y - robot_y
                yaw = math.atan2(dy, dx)
                
                # Convertește yaw în quaternion
                goal.pose.orientation.z = math.sin(yaw / 2.0)
                goal.pose.orientation.w = math.cos(yaw / 2.0)
                
                self.get_logger().info(
                    f'Goal update: ({msg.point.x:.2f}, {msg.point.y:.2f}) '
                    f'yaw: {math.degrees(yaw):.1f}°'
                )
                
            except Exception as e:
                self.get_logger().warn(
                    f'Could not calculate orientation: {e}. Using default yaw.'
                )
                # Folosește orientare default
                goal.pose.orientation.z = math.sin(self.default_yaw / 2.0)
                goal.pose.orientation.w = math.cos(self.default_yaw / 2.0)
        else:
            # Orientare default (yaw = 0)
            goal.pose.orientation.z = math.sin(self.default_yaw / 2.0)
            goal.pose.orientation.w = math.cos(self.default_yaw / 2.0)
            
            self.get_logger().info(
                f'Goal update: ({msg.point.x:.2f}, {msg.point.y:.2f}) '
                f'with default orientation'
            )
        
        # Publică goal update
        self.publisher.publish(goal)
        self.last_goal_pose = goal


def main(args=None):
    rclpy.init(args=args)
    node = ClickedPointToGoalUpdate()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down clicked_point_to_goal_update node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
