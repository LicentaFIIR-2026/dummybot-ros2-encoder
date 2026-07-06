#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
import yaml
import math
import sys
import os

class NamedWaypointNavigator(Node):
    def __init__(self, config_path):
        super().__init__('named_waypoint_navigator')
        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        
        # Încarcă waypoint-urile
        self.waypoints = self.load_waypoints(config_path)
        self.get_logger().info(f'Încărcat {len(self.waypoints)} waypoint-uri')
        
    def load_waypoints(self, filepath):
        """Încarcă waypoint-uri din named_waypoints.yaml"""
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('waypoints', {})
    
    def yaw_to_quaternion(self, yaw_degrees):
        """Convertește yaw din grade în quaternion"""
        yaw_rad = math.radians(yaw_degrees)
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw_rad / 2.0),
            'w': math.cos(yaw_rad / 2.0)
        }
    
    def send_route(self, waypoint_names):
        """Trimite o rută folosind numele waypoint-urilor"""
        goal_msg = NavigateThroughPoses.Goal()
        
        for name in waypoint_names:
            if name not in self.waypoints:
                self.get_logger().error(f'Waypoint "{name}" nu există!')
                return False
            
            wp = self.waypoints[name]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(wp['x'])
            pose.pose.position.y = float(wp['y'])
            pose.pose.position.z = 0.0
            
            # Convertește yaw în quaternion
            quat = self.yaw_to_quaternion(float(wp['yaw']))
            pose.pose.orientation.x = quat['x']
            pose.pose.orientation.y = quat['y']
            pose.pose.orientation.z = quat['z']
            pose.pose.orientation.w = quat['w']
            
            goal_msg.poses.append(pose)
            self.get_logger().info(f'  Adăugat: {name} -> ({wp["x"]}, {wp["y"]}, yaw={wp["yaw"]}°)')
        
        self.get_logger().info(f'Trimit ruta cu {len(waypoint_names)} waypoint-uri...')
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True
    
    def feedback_callback(self, feedback_msg):
        """Callback pentru feedback - ignorat pentru stabilitate"""
        pass
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal respins!')
            rclpy.shutdown()
            return
        
        self.get_logger().info('✅ Goal acceptat! Robotul navigheaza...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('🎉 Navigare completă!')
        rclpy.shutdown()

def main():
    rclpy.init()
    
    # Path către config
    config_path = os.path.expanduser(
        '~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/named_waypoints.yaml')
    
    navigator = NamedWaypointNavigator(config_path)
    
    # Afișează waypoint-urile disponibile
    print("\n🗺️  Waypoint-uri disponibile:")
    for name, wp in navigator.waypoints.items():
        print(f"  • {name}: ({wp['x']}, {wp['y']}, yaw={wp['yaw']}°)")
    
    print("\n" + "="*50)
    print("Exemple de rute:")
    print("  1. home -> usa-spate -> hol-spate")
    print("  2. hol-spate -> usa-spate -> home (patrulare)")
    print("  3. home -> hol-spate (directă)")
    print("="*50)
    
    # Definește ruta dorită (modifică aici!)
    route = ['home', 'usa-spate', 'hol-spate']
    
    print(f"\n📋 Ruta selectată: {' -> '.join(route)}")
    input("Apasă Enter pentru a începe navigarea...")
    
    if navigator.send_route(route):
        rclpy.spin(navigator)

if __name__ == '__main__':
    main()
