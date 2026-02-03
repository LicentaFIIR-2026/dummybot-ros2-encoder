#!/usr/bin/env python3
"""
Waypoint Service Node
Expune waypoint-urile ca servicii ROS2 pe care Claude le poate apela
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import yaml
import os
import math

class WaypointServiceNode(Node):
    def __init__(self):
        super().__init__('waypoint_service_node')
        
        # Path către waypoint-uri
        self.waypoints_file = os.path.expanduser(
            '~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/named_waypoints.yaml'
        )
        
        # Publisher pentru goal
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Încarcă waypoint-uri
        self.waypoints = self.load_waypoints()
        
        # Creează servicii pentru fiecare waypoint
        self.waypoint_services = {}
        for name in self.waypoints.keys():
            service_name = f'/navigate_to_{name.replace("-", "_")}'
            
            # Folosim funcție separată pentru fiecare waypoint
            def create_callback(wp_name):
                def callback(request, response):
                    return self.navigate_callback(wp_name, request, response)
                return callback
            
            self.waypoint_services[name] = self.create_service(
                Trigger,
                service_name,
                create_callback(name)
            )
            self.get_logger().info(f'Serviciu creat: {service_name}')
        
        # Serviciu pentru listare
        self.list_service = self.create_service(
            Trigger,
            '/list_waypoints',
            self.list_callback
        )
        
        self.get_logger().info(f'Waypoint Service Node pornit cu {len(self.waypoints)} waypoint-uri')
    
    def load_waypoints(self):
        """Încarcă waypoint-uri din YAML"""
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'waypoints' in data:
                        return data['waypoints']
            except Exception as e:
                self.get_logger().error(f'Eroare încărcare: {e}')
        return {}
    
    def quaternion_from_euler(self, yaw_deg):
        """Convertește yaw în quaternion"""
        yaw = math.radians(yaw_deg)
        return {
            'z': math.sin(yaw * 0.5),
            'w': math.cos(yaw * 0.5)
        }
    
    def navigate_callback(self, waypoint_name, request, response):
        """Callback pentru navigare la waypoint"""
        if waypoint_name not in self.waypoints:
            response.success = False
            response.message = f'Waypoint {waypoint_name} nu există'
            return response
        
        wp = self.waypoints[waypoint_name]
        
        # Creează PoseStamped
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = wp['x']
        goal.pose.position.y = wp['y']
        goal.pose.position.z = 0.0
        
        quat = self.quaternion_from_euler(wp['yaw'])
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = quat['z']
        goal.pose.orientation.w = quat['w']
        
        # Publică goal
        self.goal_pub.publish(goal)
        
        response.success = True
        response.message = f'Navigare la {waypoint_name}: ({wp["x"]:.2f}, {wp["y"]:.2f}, {wp["yaw"]}°)'
        
        self.get_logger().info(response.message)
        return response
    
    def list_callback(self, request, response):
        """Callback pentru listare waypoint-uri"""
        response.success = True
        
        waypoint_list = []
        for name, wp in self.waypoints.items():
            waypoint_list.append(f"{name}: ({wp['x']:.2f}, {wp['y']:.2f}, {wp['yaw']}°)")
        
        response.message = "\n".join(waypoint_list)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()