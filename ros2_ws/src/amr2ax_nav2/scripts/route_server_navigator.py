#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeAndTrackRoute
from geometry_msgs.msg import PoseStamped
import json
import math

class RouteServerNavigator(Node):
    def __init__(self, graph_path):
        super().__init__('route_server_navigator')
        self._action_client = ActionClient(
            self, ComputeAndTrackRoute, 'compute_and_track_route')
        
        # Încarcă graph-ul pentru a obține coordonatele
        self.nodes = self.load_graph(graph_path)
        self.get_logger().info(f'Încărcat {len(self.nodes)} noduri din graph')
        
    def load_graph(self, filepath):
        """Extrage nodurile din graph GeoJSON"""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        nodes = {}
        for feature in data['features']:
            if feature['geometry']['type'] == 'Point':
                node_id = feature['properties']['id']
                coords = feature['geometry']['coordinates']
                name = feature['properties'].get('name', f'node_{node_id}')
                nodes[node_id] = {
                    'id': node_id,
                    'name': name,
                    'x': coords[0],
                    'y': coords[1]
                }
        return nodes
    
    def send_route(self, start_id, goal_id):
        """Trimite rută folosind coordonatele din graph"""
        if start_id not in self.nodes or goal_id not in self.nodes:
            self.get_logger().error(f'ID-uri invalide: {start_id} sau {goal_id}')
            return False
        
        start_node = self.nodes[start_id]
        goal_node = self.nodes[goal_id]
        
        goal_msg = ComputeAndTrackRoute.Goal()
        
        # Setează start
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.header.stamp = self.get_clock().now().to_msg()
        goal_msg.start.pose.position.x = start_node['x']
        goal_msg.start.pose.position.y = start_node['y']
        goal_msg.start.pose.orientation.w = 1.0
        
        # Setează goal
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = goal_node['x']
        goal_msg.goal.pose.position.y = goal_node['y']
        goal_msg.goal.pose.orientation.w = 1.0
        
        # Folosește poziții, nu ID-uri (workaround)
        goal_msg.use_start = True
        goal_msg.use_poses = True
        
        self.get_logger().info(f'Trimit rută: {start_node["name"]} -> {goal_node["name"]}')
        self.get_logger().info(f'  Start: ({start_node["x"]}, {start_node["y"]})')
        self.get_logger().info(f'  Goal: ({goal_node["x"]}, {goal_node["y"]})')
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'📍 Nod curent: {feedback.last_node_id} -> {feedback.next_node_id}')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal respins!')
            rclpy.shutdown()
            return
        
        self.get_logger().info('✅ Rută calculată! Robotul navigheaza...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🎉 Navigare completă! Cod: {result.error_code}')
        rclpy.shutdown()

def main():
    rclpy.init()
    
    graph_path = '/home/saim/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/route_graph.json'
    navigator = RouteServerNavigator(graph_path)
    
    print("\n🗺️  Noduri disponibile:")
    for node_id, node in navigator.nodes.items():
        print(f"  {node_id}: {node['name']} ({node['x']}, {node['y']})")
    
    print("\n" + "="*50)
    start_id = 0  # home
    goal_id = 2   # hol-spate
    print(f"📋 Rută: {navigator.nodes[start_id]['name']} -> {navigator.nodes[goal_id]['name']}")
    print("="*50)
    
    input("Apasă Enter pentru a începe...")
    
    if navigator.send_route(start_id, goal_id):
        rclpy.spin(navigator)

if __name__ == '__main__':
    main()
