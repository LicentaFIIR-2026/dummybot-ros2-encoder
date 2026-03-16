#!/usr/bin/env python3
"""
semantic_waypoint_navigation.py
-------------------------------
Navigatie prin waypoint-uri folosind Route Server pentru planificare ruta
Testeaza re-rutarea dinamica când muchiile sunt blocate de obiecte

Usage:
    python3 semantic_waypoint_navigation.py
    
Apoi în terminal separat:
    ros2 topic pub /navigate_to_goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.513, y: 2.768, z: 0.0}}}" --once
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav2_msgs.srv import GetRoute
import json

# Graph noduri (din route_graph.json)
NODES = {
    0: (1.097, 2.588),  # start
    1: (2.313, 3.215),  # in1
    2: (3.334, 2.844),  # int2
    3: (4.353, 3.299),  # altA
    4: (4.469, 2.454),  # altB
    5: (5.513, 2.768)   # goal
}


class SemanticWaypointNavigator(Node):
    """Navigator care foloseste Route Server pentru planificare"""
    
    def __init__(self):
        super().__init__('semantic_waypoint_navigator')
        
        # Service client pentru Route Server
        self.route_client = self.create_client(
            GetRoute,
            '/route_server/get_route'
        )
        
        # Action client pentru Nav2 waypoints
        self.waypoint_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )
        
        # Subscriber pentru comenzi de navigatie
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/navigate_to_goal',
            self.goal_callback,
            10
        )
        
        # Asteapta servicii
        while not self.route_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Route Server...')
        
        self.get_logger().info('🚀 Semantic Waypoint Navigator ready')
        self.get_logger().info('Send goal to /navigate_to_goal topic')
        
        # State
        self.current_goal = None
        self.is_navigating = False
    
    def goal_callback(self, msg: PoseStamped):
        """Primeste goal si porneste navigatia prin waypoints"""
        if self.is_navigating:
            self.get_logger().warn('Already navigating, ignoring new goal')
            return
        
        self.current_goal = msg
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        self.get_logger().info(f'📍 New goal received: ({goal_x:.2f}, {goal_y:.2f})')
        
        # Gaseste nodul cel mai apropiat de goal
        goal_node_id = self._find_nearest_node(goal_x, goal_y)
        
        self.get_logger().info(f'Goal mapped to node {goal_node_id}')
        
        # Cere ruta de la Route Server
        # Presupunem ca start e nodul 0 (poti schimba cu pozitia curenta a robotului)
        self._request_route(start_node=0, goal_node=goal_node_id)
    
    def _find_nearest_node(self, x: float, y: float) -> int:
        """Gaseste nodul cel mai apropiat de pozitia data"""
        import math
        min_dist = float('inf')
        nearest_id = 0
        
        for node_id, (nx, ny) in NODES.items():
            dist = math.hypot(x - nx, y - ny)
            if dist < min_dist:
                min_dist = dist
                nearest_id = node_id
        
        return nearest_id
    
    def _request_route(self, start_node: int, goal_node: int):
        """Cere ruta de la Route Server"""
        request = GetRoute.Request()
        request.start_node_id = start_node
        request.goal_node_id = goal_node
        
        self.get_logger().info(f'🔍 Requesting route: {start_node} → {goal_node}')
        
        future = self.route_client.call_async(request)
        future.add_done_callback(self._route_callback)
    
    def _route_callback(self, future):
        """Proceseaza raspunsul de la Route Server"""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Route service call failed: {e}')
            return
        
        if not response.success:
            self.get_logger().error('Route Server failed to find route')
            return
        
        # Parsare ruta (secventa de noduri)
        route_nodes = response.route_node_ids  # Lista de int
        
        self.get_logger().info(f'✅ Route received: {route_nodes}')
        
        # Converteste noduri in waypoints
        waypoints = []
        for node_id in route_nodes:
            if node_id not in NODES:
                self.get_logger().warn(f'Unknown node ID: {node_id}')
                continue
            
            x, y = NODES[node_id]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            waypoints.append(pose)
        
        if not waypoints:
            self.get_logger().error('No valid waypoints generated')
            return
        
        self.get_logger().info(f'🎯 Following {len(waypoints)} waypoints')
        
        # Trimite waypoints catre Nav2
        self._follow_waypoints(waypoints)
    
    def _follow_waypoints(self, waypoints: list):
        """Trimite waypoints catre Nav2 FollowWaypoints action"""
        if not self.waypoint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 FollowWaypoints action not available')
            return
        
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        
        self.is_navigating = True
        
        self.get_logger().info('🚀 Sending waypoints to Nav2...')
        
        send_goal_future = self.waypoint_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._waypoint_response_callback)
    
    def _waypoint_response_callback(self, future):
        """Callback pentru acceptare goal"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint navigation rejected')
            self.is_navigating = False
            return
        
        self.get_logger().info('✅ Waypoint navigation accepted')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._waypoint_result_callback)
    
    def _waypoint_result_callback(self, future):
        """Callback pentru rezultat final"""
        result = future.result().result
        self.is_navigating = False
        
        if len(result.missed_waypoints) == 0:
            self.get_logger().info('🎉 Navigation completed successfully!')
        else:
            self.get_logger().warn(
                f'Navigation completed with {len(result.missed_waypoints)} missed waypoints'
            )


def main(args=None):
    rclpy.init(args=args)
    navigator = SemanticWaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
