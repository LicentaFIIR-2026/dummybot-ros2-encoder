#!/usr/bin/env python3
"""
test_route_navigation.py - WORKING NavigateThroughPoses
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeRoute, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
import time


class RouteNavigator(Node):
    def __init__(self):
        super().__init__('route_navigator')
        
        self.route_client = ActionClient(self, ComputeRoute, '/compute_route')
        self.nav_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        
        self.complete = False
        self.success = False
        
        self.get_logger().info('🚀 Waiting for servers...')
        self.route_client.wait_for_server()
        self.nav_client.wait_for_server()
        self.get_logger().info('✅ Ready!')
    
    def navigate_to_node(self, goal_node_id: int):
        self.get_logger().info(f'📍 Computing route to node {goal_node_id}')
        
        # Step 1: Route Server
        route_goal = ComputeRoute.Goal()
        route_goal.start_id = 0
        route_goal.goal_id = goal_node_id
        route_goal.use_start = False
        route_goal.use_poses = False
        
        send_future = self.route_client.send_goal_async(route_goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Route rejected!')
            self.complete = True
            return
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        
        result = result_future.result().result
        
        if result.error_code != 0:
            self.get_logger().error(f'❌ Route failed: {result.error_code}')
            self.complete = True
            return
        
        # Log rută
        route = result.route
        nodes = [n.nodeid for n in route.nodes]
        edges = [e.edgeid for e in route.edges]
        
        self.get_logger().info(
            f'🛣️  Route computed:\n'
            f'    Nodes: {nodes}\n'
            f'    Edges: {edges}\n'
            f'    Cost: {route.route_cost:.2f}\n'
            f'    Path poses: {len(result.path.poses)}'
        )
        
        # Step 2: Extrage DOAR nodurile principale (nu toate cele 98 poses)
        waypoints = self.extract_node_waypoints(result)
        
        if not waypoints:
            self.get_logger().error('❌ No waypoints!')
            self.complete = True
            return
        
        self.get_logger().info(f'🎯 Using {len(waypoints)} waypoints from graph nodes')
        
        # Step 3: Navigate through waypoints
        self.navigate_waypoints(waypoints)
    
    def extract_node_waypoints(self, route_result):
        """
        Extrage DOAR poses-urile corespunzătoare nodurilor din graph
        Nu toate cele 98 poses, ci doar ~5 noduri cheie
        """
        route = route_result.route
        current_time = self.get_clock().now().to_msg()
        
        waypoints = []
        
        # Pentru fiecare nod din rută, creează un waypoint
        for node in route.nodes:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = current_time
            pose.pose.position.x = node.position.x
            pose.pose.position.y = node.position.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            waypoints.append(pose)
            
            self.get_logger().info(
                f'  Waypoint {node.nodeid}: ({node.position.x:.2f}, {node.position.y:.2f})'
            )
        
        return waypoints
    
    def navigate_waypoints(self, waypoints):
        """Navigate through poses cu BT gol (default)"""
        
        nav_goal = NavigateThroughPoses.Goal()
        nav_goal.poses = waypoints
        nav_goal.behavior_tree = ''  # Folosește BT default
        
        self.get_logger().info(f'🚀 Starting navigation through {len(waypoints)} waypoints...')
        
        send_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.nav_feedback
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Navigation rejected!')
            self.complete = True
            return
        
        self.get_logger().info('✅ Navigation started - following waypoints...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0)
        
        if result_future.done():
            result = result_future.result()
            if result.status == 4:  # SUCCEEDED
                self.get_logger().info('🎉 All waypoints reached!')
                self.success = True
            else:
                self.get_logger().warn(f'⚠️  Navigation status: {result.status}')
        else:
            self.get_logger().error('❌ Timeout!')
        
        self.complete = True
    
    def nav_feedback(self, feedback_msg):
        """Log progres"""
        feedback = feedback_msg.feedback
        remaining = feedback.number_of_poses_remaining
        
        if remaining > 0:
            self.get_logger().info(
                f'📍 Progress: {remaining} waypoints remaining, '
                f'dist={feedback.distance_remaining:.2f}m',
                throttle_duration_sec=5.0  # Log la fiecare 5s
            )


def main():
    rclpy.init()
    nav = RouteNavigator()
    
    try:
        nav.navigate_to_node(goal_node_id=5)
        
        while rclpy.ok() and not nav.complete:
            rclpy.spin_once(nav, timeout_sec=0.1)
            time.sleep(0.1)
        
        if nav.success:
            nav.get_logger().info('✅ Mission complete!')
        
    except KeyboardInterrupt:
        nav.get_logger().info('⚠️  Interrupted')
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()