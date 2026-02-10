#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeRoute, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
import json
import os

class SmartWaypointNavigator(Node):
    def __init__(self):
        super().__init__('smart_waypoint_navigator')
        
        self.compute_client = ActionClient(
            self,
            ComputeRoute,
            '/compute_route'
        )
        
        self.navigate_client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses'
        )

    def compute_and_navigate(self, goal_x, goal_y, goal_name):
        self.get_logger().info(f'Computing route to {goal_name} ({goal_x:.3f}, {goal_y:.3f})')
        
        goal_msg = ComputeRoute.Goal()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = goal_x
        goal_msg.goal.pose.position.y = goal_y
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.use_start = False
        goal_msg.use_poses = True

        self.compute_client.wait_for_server()
        self.get_logger().info('Route Server available, computing route...')

        future = self.compute_client.send_goal_async(goal_msg)
        future.add_done_callback(self.route_computed_callback)

    def route_computed_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('ComputeRoute goal respins!')
            rclpy.shutdown()
            return
        
        self.get_logger().info('Route computation accepted...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.route_result_callback)

    def route_result_callback(self, future):
        result = future.result().result
        
        if result.error_code != 0:
            self.get_logger().error(f'Route computation failed! Error: {result.error_code}')
            rclpy.shutdown()
            return
        
        self.get_logger().info(f'Route found! Cost: {result.route.route_cost:.2f}')
        self.get_logger().info(f'Nodes: {[n.nodeid for n in result.route.nodes]}')
        self.get_logger().info(f'Edges: {[e.edgeid for e in result.route.edges]}')
        
        if len(result.route.nodes) == 0:
            self.get_logger().error('Ruta goala! Muta robotul la pozitia start si incearca din nou.')
            rclpy.shutdown()
            return
        
        poses = []
        for node in result.route.nodes:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = node.position.x
            pose.pose.position.y = node.position.y
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        
        self.get_logger().info(f'Navigating through {len(poses)} waypoints...')
        self.navigate_through_poses(poses)

    def navigate_through_poses(self, poses):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.navigate_client.wait_for_server()
        
        future = self.navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_feedback_callback
        )
        future.add_done_callback(self.navigate_response_callback)

    def navigate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        now = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, '_last_feedback_time') or now - self._last_feedback_time > 2.0:
            self._last_feedback_time = now
            self.get_logger().info(
                f'Pos: ({feedback.current_pose.pose.position.x:.2f}, '
                f'{feedback.current_pose.pose.position.y:.2f}) | '
                f'Dist: {feedback.distance_remaining:.2f}m'
            )

    def navigate_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('NavigateThroughPoses goal respins!')
            rclpy.shutdown()
            return
        
        self.get_logger().info('Navigation accepted! Moving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigate_result_callback)

    def navigate_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Navigare completa cu succes!')
        else:
            self.get_logger().error(f'Navigare esuata! Error: {result.error_code}')
        rclpy.shutdown()


def load_nodes_from_graph(graph_path):
    with open(graph_path, 'r') as f:
        graph = json.load(f)

    nodes = []
    for feature in graph['features']:
        if feature['geometry']['type'] == 'Point':
            nodes.append({
                'id': feature['properties']['id'],
                'name': feature['properties']['name'],
                'x': feature['geometry']['coordinates'][0],
                'y': feature['geometry']['coordinates'][1]
            })

    nodes.sort(key=lambda n: n['id'])
    return nodes


def main():
    rclpy.init()

    graph_path = os.path.expanduser(
        '~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/route_graph.json'
    )

    nodes = load_nodes_from_graph(graph_path)
    node_dict = {n['id']: n for n in nodes}

    print(f"\nNoduri disponibile ({len(nodes)} total):")
    print("=" * 50)
    for node in nodes:
        print(f"  [{node['id']}] {node['name']}: ({node['x']:.3f}, {node['y']:.3f})")
    print("=" * 50)

    print("\nIntrodu ID-ul nodului GOAL:")
    print("(Route Server alege singur ruta optima!)")

    try:
        goal_id = int(input("Goal ID: ").strip())
    except ValueError:
        print("ID invalid!")
        rclpy.shutdown()
        return

    if goal_id not in node_dict:
        print(f"ID {goal_id} nu exista!")
        rclpy.shutdown()
        return

    goal = node_dict[goal_id]

    print(f"\nGoal: {goal['name']} ({goal['x']:.3f}, {goal['y']:.3f})")
    print("Route Server calculeaza ruta optima, apoi robotul navigheaza!")
    input("Apasa Enter pentru start...")

    navigator = SmartWaypointNavigator()
    navigator.compute_and_navigate(goal['x'], goal['y'], goal['name'])
    rclpy.spin(navigator)
    navigator.destroy_node()


if __name__ == '__main__':
    main()