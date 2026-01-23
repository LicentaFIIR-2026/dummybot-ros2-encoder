#!/usr/bin/env python3

import os
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.waypoints = self.load_waypoints()
        self.current_waypoint_idx = 0
        self.loop = True  # Set to True for continuous looping

        # Wait for the action server to be available
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 FollowWaypoints action server not available!')
            raise SystemExit

        self.get_logger().info('Waypoint Follower initialized. Starting navigation...')
        self.send_waypoints()

    def load_waypoints(self):
        # Path to waypoints.yaml
        waypoints_file = os.path.expanduser('~/amr2AX_ws/src/amr2ax_nav2/config/waypoints.yaml')
        try:
            with open(waypoints_file, 'r') as file:
                data = yaml.safe_load(file)
                waypoints = []
                for wp in data['waypoints']:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'  # Assuming waypoints are in the map frame
                    pose.pose.position.x = float(wp['position']['x'])
                    pose.pose.position.y = float(wp['position']['y'])
                    pose.pose.position.z = float(wp['position']['z'])
                    pose.pose.orientation.x = float(wp['orientation']['x'])
                    pose.pose.orientation.y = float(wp['orientation']['y'])
                    pose.pose.orientation.z = float(wp['orientation']['z'])
                    pose.pose.orientation.w = float(wp['orientation']['w'])
                    waypoints.append(pose)
                self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {waypoints_file}')
                return waypoints
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
            raise

    def send_waypoints(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints to follow!')
            return

        # Prepare the goal message
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints  # Send all waypoints at once

        # Send the goal and set up feedback/result callbacks
        self.get_logger().info('Sending waypoints to Nav2...')
        self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Log feedback (e.g., current waypoint being followed)
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Following waypoint {feedback.current_waypoint + 1}/{len(self.waypoints)}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal rejected by server!')
            return

        self.get_logger().info('Waypoint goal accepted by server.')
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('All waypoints completed successfully!')
        else:
            self.get_logger().warn(f'Waypoint following failed with status: {status}')

        # If looping is enabled, restart the waypoint following
        if self.loop:
            self.get_logger().info('Restarting waypoint loop...')
            self.send_waypoints()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down waypoint follower...')
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()