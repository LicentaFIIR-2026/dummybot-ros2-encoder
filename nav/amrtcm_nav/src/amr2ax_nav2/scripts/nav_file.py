#! /usr/bin/env python3
import yaml
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def load_waypoints_from_yaml(file_path):
    """Încarcă punctele de navigație dintr-un fișier YAML."""
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    waypoints = []
    for key, values in data['waypoints'].items():
        pose_data = values['pose']
        orientation_data = values['orientation']

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()

        # Asigură conversia în float
        pose.pose.position.x = float(pose_data[0])
        pose.pose.position.y = float(pose_data[1])
        pose.pose.position.z = float(pose_data[2])
        pose.pose.orientation.x = float(orientation_data[0])
        pose.pose.orientation.y = float(orientation_data[1])
        pose.pose.orientation.z = float(orientation_data[2])
        pose.pose.orientation.w = float(orientation_data[3])

        waypoints.append(pose)

    return waypoints


def main():
    rclpy.init()
    global navigator  # Necesită global pentru a putea accesa timpul în `load_waypoints_from_yaml`
    navigator = BasicNavigator()

    # Așteptăm activarea Nav2
    navigator.waitUntilNav2Active()

    # Încărcăm punctele din fișierul YAML
    yaml_file_path = "waypoints.yaml"  # Schimbă acest path cu locația fișierului tău
    goal_poses = load_waypoints_from_yaml(yaml_file_path)

    # Inițiem navigarea către punctele definite
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Navigând către punctul {feedback.current_waypoint + 1}/{len(goal_poses)}')
            now = navigator.get_clock().now()

            # Timeout de siguranță (600 secunde)
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()

    # Verificăm rezultatul navigației
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Navigație finalizată cu succes!")
    elif result == TaskResult.CANCELED:
        print("Navigația a fost anulată!")
    elif result == TaskResult.FAILED:
        print("Navigație eșuată!")
    else:
        print("Stare necunoscută!")

    exit(0)


if __name__ == '__main__':
    main()
