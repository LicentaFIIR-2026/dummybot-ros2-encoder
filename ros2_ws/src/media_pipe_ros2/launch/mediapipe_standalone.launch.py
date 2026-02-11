from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch
import subprocess


def generate_launch_description():

    # Pornim mediapipe_standalone ca proces independent (nu nod ROS2)
    standalone_process = launch.actions.ExecuteProcess(
        cmd=[
            'python3',
            '/home/saim/dummybot-ros2-encoder/ros2_ws/src/'
            'media_pipe_ros2/media_pipe_ros2/mediapipe_standalone.py'
        ],
        output='screen',
        name='mediapipe_standalone'
    )

    # Pornim bridge-ul ROS2 care publica detectiile
    bridge_node = Node(
        package='media_pipe_ros2',
        executable='ros2_bridge',
        name='mediapipe_ros2_bridge',
        output='screen',
    )

    return LaunchDescription([
        standalone_process,
        bridge_node,
    ])
