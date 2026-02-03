import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Pachete
    autonomous_slam_dir = get_package_share_directory('dummybot_autonomous_slam')
    
    # Fișiere de configurare
    exploration_params_file = os.path.join(autonomous_slam_dir, 'config', 'exploration_params.yaml')
    
    # Nodul de explorare autonomă
    frontier_explorer_node = Node(
        package='dummybot_autonomous_slam',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[exploration_params_file],
        remappings=[
            ('/map', '/map'),
            ('/navigate_to_pose', '/navigate_to_pose')
        ]
    )
    
    return LaunchDescription([
        frontier_explorer_node,
    ])
