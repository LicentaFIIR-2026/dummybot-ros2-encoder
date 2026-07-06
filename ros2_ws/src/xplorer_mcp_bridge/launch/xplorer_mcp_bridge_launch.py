import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('xplorer_mcp_bridge'),
        'config',
        'xplorer_mcp_bridge_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='xplorer_mcp_bridge',
            executable='context_server',
            name='xplorer_context_server',
            parameters=[params_file],
            output='screen'
        )
    ])