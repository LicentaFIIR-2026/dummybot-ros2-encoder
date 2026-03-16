#!/usr/bin/env python3
"""Launch semantic_localizer node with parameters."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('semantic_localizer')
    params_file = os.path.join(pkg_dir, 'config', 'semantic_localizer_params.yaml')

    return LaunchDescription([
        Node(
            package='semantic_localizer',
            executable='semantic_localizer_node',
            name='semantic_localizer',
            output='screen',
            parameters=[params_file],
            # Remap if needed:
            # remappings=[
            #     ('/scan', '/scan'),
            #     ('/yolo26/detections', '/yolo26/detections'),
            # ],
        ),
    ])
