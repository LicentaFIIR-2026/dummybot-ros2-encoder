#!/usr/bin/env python3
"""
Launch file pentru testare Dynamic Object Following

Pornește nodul convertor clicked_point_to_goal_update pentru testare rapidă.
Presupune că robotul și nav2 sunt deja lansate.

Usage:
    ros2 launch amr2ax_nav2 test_dynamic_following.launch.py

Author: SAIM Xplorer Team
Date: 2025-10-10
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Parametri configurabili
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    calculate_orientation = LaunchConfiguration('calculate_orientation')
    
    # Argumentele de launch
    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/clicked_point',
        description='Topic pentru clicked points din RViz'
    )
    
    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/goal_update',
        description='Topic pentru goal updates către behavior tree'
    )
    
    declare_calculate_orientation = DeclareLaunchArgument(
        'calculate_orientation',
        default_value='true',
        description='Calculează orientarea către click (true) sau folosește default (false)'
    )
    
    # Nod convertor clicked_point
    clicked_point_converter = Node(
        package='amr2ax_nav2',
        executable='clicked_point_to_goal_update.py',
        name='clicked_point_to_goal_update',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'output_topic': output_topic,
            'base_frame': 'base_link',
            'calculate_orientation': calculate_orientation,
            'default_yaw': 0.0
        }],
        remappings=[]
    )
    
    # Log informativ
    log_info = LogInfo(
        msg=[
            '\n',
            '========================================\n',
            'Dynamic Object Following - Test Mode\n',
            '========================================\n',
            'Nodul convertor clicked_point pornit.\n',
            '\n',
            'Pentru testare:\n',
            '1. Asigură-te că robotul și nav2 rulează\n',
            '2. Deschide RViz\n',
            '3. Comută BT la dynamic_follow:\n',
            '   ros2 service call /bt_navigator/navigate_to_pose nav2_msgs/action/NavigateToPose\n',
            '4. Sau folosește comanda action:\n',
            '   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...\n',
            '5. Click pe hartă în RViz (butonul "Publish Point")\n',
            '\n',
            'Input topic: ', input_topic, '\n',
            'Output topic: ', output_topic, '\n',
            '========================================\n'
        ]
    )
    
    return LaunchDescription([
        declare_input_topic,
        declare_output_topic,
        declare_calculate_orientation,
        log_info,
        clicked_point_converter,
    ])
