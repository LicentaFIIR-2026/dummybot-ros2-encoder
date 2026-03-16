#!/usr/bin/env python3
"""
YOLO26 ROS2 Launch File
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    
    # Arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/pi/yolo26/yolo26n_ncnn_model',
        description='Path to NCNN model'
    )
    
    input_size_arg = DeclareLaunchArgument(
        'input_size',
        default_value='416',
        description='YOLO input size'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Detection confidence threshold'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )
    
    debug_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='false',
        description='Publish annotated debug image'
    )
    
    # YOLO26 Lifecycle Node
    yolo_node = LifecycleNode(
        package='yolo26_ros',
        executable='yolo26_node',
        name='yolo26_detector',
        namespace='',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'input_size': LaunchConfiguration('input_size'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'image_topic': LaunchConfiguration('image_topic'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'num_threads': 4,
            'max_detection_rate': 10.0,
        }],
    )
    
    # Auto-configure after 2 seconds
    configure_event = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda node: node.name == 'yolo26_detector',
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            ),
        ],
    )
    
    # Auto-activate after 4 seconds
    activate_event = TimerAction(
        period=4.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda node: node.name == 'yolo26_detector',
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            ),
        ],
    )
    
    return LaunchDescription([
        model_path_arg,
        input_size_arg,
        confidence_arg,
        image_topic_arg,
        debug_arg,
        yolo_node,
        configure_event,
        activate_event,
    ])