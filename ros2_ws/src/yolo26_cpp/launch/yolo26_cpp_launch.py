#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', 
                              default_value='/home/saim/yolo26/yolo26n_ncnn_model'),
        DeclareLaunchArgument('input_size', default_value='416'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.5'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),

        LifecycleNode(
            package='yolo26_cpp',
            executable='yolo26_node',
            name='yolo26_detector',
            namespace='',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'input_size': LaunchConfiguration('input_size'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'image_topic': LaunchConfiguration('image_topic'),
                'num_threads': 4,
                'max_detection_rate': 15.0,
            }],
        ),

        TimerAction(
            period=2.0,
            actions=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda node: node.name == 'yolo26_detector',
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))],
        ),

        TimerAction(
            period=4.0,
            actions=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda node: node.name == 'yolo26_detector',
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        ),
    ])
