#!/usr/bin/env python3
"""
YOLO26 C++ Launch File for DummyBot
Integrare YOLO26 cu NCNN pentru detecție obiecte (versiune C++ high-performance)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    
    # Arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/saim/yolo26/yolo26n_ncnn_model',
        description='Path to NCNN model'
    )
    
    input_size_arg = DeclareLaunchArgument(
        'input_size',
        default_value= '256', #'416',
        description='YOLO input size'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value= '0.60', #'0.5',
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

    class_filter_arg = DeclareLaunchArgument(
        'class_filter',
        default_value='[39]',  # Doar bottle (class 39 în COCO)
        description='List of class IDs to detect (empty = all classes)'
    )
    
    # YOLO26 C++ Lifecycle Node
    yolo_node = LifecycleNode(
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
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'class_filter': [39],  # ← ADAUGĂ ASTA! Doar bottle
            'num_threads': 2, #4,
            'max_detection_rate': 1.0, #15.0,
            'detections_topic': '/mediapipe/detections',  # ← Și adaugă asta pentru compatibility
        }],
    )
    
    # Lifecycle manager pentru auto-activare
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='yolo26_lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['yolo26_detector']
        }]
    )
    
    return LaunchDescription([
        model_path_arg,
        input_size_arg,
        confidence_arg,
        image_topic_arg,
        debug_arg,
        class_filter_arg,
        yolo_node,
        lifecycle_manager,
    ])
