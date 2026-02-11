from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'score_threshold',
            default_value='0.45',
            description='Confidence minima detectie'
        ),
        DeclareLaunchArgument(
            'detection_rate_hz',
            default_value='8.0',
            description='Rata procesare frames'
        ),
        Node(
            package='media_pipe_ros2',
            executable='object_detector',
            name='mediapipe_object_detector',
            output='screen',
            parameters=[{
                'model_path': '/home/saim/mediapipe_models/efficientdet_lite0_int8.tflite',
                'max_results': 5,
                'score_threshold': LaunchConfiguration('score_threshold'),
                'detection_rate_hz': LaunchConfiguration('detection_rate_hz'),
                'publish_debug_image': True,
            }]
        ),
    ])