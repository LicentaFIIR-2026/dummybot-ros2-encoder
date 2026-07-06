from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'score_threshold',
            default_value='0.50',
            description='Confidence minima detectie'
        ),
        DeclareLaunchArgument(
            'detection_rate_hz',
            default_value='2.0',
            description='Rata procesare frames'
        ),
        Node(
            package='media_pipe_ros2',
            executable='yolo_detector',  # ← Noul executable
            name='yolo_object_detector',
            output='screen',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'max_results': 1,
                'score_threshold': LaunchConfiguration('score_threshold'),
                'detection_rate_hz': 0.3,
                'input_width': 320,
                'input_height': 240,
            }]
        ),
    ])
