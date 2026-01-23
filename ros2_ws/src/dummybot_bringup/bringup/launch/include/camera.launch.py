import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments pentru flexibilitate
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )

    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='15',
        description='Camera frame rate'
    )

    return LaunchDescription([
        camera_device_arg,
        frame_rate_arg,

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_node',
            namespace='camera',
            output='screen',
            parameters=[{
                # Rezolutie
                'image_size': [640, 480],
                
                # 15 FPS - optim pentru AprilTag docking + detectii multiple
                'time_per_frame': [1, 15],
                
                # Frame ID pentru TF
                'camera_frame_id': 'camera_link_optical',
                
                # Calibrare camera
                'camera_info_url': 'file:///home/pi/saim_xplorer/src/xplorer_bringup/bringup/config/hd_pro_webcam_c920_v2.yaml',
                
                # Device video
                #'video_device': '/dev/video1',
                
                # Format pixel - YUYV e nativ pt Logitech C920, evita conversii
                'pixel_format': 'YUYV',
                
                # Buffer-e V4L2 - 2 e minim pentru flow continuu fara lag
                'io_method': 'mmap',
            }],
            # QoS overrides pentru performanta cu multi-subscriber
            # best_effort + keep_last(1) = subscriberii primesc ultimul frame disponibil
            # fara sa se acumuleze coada
            arguments=[
                '--ros-args',
                '--log-level', 'camera_node:=warn',
            ],
        ),
    ])