# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = FindPackageShare('dummybot_bringup')

    # Argument pentru a activa/dezactiva YOLO
    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo',
        default_value='false',
        description='Launch YOLO26 object detection'
    )
    
    use_yolo = LaunchConfiguration('use_yolo')

    # MediaPipe Object Detection
    use_mediapipe_arg = DeclareLaunchArgument(
        'use_mediapipe',
        default_value='true',
        description='Launch MediaPipe object detection'
    )
    use_mediapipe = LaunchConfiguration('use_mediapipe')

    launch_mediapipe = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('media_pipe_ros2'),
                'launch',
                'mediapipe_object_detector.launch.py'])]),
        condition=IfCondition(use_mediapipe)
    )

#    use_orbecc = LaunchConfiguration('use_orbecc')
#    use_orbecc_cmd = DeclareLaunchArgument(
#        'use_orbecc',
#        default_value=os.getenv('USE_ORBECC', 'false').lower(),
#        description='Set to true to launch the ORBECC configuration'
#    )

#    launch_camera = GroupAction([
#        # Realsense
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([
#                PathJoinSubstitution([
#                    bringup_dir, 'launch', 'include', 'realsense.launch.py'])]),
#            condition=UnlessCondition(PythonExpression(["'", use_orbecc, "' == 'true'"]))
#        ),
#
#        # Orbecc
#        IncludeLaunchDescription(
#           PythonLaunchDescriptionSource([
#                PathJoinSubstitution([
#                    bringup_dir, 'launch', 'include', 'orbecc.launch.py'])]),
#            condition=IfCondition(PythonExpression(["'", use_orbecc, "' == 'true'"]))
#        ),
#    ])

#    launch_imu = IncludeLaunchDescription(
#        PythonLaunchDescriptionSource([
#            PathJoinSubstitution([
#                bringup_dir, 'launch', 'include', 'mpu9250.launch.py'])]),
#    )
#    launch_BNO055 = IncludeLaunchDescription(
#        PythonLaunchDescriptionSource([
#            PathJoinSubstitution([
#                bringup_dir, 'bringup', 'launch', 'include', 'BNO055.launch.py'])]),
#    )
    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bringup_dir, 'bringup','launch', 'include', 'ld19.launch.py'])]),
    )
    launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bringup_dir, 'bringup','launch', 'include', 'camera.launch.py'])]),
    )
    
    # YOLO26 Object Detection
    launch_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bringup_dir, 'bringup','launch', 'include', 'yolo26.launch.py'])]),
        condition=IfCondition(use_yolo)
    )

    ld = LaunchDescription()
#    ld.add_action(use_orbecc_cmd)
    ld.add_action(use_yolo_arg)
    ld.add_action(launch_camera)
#    ld.add_action(launch_BNO055)
    ld.add_action(launch_lidar)
    ld.add_action(launch_yolo)
    ld.add_action(use_mediapipe_arg)
    ld.add_action(launch_mediapipe)
    return ld