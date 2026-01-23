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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')
    amr2ax_nav_dir = get_package_share_directory('amr2ax_nav2')
    amr2ax_launch_dir = os.path.join(amr2ax_nav_dir, 'launch')
    config_dir = os.path.join(get_package_share_directory('amr2ax_nav2'), 'config')
    rviz_config_dir = os.path.join(config_dir, 'navigation.rviz')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    localization_type = LaunchConfiguration('localization_type')
    nav2pose_bt_xml = LaunchConfiguration('nav2pose_bt_xml')
    start_rviz = LaunchConfiguration('start_rviz')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'default_nav_to_pose_bt_xml': nav2pose_bt_xml}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declare the launch arguments
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('amr2ax_nav2'), 'maps', 'cb204v1.yaml'),
        description='Full path to map file to load',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(amr2ax_nav_dir, 'config', 'nav2_indoor_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_localization_type_cmd = DeclareLaunchArgument(
        'localization_type',
        default_value='2D',
        description='Whether to use indoor (2D), outdoor (3D), or GPS (GPS) localization',
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'nav2pose_bt_xml',
        default_value=os.path.join(
            bt_navigator_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Which navigate to pose BT to use',
    )

    declare_start_rviz_cmd = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Whether to start RViz2 (requires rviz2 to be installed)'
    )

    # Specify the actions
    bringup_cmd_group = GroupAction([
        Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': True}],
            arguments=['--ros-args', '--log-level', 'info'],
            #remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(amr2ax_launch_dir, 'localization.launch.py')),
            launch_arguments={'map': map_yaml_file,
                              'slam': slam,
                              'use_sim_time': use_sim_time,
                              'params_file': params_file,
                              'localization_type': localization_type,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(amr2ax_launch_dir, 'include', 'navigation.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': params_file,
                              'use_composition': 'True',
                              'localization_type': localization_type,
                              'container_name': 'nav2_container',
                              'nav2pose_bt_xml': nav2pose_bt_xml}.items()),
    ])
    rviz_cmd = GroupAction([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ], condition=IfCondition(start_rviz))
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_localization_type_cmd)
    ld.add_action(declare_start_rviz_cmd)
    ld.add_action(bringup_cmd_group)
    ld.add_action(rviz_cmd)
    return ld
