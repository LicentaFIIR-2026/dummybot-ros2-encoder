import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Pachete
    autonomous_slam_dir = get_package_share_directory('dummybot_autonomous_slam')
    nav2_dir = get_package_share_directory('amr2ax_nav2')
    
    # Fișiere de configurare
    exploration_params_file = os.path.join(autonomous_slam_dir, 'config', 'exploration_params.yaml')
    nav2_params_file = os.path.join(nav2_dir, 'config', 'xplorer.yaml')
    
    # Domain ID
    domain_id = LaunchConfiguration('domain_id')
    
    arg_domain_id = DeclareLaunchArgument(
        'domain_id',
        default_value='0',
        description='ROS Domain ID (0-232)'
    )
    
    set_domain_id = SetEnvironmentVariable(
        'ROS_DOMAIN_ID',
        domain_id
    )
    
    # Launch Nav2 cu SLAM activat
    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_dir, 'launch', 'navxplorer.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'False',
            'domain_id': domain_id
        }.items()
    )
    
    # Nodul de explorare autonomă
    frontier_explorer_node = Node(
        package='dummybot_autonomous_slam',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[exploration_params_file],
        remappings=[
            ('/map', '/map'),
            ('/navigate_to_pose', '/navigate_to_pose')
        ]
    )
    
    return LaunchDescription([
        arg_domain_id,
        set_domain_id,
        nav2_slam_launch,
        frontier_explorer_node,
    ])
