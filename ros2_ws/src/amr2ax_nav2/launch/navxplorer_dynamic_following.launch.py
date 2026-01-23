import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    maps_dir = os.path.join(get_package_share_directory('amr2ax_nav2'), 'maps')
    param_dir = os.path.join(get_package_share_directory('amr2ax_nav2'), 'config')
    config_dir = os.path.join(get_package_share_directory('amr2ax_nav2'), 'config')
    map_file = os.path.join(maps_dir, '204_25.07.yaml')
    param_file = os.path.join(param_dir, 'xplorer.yaml')
    rviz_config_dir = os.path.join(config_dir, 'navigation.rviz')

    # Parametrul domain_id
    domain_id = LaunchConfiguration('domain_id')
    
    # Argumentul pentru domain_id
    arg_domain_id = DeclareLaunchArgument(
        'domain_id',
        default_value='0',
        description='ROS Domain ID (0-232)'
    )

    # Setarea variabilei de mediu ROS_DOMAIN_ID
    set_domain_id = SetEnvironmentVariable(
        'ROS_DOMAIN_ID',
        domain_id
    )
    
    return LaunchDescription([
        arg_domain_id,
        set_domain_id,
        
        # Launch Nav2 cu configurație normală
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('amr2ax_nav2'),'/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'params_file': param_file,
                'domain_id': domain_id}.items(),
        ),
        
        # Launch nodul convertor clicked_point automat
        Node(
            package='amr2ax_nav2',
            executable='clicked_point_to_goal_update.py',
            name='clicked_point_to_goal_update',
            output='screen',
            parameters=[{
                'input_topic': '/clicked_point',
                'output_topic': '/goal_update',
                'base_frame': 'base_link',
                'calculate_orientation': True,
                'default_yaw': 0.0
            }]
        ),
    ])
