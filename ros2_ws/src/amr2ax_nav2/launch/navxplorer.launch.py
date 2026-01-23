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
#    map_file = os.path.join(maps_dir, 'b42.yaml')
    map_file = os.path.join(maps_dir, 'cb204_13.12.05v4.yaml')
#    map_file = os.path.join(maps_dir, 'test.yaml')
#    map_file = os.path.join(maps_dir, 'demo_map_4x4.yaml')    
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

#    joystick = IncludeLaunchDescription(
#                PythonLaunchDescriptionSource([os.path.join(
#                    get_package_share_directory('amr2ax_nav2'),'launch','joystick.launch.py'
#                )]), launch_arguments={'use_sim_time': 'false'}.items()
#    )

#    twist_mux_params = os.path.join(get_package_share_directory('amr2ax_nav2'),'config','twist_mux.yaml')
#    twist_mux = Node(
#            package="twist_mux",
#            executable="twist_mux",
#            parameters=[twist_mux_params],
#            remappings=[('/cmd_vel_out','/cmd_vel')]
#        )
    
    return LaunchDescription([
        arg_domain_id,
        set_domain_id,
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([get_package_share_directory('turtlebot3_gazebo'),'/launch','/turtlebot3_world.launch.py'])
#        ),
#
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('amr2ax_nav2'),'/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'params_file': param_file,
                'domain_id': domain_id}.items(),
        ),
#        joystick,
#        twist_mux,

#    Node(
#        package='rviz2',
#        executable='rviz2',
#        name='rviz2_node',
#        arguments=['-d', rviz_config_dir ],
#        output='screen'

#        ),

    
    ])
