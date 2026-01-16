from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    # Include robot bringup (robot + LiDAR + controllers)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dummybot_bringup'),
                'bringup',
                'launch',
                'dummybot.launch.py'
            ])
        ])
    )

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('dummybot_navigation'),
                'config',
                'mapper_params_online_async.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/odom', '/diff_drive_controller/odom')  # ← ADAUGĂ ASTA
        ]
    )

    # RViz2 pentru vizualizare (opțional, dar FOARTE util)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('dummybot_navigation'),
        'config',
        'slam.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Lifecycle spawners pentru activare automată SLAM
    slam_configure_event = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        name='slam_configure',
        output='screen'
    )
    
    slam_activate_event = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        name='slam_activate',
        output='screen'
    )
    
    # Delay pentru a permite node-ului să pornească
    delayed_slam_configure = TimerAction(
        period=2.0,
        actions=[slam_configure_event]
    )
    
    delayed_slam_activate = TimerAction(
        period=4.0,
        actions=[slam_activate_event]
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        robot_bringup,
        slam_toolbox_node,
        delayed_slam_configure,
        delayed_slam_activate,
        # rviz_node,  # Decomentează dacă vrei RViz automat
    ])