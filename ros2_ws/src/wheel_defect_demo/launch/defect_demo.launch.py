import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dummybot_bringup'),
                'bringup', 'launch', 'robot.launch.py'
            )
        )
    )

    defect_injector = TimerAction(
        period=5.0,
        actions=[Node(
            package='wheel_defect_demo',
            executable='defect_injector_node',
            name='defect_injector',
            output='screen',
        )]
    )

    slip_compensator = TimerAction(
        period=5.0,
        actions=[Node(
            package='wheel_defect_demo',
            executable='slip_compensator_node',
            name='slip_compensator',
            output='screen',
        )]
    )

    mqtt_bridge = TimerAction(
        period=5.0,
        actions=[Node(
            package='wheel_defect_demo',
            executable='mqtt_bridge_node',
            name='mqtt_bridge',
            output='screen',
        )]
    )

    return LaunchDescription([
        robot_launch,
        defect_injector,
        slip_compensator,
        mqtt_bridge,
    ])
