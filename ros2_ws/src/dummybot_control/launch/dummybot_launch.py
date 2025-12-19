from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummybot_control',
            executable='motor_controller',
            name='dummybot_controller',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'baud_rate': 115200},
                {'wheel_separation': 0.3},
                {'max_speed': 200}
            ],
            output='screen'
        )
    ])