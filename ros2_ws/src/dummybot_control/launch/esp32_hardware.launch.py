from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummybot_control',
            executable='esp32_hardware_interface',
            name='esp32_hardware_interface',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'baud_rate': 115200},
                {'wheel_radius': 0.0325},  # 65mm / 2
                {'wheel_separation': 0.3863},  # Lx din specificații
                {'ticks_per_revolution': 1760},
                {'update_rate': 30.0}
            ],
            output='screen'
        )
    ])
