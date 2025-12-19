from setuptools import setup

package_name = 'dummybot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dummybot_launch.py']),
        ('share/' + package_name + '/launch', ['launch/esp32_hardware.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='DummyBot 4WD controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = dummybot_control.motor_controller:main',
            'esp32_hardware_interface = dummybot_control.esp32_hardware_interface:main',
            'lidar_node = dummybot_control.lidar_node:main',
            'unified_controller = dummybot_control.unified_controller:main',
            'lidar_raw_node = dummybot_control.lidar_raw_node:main',
        ],
    },
)
