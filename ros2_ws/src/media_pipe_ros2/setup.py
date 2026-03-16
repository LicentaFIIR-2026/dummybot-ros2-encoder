from setuptools import setup

package_name = 'media_pipe_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/mediapipe_object_detector.launch.py',
            'launch/mediapipe_standalone.launch.py',
            'launch/yolo_object_detector.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrei',
    maintainer_email='andrei@dummybot.com',
    description='MediaPipe Detection for DummyBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hands_detector = media_pipe_ros2.hands_detector:main',
            'hand_follower = media_pipe_ros2.hand_follower:main',
            'pose_detector = media_pipe_ros2.pose_detector:main',
            'pose_follower = media_pipe_ros2.pose_follower:main',
            'object_detector = media_pipe_ros2.object_detector:main',
            'mediapipe_standalone = media_pipe_ros2.mediapipe_standalone:main',
            'ros2_bridge = media_pipe_ros2.ros2_bridge:main',
            'yolo_detector = media_pipe_ros2.yolo_detector:main',
        ],
    },
)
