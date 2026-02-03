from setuptools import setup
import os
from glob import glob

package_name = 'dummybot_autonomous_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrei',
    maintainer_email='andrei@dummybot.ro',
    description='Autonomous SLAM exploration for DummyBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = dummybot_autonomous_slam.frontier_explorer:main',
        ],
    },
)