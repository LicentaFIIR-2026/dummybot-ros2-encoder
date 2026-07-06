from setuptools import setup
import os
from glob import glob

package_name = 'wheel_defect_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrei',
    description='Wheel defect detection and compensation demo for AIONIS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'defect_injector_node=wheel_defect_demo.defect_injector_node:main',
            'slip_compensator_node=wheel_defect_demo.slip_compensator_node:main',
            'mqtt_bridge_node=wheel_defect_demo.mqtt_bridge_node:main',
        ],
    },
)
