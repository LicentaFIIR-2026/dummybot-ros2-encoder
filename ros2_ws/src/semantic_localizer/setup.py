from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'semantic_localizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FIIR',
    maintainer_email='bogdan@pub.ro',
    description='Semantic object localization via YOLO + LiDAR 2D fusion for Nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'semantic_localizer_node = semantic_localizer.semantic_localizer_node:main',
        ],
    },
)
