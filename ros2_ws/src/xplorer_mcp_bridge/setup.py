from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xplorer_mcp_bridge'

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
    description='HTTP context server for SAIM Xplorer',
    license='MIT',
    entry_points={
        'console_scripts': [
            'context_server = xplorer_mcp_bridge.context_server:main',
        ],
    },
)