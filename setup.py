from setuptools import setup
import os
from setuptools import find_packages
from glob import glob

package_name = 'zed_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for ZED2i camera pose tracking',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_pose_node = zed_pose.zed_pose_node:main',
            'pose_logger = zed_pose.pose_logger:main',
        ],
    },
)
