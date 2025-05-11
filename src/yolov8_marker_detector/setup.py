# -*- coding: utf-8 -*-
import os
import sys
from setuptools import find_packages, setup

os.environ["PYTHON_EXECUTABLE"] = "/home/jiang/.envs/yolov8_ros2/bin/python3"
sys.executable = "/home/jiang/.envs/yolov8_ros2/bin/python3"

package_name = 'yolov8_marker_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/train_best0421.pt']),
    ],
    install_requires=['setuptools', 'ultralytics', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='jiang',
    maintainer_email='mengshuimeng@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_detector_node = yolov8_marker_detector.marker_detector_node:main',
        ],
    },
)
