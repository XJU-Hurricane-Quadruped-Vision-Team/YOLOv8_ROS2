from setuptools import setup
import os
from glob import glob

package_name = 'yolov8_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),  
    ],
    install_requires=[
        'setuptools', 'rclpy', 'sensor_msgs',
        'cv_bridge', 'ultralytics', 'opencv-python'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='YOLOv8 object detection',
    license='Apache License 2.0',
    
    entry_points={
    'console_scripts': [
        'yolov8_node = yolov8_pkg.yolov8_node:main',
    ],
},

)
