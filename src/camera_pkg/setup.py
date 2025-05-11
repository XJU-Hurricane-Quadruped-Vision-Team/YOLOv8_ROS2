from setuptools import setup
import os
from glob import glob

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Publish webcam images',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'camera_pub = camera_pkg.camera_node:main'
        ],
    },
)
