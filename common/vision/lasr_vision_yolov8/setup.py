from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lasr_vision_yolov8'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maayan',
    maintainer_email='maayan.armony@gmail.com',
    description='YOLOv8 object detection service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "yolo_service_node = lasr_vision_yolov8.lasr_vision_yolov8.services:main",
        ],
    },
)