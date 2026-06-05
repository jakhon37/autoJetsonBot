from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetson_bot_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include MobileNetSSD models
        (os.path.join('share', package_name, 'resource'), 
         ['resource/MobileNetSSD_deploy.caffemodel', 'resource/MobileNetSSD_deploy.prototxt.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakhon37',
    maintainer_email='jakhon37@gmail.com',
    description='Object detection using MobileNetSSD for autoJetsonBot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetson_bot_detection_node = jetson_bot_detection.jetson_bot_detection_node:main',
            'camera_go = jetson_bot_detection.camera_go:main',
        ],
    },
)
