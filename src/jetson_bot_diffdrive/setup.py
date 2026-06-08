from setuptools import setup
import os
from glob import glob

package_name = 'jetson_bot_diffdrive'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakhon37',
    maintainer_email='jakhon37@gmail.com',
    description='Python Serial Bridge for autoJetsonBot Motor Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diffdrive_node = jetson_bot_diffdrive.diffdrive_node:main'
        ],
    },
)
