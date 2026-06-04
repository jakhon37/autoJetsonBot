from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetson_bot_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all files in the web directory
        ('share/' + package_name + '/web', glob('web/*.*')),
        ('share/' + package_name + '/web/css', glob('web/css/*.css')),
        ('share/' + package_name + '/web/js', glob('web/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakhon37',
    maintainer_email='jakhon37@gmail.com',
    description='Web GUI for autoJetsonBot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = jetson_bot_gui.web_server:main',
            'telemetry_node = jetson_bot_gui.telemetry_node:main',
        ],
    },

)
