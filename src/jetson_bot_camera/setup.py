from glob import glob

from setuptools import find_packages, setup

package_name = 'jetson_bot_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakhon37',
    maintainer_email='jakhon37@gmail.com',
    description='Publishes an MJPEG stream, such as the robot\'s CSI camera, as ROS images.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mjpeg_camera_node = jetson_bot_camera.mjpeg_camera_node:main',
        ],
    },
)
