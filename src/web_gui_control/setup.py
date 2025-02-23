from setuptools import find_packages, setup
from glob import glob  
import os

package_name = 'web_gui_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch_web_gui', glob('launch_web_gui/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakhon37',
    maintainer_email='jakhon37@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
