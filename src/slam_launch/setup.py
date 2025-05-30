from setuptools import find_packages, setup
from glob import glob

package_name = 'slam_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/config', ['src/slam_launch/config/slam_toolbox_config.yaml']),
        ('share/' + package_name , ['config/slam_toolbox_config.yaml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.world')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        # ('share/' + package_name + '/config/slam_toolbox_config.yaml', glob('launch/*')),

        # ('share/' + package_name + '/launch', glob('launch/*.py')),
        # ('share/' + package_name + '/launch/web_gui', glob('launch/web_gui/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='jakhon37@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
