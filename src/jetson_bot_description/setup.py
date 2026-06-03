from setuptools import setup

package_name = 'jetson_bot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/description.launch.py']),
        ('share/' + package_name + '/urdf', [
            'urdf/camera.xacro',
            'urdf/esp32_hardware.xml',
            'urdf/face.xacro',
            'urdf/gazebo_control.xacro',
            'urdf/inertial_macros.xacro',
            'urdf/lidar.xacro',
            'urdf/robot_core.xacro',
            'urdf/robot.xacro',
            'urdf/ros2_control.xacro'
        ]),
        ('share/' + package_name + '/rviz', ['rviz/default.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakhon37',
    maintainer_email='jakhon37@gmail.com',
    description='URDF and physical description of the autoJetsonBot platform.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
