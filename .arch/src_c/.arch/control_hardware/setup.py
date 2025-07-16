from setuptools import find_packages, setup

package_name = 'control_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/launch', ['launch/hardware.launch.py']),
        ('share/' + package_name + '/config', ['config/controller_config.yaml']),
        # Hardware plugin XML
        ('share/' + package_name + '/hardware', ['hardware/esp32_hardware.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakhon37',
    maintainer_email='jakhon37@gmail.com',
    description='ESP32 hardware interface for ROS2 Control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
        'hardware_interface.robot_hardware_interface': [
            'control_hardware.esp32_interface = control_hardware.esp32_hardware_interface:get_hardware_interfaces',
        ],
    },
)
