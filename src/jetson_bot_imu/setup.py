from setuptools import find_packages, setup

package_name = 'jetson_bot_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakhon37',
    maintainer_email='jakhon37@gmail.com',
    description='I2C driver for MPU6050 IMU sensor on Jetson Nano.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu6050_node = jetson_bot_imu.mpu6050_node:main',
            'imu_bridge_node = jetson_bot_imu.imu_bridge_node:main',
        ],
    },

)
