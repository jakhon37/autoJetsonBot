from setuptools import setup

package_name = 'jetson_bot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sim.launch.py',
            'launch/main.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/unified_robot_config.yaml', 'config/system_config.yaml']),
        ('share/' + package_name + '/worlds', [
            'worlds/lab.world',
            'worlds/simple.world'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakhon37',
    maintainer_email='jakhon37@gmail.com',
    description='Main bringup and orchestration for autoJetsonBot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
