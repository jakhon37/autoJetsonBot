#!/usr/bin/env python3
"""
autonomous_car_launch.py
Launches the motor control, MPU6050, and includes the RPLIDAR A1 launch file.
"""
# /home/ubuntu/ros2_ws/install/sllidar_ros2/share/sllidar_ros2/launch/sllidar_a1_lauch_headless.py
# /home/ubuntu/ros2_ws/src/sllidar_ros2/launch/sllidar_a1_lauch_headless.py
# /home/ubuntu/ros2_ws/install/sllidar_ros2/share/sllidar_ros2/launch/sllidar_a1_lauch_headless.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the sllidar_ros2 package's launch file
    # sllidar_launch_path = os.path.join(
    #     get_package_share_directory('sllidar_ros2'),
    #     'launch',
    #     'sllidar_a1_lauch_headless.py'
    # )
    # pkg_share = get_package_share_directory('my_robot_slam')
    # config_file = os.path.join(pkg_share, 'config', 'slam_toolbox_config.yaml')
    # config_file = os.path.join(
    #     get_package_share_directory('slam_launch'),
    #     'config',
    #     'slam_toolbox_config.yaml'
    # )
    
    return LaunchDescription([
        # # Motor Controller Node
        # Node(
        #     package='motor_control',
        #     executable='motor_controller',
        #     name='motor_controller',
        #     output='screen'
        # ),
        
        # encMotor Controller Node
        # Node(
        #     package='enc_motor_control',
        #     executable='enc_motor_controller',
        #     name='enc_motor_controller',
        #     output='screen'
        # ),


        # camera Node
        # Node(
        #     package='object_detection',
        #     executable='camera_go',
        #     name='camera_go',
        #     output='screen'
        # ),        
                
        # object detection Node
        # Node(
        #     package='object_detection',
        #     executable='object_detection_node',
        #     name='object_detection_node',
        #     output='screen'
        # ),        
        
        # dual Motor Controller Node
        Node(
            package='dual_motor',
            executable='dual_motor_controller',
            name='dual_motor_controller',
            output='screen'
        ),
        
        # Encoder Controller Node
        Node(
            package='motor_encoder',
            executable='dual_encoder_node',
            name='dual_encoder_node',
            output='screen'
        ),
        # MPU6050 IMU Node
        # Node(
        #     package='mpu6050_imu',
        #     executable='mpu6050_node',
        #     name='mpu6050_node',
        #     output='screen'
        # ),
        # Include the sllidar_ros2 A1 launch file
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0', #'/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
        #         'serial_baudrate': 115200,
        #         'frame_id': 'laser_frame',
        #         'angle_compensate': True,
        #         'scan_mode': 'Standard',
        #         # 'scan_mode': 'DenseBoost',  # Less CPU intensive mode
        #         # 'scan_frequency': 5.0,  # Limit to 5Hz

        #     }]
        # ),
        
        #     package='slam_toolbox',
        #     executable='sync_slam_toolbox_node',  # Alternatively, use 'async_slam_toolbox_node' if desired
        #     name='slam_toolbox',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', 'warn'],  # Set log level to warn
        #     parameters=[
        #         {'use_sim_time': False},  # disable simulation time
        #         # Use the configuration file below (adjust the file path as needed)
        #         config_file
        #     ],
        #     # Remap the scan topic to match your sensor (if necessary)
        #     remappings=[('scan', '/scan')]
        # )
                
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',  # For asynchronous mapping (use "sync_slam_toolbox_node" if preferred)
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[config_file]
        # )
                
                
    ])

if __name__ == '__main__':
    generate_launch_description()
