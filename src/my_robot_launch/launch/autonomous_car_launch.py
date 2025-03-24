#!/usr/bin/env python3
"""
autonomous_car_launch.py

Launches nodes required to run the autonomous car system including:
- Web GUI
- ROS Bridge
- Serial Motor Controller
- RPLidar Node
- SLAM Toolbox Node
- Robot State Publisher (for robot description)

Author: Jakhongir Nodirov
Date: 2025-02-25
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # -------------------------
    # Web GUI Controller Launch
    web_gui_pkg = get_package_share_directory('web_gui_control')
    web_gui_path = os.path.join(web_gui_pkg, 'launch_web_gui')
    port_web = '8080'
    webserver = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', port_web],
        cwd=web_gui_path,
        output='screen'
    )
    print(f'Web server successfully started on port {port_web}.')

    # -------------------------
    # ROS Bridge Launch
    rosbridge = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket', '--port', '9090'],
        output='screen'
    )
    print('Rosbridge server successfully started.')

    # -------------------------
    # Serial Motor Controller Node
    serial_motor = Node(
        package='motor_usbserial',
        executable='motor_usbserial_node',
        name='motor_usbserial',
        output='screen'
    )

    # -------------------------
    # RPLidar Node
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )

    # -------------------------
    # SLAM Toolbox Node
    slam_pkg = get_package_share_directory('slam_launch')
    slam_config_file = os.path.join(slam_pkg, 'config/slam_toolbox_config.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': False}, slam_config_file],
        remappings=[('scan', '/scan')]
    )

    # -------------------------
    # Robot Description Node (Robot State Publisher)
    # Locate your robot description file (URDF or XACRO)
    # robot_description_pkg = get_package_share_directory('robot_description')
    # robot_description_file = os.path.join(robot_description_pkg, 'urdf', 'robot.urdf.xacro')
    
    # # Process the xacro file to produce a valid URDF XML string
    # doc = xacro.process_file(robot_description_file)
    # robot_description_xml = doc.toxml()

    # # Create the robot_state_publisher node and load the robot description
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description_xml}]
    # )

    # -------------------------
    # Return the complete LaunchDescription with all nodes
    return LaunchDescription([
        webserver,
        rosbridge,
        serial_motor,
        rplidar,
        slam_node,
        # robot_state_publisher_node,
    ])

if __name__ == '__main__':
    generate_launch_description()

