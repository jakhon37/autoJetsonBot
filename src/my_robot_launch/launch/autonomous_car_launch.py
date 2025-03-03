#!/usr/bin/env python3
"""
autonomous_car_launch.py

Launches the nodes required to run the autonomous car system.
This includes:
- Web GUI: Serves the web interface.
- ROS Bridge: Connects the web interface to ROS.
- Micro-ROS Agents: 
    - Serial connection to a microcontroller.
    - UDP connection over Wi-Fi.
- Serial Motor Controller: Controls the motors via USB-serial.
- RPLidar Node: Reads LIDAR values.
- SLAM Toolbox Node: Performs SLAM.

Author: Jakhongir Nodirov
Date: 2025-02-25
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # -------------------------
    # Web GUI Controller Launch
    # Launch an HTTP server to serve the web GUI on port 8000.
    web_gui_pkg = get_package_share_directory('web_gui_control')
    web_gui_path = os.path.join(web_gui_pkg, 'launch_web_gui')
    port_web = '8081'
    webserver = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', port_web ],
        cwd=web_gui_path,
        output='screen'
    )
    print(f'Web server successfully started on port {port_web}.')

    # -------------------------
    # ROS Bridge Launch
    # Launch the rosbridge server (runs rosbridge_websocket on port 9090).
    rosbridge = ExecuteProcess(
        # cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket', '--port', '9090'],

        output='screen'
    )
    print('Rosbridge server successfully started.')

    # -------------------------
    # Micro-ROS Agent Launch (Serial)
    # micro_ros_agent_serial = ExecuteProcess(
    #     cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
    #          'serial', '--dev', '/dev/ttyACM0'],
    #     output='screen'
    # )
    # print('Micro-ROS agent (Serial) successfully started.')

    # -------------------------
    # # Micro-ROS Agent Launch (UDP over Wi-Fi)
    # micro_ros_agent_udp = ExecuteProcess(
    #     cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
    #          'udp4', '--port', '8888'],
    #     output='screen'
    # )
    # print('Micro-ROS agent (UDP) successfully started.')

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
    # (Optional) Additional nodes such as camera, object detection, etc. can be added here.
    # -------------------------

    return LaunchDescription([
        webserver,
        rosbridge,
        # micro_ros_agent_serial,
        # micro_ros_agent_udp,
        serial_motor,
        rplidar,
        slam_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
