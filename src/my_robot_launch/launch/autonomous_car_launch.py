#!/usr/bin/env python3
"""
autonomous_car_launch.py
Launches the nodes required to run the autonomous car system.
This includes the following nodes:
- camera_go: Camera node that captures images from the camera
- object_detection_node: Object detection node that processes the images and detects objects
- dual_motor_controller: Dual motor controller node that controls the motors
- dual_encoder_node: Dual encoder node that reads the encoder values
- mpu6050_node: MPU6050 IMU node that reads the IMU values
- rplidar_composition: RPLidar A1 node that reads the LIDAR values
- slam_toolbox: SLAM Toolbox node that performs SLAM
- web_gui: Web GUI node that serves the web interface
- rosbridge: ROS Bridge node that connects the web interface to ROS
- micro_ros_agent: Micro-ROS Agent node that connects the microcontroller to ROS
Author: Jakhongir Nodirov
Date: 2025-02-25
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    
    # -------------------------
    # WEB GUI CONTROLLER LAUNCH
    # Get the path to the web_gui package
    web_gui_pkg = get_package_share_directory('web_gui_control')
    web_gui_path = os.path.join(web_gui_pkg, 'launch_web_gui')
    # print(f'web gui path: {web_gui_path}')
    # Launch an HTTP server to serve the web GUI on port 8000
    webserver = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8001'],
        cwd=web_gui_path,
        output='screen'
    )
    print(f'wevserver succesfully run: {1}')
    # -------------------------
    
    
    # -------------------------
    # ROS BRIDGE LAUNCH
    # Launch the rosbridge server (runs rosbridge_websocket on port 9090)
    # sudo apt install ros-jazzy-rosbridge-server
    rosbridge = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
        output='screen'
    )
    print(f'rosbridge succesfully run')
    # -------------------------     
    
    # -------------------------
    # MICRO-ROS MOTOR CONTROL & ENCODER LAUNCH
    # git clone --branch foxy https://github.com/micro-ROS/micro_ros_setup.git 
    # cd ..
    # calcon install --packages-select micro_ros_setup
    # source /install/setup.bash
    # ros2 run micro_ros_setup create_agent_ws.sh
    # ros2 run micro_ros_setup build_agent.sh
    # source install/local_setup.sh
    # ros2 run micro_ros_agent micro_ros_agent [parameters]
    # ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1
    
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
             'serial', 
             '--dev', 
             '/dev/ttyACM0'],
        output='screen'
    )
    print(f'micro_ros_agent succesfully run')
    # -------------------------
    
    
    
    # -------------------------
    # MICRO-ROS MOTOR CONTROL & ENCODER LAUNCH OVER WIFI
    # ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    # Example: If your agent supports these flags:
    # ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6 \
    #   --max-input-buffer-length 65536 \
    #   --max-output-buffer-length 65536 \
    #   --max-participants 10 \
    #   --max-datawriters 20 \
    #   --max-datareaders 20

    # micro_ros_agent = ExecuteProcess( 
    #     cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
    #          'udp4', 
    #          '--port', 
    #          '8888',
    #          '-v6',
    #          '--max-input-buffer-length',
    #          '65536',
    #          '--max-output-buffer-length',
    #          '65536',
    #          '--max-participants',
    #          '10',
    #          '--max-datawriters',
    #          '20',
    #          '--max-datareaders',
    #          '20'],
    #     output='screen'
    # )
    print(f'micro_ros_agent_WIFI succesfully run')  
    # -------------------------
    
    
    
    
    
    
    # -------------------------
    # other nodes
    # camera Node
    #...
    # -------------------------
    
    return LaunchDescription([
        webserver,
        rosbridge,
        micro_ros_agent,
        #micro_ros_agent_WIFI
    ])  
    

    
    
if __name__ == '__main__':
    generate_launch_description()
    
    # return LaunchDescription([



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
        # Node(
        #     package='dual_motor',
        #     executable='dual_motor_controller',
        #     name='dual_motor_controller',
        #     output='screen'
        # ),
        
        # # Encoder Controller Node
        # Node(
        #     package='motor_encoder',
        #     executable='dual_encoder_node',
        #     name='dual_encoder_node',
        #     output='screen'
        # ),
        # MPU6050 IMU Node
        # Node(
        #     package='mpu6050_imu',
        #     executable='mpu6050_node',
        #     name='mpu6050_node',
        #     output='screen'
        # ),
        # # Include the sllidar_ros2 A1 launch file
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB2', #'/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
        #         'serial_baudrate': 115200,
        #         'frame_id': 'laser_frame',
        #         'angle_compensate': True,
        #         'scan_mode': 'Standard',
        #         # 'scan_mode': 'DenseBoost',  # Less CPU intensive mode
        #         # 'scan_frequency': 5.0,  # Limit to 5Hz

        #     }]
        # ),
        
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
    
        # Node(
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
                
                
    # ])

if __name__ == '__main__':
    generate_launch_description()
