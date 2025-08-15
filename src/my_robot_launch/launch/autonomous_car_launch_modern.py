#!/usr/bin/env python3
"""
autonomous_car_launch_modern.py - Modern unified launch file
Supports both real robot and Gazebo simulation with a simple flag
Author: RovoDev Assistant - Updated Version
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Modern unified launch description
    Usage:
      ros2 launch my_robot_launch autonomous_car_launch_modern.py
      ros2 launch my_robot_launch autonomous_car_launch_modern.py use_sim:=true
      ros2 launch my_robot_launch autonomous_car_launch_modern.py use_sim:=false
    """
    
    # =============================================================================
    # LAUNCH ARGUMENTS
    # =============================================================================
    
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use Gazebo simulation if true, real hardware if false',
        choices=['true', 'false']
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='jetson_bot',
        description='Name of the robot'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='lab.world',
        description='Gazebo world file for simulation'
    )
    
    # =============================================================================
    # CONFIGURATION VARIABLES
    # =============================================================================
    
    use_sim = LaunchConfiguration('use_sim')
    robot_name = LaunchConfiguration('robot_name')
    world_file = LaunchConfiguration('world_file')
    
    # Package directories
    pkg_my_robot_launch = FindPackageShare('my_robot_launch')
    pkg_web_gui_control = FindPackageShare('web_gui_control')
    pkg_slam_launch = FindPackageShare('slam_launch')
    
    # =============================================================================
    # ROBOT DESCRIPTION
    # =============================================================================
    
    xacro_file = os.path.join(
        get_package_share_directory('my_robot_launch'),
        'urdf',
        'robot.xacro'
    )
    
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=true',
        ' sim_mode:=', use_sim
    ])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim
        }]
    )
    
    # =============================================================================
    # GAZEBO SIMULATION (Conditional)
    # =============================================================================
    
    # Gazebo Launch
    gazebo_params_file = os.path.join(
        get_package_share_directory('my_robot_launch'),
        'config',
        'gazebo_params.yaml'
    )
    
    world_path = os.path.join(
        get_package_share_directory('my_robot_launch'),
        'config',
        'lab.world'
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items(),
        condition=IfCondition(use_sim)
    )
    
    # Spawn Robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robot_name],
        output='screen',
        condition=IfCondition(use_sim)
    )
    
    # =============================================================================
    # ROS2 CONTROL (Both Sim and Real)
    # =============================================================================
    
    # Controller configuration files
    controller_config_real = os.path.join(
        get_package_share_directory('my_robot_launch'),
        'config',
        'my_controllers.yaml'
    )
    
    controller_config_sim = os.path.join(
        get_package_share_directory('my_robot_launch'),
        'config',
        'my_controllers_use_sim.yaml'
    )
    
    # Controller Manager (Real Hardware)
    controller_manager_real = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            controller_config_real,
            {'use_sim_time': False}
        ],
        condition=UnlessCondition(use_sim)
    )
    
    # Controller Spawners
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output='screen'
    )
    
    joint_broad_spawner = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["joint_broad"],
        output='screen'
    )
    
    # =============================================================================
    # SENSORS (Real Hardware Only)
    # =============================================================================
    
    # RPLidar
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': False
        }],
        condition=UnlessCondition(use_sim)
    )
    
    # IMU Node
    imu_node = Node(
        package='mpu6050_imu',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        parameters=[{
            'publish_rate': 50,
            'use_sim_time': False
        }],
        condition=UnlessCondition(use_sim)
    )
    
    # Motor USB Serial (Real Hardware)
    motor_usbserial_node = Node(
        package='motor_usbserial',
        executable='motor_usbserial_node',
        name='motor_usbserial',
        output='screen',
        condition=UnlessCondition(use_sim)
    )
    
    # =============================================================================
    # SLAM INTEGRATION
    # =============================================================================
    
    # SLAM Configuration Selection
    slam_config_real = os.path.join(
        get_package_share_directory('slam_launch'),
        'config',
        'mapper_params_online_async.yaml'
    )
    
    slam_config_sim = os.path.join(
        get_package_share_directory('slam_launch'),
        'config', 
        'mapper_params_online_async-sim.yaml'
    )
    
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim},
            PythonExpression([
                "'", slam_config_sim, "' if '", use_sim, "' == 'true' else '", slam_config_real, "'"
            ])
        ],
        remappings=[('scan', '/scan')]
    )
    
    # =============================================================================
    # WEB INTERFACE & COMMUNICATION
    # =============================================================================
    
    # ROSBridge WebSocket Server
    rosbridge_server = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
        output='screen'
    )
    
    # Web GUI HTTP Server
    web_gui_path = os.path.join(
        get_package_share_directory('web_gui_control'),
        'launch_web_gui'
    )
    
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd=web_gui_path,
        output='screen'
    )
    
    # =============================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # =============================================================================
    
    # Simulation Group
    simulation_group = GroupAction([
        gazebo,
        spawn_entity,
        TimerAction(period=5.0, actions=[diff_drive_spawner]),
        TimerAction(period=5.0, actions=[joint_broad_spawner]),
    ], condition=IfCondition(use_sim))
    
    # Real Hardware Group
    real_hardware_group = GroupAction([
        controller_manager_real,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_real,
                on_start=[TimerAction(period=3.0, actions=[diff_drive_spawner])]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_real,
                on_start=[TimerAction(period=4.0, actions=[joint_broad_spawner])]
            )
        ),
        rplidar_node,
        imu_node,
        motor_usbserial_node,
    ], condition=UnlessCondition(use_sim))
    
    # Common Services Group
    common_services_group = GroupAction([
        robot_state_publisher,
        rosbridge_server,
        web_server,
        slam_node,
    ])
    
    return LaunchDescription([
        # Launch Arguments
        declare_use_sim,
        declare_robot_name,
        declare_world_file,
        
        # Launch Groups
        simulation_group,
        real_hardware_group,
        common_services_group,
    ])

if __name__ == '__main__':
    generate_launch_description()