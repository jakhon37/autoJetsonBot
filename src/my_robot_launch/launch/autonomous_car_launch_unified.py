#!/usr/bin/env python3
"""
autonomous_car_launch_unified.py - Unified launch file for both sim and real robot
Combines working simulation and real robot configurations with a single flag
Author: RovoDev Assistant - Unified Version
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    # =============================================================================
    # WEB INTERFACE & COMMUNICATION (Common for both)
    # =============================================================================
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Unified launch description for both simulation and real robot
    Usage:
      ros2 launch my_robot_launch autonomous_car_launch_unified.py use_sim:=true   # For simulation
      ros2 launch my_robot_launch autonomous_car_launch_unified.py use_sim:=false  # For real robot
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
        default_value='my_bot',
        description='Name of the robot'
    )
    
    # =============================================================================
    # CONFIGURATION VARIABLES
    # =============================================================================
    
    use_sim = LaunchConfiguration('use_sim')
    robot_name = LaunchConfiguration('robot_name')
    package_name = 'my_robot_launch'
    
    # =============================================================================
    # ROBOT DESCRIPTION (Common for both sim and real)
    # =============================================================================
    
    # Robot State Publisher (using working approach from both files)
    rsp_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'robot_body_launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items(),
        condition=IfCondition(use_sim)
    )
    
    rsp_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'robot_body_launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items(),
        condition=UnlessCondition(use_sim)
    )
    
    # =============================================================================
    # GAZEBO SIMULATION (Only when use_sim:=true)
    # =============================================================================
    
    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), 
        'config', 
        'gazebo_params.yaml'
    )
    
    # Use the specific world file path you provided
    world_path = '/home/jakhon37/myspace/robotics/autoJetsonBot/src/my_robot_launch/config/lab.world'
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items(),
        condition=IfCondition(use_sim)
    )
    
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robot_name],
        output='screen',
        condition=IfCondition(use_sim)
    )
    
    # =============================================================================
    # CONTROLLERS (Different approach for sim vs real)
    # =============================================================================
    
    # === SIMULATION CONTROLLERS ===
    diff_drive_spawner_sim = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
        output='screen',
        remappings=[
            ('/diff_cont/cmd_vel_unstamped', '/cmd_vel'),
        ],
        condition=IfCondition(use_sim)
    )

    joint_broad_spawner_sim = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
        output='screen',
        condition=IfCondition(use_sim)
    )
    
    # === REAL ROBOT CONTROLLERS ===
    # Get robot description for real robot controller manager
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    robot_description_content = Command([
        'xacro ', xacro_file, 
        ' use_ros2_control:=true', 
        ' sim_mode:=false'
    ])
    
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )
    hardware_yaml = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'diffbot_hardware.yaml'
    )
    
    controller_manager_real = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description_content},
            hardware_yaml,
            controller_params_file
        ],
        output='screen',
        condition=UnlessCondition(use_sim)
    )

    diff_drive_spawner_real = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
        condition=UnlessCondition(use_sim)
    )

    joint_broad_spawner_real = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
        condition=UnlessCondition(use_sim)
    )
    
    # =============================================================================
    # SENSORS (Real Hardware Only)
    # =============================================================================
    
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
        }],
        condition=UnlessCondition(use_sim)
    )
    
    # =============================================================================
    # SLAM TOOLBOX (Different configs for sim vs real)
    # =============================================================================
    
    slam_pkg = get_package_share_directory('slam_launch')
    slam_config_real = os.path.join(slam_pkg, 'config', 'mapper_params_online_async.yaml')
    slam_config_sim = os.path.join(slam_pkg, 'config', 'mapper_params_online_async-sim.yaml')

    slam_node_sim = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[
            {'use_sim_time': True},
            slam_config_sim
        ],
        remappings=[('scan', '/scan')],
        condition=IfCondition(use_sim)
    )
    
    slam_node_real = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[
            {'use_sim_time': False},
            slam_config_real
        ],
        remappings=[('scan', '/scan')],
        condition=UnlessCondition(use_sim)
    )
    
    # =============================================================================
    # DEBUGGING & DIAGNOSTICS
    # =============================================================================
    
    # Scan topic monitor (for debugging scan data issues)
    scan_monitor = Node(
        package='ros2',
        executable='topic',
        arguments=['echo', '/scan', '--field', 'header'],
        output='screen',
        condition=IfCondition(use_sim),
        # Comment out this node after debugging
    )
    
    # ROSBridge WebSocket Server
    rosbridge_server = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
        output='screen'
    )
    
    # Web GUI HTTP Server
    web_gui_pkg = get_package_share_directory('web_gui_control')
    web_gui_path = os.path.join(web_gui_pkg, 'launch_web_gui')
    
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd=web_gui_path,
        output='screen'
    )
    
    # =============================================================================
    # LAUNCH GROUPS WITH PROPER TIMING
    # =============================================================================
    
    # Simulation Group (based on working robot_body_launch_sim.py)
    simulation_group = GroupAction([
        rsp_sim,
        gazebo,
        spawn_entity,
        TimerAction(period=15.0, actions=[diff_drive_spawner_sim]),
        TimerAction(period=15.0, actions=[joint_broad_spawner_sim]),
        TimerAction(period=20.0, actions=[slam_node_sim]),  # Start SLAM after everything else
    ], condition=IfCondition(use_sim))
    
    # Real Robot Group (based on working robot_body_launch_robot.py)
    real_robot_group = GroupAction([
        rsp_real,
        TimerAction(period=5.0, actions=[controller_manager_real]),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_real,
                on_start=[diff_drive_spawner_real],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_real,
                on_start=[joint_broad_spawner_real],
            )
        ),
        rplidar_node,
        slam_node_real,
    ], condition=UnlessCondition(use_sim))
    
    # Common Services (for both sim and real)
    common_services_group = GroupAction([
        rosbridge_server,
        web_server,
    ])
    
    return LaunchDescription([
        # Launch Arguments
        declare_use_sim,
        declare_robot_name,
        
        # Launch Groups
        simulation_group,
        real_robot_group,
        common_services_group,
    ])

if __name__ == '__main__':
    generate_launch_description()