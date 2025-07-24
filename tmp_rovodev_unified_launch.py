#!/usr/bin/env python3
"""
unified_robot_launch.py - Long-term solution for ROS2 Control Integration
Unified launch file supporting seamless switching between simulation and real hardware
Author: RovoDev Assistant
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
    Unified launch description supporting both simulation and real hardware
    with proper diffdrive_arduino integration
    """
    
    # =============================================================================
    # LAUNCH ARGUMENTS - Dynamic Configuration
    # =============================================================================
    
    declare_sim_mode = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Use simulation if true, real hardware if false',
        choices=['true', 'false']
    )
    
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control framework (recommended)',
        choices=['true', 'false']
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='jetson_bot',
        description='Name of the robot for multi-robot scenarios'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='lab.world',
        description='Gazebo world file for simulation'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='main.rviz',
        description='RViz configuration file'
    )
    
    declare_enable_slam = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM functionality',
        choices=['true', 'false']
    )
    
    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='false',
        description='Enable navigation stack',
        choices=['true', 'false']
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level',
        choices=['debug', 'info', 'warn', 'error']
    )
    
    # =============================================================================
    # CONFIGURATION VARIABLES
    # =============================================================================
    
    sim_mode = LaunchConfiguration('sim_mode')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    robot_name = LaunchConfiguration('robot_name')
    world_file = LaunchConfiguration('world_file')
    rviz_config = LaunchConfiguration('rviz_config')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_navigation = LaunchConfiguration('enable_navigation')
    log_level = LaunchConfiguration('log_level')
    
    # Package directories
    pkg_my_robot_launch = FindPackageShare('my_robot_launch')
    pkg_web_gui_control = FindPackageShare('web_gui_control')
    pkg_slam_launch = FindPackageShare('slam_launch')
    
    # =============================================================================
    # ROBOT DESCRIPTION - Dynamic URDF Generation
    # =============================================================================
    
    xacro_file = os.path.join(
        get_package_share_directory('my_robot_launch'),
        'urdf',
        'robot.xacro'
    )
    
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', sim_mode,
        ' robot_name:=', robot_name
    ])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': sim_mode
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # =============================================================================
    # HARDWARE INTERFACE - Conditional Loading
    # =============================================================================
    
    # Real Hardware Configuration
    real_hardware_config = os.path.join(
        get_package_share_directory('my_robot_launch'),
        'config',
        'diffbot_hardware.yaml'
    )
    
    # Controller Configuration - Environment Specific
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
    
    # =============================================================================
    # ROS2 CONTROL MANAGER - Real Hardware
    # =============================================================================
    
    controller_manager_real = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            real_hardware_config,
            controller_config_real,
            {'use_sim_time': False}
        ],
        condition=UnlessCondition(sim_mode),
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # =============================================================================
    # CONTROLLER SPAWNERS - Real Hardware
    # =============================================================================
    
    diff_drive_spawner_real = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        output='screen',
        condition=UnlessCondition(sim_mode)
    )
    
    joint_broad_spawner_real = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        output='screen',
        condition=UnlessCondition(sim_mode)
    )
    
    # Delayed spawning for real hardware
    delayed_diff_drive_spawner_real = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_real,
            on_start=[TimerAction(period=3.0, actions=[diff_drive_spawner_real])]
        ),
        condition=UnlessCondition(sim_mode)
    )
    
    delayed_joint_broad_spawner_real = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_real,
            on_start=[TimerAction(period=4.0, actions=[joint_broad_spawner_real])]
        ),
        condition=UnlessCondition(sim_mode)
    )
    
    # =============================================================================
    # GAZEBO SIMULATION - Conditional
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
        world_file
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items(),
        condition=IfCondition(sim_mode)
    )
    
    # Spawn Robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robot_name],
        output='screen',
        condition=IfCondition(sim_mode)
    )
    
    # Controller Spawners for Simulation
    diff_drive_spawner_sim = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output='screen',
        condition=IfCondition(sim_mode)
    )
    
    joint_broad_spawner_sim = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["joint_broad"],
        output='screen',
        condition=IfCondition(sim_mode)
    )
    
    # =============================================================================
    # SENSOR NODES - Environment Specific
    # =============================================================================
    
    # RPLidar - Real Hardware Only
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
        condition=UnlessCondition(sim_mode),
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # IMU Node - Real Hardware Only
    imu_node = Node(
        package='mpu6050_imu',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        parameters=[{
            'publish_rate': 50,
            'use_sim_time': False
        }],
        condition=UnlessCondition(sim_mode),
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # =============================================================================
    # SLAM INTEGRATION - Conditional
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
            {'use_sim_time': sim_mode},
            PythonExpression([
                "'", slam_config_sim, "' if '", sim_mode, "' == 'true' else '", slam_config_real, "'"
            ])
        ],
        remappings=[('scan', '/scan')],
        condition=IfCondition(enable_slam),
        arguments=['--ros-args', '--log-level', log_level]
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
    # DIAGNOSTIC & MONITORING
    # =============================================================================
    
    # Robot Diagnostic Node
    diagnostic_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[{
            'use_sim_time': sim_mode
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # =============================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # =============================================================================
    
    # Real Hardware Group
    real_hardware_group = GroupAction([
        controller_manager_real,
        delayed_diff_drive_spawner_real,
        delayed_joint_broad_spawner_real,
        rplidar_node,
        imu_node,
    ], condition=UnlessCondition(sim_mode))
    
    # Simulation Group  
    simulation_group = GroupAction([
        gazebo,
        spawn_entity,
        TimerAction(period=5.0, actions=[diff_drive_spawner_sim]),
        TimerAction(period=5.0, actions=[joint_broad_spawner_sim]),
    ], condition=IfCondition(sim_mode))
    
    # Common Services Group
    common_services_group = GroupAction([
        robot_state_publisher,
        rosbridge_server,
        web_server,
        slam_node,
        diagnostic_node,
    ])
    
    return LaunchDescription([
        # Launch Arguments
        declare_sim_mode,
        declare_use_ros2_control,
        declare_robot_name,
        declare_world_file,
        declare_rviz_config,
        declare_enable_slam,
        declare_enable_navigation,
        declare_log_level,
        
        # Launch Groups
        real_hardware_group,
        simulation_group,
        common_services_group,
    ])

if __name__ == '__main__':
    generate_launch_description()