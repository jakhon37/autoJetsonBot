#!/usr/bin/env python3
"""
navigation_launch.py - Dedicated Navigation Launch File
Clean navigation setup using AMCL localization with existing maps
Author: RovoDev Assistant - Production Navigation Solution
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    GroupAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Clean navigation launch file for autonomous operation with existing maps
    Usage:
      ros2 launch my_robot_launch navigation_launch.py
      ros2 launch my_robot_launch navigation_launch.py map_file:=custom_map.yaml
    """
    
    # =============================================================================
    # LAUNCH ARGUMENTS
    # =============================================================================
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='lab_map.yaml',
        description='Map file for navigation (should be in workspace root)'
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='my_bot',
        description='Name of the robot entity'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # =============================================================================
    # CONFIGURATION VARIABLES
    # =============================================================================
    
    map_file = LaunchConfiguration('map_file')
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    
    package_name = 'my_robot_launch'
    use_sim_time = 'true'
    
    # =============================================================================
    # ROBOT DESCRIPTION
    # =============================================================================
    
    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'robot_body_launch.py'
        )]), 
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': 'true'}.items()
    )
    
    # =============================================================================
    # GAZEBO SIMULATION
    # =============================================================================
    
    # Gazebo World
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    # world_file = os.path.join(get_package_share_directory(package_name), 'config', 'lab.world')
    world_file = os.path.join(get_package_share_directory(package_name), 'config', 'simple.world')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )
    
    # Spawn Robot Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robot_name],
        output='screen'
    )
    
    # =============================================================================
    # ROBOT CONTROLLERS
    # =============================================================================
    
    # Differential Drive Controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
        output='screen',
        remappings=[
            ('/diff_cont/cmd_vel_unstamped', '/cmd_vel'),
        ]
    )
    
    # Joint State Broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
        output='screen'
    )
    
    # =============================================================================
    # MAP SERVER
    # =============================================================================
    
    # Map Server Node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file
        }]
    )
    
    # Map Server Lifecycle Manager
    map_server_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_server_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # =============================================================================
    # AMCL LOCALIZATION
    # =============================================================================
    
    # AMCL Node for localization with existing map
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'base_footprint',
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 100.0,
            'laser_min_range': -1.0,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': 'odom',
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.0,
            'recovery_alpha_slow': 0.0,
            'resample_interval': 1,
            'robot_model_type': 'differential',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
        }]
    )
    
    # AMCL Lifecycle Manager
    amcl_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='amcl_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['amcl']
        }]
    )
    
    # =============================================================================
    # NAVIGATION STACK (NAV2)
    # =============================================================================
    
    # Nav2 Navigation Launch
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_launch_file_dir, 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    
    # =============================================================================
    # VISUALIZATION
    # =============================================================================
    
    # RViz for Navigation Visualization
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'lab_slam.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # =============================================================================
    # WEB INTERFACE
    # =============================================================================
    
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
    # INITIAL POSE SETTING
    # =============================================================================
    
    # Set initial pose for AMCL localization
    initial_pose_publisher = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/initialpose', 
             'geometry_msgs/msg/PoseWithCovarianceStamped',
             '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'],
        output='screen'
    )
    
    # =============================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # =============================================================================
    
    return LaunchDescription([
        # Launch Arguments
        declare_map_file,
        declare_robot_name,
        declare_use_rviz,
        
        # Core Robot System
        rsp,
        gazebo,
        spawn_entity,
        
        # Controllers (delayed start)
        TimerAction(period=15.0, actions=[diff_drive_spawner]),
        TimerAction(period=15.0, actions=[joint_broad_spawner]),
        
        # Map and Localization
        TimerAction(period=18.0, actions=[map_server]),
        TimerAction(period=19.0, actions=[map_server_manager]),
        TimerAction(period=20.0, actions=[amcl]),
        TimerAction(period=21.0, actions=[amcl_manager]),
        
        # Navigation Stack
        TimerAction(period=25.0, actions=[navigation_launch]),
        
        # Initial Pose (after AMCL is ready)
        TimerAction(period=30.0, actions=[initial_pose_publisher]),
        
        # Visualization and Interface
        TimerAction(period=17.0, actions=[rviz_node]),
        rosbridge_server,
        web_server,
    ])

if __name__ == '__main__':
    generate_launch_description()