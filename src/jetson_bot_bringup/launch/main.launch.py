import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_bringup = get_package_share_directory('jetson_bot_bringup')
    pkg_description = get_package_share_directory('jetson_bot_description')
    pkg_navigation = get_package_share_directory('jetson_bot_navigation')
    pkg_slam = get_package_share_directory('jetson_bot_slam')
    pkg_imu = get_package_share_directory('jetson_bot_imu')

    # 1. Load Config
    config_path = os.path.join(pkg_bringup, 'config', 'system_config.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    sys_cfg = config.get('system', {})
    op_mode = sys_cfg.get('operational_mode', 'simulation')
    act_mode = sys_cfg.get('activity_mode', 'mapping')
    use_gui = sys_cfg.get('use_gui', True)

    entities = []

    # 2. Physical Description (Always needed)
    entities.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'description.launch.py')]),
        launch_arguments={'sim_mode': 'true' if op_mode == 'simulation' else 'false'}.items()
    ))

    # 3. Hardware vs Simulation specific nodes
    if op_mode == 'hardware':
        # IMU Driver
        entities.append(Node(
            package='jetson_bot_imu',
            executable='mpu6050_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ))
        # Note: Real hardware would also start the serial/arduino node here
    else:
        # Gazebo Simulation
        entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'gui': 'true' if use_gui else 'false'}.items()
        ))
        # Spawn robot
        entities.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'jetson_bot'],
            output='screen'
        ))
        # Simulation Controllers
        entities.append(TimerAction(period=5.0, actions=[
            Node(package="controller_manager", executable="spawner.py", arguments=["diff_cont"]),
            Node(package="controller_manager", executable="spawner.py", arguments=["joint_broad"])
        ]))

    # 4. Web Interface & ROSBridge
    entities.append(Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'use_sim_time': True if op_mode == 'simulation' else False}]
    ))

    entities.append(Node(
        package='jetson_bot_gui',
        executable='web_server',
        name='web_gui_server',
        output='screen',
        parameters=[{'port': 8000}]
    ))

    # 5. Activity Logic (Mapping vs Navigation)
    if act_mode == 'mapping':
        entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
            launch_arguments={
                'use_sim_time': 'true' if op_mode == 'simulation' else 'false',
                'params_file': os.path.join(pkg_slam, 'config', 'mapper_params.yaml')
            }.items()
        ))
    else:
        entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
            launch_arguments={
                'use_sim_time': 'true' if op_mode == 'simulation' else 'false',
                'params_file': os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'),
                'map': os.path.join(pkg_bringup, 'worlds', 'lab_map.yaml')
            }.items()
        ))

    # 5. RViz2 (if GUI enabled)
    if use_gui:
        rviz_config = os.path.join(pkg_description, 'rviz', 'default.rviz')
        entities.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            output='screen',
            parameters=[{'use_sim_time': True if op_mode == 'simulation' else False}]
        ))

    return entities

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
