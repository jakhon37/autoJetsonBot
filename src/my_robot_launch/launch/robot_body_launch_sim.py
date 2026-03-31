
# /home/jetson/myspace/autoJetsonBot/src/my_robot_launch/launch/robot_body_launch_sim.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    package_name = 'my_robot_launch'
    
    # Launch arguments
    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav',
        default_value='true',
        description='Enable navigation stack (Nav2)'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='lab_map.yaml',
        description='Map file for navigation'
    )
    
    enable_nav = LaunchConfiguration('enable_nav')
    map_file = LaunchConfiguration('map_file')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'robot_body_launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Gazebo World and Parameters Configuration
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    world_file = os.path.join(get_package_share_directory(package_name), 'config', 'simple.world')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    # -------------------------
    # WEB GUI CONTROLLER LAUNCH
    web_gui_pkg = get_package_share_directory('web_gui_control')
    web_gui_path = os.path.join(web_gui_pkg, 'launch_web_gui')
    
    webserver = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd=web_gui_path,
        output='screen'
    )
    
    # -------------------------
    # ROS BRIDGE LAUNCH
    rosbridge = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
        output='screen'
    )
    
    # -------------------------
    # SLAM TOOLBOX — mapping mode (used when nav is disabled)
    slam_pkg = get_package_share_directory('slam_launch')
    mapping_config = os.path.join(slam_pkg, 'config/mapper_params_online_async.yaml')
    localization_config = os.path.join(slam_pkg, 'config/slam_config_localization.yaml')
    
    slam_mapping_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[
            {'use_sim_time': True},
            mapping_config
        ],
        remappings=[('scan', '/scan')],
        condition=UnlessCondition(enable_nav)
    )
    
    slam_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[
            {'use_sim_time': True},
            localization_config
        ],
        remappings=[('scan', '/scan')],
        condition=IfCondition(enable_nav)
    )

    # -------------------------
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
        output='screen',
        remappings=[
            ('/diff_cont/cmd_vel_unstamped', '/cmd_vel'),
        ]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
        output='screen'
    )

    # -------------------------
    # RVIZ VISUALIZATION
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'lab_slam.rviz')
    
    if os.path.exists(rviz_config_file):
        rviz_args = ['-d', rviz_config_file]
    else:
        rviz_args = []
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # -------------------------
    # NAVIGATION STACK (NAV2)
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_launch_file_dir, 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_file
        }.items(),
        condition=IfCondition(enable_nav)
    )
    
    # Map server for navigation mode
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': LaunchConfiguration('map_file')
        }],
        condition=IfCondition(enable_nav)
    )
    
    # Lifecycle manager for map server
    map_server_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_server_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }],
        condition=IfCondition(enable_nav)
    )
    
    # Set initial pose after navigation starts
    initial_pose_publisher = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/initialpose', 
             'geometry_msgs/msg/PoseWithCovarianceStamped',
             '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'],
        output='screen',
        condition=IfCondition(enable_nav)
    )
    # -------------------------

    delay = 15.0

    return LaunchDescription([
        # Launch arguments
        declare_enable_nav,
        declare_map_file,
        
        # Core system
        rsp,
        gazebo,
        spawn_entity,
        TimerAction(period=delay, actions=[diff_drive_spawner]),
        TimerAction(period=delay, actions=[joint_broad_spawner]),
        TimerAction(period=delay+2, actions=[rviz_node]),
        
        # Services
        rosbridge,
        webserver,
        slam_mapping_node,
        slam_localization_node,
        
        # Navigation (conditional)
        TimerAction(period=delay+3, actions=[map_server]),
        TimerAction(period=delay+4, actions=[map_server_manager]),
        TimerAction(period=delay+6, actions=[navigation_launch]),
        TimerAction(period=delay+18, actions=[initial_pose_publisher]),
    ])