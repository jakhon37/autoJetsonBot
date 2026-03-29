

# /home/jetson/myspace/autoJetsonBot/src/my_robot_launch/launch/robot_body_launch_sim.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    use_ros2_control = 'true'
    use_sim_time = 'true'
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
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )

    # Gazebo World and Parameters Configuration
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

    # -------------------------
    # WEB GUI CONTROLLER LAUNCH
    # Get the path to the web_gui package
    web_gui_pkg = get_package_share_directory('web_gui_control')
    web_gui_path = os.path.join(web_gui_pkg, 'launch_web_gui')
    # print(f'web gui path: {web_gui_path}')
    # Launch an HTTP server to serve the web GUI on port 8000
    webserver = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
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
    # SLAM TOOLBOX OR LOCALIZATION
    slam_pkg = get_package_share_directory('slam_launch')
    
    # Choose between mapping and localization based on navigation mode
    mapping_config = os.path.join(slam_pkg, 'config/mapper_params_online_async-sim.yaml')
    localization_config = os.path.join(slam_pkg, 'config/slam_config_localization.yaml')
    
    # Use localization mode when navigation is enabled, mapping mode otherwise
    slam_node =  Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node' if enable_nav else 'async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[
                {'use_sim_time': True},
                localization_config if enable_nav else mapping_config
            ],
            remappings=[('scan', '/scan')]
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


    pr = 15.0
    # -------------------------
    # RVIZ VISUALIZATION
    # Launch RViz with lab environment configuration
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'lab_slam.rviz')
    
    # Try to use config file if it exists, otherwise use default RViz
    try:
        if os.path.exists(rviz_config_file):
            rviz_args = ['-d', rviz_config_file]
            print(f'Using RViz config: {rviz_config_file}')
        else:
            rviz_args = []
            print(f'Config file not found: {rviz_config_file}, using default RViz')
    except:
        rviz_args = []
        print('Using default RViz configuration')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    print(f'RViz launched with lab configuration: {rviz_config_file}')
    # -------------------------
    
    # -------------------------
    # NAVIGATION STACK (NAV2)
    # Launch Nav2 navigation stack conditionally
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
    
    # Map server for navigation mode (loads existing map)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file
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
    
    # Set initial pose after navigation starts (delayed)
    initial_pose_publisher = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/initialpose', 
             'geometry_msgs/msg/PoseWithCovarianceStamped',
             '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'],
        output='screen',
        condition=IfCondition(enable_nav)
    )
    
    print(f'Navigation will be enabled: {enable_nav}')
    print(f'Map file: {map_file}')
    # -------------------------

    return LaunchDescription([
        # Launch arguments
        declare_enable_nav,
        declare_map_file,
        
        # Core system
        rsp,
        gazebo,
        spawn_entity,
        TimerAction(period=pr, actions=[diff_drive_spawner]),  # Delay for controllers
        TimerAction(period=pr, actions=[joint_broad_spawner]),  # Delay for controllers
        TimerAction(period=pr+2, actions=[rviz_node]),  # Launch RViz after controllers
        
        # Services
        rosbridge,
        webserver,
        slam_node,
        
        # Navigation (conditional)
        TimerAction(period=pr+3, actions=[map_server]),  # Map server first
        TimerAction(period=pr+4, actions=[map_server_manager]),  # Then lifecycle manager
        TimerAction(period=pr+6, actions=[navigation_launch]),  # Nav2 after map server
        TimerAction(period=pr+18, actions=[initial_pose_publisher]),  # Set pose after Nav2
    ])