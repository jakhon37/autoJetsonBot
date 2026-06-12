import os
import yaml
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_bringup = get_package_share_directory('jetson_bot_bringup')
    pkg_description = get_package_share_directory('jetson_bot_description')
    pkg_navigation = get_package_share_directory('jetson_bot_navigation')
    pkg_slam = get_package_share_directory('jetson_bot_slam')
    pkg_imu = get_package_share_directory('jetson_bot_imu')
    pkg_gui = get_package_share_directory('jetson_bot_gui')

    # 1. Load Unified Config
    config_path = os.path.join(pkg_bringup, 'config', 'unified_robot_config.yaml')
    
    # Defaults
    use_sim = True
    mode = 'mapping'
    viz = True
    headless = False
    use_sim_time = True
    map_name = 'lab_map'
    map_dir = os.path.join(pkg_bringup, 'worlds')
    gazebo_world = 'lab.world'
    rviz_config_file = 'default.rviz'
    spawn_x = 0.0
    spawn_y = 0.0
    spawn_z = 0.06
    spawn_yaw = 0.0

    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
                if 'global' in data and 'ros__parameters' in data['global']:
                    params = data['global']['ros__parameters']
                    use_sim = params.get('use_sim', use_sim)
                    mode = params.get('mode', mode)
                    viz = params.get('viz', viz)
                    headless = params.get('headless', headless)
                    use_sim_time = params.get('use_sim_time', use_sim_time)
                    map_name = params.get('map_name', map_name)
                    map_dir = params.get('map_dir', map_dir)
                    gazebo_world = params.get('gazebo_world', gazebo_world)
                    rviz_config_file = params.get('rviz_config', rviz_config_file)
                    spawn_x = params.get('spawn_x', spawn_x)
                    spawn_y = params.get('spawn_y', spawn_y)
                    spawn_z = params.get('spawn_z', spawn_z)
                    spawn_yaw = params.get('spawn_yaw', spawn_yaw)
        except Exception as e:
            print(f"Warning: Failed to load unified config: {e}")

    # Override with CLI arguments if provided
    cli_use_sim = LaunchConfiguration('sim', default='').perform(context)
    cli_mode = LaunchConfiguration('mode', default='').perform(context)
    cli_viz = LaunchConfiguration('viz', default='').perform(context)
    cli_headless = LaunchConfiguration('headless', default='').perform(context)

    # Final variables to use (CLI takes precedence over YAML)
    final_use_sim = (cli_use_sim.lower() == 'true') if cli_use_sim != '' else use_sim
    final_mode = cli_mode if cli_mode != '' else mode
    final_viz = (cli_viz.lower() == 'true') if cli_viz != '' else viz
    final_headless = (cli_headless.lower() == 'true') if cli_headless != '' else headless
    
    # Map full path
    final_map_path = os.path.join(map_dir, f"{map_name}.yaml")
    
    # Export config for Web UI
    web_config = {
        'use_sim': final_use_sim,
        'mode': final_mode,
        'viz': final_viz,
        'headless': final_headless,
        'map_name': map_name,
        'map_dir': map_dir,
        'gazebo_world': gazebo_world,
        'rviz_config': rviz_config_file
    }
    
    # Try to write to both share and src (for persistence if symlinked)
    web_config_paths = [
        os.path.join(pkg_gui, 'web', 'config.json'),
        os.path.join(pkg_gui, '..', '..', '..', '..', 'src', 'jetson_bot_gui', 'web', 'config.json')
    ]
    
    for p in web_config_paths:
        try:
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, 'w') as f:
                json.dump(web_config, f, indent=2)
        except Exception:
            pass

    entities = []

    # 2. Physical Model (Always)
    entities.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'description.launch.py')]),
        launch_arguments={'sim_mode': 'true' if final_use_sim else 'false'}.items()
    ))

    # 3. Hardware vs Simulation Logic
    hardware_active = False
    if final_use_sim:
        hardware_active = True # Simulation always counts as active
        # --- GAZEBO STACK ---
        entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={
                'world': os.path.join(pkg_bringup, 'worlds', gazebo_world),
                'gui': 'false' if final_headless else 'true'
            }.items()
        ))
        
        # Spawn Entity (Delayed)
        entities.append(TimerAction(period=5.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'jetson_bot', '-z', '0.06'],
                output='screen'
            )
        ]))

        # Controllers (Delayed)
        entities.append(TimerAction(period=90.0, actions=[
            Node(package="controller_manager", executable="spawner.py", arguments=["diff_cont"]),
            Node(package="controller_manager", executable="spawner.py", arguments=["joint_broad"])
        ]))
    else:
        # --- REAL HARDWARE STACK ---
        print("\n" + "="*50)
        print("🔍 HARDWARE AUDIT STARTING...")
        
        # 1. IMU Bridge & Filter
        entities.append(Node(
            package='jetson_bot_imu',
            executable='imu_bridge_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ))
        
        # Madgwick Filter to convert raw IMU -> Orientation
        entities.append(Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'stateless': True,
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'fixed_frame': 'odom'
            }],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/data', '/imu/data')
            ]
        ))

        # 2. EKF Sensor Fusion
        entities.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_bringup, 'config', 'ekf.yaml'), {'use_sim_time': False}],
            remappings=[('/odometry/filtered', '/odom_filtered')]
        ))
        
        # 3. Python Serial Bridge (Check if port exists)
        motor_port = '/dev/ttyACM0'
        if os.path.exists(motor_port):
            hardware_active = True
            entities.append(Node(
                package='jetson_bot_diffdrive',
                executable='diffdrive_node',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'port': motor_port,
                    'baud': 115200,
                    'wheel_separation': 0.212, # Manual Measurement
                    'wheel_radius': 0.034,
                    'encoder_cpr': 3436,
                    'publish_tf': False # Disable for EKF compatibility
                }]
            ))
            print(f"✅ [HARDWARE] Motor Bridge: FOUND at {motor_port}")
        else:
            print(f"❌ [HARDWARE ERROR] Motor port {motor_port} NOT FOUND. Connect ESP32 via USB.")

        # 3. RPLidar A1 (Check if port exists)
        lidar_port = '/dev/ttyUSB0'
        if os.path.exists(lidar_port):
            hardware_active = True
            entities.append(Node(
                package='rplidar_ros',
                executable='rplidar_composition',
                output='screen',
                parameters=[{
                    'channel_type': 'serial',
                    'serial_port': lidar_port,
                    'serial_baudrate': 115200,
                    'frame_id': 'laser_frame',
                    'inverted': False,
                    'angle_compensate': True,
                    'scan_mode': 'Standard'  # Force standard mode for A1
                }]
            ))
            print(f"✅ [HARDWARE] RPLidar: FOUND at {lidar_port}")
        else:
            print(f"❌ [HARDWARE ERROR] Lidar port {lidar_port} NOT FOUND. Connect RPLidar via USB.")
        
        if not hardware_active:
            print("🛑 [FATAL WARNING] No critical sensors found. SLAM/Navigation will be DISABLED to prevent crash.")
            print("🚀 [INFO] Web UI and Telemetry will remain active for debugging.")
        
        print("="*50 + "\n")

    # 4. Global Infrastructure
    entities.append(Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        parameters=[{'use_sim_time': final_use_sim}],
        output='screen'
    ))

    entities.append(Node(
        package='jetson_bot_gui',
        executable='web_server',
        parameters=[{'port': 8000}],
        output='screen'
    ))

    entities.append(Node(
        package='jetson_bot_gui',
        executable='telemetry_node',
        parameters=[{'use_sim_time': final_use_sim}],
        output='screen'
    ))

    # Camera MJPEG stream — serves /camera/image_raw on port 8080
    # Guarded: skip gracefully if web_video_server is not installed
    try:
        get_package_share_directory('web_video_server')
        entities.append(Node(
            package='web_video_server',
            executable='web_video_server',
            parameters=[{'port': 8080}],
            output='screen'
        ))
    except Exception:
        print("Warning: web_video_server not found — camera stream disabled.")

    # 5. Activity (Mapping vs Navigation)
    if hardware_active:
        if final_mode == 'mapping':
            entities.append(TimerAction(period=120.0 if final_use_sim else 2.0, actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
                    launch_arguments={
                        'use_sim_time': 'true' if final_use_sim else 'false',
                        'params_file': os.path.join(pkg_slam, 'config', 'mapper_params.yaml')
                    }.items()
                )
            ]))
        else:
            entities.append(TimerAction(period=120.0 if final_use_sim else 2.0, actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
                    launch_arguments={
                        'use_sim_time': 'true' if final_use_sim else 'false',
                        'params_file': os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'),
                        'map': final_map_path
                    }.items()
                )
            ]))

    # 6. Visualization (RViz2)
    if final_viz:
        # Try to find the specific rviz config
        rviz_config = os.path.join(pkg_bringup, 'config', rviz_config_file)
        
        # Fallback to lab_slam.rviz if not found
        if not os.path.exists(rviz_config):
            rviz_config = os.path.join(pkg_bringup, 'config', 'default.rviz')
        
        # Absolute fallback for container environment
        if not os.path.exists(rviz_config):
            rviz_config = "/autonomous_ROS/install/jetson_bot_bringup/share/jetson_bot_bringup/config/default.rviz"
            
        entities.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': final_use_sim}],
            output='screen'
        ))

    return entities

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='', description='Use simulation (true/false)'),
        DeclareLaunchArgument('mode', default_value='', description='mapping or navigation'),
        DeclareLaunchArgument('viz', default_value='', description='Open RViz (true/false)'),
        DeclareLaunchArgument('headless', default_value='', description='Gazebo GUI off (true/false)'),
        OpaqueFunction(function=launch_setup)
    ])
