# /home/jetson/myspace/autoJetsonBot/src/robot_body/launch/robot_body_launch_robot.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler

from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name='robot_body'

    # Get URDF via xacro
    use_ros2_control = 'true'
    use_sim_time = 'false'
   
    # use_ros2_control = 'false'
    # use_sim_time = 'true'
     
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    robot_description_content = Command([
        'xacro ', xacro_file, 
        ' use_ros2_control:=', use_ros2_control, 
        ' sim_mode:=', use_sim_time
    ])

    # Include launch file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_body_launch.py'
                )]), 
                launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    hardware_yaml = os.path.join(get_package_share_directory(package_name),'config','diffbot_hardware.yaml')
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description_content},
                    hardware_yaml,
                    controller_params_file],
        output='screen'
    )
# parameters=[hardware_yaml, controllers_yaml],

    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
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
    
    
        # ------------------------------
    # Include the sllidar_ros2 A1 launch file
    rplidar =  Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0', #'/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                # 'scan_mode': 'DenseBoost',  # Less CPU intensive mode
                # 'scan_frequency': 5.0,  # Limit to 5Hz

            }]
        )
    # -------------------------
    # slam toolbox 
    slam_pkg = get_package_share_directory('slam_launch')
    # slam_config_file = os.path.join(slam_pkg, 'config/slam_toolbox_config.yaml') #mapper_params_online_async
    slam_config_file = os.path.join(slam_pkg, 'config/mapper_params_online_async.yaml') #mapper_params_online_async

    slam_node =  Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',  # Alternatively, use 'async_slam_toolbox_node' sync_slam_toolbox_node if desired
            name='slam_toolbox',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],  # Set log level to warn
            parameters=[
                {'use_sim_time': False},  # disable simulation time
                # Use the configuration file below (adjust the file path as needed)
                slam_config_file
            ],
            # Remap the scan topic to match your sensor (if necessary)
            remappings=[('scan', '/scan')]
        )


    # -------------------------
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        webserver,
        rosbridge,
        rplidar,
        # slam_node,
        # serail_motor
        # other nodes
        # camera Node
        #...
    ])