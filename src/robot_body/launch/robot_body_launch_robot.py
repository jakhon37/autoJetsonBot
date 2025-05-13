import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    package_name = 'robot_body'

    # Log URDF path for debugging
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'robot.xacro')
    log_urdf_path = LogInfo(msg=f"URDF path: {urdf_path}")

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'robot_body_launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(['xacro ', urdf_path, ' use_ros2_control:=true sim_mode:=false'])}]
    )

    # Controller manager
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": Command(['xacro ', urdf_path, ' use_ros2_control:=true sim_mode:=false'])},
            controller_params_file
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Delay controller manager until robot_state_publisher starts
    controller_manager_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[controller_manager]
        )
    )

    # Spawner for diff_drive_controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )

    # Spawner for joint_state_broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    # Serial bridge node
    bridge_node = Node(
        package='robot_body',
        executable='serial_bridge',
        name='serial_bridge',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baudrate': 115200,
            'wheel_separation': 0.18,  # Match my_controllers.yaml
            'wheel_radius': 0.035,    # Match my_controllers.yaml
            'max_wheel_linear': 0.5
        }]
    )

    # Launch everything
    return LaunchDescription([
        log_urdf_path,
        rsp,
        robot_state_publisher,
        controller_manager_event_handler,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        bridge_node
    ])