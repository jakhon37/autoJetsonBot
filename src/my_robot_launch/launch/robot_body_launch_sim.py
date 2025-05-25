import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction

def generate_launch_description():
    use_ros2_control = 'true'
    use_sim_time = 'true'
    package_name = 'my_robot_launch'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'robot_body_launch.py'
        )]), 
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
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
        output='screen'
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        TimerAction(period=10.0, actions=[diff_drive_spawner]),  # Delay by 5 seconds
        TimerAction(period=10.0, actions=[joint_broad_spawner]),  # Delay by 5 seconds
        rosbridge,
        webserver
    ])