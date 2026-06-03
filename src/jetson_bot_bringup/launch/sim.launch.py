import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('jetson_bot_bringup')
    pkg_description = get_package_share_directory('jetson_bot_description')
    pkg_navigation = get_package_share_directory('jetson_bot_navigation')
    pkg_slam = get_package_share_directory('jetson_bot_slam')
    
    # 2. Arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='Operational mode: "mapping" or "navigation"'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Start Gazebo Client (3D window)'
    )

    # 3. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': os.path.join(pkg_bringup, 'worlds', 'lab.world'),
            'gui': LaunchConfiguration('gui')
        }.items()
    )

    # 2. Robot State Publisher
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_description, 'launch', 'description.launch.py')]),
        launch_arguments={'sim_mode': 'true'}.items()
    )

    # 3. Spawn (Delayed slightly to ensure Gazebo is ready)
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'jetson_bot'],
                output='screen'
            )
        ]
    )

    # 4. Controllers (Delayed further)
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    # 5. Web Interface & ROSBridge
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    web_gui = Node(
        package='jetson_bot_gui',
        executable='web_server',
        name='web_gui_server',
        output='screen',
        parameters=[{'port': 8000}]
    )

    # 6. SLAM / Nav (Delayed even further)
    slam = TimerAction(
        period=20.0,
        actions=[
            # Mapping Mode
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"])),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': os.path.join(pkg_slam, 'config', 'mapper_params.yaml')
                }.items()
            ),
            # Navigation/Localization Mode
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'navigation'"])),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': os.path.join(pkg_slam, 'config', 'localization_params.yaml')
                }.items()
            )
        ]
    )

    return LaunchDescription([
        mode_arg,
        gui_arg,
        gazebo,
        robot_description,
        spawn_entity,
        rosbridge,
        web_gui,
        TimerAction(period=15.0, actions=[diff_drive_spawner, joint_broad_spawner]),
        slam
    ])
