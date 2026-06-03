import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Arguments
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Use simulation mode (Gazebo)'
    )

    # 2. Path to the XACRO file
    pkg_description = get_package_share_directory('jetson_bot_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.xacro')
    
    # 3. Process XACRO
    # Note: Xacro needs to be processed with the mappings
    def process_xacro(context):
        sim_mode = LaunchConfiguration('sim_mode').perform(context)
        robot_description_config = xacro.process_file(xacro_file, mappings={'sim_mode': sim_mode})
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config.toxml(), 'use_sim_time': True if sim_mode == 'true' else False}]
        )]

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        sim_mode_arg,
        OpaqueFunction(function=process_xacro)
    ])
