from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('my_robot_launch')
    
    # Process the URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()
    
    # Create robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': False}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    return LaunchDescription([
        node_robot_state_publisher
    ])
