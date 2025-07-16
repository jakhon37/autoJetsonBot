#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory of your package
    pkg_share = get_package_share_directory('autonomous_ROS')
    
    # Path to the autonomous car launch file (starts motor control, encoder, etc.)
    autonomous_car_launch = os.path.join(pkg_share, 'launch', 'autonomous_car_launch.py')
    autonomous_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(autonomous_car_launch)
    )
    
    # Launch the rosbridge server (runs rosbridge_websocket on port 9090)
    rosbridge = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
        output='screen'
    )
    
    # Path to the web GUI directory (contains index.html)
    web_gui_path = os.path.join(pkg_share, 'launch', 'web_gui')
    # Launch an HTTP server to serve the web GUI on port 8000
    webserver = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd=web_gui_path,
        output='screen'
    )
    
    return LaunchDescription([
        autonomous_car,
        rosbridge,
        webserver,
    ])

if __name__ == '__main__':
    generate_launch_description()
