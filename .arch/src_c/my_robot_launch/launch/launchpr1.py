
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory of your ROS package
    # pkg_share = get_package_share_directory('autonomous_ROS')
    
    # Include your existing autonomous car launch (which starts motor control, encoder, etc.)
    autonomous_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('launch', 'autonomous_car_launch.py')
            # os.path.join(pkg_share, 'launch', 'autonomous_car_launch.py')
        )
    )
    
    # Launch rosbridge server (non-ROS node) as an external process
    rosbridge = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
        output='screen'
    )
    
    # Launch the web server for your web GUI
    webserver = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd=os.path.join('launch', 'web_gui'),
        # cwd=os.path.join(pkg_share, 'src', 'launch', 'web_gui'),
        output='screen'
    )
    
    return LaunchDescription([
        autonomous_car,
        rosbridge,
        webserver,
    ])

if __name__ == '__main__':
    generate_launch_description()
