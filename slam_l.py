from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',  # For creating a map
                'transform_publish_period': 0.02,
                'map_update_interval': 5.0,
                'minimum_travel_distance': 0.5,
                'minimum_travel_heading': 0.5,
            }]
        ),
    ])
    
    # ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/Twist '{linear: {x:  0.1}, angular: {z: 0.0}}' 