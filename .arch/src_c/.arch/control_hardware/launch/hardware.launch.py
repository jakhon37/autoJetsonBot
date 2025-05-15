import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    # package_dir = get_package_share_directory('your_robot_package')
    package_dir = get_package_share_directory('control_hardware')
    robot_description_dir = get_package_share_directory('robot_body')
    
    # Declare use_sim_time parameter
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Load robot description
    # xacro_file = os.path.join(pkg_path,'urdf','robot.xacro')
    
    robot_description_path = os.path.join(
        robot_description_dir, 'urdf', 'robot.urdf.xacro'
    )
    robot_description = {
        'robot_description': 
        {'xacro_file': robot_description_path,
         'use_sim_time': use_sim_time},
    }
    
    # Load controllers configuration
    controller_config = os.path.join(
        package_dir, 'config', 'my_controllers.yaml'
    )
    
    # Launch controller manager with appropriate controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )
    
    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Diff drive controller
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        controller_manager,
        joint_state_broadcaster,
        diff_drive_controller,
    ])