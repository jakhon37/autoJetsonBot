
# apt install ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gazebo-ros2-control

# ros2 control list_hardware_interfaces

#  ros2 control list_controllers 


# ros2 run controller_manager spawner.py diff_cont

# ros2 run controller_manager spawner.py joint_broad



# ros2 run teleop_twist_keyboard  teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped