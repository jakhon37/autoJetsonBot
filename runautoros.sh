

#!/bin/bash
# Source ROS 2 and workspace environments
# source /opt/ros/foxy/setup.bash
# source /home/ubuntu/my_space/autonomous_ROS/install/setup.bash
source install/setup.bash

# Launch the unified ROS launch file
# ros2 launch my_robot_slam all_nodes_launch.py
# ros2 launch launch/autonomous_car_launch.py
# /home/ubuntu/my_space/autonomous_ROS/launch/launchpr.py

# ros2 launch launchpr.py

# ros2 launch my_robot_launch launchpr.py
ros2 launch my_robot_launch autonomous_car_launch.py




