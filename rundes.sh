

colcon build --packages-select robot_body
source install/setup.bash


ros2 launch robot_body robot_body_launch_sim.py




# ros2 run gazebo_ros spawn_entity.py -topic /robot_body -entity my_robot 

# ros2 launch robot_body robot_body_launch.py use_sim_time:=True

# ros2 run joint_state_publisher_gui joint_state_publisher_gui 


# ros2 run joint_state_publisher_gui joint_state_publisher_gui 



# GAZEBO HORMONIC FOR UBUNTU 24.04 ROS2 JAZZY

# sudo apt-get update
# sudo apt-get install curl lsb-release gnupg

# sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# sudo apt-get update
# sudo apt-get install gz-harmonic

# gz sim 


# GAZEBO FORTRESS FOR UBUNTU 22.04 ROS2 HUMBLE

# sudo apt-get update
# sudo apt-get install lsb-release gnupg
# sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# sudo apt-get update
# sudo apt-get install ignition-fortress



# ros2 run teleop_twist_keyboard teleop_twist_keyboard 