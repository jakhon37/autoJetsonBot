

colcon build --packages-select my_robot_launch
# colcon build --packages-select web_gui_control
# colcon build --packages-select slam_launch
source install/setup.bash
# LC_NUMERIC="en_US.UTF-8"; ros2 launch my_robot_launch test_body.py
# ros2 launch my_robot_launch test_body.py
# ros2 launch my_robot_launch robot_body_launch.py

ros2 launch my_robot_launch robot_body_launch_sim.py 

# ros2 launch my_robot_launch robot_body_launch_robot.py sim_mode:=true
# ros2 launch my_robot_launch robot_body_launch_robot.py
# ros2 launch robot_body robot_body_launch_robot.py sim_mode:=true
# ros2 launch robot_body robot_body_launch_sim.py  world:=./src/robot_body/urdf/lab.world

# ros2 launch slam_toolbox online_async_launch.py params_file:=./src/slam_launch/config/mapper_params_online_async.yaml 

 

# ros2 run gazebo_ros spawn_entity.py -topic /robot_body -entity my_robot 

# ros2 launch robot_body robot_body_launch.py use_sim_time:=True

# ros2 run joint_state_publisher_gui joint_state_publisher_gui 


# ros2 run joint_state_publisher_gui joint_state_publisher_gui 

# ros2 run teleop_twist_keyboard teleop_twist_keyboard 
# ros2 run teleop_twist_keyboard  teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped


# LIDAR 

# ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard'

# CAMERA 

# ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_raw/uncompressed

# sudo apt install libraspberrypi-bin v4l-utils ros-foxy-v4l2-camera
# sudo usermod -a -G video $USER
#  vcgencmd get_camera ?
# raspistill -k  ?
# v4l2-ctl --list-devices
#  ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" -p camera_frame_id:=camera_optical_link 



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


