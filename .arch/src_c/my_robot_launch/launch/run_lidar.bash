

# ls /dev | grep ttyUSB

    #   sudo apt install ros-humble-rplidar-ros


ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200 \
    -p frame_id:=lidar_link \
    -p angle_compensate:=true \
    -p scan_mode:=Standard




# https://github.com/Slamtec/sllidar_ros2.git
# ros2 launch sllidar_ros2 view_sllidar_a1_launch.py



# ros2 launch  rplidar_ros  view_rplidar_a1_launch.py