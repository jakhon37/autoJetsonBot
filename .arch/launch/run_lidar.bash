

    #   sudo apt install ros-humble-rplidar-ros


ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200 \
    -p frame_id:=lidar_link \
    -p angle_compensate:=true \
    -p scan_mode:=Standard



