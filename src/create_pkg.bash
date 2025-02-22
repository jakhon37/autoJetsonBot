
ros2 pkg create --build-type ament_python motor_control --dependencies rclpy geometry_msgs
ros2 pkg create --build-type ament_python mpu6050_imu --dependencies rclpy sensor_msgs
pip3 install smbus2

ros2 pkg create --build-type ament_python object_detection --dependencies rclpy sensor_msgs cv_bridge
ros2 pkg create --build-type ament_python camera --dependencies rclpy sensor_msgs cv_bridge

ros2 pkg create --build-type ament_python my_robot_slam --dependencies rclpy sensor_msgs geometry_msgs std_msgs launch launch_ros ament_index_python rplidar_ros slam_toolbox


ros2 pkg create --build-type ament_python my_robot_launch --dependencies rclpy launch launch_ros ament_index_python
