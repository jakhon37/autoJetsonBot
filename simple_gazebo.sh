#!/bin/bash
# Simple working Gazebo launcher

echo "🎮 Starting Simple Gazebo Simulation"

# Clean start
docker exec auto_ros_foxy bash -c "pkill -f gazebo && pkill -f ros2" || true
sleep 2

echo "Building workspace..."
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    colcon build --packages-select my_robot_launch
"

echo "🚀 Starting Gazebo with robot..."
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    source install/setup.bash && 
    export DISPLAY=host.docker.internal:0 && 
    export LIBGL_ALWAYS_INDIRECT=1 && 
    
    # Start Gazebo server first
    echo 'Starting Gazebo server...' &&
    gzserver worlds/empty.world &
    sleep 3 &&
    
    # Spawn robot
    echo 'Spawning robot...' &&
    ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_bot &
    sleep 2 &&
    
    # Start robot state publisher
    echo 'Starting robot state publisher...' &&
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(xacro src/my_robot_launch/urdf/robot.xacro)\" &
    sleep 2 &&
    
    # Try to start Gazebo client
    echo 'Starting Gazebo GUI...' &&
    gzclient &
    
    echo 'Gazebo simulation running!'
    echo 'Press Ctrl+C to stop'
    wait
"