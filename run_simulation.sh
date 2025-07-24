#!/bin/bash
echo "Starting Autonomous Robot Simulation..."
cd /autonomous_ROS
source /opt/ros/foxy/setup.bash
source install/setup.bash

echo "Launching simulation..."
ros2 launch my_robot_launch robot_body_launch_sim.py
