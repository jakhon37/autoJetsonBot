#!/bin/bash
# Test script for robot movement

echo "=== Testing Robot Movement ==="
cd /autonomous_ROS
source /opt/ros/foxy/setup.bash
source install/setup.bash

echo "=== Available topics ==="
ros2 topic list

echo "=== Testing /cmd_vel topic ==="
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo "=== Testing /diff_cont/cmd_vel_unstamped topic ==="
ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo "=== Checking controller status ==="
ros2 control list_controllers

echo "=== Test completed ==="