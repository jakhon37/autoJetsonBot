#!/bin/bash
# Complete simulation startup script with robot movement testing

echo "🚀 Starting Autonomous Robot Simulation..."
cd /autonomous_ROS
source /opt/ros/foxy/setup.bash
source install/setup.bash

echo "📦 Building project..."
colcon build --symlink-install
source install/setup.bash

echo "🎮 Launching simulation..."
ros2 launch my_robot_launch robot_body_launch_sim.py &
LAUNCH_PID=$!

echo "⏳ Waiting for simulation to start..."
sleep 15

echo "🎯 Testing robot movement..."
echo "Moving forward..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

sleep 2

echo "Turning left..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

sleep 2

echo "Stopping..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo "✅ Robot movement test completed!"
echo "🌐 Web GUI available at: http://localhost:8000"
echo "📊 Available topics:"
ros2 topic list

echo "Press Ctrl+C to stop simulation"
wait $LAUNCH_PID