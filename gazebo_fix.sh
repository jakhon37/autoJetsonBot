#!/bin/bash
# Fixed Gazebo launcher that actually works

echo "🎮 Starting Gazebo Simulation (Fixed Version)"

# Restart container clean
echo "Restarting container for clean state..."
docker restart auto_ros_foxy
sleep 5

# Set up display for macOS
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Setting up display for macOS..."
    xhost +localhost 2>/dev/null || echo "Note: Install XQuartz for GUI"
    DISPLAY_VAR="host.docker.internal:0"
else
    DISPLAY_VAR="${DISPLAY:-:0}"
fi

echo "Building workspace..."
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    colcon build --packages-select my_robot_launch web_gui_control
"

echo "🚀 Launching Gazebo with robot..."
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    source install/setup.bash && 
    export DISPLAY=$DISPLAY_VAR && 
    export LIBGL_ALWAYS_INDIRECT=1 && 
    export GAZEBO_MODEL_PATH=/autonomous_ROS/install/my_robot_launch/share/my_robot_launch && 
    ros2 launch my_robot_launch robot_body_launch_sim.py
"