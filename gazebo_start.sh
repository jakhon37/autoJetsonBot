#!/bin/bash
# Gazebo Simulation Startup Script with GUI

echo "🎮 Starting Gazebo Simulation with Robot..."

# Start container if needed
if ! docker ps | grep -q auto_ros_foxy; then
    echo "Starting container..."
    docker start auto_ros_foxy
    sleep 3
fi

# Set up X11 forwarding for macOS
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Setting up X11 forwarding for macOS..."
    # Check if XQuartz is running
    if ! pgrep -x "Xquartz" > /dev/null; then
        echo "⚠️  XQuartz not running. Please start XQuartz first:"
        echo "   Applications > Utilities > XQuartz"
        echo "   Then run: xhost +localhost"
        read -p "Press Enter when XQuartz is ready..."
    fi
    xhost +localhost 2>/dev/null || echo "Note: xhost command not found"
    DISPLAY_VAR="host.docker.internal:0"
else
    # Linux
    DISPLAY_VAR="${DISPLAY:-:0}"
fi

echo "Building workspace..."
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    colcon build --packages-select my_robot_launch web_gui_control
"

echo "🚀 Launching Gazebo simulation..."
echo "This will open:"
echo "  🎮 Gazebo GUI - Robot simulation environment"
echo "  🌐 Web Interface - http://localhost:8000"
echo "  🔌 ROSBridge - ws://localhost:9090"
echo ""

# Launch with proper display settings
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    source install/setup.bash && 
    export DISPLAY=$DISPLAY_VAR && 
    export QT_X11_NO_MITSHM=1 && 
    ros2 launch my_robot_launch robot_body_launch_sim.py
"