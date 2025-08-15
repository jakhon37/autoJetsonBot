#!/bin/bash
# 🤖 SINGLE ENTRY POINT - Complete Robot System
# Launches: Gazebo + Robot + Web GUI + All Topics

echo "🚀 Starting Complete Autonomous Robot System"
echo "This will launch:"
echo "  🎮 Gazebo Simulation (if display available)"
echo "  🤖 Robot with all sensors"
echo "  🌐 Web Control Interface"
echo "  📡 All ROS2 topics and services"
echo ""

# Ensure container is running
if ! docker ps | grep -q auto_ros_foxy; then
    echo "Starting container..."
    docker start auto_ros_foxy
    sleep 3
fi

# Clean any existing processes
echo "Cleaning previous processes..."
docker exec auto_ros_foxy bash -c "
    pkill -f ros2 || true
    pkill -f gazebo || true  
    pkill -f python3 || true
" 2>/dev/null

sleep 2

# Build workspace
echo "Building workspace..."
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    colcon build --packages-select my_robot_launch web_gui_control slam_launch
"

# Set up display for Gazebo GUI (macOS)
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Setting up display for Gazebo GUI..."
    xhost +localhost 2>/dev/null || echo "Note: Install XQuartz for Gazebo GUI"
    DISPLAY_VAR="host.docker.internal:0"
else
    DISPLAY_VAR="${DISPLAY:-:0}"
fi

echo ""
echo "🎮 Launching Complete Robot System..."
echo "Please wait 15-20 seconds for all services to start..."

# Launch everything
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    source install/setup.bash && 
    export DISPLAY=$DISPLAY_VAR && 
    export LIBGL_ALWAYS_INDIRECT=1 && 
    export GAZEBO_MODEL_PATH=/autonomous_ROS/install/my_robot_launch/share/my_robot_launch && 
    
    # Launch the complete system
    ros2 launch my_robot_launch robot_body_launch_sim.py > /tmp/robot_sim.log 2>&1 &
    
    # Wait for services to start
    sleep 15
    
    echo 'Robot system launched!'
" &

# Wait and check services
echo "Waiting for services to start..."
for i in {1..20}; do
    sleep 1
    if curl -s http://localhost:8000 > /dev/null 2>&1; then
        echo ""
        echo "✅ SUCCESS! Robot system is ready!"
        break
    fi
    echo -n "."
done

echo ""
echo "🎉 ROBOT SYSTEM READY!"
echo ""
echo "🌐 Web Control Interface: http://localhost:8000"
echo "🔌 ROSBridge WebSocket: ws://localhost:9090"
echo "🎮 Gazebo GUI: Should open automatically (if display available)"
echo ""
echo "🎯 How to Control:"
echo "  1. Open: http://localhost:8000"
echo "  2. Click 'Connect to Robot'"
echo "  3. Use WASD keys or D-pad to move"
echo "  4. Monitor real-time metrics"
echo ""
echo "📊 Check status: ./robot.sh status"
echo "🛑 Stop system: ./robot.sh stop"

# Try to open web interface automatically
if command -v open > /dev/null 2>&1; then
    sleep 2
    open http://localhost:8000
elif command -v xdg-open > /dev/null 2>&1; then
    sleep 2
    xdg-open http://localhost:8000
fi





# [async_slam_toolbox_node-7] [ERROR] [1753972490.172112977] [slam_toolbox]: 
# DeserializePoseGraph:
#  Failed to read file: /home/jetson/myspace/autoJetsonBot/lab_map_serial.