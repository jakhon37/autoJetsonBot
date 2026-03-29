#!/bin/bash
# 🚀 Complete Lab Environment with Navigation
# Launches everything: Gazebo + SLAM + Navigation + Web Interface

echo "🚀 Starting Complete Lab Environment with Navigation..."
echo "This will launch:"
echo "  🌍 Gazebo with lab.world environment"
echo "  🤖 Robot with all sensors and controllers"
echo "  📊 RViz with lab visualization config"
echo "  🌐 Web control interface"
echo "  🗺️ SLAM mapping system"
echo "  🎯 Nav2 navigation stack"
echo ""

# Check if map file exists
MAP_FILE="lab_map.yaml"
if [ ! -f "$MAP_FILE" ]; then
    echo "⚠️  Map file $MAP_FILE not found!"
    echo "To create a map first, run:"
    echo "  ./launch_lab_sim.sh     # Start SLAM mapping"
    echo "  # Drive around to map environment"
    echo "  # Then save map and run this script"
    exit 1
fi

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
    pkill -f rviz || true
    pkill -f python3 || true
" 2>/dev/null

sleep 2

# Set up display for GUI applications
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Setting up display for macOS..."
    xhost +localhost 2>/dev/null || echo "Note: Install XQuartz for GUI"
    DISPLAY_VAR="host.docker.internal:0"
else
    DISPLAY_VAR="${DISPLAY:-:0}"
    xhost +local:docker 2>/dev/null || echo "Note: GUI display may not work"
fi

echo ""
echo "🚀 Launching Complete Lab Environment with Navigation..."
echo "Please wait 30-35 seconds for all services to start..."

# Launch the complete system with navigation
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    source install/setup.bash && 
    export DISPLAY=$DISPLAY_VAR && 
    export LIBGL_ALWAYS_INDIRECT=1 && 
    export GAZEBO_MODEL_PATH=/autonomous_ROS/install/my_robot_launch/share/my_robot_launch && 
    
    # Launch the complete system with navigation enabled
    ros2 launch my_robot_launch robot_body_launch_sim.py enable_nav:=true map_file:=$MAP_FILE > /tmp/complete_lab.log 2>&1 &
    
    echo 'Complete lab environment with navigation launched!'
" &

# Wait and check services
echo "Waiting for services to start..."
for i in {1..35}; do
    sleep 1
    if curl -s http://localhost:8000 > /dev/null 2>&1; then
        echo ""
        echo "✅ SUCCESS! Complete Lab Environment Ready!"
        break
    fi
    if [ $((i % 5)) -eq 0 ]; then
        echo -n " ${i}s"
    else
        echo -n "."
    fi
done

echo ""
echo "🚀 COMPLETE LAB ENVIRONMENT WITH NAVIGATION READY!"
echo ""
echo "🌍 Gazebo: Lab world environment loaded"
echo "📊 RViz: Lab visualization configuration active"
echo "🌐 Web Control: http://localhost:8000"
echo "🔌 ROSBridge: ws://localhost:9090"
echo "🗺️ SLAM: Active localization mode"
echo "🎯 Navigation: Nav2 stack ready for autonomous navigation"
echo ""
echo "🎯 Navigation Ready! You can now:"
echo "  1. Use web interface for manual control"
echo "  2. Send navigation goals via command line:"
echo "     ./navigate_robot.sh goal 2.0 1.5"
echo "  3. Monitor autonomous navigation in RViz"
echo "  4. Cancel navigation: ./navigate_robot.sh cancel"
echo ""
echo "📊 Check status: ./robot.sh status"
echo "🛑 Stop system: ./robot.sh stop"

# Try to open web interface automatically
if command -v open > /dev/null 2>&1; then
    sleep 3
    open http://localhost:8000
elif command -v xdg-open > /dev/null 2>&1; then
    sleep 3
    xdg-open http://localhost:8000
fi