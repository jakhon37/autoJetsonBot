#!/bin/bash
# 🎯 Clean Navigation Launch Script
# Uses dedicated navigation launch file with AMCL localization

echo "🎯 Starting Clean Navigation System..."
echo "This will launch:"
echo "  🌍 Gazebo with lab.world environment"
echo "  🤖 Robot with differential drive controllers"
echo "  🗺️ Map server with your existing lab_map.yaml"
echo "  📍 AMCL localization for precise robot positioning"
echo "  🧭 Complete Nav2 navigation stack"
echo "  📊 RViz with navigation visualization"
echo "  🌐 Web control interface"
echo ""

# Check if map file exists
MAP_FILE="lab_map.yaml"
if [ ! -f "$MAP_FILE" ]; then
    echo "⚠️  Map file $MAP_FILE not found!"
    echo "Please create a map first using:"
    echo "  ./launch_lab_sim.sh     # Start SLAM mapping"
    echo "  # Drive around to map environment"
    echo "  # Save map: ros2 run nav2_map_server map_saver_cli -f lab_map"
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
echo "🚀 Launching Clean Navigation System..."
echo "Please wait 35-40 seconds for all components to start..."

# Launch the clean navigation system
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    source install/setup.bash && 
    export DISPLAY=$DISPLAY_VAR && 
    export LIBGL_ALWAYS_INDIRECT=1 && 
    export GAZEBO_MODEL_PATH=/autonomous_ROS/install/my_robot_launch/share/my_robot_launch && 
    
    # Launch the clean navigation system
    ros2 launch my_robot_launch navigation_launch.py map_file:=$MAP_FILE > /tmp/clean_navigation.log 2>&1 &
    
    echo 'Clean navigation system launched!'
" &

# Wait and check services
echo "Waiting for services to start..."
for i in {1..40}; do
    sleep 1
    if curl -s http://localhost:8000 > /dev/null 2>&1; then
        echo ""
        echo "✅ SUCCESS! Clean Navigation System Ready!"
        break
    fi
    if [ $((i % 5)) -eq 0 ]; then
        echo -n " ${i}s"
    else
        echo -n "."
    fi
done

echo ""
echo "🎯 CLEAN NAVIGATION SYSTEM READY!"
echo ""
echo "🌍 Gazebo: Lab world with robot"
echo "🗺️ Map Server: Your lab_map.yaml loaded"
echo "📍 AMCL: Precise localization active"
echo "🧭 Nav2: Complete navigation stack running"
echo "📊 RViz: Navigation visualization ready"
echo "🌐 Web Control: http://localhost:8000"
echo "🔌 ROSBridge: ws://localhost:9090"
echo ""
echo "🎯 Ready for Autonomous Navigation!"
echo ""
echo "📋 How to Navigate:"
echo "  1. **RViz Method (Recommended):**"
echo "     - Click '2D Pose Estimate' → Set robot position"
echo "     - Click '2D Nav Goal' → Set destination"
echo "     - Watch autonomous navigation!"
echo ""
echo "  2. **Command Line:**"
echo "     ./navigate_robot.sh goal 2.0 1.5"
echo ""
echo "  3. **Web Interface:**"
echo "     http://localhost:8000 for manual control"
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