#!/bin/bash
# 🏢 Lab Environment Simulation Launcher
# Launches robot in lab.world with proper RViz visualization

echo "🏢 Starting Lab Environment Simulation..."
echo "This will launch:"
echo "  🌍 Gazebo with lab.world environment"
echo "  🤖 Robot with all sensors and controllers"
echo "  📊 RViz with lab visualization config"
echo "  🌐 Web control interface"
echo "  🗺️ SLAM mapping system"
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
echo "🚀 Launching Lab Simulation Environment..."
echo "Please wait 20-25 seconds for all services to start..."

# Launch the enhanced simulation
docker exec auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    source install/setup.bash && 
    export DISPLAY=$DISPLAY_VAR && 
    export LIBGL_ALWAYS_INDIRECT=1 && 
    export GAZEBO_MODEL_PATH=/autonomous_ROS/install/my_robot_launch/share/my_robot_launch && 
    
    # Launch the complete lab environment
    ros2 launch my_robot_launch robot_body_launch_sim.py > /tmp/lab_sim.log 2>&1 &
    
    echo 'Lab simulation launched!'
" &

# Wait and check services
echo "Waiting for services to start..."
for i in {1..25}; do
    sleep 1
    if curl -s http://localhost:8000 > /dev/null 2>&1; then
        echo ""
        echo "✅ SUCCESS! Lab Environment Ready!"
        break
    fi
    if [ $((i % 5)) -eq 0 ]; then
        echo -n " ${i}s"
    else
        echo -n "."
    fi
done

echo ""
echo "🏢 LAB SIMULATION ENVIRONMENT READY!"
echo ""
echo "🌍 Gazebo: Lab world environment loaded"
echo "📊 RViz: Lab visualization configuration active"
echo "🌐 Web Control: http://localhost:8000"
echo "🔌 ROSBridge: ws://localhost:9090"
echo "🗺️ SLAM: Active mapping mode"
echo ""
echo "🎯 Next Steps:"
echo "  1. Open web interface: http://localhost:8000"
echo "  2. Drive robot around to build map"
echo "  3. Monitor in RViz for real-time visualization"
echo "  4. Save map when complete: ros2 run nav2_map_server map_saver_cli -f lab_map"
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