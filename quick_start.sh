#!/bin/bash
# Quick and reliable robot startup script

echo "🤖 Starting Autonomous Robot..."

# Start container if needed
if ! docker ps | grep -q auto_ros_foxy; then
    echo "Starting container..."
    docker start auto_ros_foxy
    sleep 3
fi

# Launch robot with proper waiting
echo "Launching robot system..."
docker exec -d auto_ros_foxy bash -c "
    cd /autonomous_ROS && 
    source /opt/ros/foxy/setup.bash && 
    source install/setup.bash && 
    ros2 launch my_robot_launch autonomous_car_launch_1.py > /tmp/robot.log 2>&1
" &

# Wait for services to start
echo "Waiting for services to start..."
for i in {1..30}; do
    sleep 1
    if curl -s http://localhost:8000 > /dev/null 2>&1; then
        echo "✅ Web interface ready!"
        break
    fi
    echo -n "."
done

echo ""
echo "🎉 Robot system ready!"
echo "🌐 Web interface: http://localhost:8000"
echo "🔌 ROSBridge: ws://localhost:9090"
echo ""
echo "Open your browser to http://localhost:8000 to control the robot!"