#!/bin/bash
# 🤖 Autonomous Jetson Robot - Simple Control Script
# Usage: ./robot.sh [command]

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
CONTAINER_NAME="auto_ros_foxy"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

show_help() {
    cat << EOF
🤖 Autonomous Jetson Robot Control

USAGE:
    ./robot.sh [COMMAND]

COMMANDS:
    sim         Start Gazebo simulation
    robot       Start real robot (no simulation)
    web         Open web interface in browser
    shell       Enter robot container
    build       Build the workspace
    stop        Stop all robot processes
    status      Show robot status
    logs        Show robot logs
    clean       Clean build files
    help        Show this help

EXAMPLES:
    ./robot.sh sim          # Start simulation + web interface
    ./robot.sh robot        # Start real robot
    ./robot.sh web          # Open web control
    ./robot.sh shell        # Debug in container

WEB INTERFACE:
    http://localhost:8000   # Robot control interface
    ws://localhost:9090     # ROSBridge WebSocket

KEYBOARD CONTROLS:
    WASD or Arrow Keys      # Move robot
    Spacebar               # Emergency stop

EOF
}

check_container() {
    # Check if container exists
    if ! docker ps -a | grep -q $CONTAINER_NAME; then
        log_error "Container $CONTAINER_NAME does not exist"
        log_info "Please create the container first using your existing setup"
        exit 1
    fi
    
    # Check if container is running
    if ! docker ps | grep -q $CONTAINER_NAME; then
        log_info "Container $CONTAINER_NAME is stopped. Starting..."
        docker start $CONTAINER_NAME || {
            log_error "Failed to start container"
            exit 1
        }
        log_success "Container started successfully"
        sleep 3  # Give container time to fully start
    fi
    
    # Verify container is actually running
    if ! docker ps | grep -q $CONTAINER_NAME; then
        log_error "Container failed to start properly"
        exit 1
    fi
}

build_workspace() {
    log_info "Building robot workspace..."
    check_container
    
    # Build with better error handling
    if docker exec $CONTAINER_NAME bash -c "
        cd /autonomous_ROS && 
        source /opt/ros/foxy/setup.bash && 
        colcon build --packages-select my_robot_launch web_gui_control
    "; then
        log_success "Workspace built successfully"
    else
        log_error "Build failed"
        log_info "Try: ./robot.sh shell  # to debug"
        exit 1
    fi
}

start_simulation() {
    log_info "🎮 Starting Gazebo simulation..."
    check_container
    
    # Setup X11 forwarding (same as runrosenv.sh)
    xhost +local:docker 2>/dev/null || echo "Note: xhost not available"
    
    # Stop any existing processes
    docker exec $CONTAINER_NAME bash -c "pkill -f 'ros2 launch' || true"
    sleep 2
    
    # Start simulation with proper GUI environment
    log_info "Launching Gazebo with GUI..."
    docker exec -d $CONTAINER_NAME bash -c "
        cd /autonomous_ROS && 
        source /opt/ros/foxy/setup.bash && 
        source install/setup.bash && 
        export DISPLAY=:0 &&
        export QT_X11_NO_MITSHM=1 &&
        nohup ros2 launch my_robot_launch robot_body_launch_sim.py > /tmp/sim.log 2>&1 &
        sleep 2
    "
    
    # Wait and check if it started
    log_info "Waiting for simulation to start..."
    for i in {1..30}; do
        sleep 1
        if curl -s http://localhost:8000 > /dev/null 2>&1; then
            break
        fi
        if [ $((i % 5)) -eq 0 ]; then
            echo -n " ${i}s"
        else
            echo -n "."
        fi
    done
    echo ""
    
    # Check status
    if curl -s http://localhost:8000 > /dev/null 2>&1; then
        log_success "🎉 Simulation started successfully!"
        log_info "🌐 Web interface: http://localhost:8000"
        log_info "🔌 ROSBridge: ws://localhost:9090"
        log_info "🎮 Use WASD keys or web interface to control robot"
        log_info ""
        log_info "💡 Tip: Run './robot.sh web' to open browser automatically"
    else
        log_error "Simulation failed to start properly"
        log_info "Check logs with: ./robot.sh logs"
    fi
}

start_robot() {
    log_info "🤖 Starting real robot..."
    check_container
    
    # Stop any existing processes
    docker exec $CONTAINER_NAME bash -c "pkill -f 'ros2 launch' || true"
    sleep 2
    
    # Start real robot
    log_info "Launching robot nodes..."
    docker exec -d $CONTAINER_NAME bash -c "
        cd /autonomous_ROS && 
        source /opt/ros/foxy/setup.bash && 
        source install/setup.bash && 
        nohup ros2 launch my_robot_launch autonomous_car_launch_1.py > /tmp/robot.log 2>&1 &
        sleep 2
    "
    
    # Wait and check if it started
    log_info "Waiting for robot to start..."
    for i in {1..20}; do
        sleep 1
        if curl -s http://localhost:8000 > /dev/null 2>&1; then
            break
        fi
        if [ $((i % 5)) -eq 0 ]; then
            echo -n " ${i}s"
        else
            echo -n "."
        fi
    done
    echo ""
    
    # Check status
    if curl -s http://localhost:8000 > /dev/null 2>&1; then
        log_success "🎉 Robot started successfully!"
        log_info "🌐 Web interface: http://localhost:8000"
        log_info "🔌 ROSBridge: ws://localhost:9090"
        log_info "💡 Tip: Run './robot.sh web' to open browser automatically"
    else
        log_error "Robot failed to start properly"
        log_info "Check logs with: ./robot.sh logs"
    fi
}

open_web() {
    log_info "🌐 Opening web interface..."
    
    # Check if web server is running
    if curl -s http://localhost:8000 > /dev/null 2>&1; then
        log_success "Web interface is running"
        
        # Try to open browser (macOS/Linux)
        if command -v open > /dev/null 2>&1; then
            open http://localhost:8000
        elif command -v xdg-open > /dev/null 2>&1; then
            xdg-open http://localhost:8000
        else
            log_info "Please open: http://localhost:8000"
        fi
    else
        log_error "Web interface not running. Start robot first:"
        log_info "./robot.sh sim    # or"
        log_info "./robot.sh robot"
    fi
}

enter_shell() {
    log_info "🐚 Entering robot container..."
    check_container
    docker exec -it $CONTAINER_NAME bash -c "
        cd /autonomous_ROS && 
        source /opt/ros/foxy/setup.bash && 
        source install/setup.bash 2>/dev/null || true &&
        echo '🤖 Welcome to Robot Container!' &&
        echo 'Available commands:' &&
        echo '  ros2 topic list' &&
        echo '  ros2 node list' &&
        echo '  ros2 launch my_robot_launch robot_body_launch_sim.py' &&
        echo '' &&
        bash
    "
}

stop_robot() {
    log_info "🛑 Stopping robot processes..."
    check_container
    
    docker exec $CONTAINER_NAME bash -c "
        pkill -f 'ros2 launch' || true
        pkill -f 'python3 -m http.server' || true
        pkill -f 'rosbridge_websocket' || true
        pkill -f 'gazebo' || true
        pkill -f 'gzserver' || true
        pkill -f 'gzclient' || true
    "
    
    log_success "Robot stopped"
}

show_status() {
    log_info "📊 Robot status:"
    
    echo "Container Status:"
    docker ps -f name=$CONTAINER_NAME --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
    
    echo ""
    echo "Services:"
    
    # Check web server
    if curl -s http://localhost:8000 > /dev/null 2>&1; then
        echo "✅ Web Interface: http://localhost:8000"
    else
        echo "❌ Web Interface: Not running"
    fi
    
    # Check ROSBridge
    if netstat -ln 2>/dev/null | grep -q :9090; then
        echo "✅ ROSBridge: ws://localhost:9090"
    else
        echo "❌ ROSBridge: Not running"
    fi
    
    # Check ROS nodes (if container is running)
    if docker ps | grep -q $CONTAINER_NAME; then
        echo ""
        echo "Active ROS Nodes:"
        docker exec $CONTAINER_NAME bash -c "
            source /opt/ros/foxy/setup.bash 2>/dev/null && 
            source install/setup.bash 2>/dev/null && 
            ros2 node list 2>/dev/null || echo 'No ROS nodes running'
        " 2>/dev/null
    fi
}

show_logs() {
    log_info "📝 Robot logs:"
    check_container
    docker logs --tail 50 $CONTAINER_NAME
}

clean_build() {
    log_info "🧹 Cleaning build files..."
    check_container
    
    docker exec $CONTAINER_NAME bash -c "
        cd /autonomous_ROS && 
        rm -rf build install log || true
    "
    
    log_success "Build files cleaned"
}

# Main command handling
case "${1:-help}" in
    sim|simulation)
        build_workspace
        start_simulation
        ;;
    robot|real)
        build_workspace
        start_robot
        ;;
    web|gui)
        open_web
        ;;
    shell|bash)
        enter_shell
        ;;
    build)
        build_workspace
        ;;
    stop)
        stop_robot
        ;;
    status)
        show_status
        ;;
    logs)
        show_logs
        ;;
    clean)
        clean_build
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        log_error "Unknown command: $1"
        echo ""
        show_help
        exit 1
        ;;
esac