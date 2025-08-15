#!/bin/bash

# 🗺️ SLAM Mode Control Script
# Easy switching between mapping and localization modes

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    echo "🗺️ SLAM Mode Control Script"
    echo ""
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  mapping [sim|robot]     - Start robot in mapping mode (creates new maps)"
    echo "  localization [sim|robot] - Start robot in localization mode (uses existing maps)"
    echo "  status                  - Show current robot and SLAM status"
    echo "  save-map [name]         - Save current map with optional name"
    echo "  list-maps               - List available saved maps"
    echo ""
    echo "Examples:"
    echo "  $0 mapping sim          - Start simulation in mapping mode"
    echo "  $0 localization robot   - Start real robot using existing maps"
    echo "  $0 save-map office      - Save current map as 'office'"
    echo ""
    echo "Default mode: mapping"
    echo "Default environment: sim"
}

# Function to check if maps exist
check_maps() {
    if [ -f "maps/lab_map_serial.data" ] && [ -f "maps/lab_map_serial.posegraph" ]; then
        return 0
    else
        return 1
    fi
}

# Function to start robot with SLAM mode
start_robot() {
    local slam_mode=$1
    local env_mode=$2
    
    print_info "🚀 Starting robot in $slam_mode mode ($env_mode environment)..."
    
    # Check if localization mode has required maps
    if [ "$slam_mode" = "localization" ]; then
        if ! check_maps; then
            print_error "No maps found for localization mode!"
            print_info "Available options:"
            echo "  1. Run in mapping mode first: $0 mapping $env_mode"
            echo "  2. Copy existing maps to ./maps/ directory"
            exit 1
        fi
        print_success "Maps found for localization mode"
    fi
    
    # Set use_sim flag
    local use_sim="false"
    if [ "$env_mode" = "sim" ]; then
        use_sim="true"
    fi
    
    # Stop any existing robot processes
    print_info "Stopping existing robot processes..."
    ./robot.sh stop > /dev/null 2>&1 || true
    
    # Start robot with specified SLAM mode
    print_info "Launching robot with slam_mode:=$slam_mode use_sim:=$use_sim"
    
    # Build workspace first
    ./robot.sh build
    
    # Start the robot
    docker exec -d auto_ros_foxy bash -c "
        source /opt/ros/foxy/setup.bash && 
        source /ros2/install/setup.bash && 
        ros2 launch my_robot_launch autonomous_car_launch_1.py use_sim:=$use_sim slam_mode:=$slam_mode
    " || {
        print_error "Failed to start robot"
        exit 1
    }
    
    print_success "Robot started successfully!"
    print_info "🌐 Web interface: http://localhost:8000"
    print_info "🔗 ROSBridge: ws://localhost:9090"
    
    if [ "$slam_mode" = "mapping" ]; then
        print_info "📍 Robot is creating new maps as it moves"
        print_info "💾 Use '$0 save-map [name]' to save maps when done"
    else
        print_info "📍 Robot is using existing maps for localization"
    fi
}

# Function to save current map
save_map() {
    local map_name=${1:-"map_$(date +%s)"}
    
    print_info "💾 Saving current map as '$map_name'..."
    
    # Create maps directory if it doesn't exist
    mkdir -p maps
    
    # Save map using SLAM toolbox service
    docker exec auto_ros_foxy bash -c "
        source /opt/ros/foxy/setup.bash && 
        ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '/ros2/maps/$map_name'}}\"
    " || {
        print_error "Failed to save map. Is SLAM running?"
        exit 1
    }
    
    print_success "Map saved as '$map_name'"
    print_info "Files created:"
    echo "  - maps/${map_name}.data"
    echo "  - maps/${map_name}.posegraph"
}

# Function to list available maps
list_maps() {
    print_info "📋 Available maps:"
    
    if [ ! -d "maps" ] || [ -z "$(ls -A maps 2>/dev/null)" ]; then
        print_warning "No maps found in ./maps/ directory"
        return
    fi
    
    echo ""
    for file in maps/*.data; do
        if [ -f "$file" ]; then
            local basename=$(basename "$file" .data)
            local size=$(du -h "$file" | cut -f1)
            local date=$(stat -c %y "$file" | cut -d' ' -f1)
            echo "  📍 $basename (${size}, $date)"
        fi
    done
    echo ""
}

# Function to show status
show_status() {
    print_info "📊 Robot and SLAM Status:"
    echo ""
    
    # Show robot status
    ./robot.sh status
    
    echo ""
    print_info "🗺️ SLAM Status:"
    
    # Check if SLAM node is running
    if docker exec auto_ros_foxy bash -c "source /opt/ros/foxy/setup.bash && ros2 node list" 2>/dev/null | grep -q slam_toolbox; then
        print_success "SLAM toolbox is running"
        
        # Get SLAM mode
        local slam_mode=$(docker exec auto_ros_foxy bash -c "source /opt/ros/foxy/setup.bash && ros2 param get /slam_toolbox mode" 2>/dev/null | grep -o 'mapping\|localization' || echo "unknown")
        echo "  Mode: $slam_mode"
        
        # Check topics
        echo "  Topics:"
        docker exec auto_ros_foxy bash -c "source /opt/ros/foxy/setup.bash && ros2 topic list" 2>/dev/null | grep -E "(map|scan)" | sed 's/^/    /' || echo "    No SLAM topics found"
    else
        print_warning "SLAM toolbox is not running"
    fi
    
    echo ""
    list_maps
}

# Main script logic
case "${1:-help}" in
    "mapping")
        start_robot "mapping" "${2:-sim}"
        ;;
    "localization")
        start_robot "localization" "${2:-sim}"
        ;;
    "save-map")
        save_map "$2"
        ;;
    "list-maps")
        list_maps
        ;;
    "status")
        show_status
        ;;
    "help"|"-h"|"--help")
        show_usage
        ;;
    *)
        print_error "Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac


# ros2 launch my_robot_launch autonomous_car_launch_1.py use_sim:=mapping slam_mode:=sim