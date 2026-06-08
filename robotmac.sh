#!/bin/bash
# autoJetsonBot - macOS Hardware Bridge
# This script enables physical ESP32 connection from Docker on Mac using socat.

# Configuration - AUTO DETECT PORTS
MOTOR_BRIDGE_PORT=2000
LIDAR_BRIDGE_PORT=2001
CONTAINER_NAME="auto_ros_foxy"

# Colors
BLUE='\033[0;34m'
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info()    { echo -e "${BLUE}[MAC-INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[MAC-SUCCESS]${NC} $1"; }
log_error()   { echo -e "${RED}[MAC-ERROR]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[MAC-WARN]${NC} $1"; }

setup_socat_mac() {
    # 1. Aggressive Host-Side Cleanup
    log_info "Cleaning up existing bridges..."
    pkill -9 -f "while true; do MAC_MOTOR_PORT" || true
    pkill -9 -f "while true; do MAC_LIDAR_PORT" || true
    pkill -9 -f "socat TCP-LISTEN:${MOTOR_BRIDGE_PORT}" || true
    pkill -9 -f "socat TCP-LISTEN:${LIDAR_BRIDGE_PORT}" || true
    sleep 1

    log_info "Starting host-side bridges (Auto-Restart Monitor enabled)..."
    
    # 1. Motor Bridge Monitor
    (while true; do
        MAC_MOTOR_PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1)
        if [ -n "$MAC_MOTOR_PORT" ] && [ -e "$MAC_MOTOR_PORT" ]; then
            stty -f "$MAC_MOTOR_PORT" 115200 raw -echo -hupcl 2>/dev/null
            socat TCP-LISTEN:${MOTOR_BRIDGE_PORT},reuseaddr,fork,range=127.0.0.1/32 FILE:${MAC_MOTOR_PORT},nonblock,raw,echo=0,ispeed=115200,ospeed=115200
        fi
        sleep 2
    done) &
    log_success "Motor bridge monitor started (Port ${MOTOR_BRIDGE_PORT})"

    # 2. Lidar Bridge Monitor
    (while true; do
        MAC_LIDAR_PORT=$(ls /dev/cu.usbserial* 2>/dev/null | head -n 1)
        if [ -n "$MAC_LIDAR_PORT" ] && [ -e "$MAC_LIDAR_PORT" ]; then
            stty -f "$MAC_LIDAR_PORT" 115200 raw -echo -hupcl 2>/dev/null
            socat TCP-LISTEN:${LIDAR_BRIDGE_PORT},reuseaddr,fork,range=127.0.0.1/32 FILE:${MAC_LIDAR_PORT},nonblock,raw,echo=0,ispeed=115200,ospeed=115200
        fi
        sleep 2
    done) &
    log_success "Lidar bridge monitor started (Port ${LIDAR_BRIDGE_PORT})"
}

setup_socat_docker() {
    log_info "Configuring container-side bridges..."
    
    # Install socat inside container if missing
    docker exec "$CONTAINER_NAME" bash -c "command -v socat >/dev/null || (apt-get update && apt-get install -y socat)"
    
    # 1. Map Motor -> /dev/ttyACM0
    docker exec "$CONTAINER_NAME" pkill -9 -f "socat PTY,link=/dev/ttyACM0" || true
    docker exec -d "$CONTAINER_NAME" bash -c "socat PTY,link=/dev/ttyACM0,raw,echo=0 TCP:host.docker.internal:${MOTOR_BRIDGE_PORT}"
    
    # 2. Map Lidar -> /dev/ttyUSB0
    docker exec "$CONTAINER_NAME" pkill -9 -f "socat PTY,link=/dev/ttyUSB0" || true
    docker exec -d "$CONTAINER_NAME" bash -c "socat PTY,link=/dev/ttyUSB0,raw,echo=0 TCP:host.docker.internal:${LIDAR_BRIDGE_PORT}"
    
    sleep 1
    docker exec "$CONTAINER_NAME" chmod 666 /dev/ttyACM0 /dev/ttyUSB0 2>/dev/null || true
    log_success "Container-side virtual devices are ready"
}

start_robot() {
    log_info "Synchronizing Unified Config to Hardware Mode..."
    # Set use_sim and use_sim_time to false in the YAML
    sed -i '' 's/use_sim: true/use_sim: false/g' src/jetson_bot_bringup/config/unified_robot_config.yaml
    sed -i '' 's/use_sim_time: true/use_sim_time: false/g' src/jetson_bot_bringup/config/unified_robot_config.yaml
    
    setup_socat_mac
    setup_socat_docker
    log_info "Launching Robot Hardware Mode via robot.sh..."
    ./robot.sh robot
}

clean_bridges() {
    log_info "Aggressively cleaning all bridge processes..."
    pkill -9 -f "while true; do MAC_MOTOR_PORT" || true
    pkill -9 -f "while true; do MAC_LIDAR_PORT" || true
    pkill -9 -f "socat TCP-LISTEN:${MOTOR_BRIDGE_PORT}" || true
    pkill -9 -f "socat TCP-LISTEN:${LIDAR_BRIDGE_PORT}" || true
    pkill -9 -f "robotmac.sh" || true
    
    # Clean container side too
    if docker ps | grep -q "$CONTAINER_NAME"; then
        docker exec "$CONTAINER_NAME" pkill -9 -f "socat PTY" || true
    fi
    log_success "Cleanup complete."
}

stop_all() {
    log_info "Stopping everything (Mac & Docker)..."
    ./robot.sh stop
    clean_bridges
}

# Main
case "$1" in
    up|start|robot)
        start_robot
        ;;
    stop)
        stop_all
        ;;
    clean)
        clean_bridges
        ;;
    *)
        echo "Usage: ./robotmac.sh [robot|stop|clean]"
        echo "  robot : Start macOS serial bridge and launch hardware mode"
        echo "  stop  : Stop all processes and clean up bridges"
        echo "  clean : Aggressively kill zombie bridge processes"
        ;;
esac
