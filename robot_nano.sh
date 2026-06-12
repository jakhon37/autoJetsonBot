#!/bin/bash
# robot_nano.sh - Native control script for autoJetsonBot (No Docker)
# Ported from robot.sh for physical Jetson Nano hardware.

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VNC_PORT=5900
VNC_RESOLUTION="1280x720x24"
XVFB_DISPLAY=":99"
XVFB_AUTH="/tmp/xvfb99.auth"

log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }

# ── Environment ──────────────────────────────────────────────────────────────

source_env() {
    if [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
    else
        log_error "ROS 2 Foxy not found at /opt/ros/foxy/. Please run setup_jetson.sh first."
        exit 1
    fi

    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
    else
        log_warn "Workspace setup.bash not found. Did you run 'colcon build'?"
    fi
}

# ── Launch Logic ──────────────────────────────────────────────────────────────

_launch() {
    local EXTRA_ARGS="$*"
    log_info "Launching Native Robot System with: ${EXTRA_ARGS:-"YAML Defaults"}"
    
    source_env

    # 1. Kill any existing processes
    stop_robot || true
    
    # 2. Virtual Display & VNC (Optional for headless)
    start_virtual_display
    start_vnc_server
    start_openbox

    # 3. Launch ROS Stack
    export DISPLAY="${XVFB_DISPLAY}"
    export XAUTHORITY="${XVFB_AUTH}"
    export QT_X11_NO_MITSHM=1
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=softpipe

    # Run in background
    nohup ros2 launch jetson_bot_bringup main.launch.py ${EXTRA_ARGS} > /tmp/robot_nano.log 2>&1 &
    
    log_success "System is booting. Access Web UI at http://localhost:8000"
    log_info "Tailing logs... (Ctrl+C to stop tailing, system will keep running)"
    tail -f /tmp/robot_nano.log
}

# ── Infrastructure Helpers ────────────────────────────────────────────────────

start_virtual_display() {
    pgrep -f "Xvfb ${XVFB_DISPLAY}" > /dev/null && return
    log_info "Starting virtual framebuffer (${XVFB_DISPLAY})..."
    
    rm -f /tmp/.X${XVFB_DISPLAY#:}-lock
    touch ${XVFB_AUTH}
    # Simple xauth cookie generation
    COOKIE=$(head -c 16 /dev/urandom | od -An -tx1 | tr -d ' \n')
    xauth -f ${XVFB_AUTH} add ${XVFB_DISPLAY} . $COOKIE > /dev/null 2>&1 || true

    nohup Xvfb ${XVFB_DISPLAY} -screen 0 ${VNC_RESOLUTION} -auth ${XVFB_AUTH} > /tmp/xvfb_nano.log 2>&1 &
    sleep 1
}

start_vnc_server() {
    pgrep x11vnc > /dev/null && return
    log_info "Starting VNC server on port ${VNC_PORT}..."
    nohup x11vnc -display ${XVFB_DISPLAY} -auth ${XVFB_AUTH} -nopw -forever -shared -listen 0.0.0.0 -rfbport ${VNC_PORT} > /tmp/x11vnc_nano.log 2>&1 &
}

start_openbox() {
    pgrep openbox > /dev/null && return
    log_info "Starting Openbox window manager..."
    DISPLAY="${XVFB_DISPLAY}" XAUTHORITY="${XVFB_AUTH}" nohup openbox --sm-disable > /dev/null 2>&1 &
}

# ── Utility Commands ──────────────────────────────────────────────────────────

stop_robot() {
    log_info "Stopping all robot processes (Aggressive Sudo Cleanup)..."
    
    # 1. Kill ROS Launch and nodes (Using sudo to clear zombie processes)
    sudo pkill -9 -f "ros2 launch"      || true
    sudo pkill -9 -f "ros2 run"         || true
    sudo pkill -9 -f "node"             || true
    sudo pkill -9 -f "python3"          || true
    
    # 2. Kill infrastructure
    sudo pkill -9 -f gzserver           || true
    sudo pkill -9 -f gzclient           || true
    sudo pkill -9 -f rviz2              || true
    sudo pkill -9 -f rosbridge          || true
    sudo pkill -9 -f web_server         || true
    sudo pkill -9 -f x11vnc             || true
    sudo pkill -9 -f Xvfb               || true
    
    log_success "All processes terminated."
}

build_workspace() {
    log_info "Building workspace..."
    source /opt/ros/foxy/setup.bash || true
    cd "$WORKSPACE_DIR"
    colcon build --symlink-install --continue-on-error
}

show_status() {
    log_info "Native Status:"
    
    # Check Web UI
    if curl -s --connect-timeout 1 http://localhost:8000 >/dev/null 2>&1; then
        echo -e "Web UI:    ${GREEN}OK${NC}"
    else
        echo -e "Web UI:    ${RED}Offline${NC}"
    fi

    # Check ROS Launch
    if pgrep -f "main.launch.py" > /dev/null; then
        echo -e "ROS:       ${GREEN}Running${NC}"
        source_env
        NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
        echo -e "Nodes:     ${GREEN}${NODE_COUNT}${NC}"
    else
        echo -e "ROS:       ${RED}Stopped${NC}"
    fi
}

# ── Main Dispatcher ───────────────────────────────────────────────────────────

case "${1:-help}" in
    up)              shift; _launch "$@" ;;
    robot)           shift; _launch "sim:=false viz:=false $@" ;;
    nav)             shift; _launch "mode:=navigation sim:=false viz:=false $@" ;;
    stop)            stop_robot ;;
    build)           build_workspace ;;
    status)          show_status ;;
    logs)            tail -f /tmp/robot_nano.log ;;
    *)
        echo "Usage: ./robot_nano.sh {up|robot|nav|stop|build|status|logs}"
        ;;
esac
