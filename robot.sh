#!/bin/bash
# Autonomous Jetson Robot - Control Script
# Usage: ./robot.sh [command]

set -e
trap 'echo -e "\033[0;31m[ERROR]\033[0m Script exited unexpectedly at line $LINENO" >&2' ERR

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
CONTAINER_NAME="auto_ros_foxy"
VNC_PORT=5900
VNC_RESOLUTION=${VNC_RESOLUTION:-"1920x1080x24"}
XVFB_DISPLAY=":99"
XVFB_AUTH="/tmp/xvfb99.auth"

log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }

show_help() {
    cat << 'EOH'
Autonomous Jetson Robot Control

USAGE:
    ./robot.sh [COMMAND] [ARGS...]

CORE COMMANDS:
    up          Start system using config file defaults (YAML)
    sim         Force Simulation Mode (sim:=true viz:=true)
    robot       Force Hardware Mode (sim:=false viz:=false)
    nav         Force Navigation Mode (auto-saves map if was mapping)
    map2nav     Save current map (if mapping) + switch to Nav2 (recommended for hardware)
    stop        Stop all robot processes and clean up ports

UTILITY COMMANDS:
    build       Build the modular workspace (jetson_bot_*)
    auto        Install missing dependencies and build
    status      Show container and node health
    watch       Auto-refresh status every 10s
    logs        Tail the live ROS launch log (/tmp/sim.log)
    shell       Enter container (or run command: ./robot.sh shell "ls")
    verify      Run EKF/fusion + basic health checks (inside Docker)
    clean       Delete build, install, and log folders
    help        Show this help

EXAMPLES:
    ./robot.sh sim
    ./robot.sh sim mode:=navigation
    ./robot.sh up
    ./robot.sh nav
    ./robot.sh map2nav          # save map + switch to nav (real hardware friendly)
    ./robot.sh sim x:=0.5 y:=0 z:=0.06 yaw:=0.0   # spawn pose (passed to launch)
EOH
}

# ── Container lifecycle ───────────────────────────────────────────────────────

check_container() {
    if ! docker ps -a | grep -q "$CONTAINER_NAME"; then
        log_warn "Container $CONTAINER_NAME does not exist. Attempting basic creation..."
        docker run -d \
            --name "$CONTAINER_NAME" \
            --network host \
            -v "$(pwd)":/autonomous_ROS \
            -p 5900:5900 -p 8000:8000 -p 9090:9090 \
            --privileged \
            auto_ros:foxy \
            bash -c "while true; do sleep 3600; done" || {
                log_error "Create failed. Run: docker build -f Dockerfile.foxy -t auto_ros:foxy ."
                exit 1
            }
        sleep 3
    fi

    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        log_info "Container $CONTAINER_NAME is stopped — starting..."
        docker start "$CONTAINER_NAME"
        sleep 2
    fi
}

# ── Build ─────────────────────────────────────────────────────────────────────

build_workspace() {
    log_info "Building modular workspace..."
    check_container
    docker exec "$CONTAINER_NAME" bash -c "
        cd /autonomous_ROS &&
        source /opt/ros/foxy/setup.bash &&
        colcon build --symlink-install --continue-on-error
    "
}

# ── Dependencies ──────────────────────────────────────────────────────────────

install_deps() {
    log_info "Checking for missing dependencies..."
    check_container
    
    # 1. web_video_server
    if ! docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/foxy/setup.bash && ros2 pkg list 2>/dev/null" | grep -q web_video_server; then
        log_warn "web_video_server not found. Installing..."
        docker exec "$CONTAINER_NAME" apt-get update
        docker exec "$CONTAINER_NAME" apt-get install -y ros-foxy-web-video-server
        log_success "web_video_server installed."
    else
        log_success "web_video_server is already installed."
    fi
    
    log_success "All dependencies verified."
}

# ── Launch Logic ──────────────────────────────────────────────────────────────

_launch() {
    local EXTRA_ARGS="$*"
    log_info "Launching Robot System with: ${EXTRA_ARGS:-"YAML Defaults"}"
    check_container
    
    # Infrastructure
    start_virtual_display
    start_vnc_server
    
    # X11 Auth Fix
    docker exec "$CONTAINER_NAME" bash -c "DISPLAY=${XVFB_DISPLAY} XAUTHORITY=${XVFB_AUTH} xhost +local: >/dev/null 2>&1 || true"
    
    start_openbox

    # Launch ROS Stack
    docker exec -d \
        -e DISPLAY="${XVFB_DISPLAY}" \
        -e XAUTHORITY="${XVFB_AUTH}" \
        -e QT_X11_NO_MITSHM=1 \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e GALLIUM_DRIVER=softpipe \
        "$CONTAINER_NAME" bash -c "
            cd /autonomous_ROS &&
            source /opt/ros/foxy/setup.bash &&
            source install/setup.bash 2>/dev/null || true &&
            nohup ros2 launch jetson_bot_bringup main.launch.py ${EXTRA_ARGS} > /tmp/sim.log 2>&1
        "

    log_success "System is booting. Access Web UI at http://localhost:8000 or VNC at localhost:5900"
}

# ── Infrastructure Helpers ────────────────────────────────────────────────────

start_virtual_display() {
    docker exec "$CONTAINER_NAME" bash -c "pgrep -f 'Xvfb ${XVFB_DISPLAY}'" &>/dev/null && return
    log_info "Starting virtual framebuffer (${XVFB_DISPLAY})..."
    
    docker exec "$CONTAINER_NAME" bash -c "
        rm -f /tmp/.X${XVFB_DISPLAY#:}-lock
        touch ${XVFB_AUTH}
        if command -v xauth >/dev/null; then
            COOKIE=\$(mcookie 2>/dev/null || head -c 16 /dev/urandom | od -An -tx1 | tr -d ' \n' 2>/dev/null || echo '00000000000000000000000000000000')
            xauth -f ${XVFB_AUTH} add ${XVFB_DISPLAY} . \$COOKIE 2>/dev/null || true
        fi
    " || true

    docker exec -d "$CONTAINER_NAME" bash -c "Xvfb ${XVFB_DISPLAY} -screen 0 ${VNC_RESOLUTION} -auth ${XVFB_AUTH} > /tmp/xvfb.log 2>&1"
    sleep 1
}

start_vnc_server() {
    docker exec "$CONTAINER_NAME" bash -c "pgrep x11vnc" &>/dev/null && return
    log_info "Starting VNC server on port ${VNC_PORT}..."
    docker exec -d "$CONTAINER_NAME" bash -c "x11vnc -display ${XVFB_DISPLAY} -auth ${XVFB_AUTH} -nopw -forever -shared -listen 0.0.0.0 -rfbport ${VNC_PORT} > /tmp/x11vnc.log 2>&1"
}

start_openbox() {
    docker exec "$CONTAINER_NAME" bash -c "pgrep openbox" &>/dev/null && return
    log_info "Starting Openbox window manager..."
    docker exec -d \
        -e DISPLAY="${XVFB_DISPLAY}" \
        -e XAUTHORITY="${XVFB_AUTH}" \
        "$CONTAINER_NAME" openbox --sm-disable
}

# ── Utility Commands ──────────────────────────────────────────────────────────

stop_robot() {
    log_info "Stopping all processes (Aggressive Cleanup)..."
    
    if ! docker ps --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
        log_warn "Container $CONTAINER_NAME is not running."
        return 0
    fi

    attempt_save_map || true

    # Kill everything inside container
    docker exec "$CONTAINER_NAME" bash -c '
        # 1. Kill ROS Launch and nodes
        pkill -9 -f "ros2 launch"      || true
        pkill -9 -f "ros2 run"         || true
        pkill -9 -f "node"             || true
        pkill -9 -f "python3"          || true
        
        # 2. Kill core physics/viz
        pkill -9 -f gzserver           || true
        pkill -9 -f gzclient           || true
        pkill -9 -f gazebo             || true
        pkill -9 -f rviz2              || true
        
        # 3. Kill bridges and servers
        pkill -9 -f rosbridge          || true
        pkill -9 -f web_server         || true
        pkill -9 -f socat              || true
    ' || true
    
    _cleanup_ports
    log_success "All processes terminated."
    sleep 2
}

# Attempt to save map via slam_toolbox if we are in mapping mode and slam is running.
# This provides lightweight auto-save before mode switches / stops.
attempt_save_map() {
    if ! docker ps --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
        return 0
    fi

    # Detect current mode from the exported config (written by launch)
    local cur_mode
    cur_mode=$(docker exec "$CONTAINER_NAME" python3 -c "
import json,os
p='/autonomous_ROS/src/jetson_bot_gui/web/config.json'
print(json.load(open(p)).get('mode','') if os.path.exists(p) else '')
" 2>/dev/null || echo "")

    if [[ "$cur_mode" != "mapping" ]]; then
        return 0
    fi

    # Quick check if slam_toolbox is present
    if ! docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/foxy/setup.bash && ros2 node list 2>/dev/null | grep -q slam_toolbox" 2>/dev/null; then
        return 0
    fi

    local map_name map_dir full_path
    map_name=$(docker exec "$CONTAINER_NAME" python3 -c "
import json,os
p='/autonomous_ROS/src/jetson_bot_gui/web/config.json'
print(json.load(open(p)).get('map_name','lab_map_v2') if os.path.exists(p) else 'lab_map_v2')
" 2>/dev/null || echo "lab_map_v2")

    map_dir=$(docker exec "$CONTAINER_NAME" python3 -c "
import json,os
p='/autonomous_ROS/src/jetson_bot_gui/web/config.json'
print(json.load(open(p)).get('map_dir','/autonomous_ROS/maps') if os.path.exists(p) else '/autonomous_ROS/maps')
" 2>/dev/null || echo "/autonomous_ROS/maps")

    full_path="${map_dir}/${map_name}"

    log_info "Auto-saving current map before switch/stop: ${full_path}"

    docker exec "$CONTAINER_NAME" bash -c "
        source /opt/ros/foxy/setup.bash 2>/dev/null || true
        source /autonomous_ROS/install/setup.bash 2>/dev/null || true
        ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '${full_path}'}}\" 2>/dev/null || \
        ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: '${full_path}'}\" 2>/dev/null || true
    " || true

    sleep 1
}

_cleanup_ports() {
    if ! docker ps --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
        return 0
    fi

    docker exec "$CONTAINER_NAME" bash -c '
        for port in 8000 9090 5900 8080; do
            pids=$(netstat -tulpn 2>/dev/null | grep ":$port " | awk "{print \$7}" | cut -d/ -f1 | grep -E "^[0-9]+$" || true)
            for pid in $pids; do
                if [ -n "$pid" ]; then
                    kill -9 "$pid" 2>/dev/null || true
                fi
            done
        done
    ' || true
}

enter_shell() {
    check_container
    if [ $# -gt 0 ]; then
        docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash 2>/dev/null || true && $*"
    else
        log_info "Entering container..."
        docker exec -it "$CONTAINER_NAME" bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash 2>/dev/null || true && exec bash"
    fi
}

show_status() {
    log_info "Status:"
    docker ps -f "name=$CONTAINER_NAME" --format "table {{.Names}}\t{{.Status}}"

    # Mac compatibility: use localhost as primary for display
    LOCAL_IP="localhost"
    
    if curl -s --connect-timeout 1 http://localhost:8000 >/dev/null 2>&1; then
        echo -e "Web UI:    ${GREEN}OK${NC}  http://${LOCAL_IP}:8000"
    else
        echo -e "Web UI:    ${RED}Offline${NC}"
    fi

    if docker exec "$CONTAINER_NAME" pgrep x11vnc >/dev/null 2>&1; then
        echo -e "VNC:       ${GREEN}OK${NC}  ${LOCAL_IP}:5900"
    else
        echo -e "VNC:       ${RED}Offline${NC}"
    fi

    echo "--- ROS Health ---"
    if ! docker exec "$CONTAINER_NAME" pgrep -f "main.launch.py" >/dev/null 2>&1; then
        echo -e "Launch:    ${RED}Crashed/Stop${NC}"
        return 0
    fi
    echo -e "Launch:    ${GREEN}Running${NC}"

    NODE_COUNT=$(docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/foxy/setup.bash && ros2 node list 2>/dev/null | wc -l" 2>/dev/null || echo "0")
    echo -e "Nodes:     ${GREEN}${NODE_COUNT}${NC}"

    USE_SIM=$(docker exec "$CONTAINER_NAME" python3 -c "import json,os; p='/autonomous_ROS/src/jetson_bot_gui/web/config.json'; print(json.load(open(p))['use_sim']) if os.path.exists(p) else print('unknown')" 2>/dev/null || echo "unknown")
    CURRENT_MODE=$(docker exec "$CONTAINER_NAME" python3 -c "import json,os; p='/autonomous_ROS/src/jetson_bot_gui/web/config.json'; print(json.load(open(p))['mode']) if os.path.exists(p) else print('unknown')" 2>/dev/null || echo "unknown")
    echo -e "Mode:      ${BLUE}${CURRENT_MODE}${NC} ($( [[ "$USE_SIM" == "False" ]] && echo "Hardware" || echo "Sim" ))"

    if [[ "$USE_SIM" == "False" ]]; then
        if docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/foxy/setup.bash && ros2 node list 2>/dev/null | grep -q diffdrive_bridge" 2>/dev/null; then
            echo -e "Control:   ${GREEN}OK (HW Bridge)${NC}"
        else
            echo -e "Control:   ${RED}Bridge Offline/Crashed${NC}"
        fi
    else
        if docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/foxy/setup.bash && ros2 node list 2>/dev/null | grep -q controller_manager" 2>/dev/null; then
            echo -e "Control:   ${GREEN}OK (Simulation)${NC}"
        else
            echo -e "Control:   ${YELLOW}Starting...${NC}"
        fi
    fi

    echo "--- Hardware Audit (from logs) ---"
    if docker exec "$CONTAINER_NAME" [ -f /tmp/sim.log ]; then
        docker exec "$CONTAINER_NAME" awk '/🔍 HARDWARE AUDIT STARTING.../,/==================================================/' /tmp/sim.log | sed 's/🔍//g; s/❌//g; s/✅//g; s/🛑//g; s/🚀//g'
    else
        echo -e "${YELLOW}No audit data available yet.${NC}"
    fi
}

verify_fusion() {
    log_info "Running EKF/fusion + health verification inside Docker..."
    docker exec "$CONTAINER_NAME" bash -c '
        source /opt/ros/foxy/setup.bash 2>/dev/null || true
        source /autonomous_ROS/install/setup.bash 2>/dev/null || true
        echo "=== Odom rates ==="
        timeout 5 ros2 topic hz /odom 2>/dev/null | head -3 || echo "no /odom (start stack)"
        timeout 5 ros2 topic hz /odom_filtered 2>/dev/null | head -3 || echo "no /odom_filtered (launch with EKF)"
        echo "=== TF / IMU link ==="
        ros2 topic echo /tf_static --once 2>/dev/null | grep -E "imu_link|base_link" | head -3 || echo "check rsp or launch"
        echo "=== Fusion nodes (if running) ==="
        ros2 node list 2>/dev/null | grep -E "ekf|diffdrive|imu_filter" | head -5 || echo "no fusion nodes active"
    '
}

# ── Main Dispatcher ───────────────────────────────────────────────────────────

case "${1:-help}" in
    up)              shift; stop_robot; _launch "$@" ;;
    sim)             shift; stop_robot; _launch "sim:=true viz:=true $@" ;;
    robot)           shift; stop_robot; _launch "sim:=false viz:=false $@" ;;
    nav)             shift; attempt_save_map || true; stop_robot; _launch "mode:=navigation $@" ;;
    map2nav)         shift; attempt_save_map || true; stop_robot; _launch "mode:=navigation $@" ;;
    build)           build_workspace ;;
    auto)            install_deps; build_workspace ;;
    status)          show_status ;;
    logs)            docker exec "$CONTAINER_NAME" tail -f /tmp/sim.log ;;
    verify)          verify_fusion ;;
    watch)
        log_info "Running EKF/fusion + health verification inside Docker..."
        docker exec "$CONTAINER_NAME" bash -c '
            source /opt/ros/foxy/setup.bash 2>/dev/null || true
            source /autonomous_ROS/install/setup.bash 2>/dev/null || true
            echo "=== Odom rates ==="
            timeout 5 ros2 topic hz /odom 2>/dev/null | head -3 || echo "no /odom"
            timeout 5 ros2 topic hz /odom_filtered 2>/dev/null | head -3 || echo "no /odom_filtered (launch stack first)"
            echo "=== TF / IMU ==="
            ros2 topic echo /tf_static --once 2>/dev/null | grep -E "imu_link|base_link" || echo "check TF"
            echo "=== Nodes (if stack running) ==="
            ros2 node list 2>/dev/null | grep -E "ekf|diffdrive|imu" | head -5 || echo "no fusion nodes (use ./robot.sh sim/robot)"
        '
        ;;
    watch)
        log_info "Watching status (Ctrl+C to stop)..."
        while true; do
            clear
            show_status
            sleep 10
        done
        ;;
    clean)           docker exec "$CONTAINER_NAME" bash -c "cd /autonomous_ROS && rm -rf build install log" ;;
    shell)           shift; enter_shell "$@" ;;
    stop)
        shift
        if [[ "$1" == "f" ]]; then
            log_info "Force restarting container..."
            docker restart "$CONTAINER_NAME"
            sleep 2
        fi
        stop_robot
        ;;
    *)               show_help ;;
esac
