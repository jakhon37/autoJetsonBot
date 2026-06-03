#!/bin/bash
# 🤖 Autonomous Jetson Robot - Control Script
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
XVFB_DISPLAY=":99"
XVFB_AUTH="/tmp/xvfb99.auth"

log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }

show_help() {
    cat << EOF
🤖 Autonomous Jetson Robot Control

USAGE:
    ./robot.sh [COMMAND]

COMMANDS:
    sim         Start Gazebo simulation (headless)
    gui         Start Gazebo + RViz2 via VNC (connect to localhost:${VNC_PORT})
    vnc-restart Restart VNC server (fixes stuck/refused connections)
    robot       Start real robot (no simulation)
    web         Open web interface in browser
    shell       Enter robot container
    build       Build the workspace
    stop        Stop all robot processes
    status      Show robot status
    logs        Show robot logs
    clean       Clean build files
    help        Show this help

GUI / VNC:
    './robot.sh gui' starts a virtual display (Xvfb :99) inside the container
    and exposes it over VNC on port ${VNC_PORT}. No XQuartz needed.

    Mac:   open vnc://localhost:${VNC_PORT}   (or RealVNC Viewer — leave password blank)
    Linux: vncviewer localhost:${VNC_PORT}

WEB INTERFACE:
    http://localhost:8000   Robot control interface
    ws://localhost:9090     ROSBridge WebSocket

KEYBOARD CONTROLS (web UI):
    WASD / Arrow Keys  Move robot
    Spacebar           Emergency stop

EOF
}

# ── Container lifecycle ───────────────────────────────────────────────────────

check_container() {
    if ! docker ps -a | grep -q "$CONTAINER_NAME"; then
        log_error "Container $CONTAINER_NAME does not exist."
        log_info  "Create it first with: ./runrosenv.sh"
        exit 1
    fi

    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        log_info "Container $CONTAINER_NAME is stopped — starting..."
        docker start "$CONTAINER_NAME" || { log_error "Failed to start container"; exit 1; }
        log_success "Container started"
        sleep 3
    fi
}

# ── Build ─────────────────────────────────────────────────────────────────────

build_workspace() {
    log_info "Building robot workspace..."
    check_container

    if docker exec "$CONTAINER_NAME" bash -c "
        cd /autonomous_ROS &&
        source /opt/ros/foxy/setup.bash &&
        colcon build --symlink-install
    "; then
        log_success "Workspace built successfully"
    else
        log_error "Build failed — run './robot.sh shell' to debug"
        exit 1
    fi
}

# ── Simulation launch (shared by sim and gui) ─────────────────────────────────

_launch_sim_nodes() {
    # Parse extra arguments (e.g., mode:=navigation)
    EXTRA_ARGS="${@:-mode:=mapping}"

    docker exec -d \
        -e DISPLAY="${XVFB_DISPLAY}" \
        -e QT_X11_NO_MITSHM=1 \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e MESA_GL_VERSION_OVERRIDE=3.3 \
        -e GALLIUM_DRIVER=softpipe \
        -e GAZEBO_MODEL_PATH="/autonomous_ROS/install/jetson_bot_description/share/jetson_bot_description" \
        "$CONTAINER_NAME" bash -c "
            cd /autonomous_ROS &&
            source /opt/ros/foxy/setup.bash &&
            source install/setup.bash &&
            nohup ros2 launch jetson_bot_bringup sim.launch.py ${EXTRA_ARGS} > /tmp/sim.log 2>&1
        "
}

_wait_for_web() {
    local timeout=${1:-60}
    log_info "Waiting for web server (up to ${timeout}s)..."
    for i in $(seq 1 "$timeout"); do
        sleep 1
        if curl -s http://localhost:8000 > /dev/null 2>&1; then
            return 0
        fi
        [ $((i % 5)) -eq 0 ] && echo -n " ${i}s" || echo -n "."
    done
    echo ""
    return 1
}

start_robot_system() {
    log_info "🚀 Starting Simulation System..."

    # 1. Setup Display (always lightweight, ensures GUI apps can run)
    start_virtual_display
    start_vnc_server
    start_openbox

    # 2. Launch Simulation Nodes
    _launch_sim_nodes "$@"

    log_success "Simulation system is booting. Check VNC or Web UI."
    open_vnc_viewer
}


# ── VNC / Xvfb helpers ────────────────────────────────────────────────────────

start_virtual_display() {
    if docker exec "$CONTAINER_NAME" bash -c "pgrep -f 'Xvfb ${XVFB_DISPLAY}'" &>/dev/null; then
        log_info "Xvfb already running on ${XVFB_DISPLAY}"
        return
    fi

    # Install xauth if missing (needed to create the auth file)
    if ! docker exec "$CONTAINER_NAME" bash -c "command -v xauth" &>/dev/null; then
        log_info "Installing xauth..."
        docker exec "$CONTAINER_NAME" bash -c "apt-get install -y --no-install-recommends xauth" \
            || { log_error "Failed to install xauth"; exit 1; }
    fi

    # Remove any stale lock/socket files from a previous crashed Xvfb
    local display_num="${XVFB_DISPLAY#:}"
    docker exec "$CONTAINER_NAME" bash -c "
        rm -f /tmp/.X${display_num}-lock
        rm -f /tmp/.X11-unix/X${display_num}
    " || true

    # Pre-create a valid empty xauth file (Xvfb -auth requires this to already exist)
    docker exec "$CONTAINER_NAME" bash -c "
        touch ${XVFB_AUTH} && xauth -f ${XVFB_AUTH} generate ${XVFB_DISPLAY} . trusted 2>/dev/null || true
    "

    log_info "Starting virtual framebuffer (Xvfb ${XVFB_DISPLAY})..."
    docker exec -d "$CONTAINER_NAME" bash -c \
        "Xvfb ${XVFB_DISPLAY} -screen 0 1280x1024x24 -auth ${XVFB_AUTH} > /tmp/xvfb.log 2>&1"
    sleep 2

    if ! docker exec "$CONTAINER_NAME" bash -c "pgrep -f 'Xvfb ${XVFB_DISPLAY}'" &>/dev/null; then
        log_error "Xvfb failed — log output:"
        docker exec "$CONTAINER_NAME" bash -c "cat /tmp/xvfb.log" || true
        exit 1
    fi
    log_success "Xvfb running on ${XVFB_DISPLAY}"
}

start_vnc_server() {
    if docker exec "$CONTAINER_NAME" bash -c "pgrep x11vnc" &>/dev/null; then
        log_info "x11vnc already running"
        return
    fi

    log_info "Starting VNC server on port ${VNC_PORT}..."
    docker exec -d "$CONTAINER_NAME" bash -c \
        "x11vnc -display ${XVFB_DISPLAY} \
         -auth ${XVFB_AUTH} \
         -nopw -forever -shared \
         -noipv6 -permitfiletransfer \
         -listen 0.0.0.0 -rfbport ${VNC_PORT} \
         > /tmp/x11vnc.log 2>&1"
    sleep 1

    if ! docker exec "$CONTAINER_NAME" bash -c "pgrep x11vnc" &>/dev/null; then
        log_error "x11vnc failed — check: docker exec $CONTAINER_NAME cat /tmp/x11vnc.log"
        exit 1
    fi
    log_success "VNC server listening on localhost:${VNC_PORT}"
}

start_openbox() {
    if docker exec "$CONTAINER_NAME" bash -c "pgrep openbox" &>/dev/null; then
        log_info "Openbox already running"
        return
    fi

    log_info "Starting Openbox window manager..."
    docker exec -d "$CONTAINER_NAME" bash -c \
        "DISPLAY=${XVFB_DISPLAY} openbox --sm-disable > /tmp/openbox.log 2>&1"
    sleep 1

    if docker exec "$CONTAINER_NAME" bash -c "pgrep openbox" &>/dev/null; then
        log_success "Openbox running — windows will have title bars"
    else
        log_warn "Openbox failed — windows won't be draggable"
        log_warn "Check: docker exec $CONTAINER_NAME cat /tmp/openbox.log"
    fi
}

check_vnc_port_exposed() {
    if ! docker port "$CONTAINER_NAME" "$VNC_PORT" &>/dev/null; then
        log_warn "Port ${VNC_PORT} is NOT forwarded to the host."
        log_warn "Recreate the container with -p ${VNC_PORT}:${VNC_PORT} (runrosenv.sh already does this)."
    fi
}

launch_gui_apps() {
    local rviz_config="/autonomous_ROS/install/my_robot_launch/share/my_robot_launch/config/lab_slam.rviz"

    log_info "Launching gzclient on display ${XVFB_DISPLAY}..."
    docker exec -d \
        -e DISPLAY="${XVFB_DISPLAY}" \
        -e QT_X11_NO_MITSHM=1 \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e MESA_GL_VERSION_OVERRIDE=3.3 \
        -e GALLIUM_DRIVER=softpipe \
        -e GAZEBO_MODEL_PATH="/autonomous_ROS/install/jetson_bot_description/share" \
        "$CONTAINER_NAME" bash -c "
            source /opt/ros/foxy/setup.bash &&
            source /autonomous_ROS/install/setup.bash &&
            gzclient > /tmp/gzclient.log 2>&1
        "

    log_info "Launching RViz2 on display ${XVFB_DISPLAY}..."
    docker exec -d \
        -e DISPLAY="${XVFB_DISPLAY}" \
        -e QT_X11_NO_MITSHM=1 \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e MESA_GL_VERSION_OVERRIDE=3.3 \
        "$CONTAINER_NAME" bash -c "
            source /opt/ros/foxy/setup.bash &&
            source /autonomous_ROS/install/setup.bash &&
            if [ -f '${rviz_config}' ]; then
                rviz2 -d '${rviz_config}' --ros-args -p use_sim_time:=true > /tmp/rviz2.log 2>&1
            else
                rviz2 --ros-args -p use_sim_time:=true > /tmp/rviz2.log 2>&1
            fi
        "
}

open_vnc_viewer() {
    log_info "────────────────────────────────────────────────"
    log_info "VNC is ready. Connect with one of these:"
    log_info ""
    log_info "  Option 1 — RealVNC Viewer (recommended, free):"
    log_info "    https://www.realvnc.com/en/connect/download/viewer/"
    log_info "    Address: localhost:${VNC_PORT}   (leave password blank)"
    log_info ""
    log_info "  Option 2 — macOS Screen Sharing:"
    log_info "    Finder → Go → Connect to Server → vnc://localhost:${VNC_PORT}"
    log_info "────────────────────────────────────────────────"
}

start_gui() {
    log_info "🖥  Starting GUI mode (Xvfb + VNC)..."
    check_container
    start_virtual_display
    start_vnc_server
    start_openbox
    check_vnc_port_exposed

    if ! curl -s http://localhost:8000 > /dev/null 2>&1; then
        log_info "Simulation not running — starting it first..."
        _launch_sim_nodes "gui:=true $@"
        if ! _wait_for_web 60; then
            echo ""
            log_warn "Web server not up yet — sim may still be loading"
            log_info "Check: docker exec $CONTAINER_NAME tail -20 /tmp/sim.log"
        else
            echo ""
        fi
    else
        log_info "Simulation already running — attaching GUI"
    fi

    log_info "Waiting for gzserver..."
    for i in $(seq 1 20); do
        sleep 1
        docker exec "$CONTAINER_NAME" bash -c "pgrep gzserver" &>/dev/null && break
        echo -n "."
    done
    echo ""

    launch_gui_apps

    log_success "🎉 GUI launched!"
    log_info "🌐 Web interface: http://localhost:8000"
    log_info "🔌 ROSBridge:     ws://localhost:9090"
    log_info ""
    log_info "Logs inside container:"
    log_info "  /tmp/xvfb.log  /tmp/x11vnc.log  /tmp/gzclient.log  /tmp/rviz2.log"
    log_info ""
    open_vnc_viewer
}

restart_vnc() {
    log_info "Restarting x11vnc..."
    check_container
    docker exec "$CONTAINER_NAME" bash -c "pkill x11vnc || true"
    sleep 1
    start_vnc_server
    log_success "VNC restarted on localhost:${VNC_PORT}"
    open_vnc_viewer
}

# ── Real robot ────────────────────────────────────────────────────────────────

start_robot() {
    log_info "🤖 Starting real robot..."

    docker exec -d "$CONTAINER_NAME" bash -c "
        cd /autonomous_ROS &&
        source /opt/ros/foxy/setup.bash &&
        source install/setup.bash &&
        nohup ros2 launch jetson_bot_bringup main.launch.py > /tmp/robot.log 2>&1
    "

    if _wait_for_web 20; then
        echo ""
        log_success "🎉 Robot started!"
        log_info "🌐 Web interface: http://localhost:8000"
        log_info "🔌 ROSBridge:     ws://localhost:9090"
    else
        echo ""
        log_error "Robot failed to start — check: ./robot.sh logs"
    fi
}

# ── Utility commands ──────────────────────────────────────────────────────────

open_web() {
    if curl -s http://localhost:8000 > /dev/null 2>&1; then
        log_success "Web interface is running"
        if command -v open > /dev/null 2>&1; then
            open http://localhost:8000
        elif command -v xdg-open > /dev/null 2>&1; then
            xdg-open http://localhost:8000
        else
            log_info "Open: http://localhost:8000"
        fi
    else
        log_error "Web interface not running."
        log_info "Start first with: ./robot.sh sim  (or)  ./robot.sh gui"
    fi
}

enter_shell() {
    log_info "🐚 Entering robot container..."
    check_container
    
    # Allow passing a command to run non-interactively
    if [ $# -gt 0 ]; then
        docker exec "$CONTAINER_NAME" bash -c "
            cd /autonomous_ROS &&
            source /opt/ros/foxy/setup.bash &&
            source install/setup.bash 2>/dev/null || true &&
            $*
        "
    else
        docker exec -it "$CONTAINER_NAME" bash -c "
            cd /autonomous_ROS &&
            source /opt/ros/foxy/setup.bash &&
            source install/setup.bash 2>/dev/null || true &&
            echo '🤖 Robot container — useful commands:' &&
            echo '  ros2 node list  |  ros2 topic list  |  ros2 launch ...' &&
            exec bash
        "
    fi
}
stop_robot() {
    log_info "🛑 Stopping all processes..."
    check_container

    log_info "Killing ROS/Gazebo/Web processes inside container..."

    # Aggregate kill command for efficiency
    docker exec "$CONTAINER_NAME" bash -c "
        pkill -9 -f 'ros2' || true
        pkill -9 -f 'gzserver' || true
        pkill -9 -f 'gzclient' || true
        pkill -9 -f 'gazebo' || true
        pkill -9 -f 'rviz2' || true
        pkill -9 -f 'slam_toolbox' || true
        pkill -9 -f 'robot_state_publisher' || true
        pkill -9 -f 'controller_manager' || true
        pkill -9 -f 'spawner' || true
        pkill -9 -f 'rosbridge' || true
        pkill -9 -f 'web_server' || true
        pkill -9 -f 'python3 -m http.server' || true
        pkill -9 -f 'Xvfb' || true
        pkill -9 -f 'x11vnc' || true
        pkill -9 -f 'openbox' || true
        pkill -9 -f 'jetson_bot' || true
    " || true

    _cleanup_ports

    log_success "All processes stopped"
}

_cleanup_ports() {
    log_info "Cleaning up ports 8000, 9090, 5900..."
    
    docker exec "$CONTAINER_NAME" bash -c "
        for port in 8000 9090 5900; do
            # Use netstat to find PIDs (netstat -p shows 'PID/Program')
            pids=\$(netstat -tulpn 2>/dev/null | grep \":\$port \" | awk '{print \$7}' | cut -d'/' -f1 | grep -E '^[0-9]+\$' || true)
            for pid in \$pids; do
                if [ ! -z \"\$pid\" ]; then
                    echo \"Killing process \$pid holding port \$port\"
                    kill -9 \$pid 2>/dev/null || true
                fi
            done
        done
    " || true

    # Give a moment for the OS to release ports
    sleep 2

    # Quick check from host - if ports are still busy, the container needs a restart
    if lsof -i :9090 -t >/dev/null || lsof -i :8000 -t >/dev/null; then
        log_warn "Ports still blocked after kill. Force-restarting container..."
        docker restart "$CONTAINER_NAME" >/dev/null
        sleep 5
    fi
}

show_status() {
    log_info "📊 Robot status:"

    echo ""
    echo "Container:"
    docker ps -f "name=$CONTAINER_NAME" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"

    echo ""
    echo "Services:"
    curl -s http://localhost:8000 > /dev/null 2>&1 \
        && echo "✅ Web Interface: http://localhost:8000" \
        || echo "❌ Web Interface: not running"

    docker exec "$CONTAINER_NAME" bash -c "netstat -ln 2>/dev/null | grep -q ':9090'" &>/dev/null \
        && echo "✅ ROSBridge: ws://localhost:9090" \
        || echo "❌ ROSBridge: not running"

    docker exec "$CONTAINER_NAME" bash -c "pgrep x11vnc" &>/dev/null \
        && echo "✅ VNC: localhost:${VNC_PORT}" \
        || echo "❌ VNC: not running  (start with: ./robot.sh gui)"

    if docker ps | grep -q "$CONTAINER_NAME"; then
        echo ""
        echo "Xvfb / VNC:"
        docker exec "$CONTAINER_NAME" bash -c "pgrep -a Xvfb   || echo '  Xvfb    not running'"
        docker exec "$CONTAINER_NAME" bash -c "pgrep -a x11vnc || echo '  x11vnc  not running'"

        echo ""
        echo "Active ROS Nodes:"
        docker exec "$CONTAINER_NAME" bash -c "
            source /opt/ros/foxy/setup.bash 2>/dev/null &&
            source /autonomous_ROS/install/setup.bash 2>/dev/null &&
            ros2 node list 2>/dev/null || echo '  No ROS nodes running'
        " 2>/dev/null
    fi
}

show_logs() {
    log_info "📝 Simulation log (last 50 lines):"
    check_container
    docker exec "$CONTAINER_NAME" bash -c "tail -50 /tmp/sim.log 2>/dev/null || docker logs --tail 50 $CONTAINER_NAME" || \
        docker logs --tail 50 "$CONTAINER_NAME"
}

clean_build() {
    log_info "🧹 Cleaning build files..."
    check_container
    docker exec "$CONTAINER_NAME" bash -c "cd /autonomous_ROS && rm -rf build install log || true"
    log_success "Build files cleaned"
}

# ── Main dispatcher ───────────────────────────────────────────────────────────

case "${1:-help}" in
    sim|start)       shift; stop_robot; _cleanup_ports; build_workspace; start_robot_system "$@" ;;
    gui)             shift; stop_robot; _cleanup_ports; build_workspace; start_gui "$@" ;;
    robot)           shift; stop_robot; _cleanup_ports; build_workspace; start_robot ;;
    vnc-restart)     restart_vnc ;;
    build)           build_workspace ;;
    stop)            stop_robot ;;
    status)          show_status ;;
    shell|bash)      shift; enter_shell "$@" ;;
    logs)            show_logs ;;
    clean)           clean_build ;;
    help|--help|-h)  show_help ;;
    *)
        log_error "Unknown command: $1"
        echo ""
        show_help
        exit 1
        ;;
esac


docker exec auto_ros_foxy bash -c "
  echo '=== SIM LOG (last 30) ===' && tail -30 /tmp/sim.log
  echo '=== GZCLIENT LOG ===' && cat /tmp/gzclient.log 2>/dev/null
  echo '=== RVIZ LOG ===' && cat /tmp/rviz2.log 2>/dev/null
"

docker exec auto_ros_foxy bash -c " 
  echo '=== GZCLIENT LOG ===' && cat /tmp/gzclient.log 2>/dev/null "

docker exec auto_ros_foxy bash -c " 
  echo '=== RVIZ LOG ===' && cat /tmp/rviz2.log 2>/dev/null "