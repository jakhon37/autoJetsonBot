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

# Configuration — default is Humble. Override with ROS_DISTRO=foxy ./robot.sh ...
ROS_DISTRO="${ROS_DISTRO:-humble}"
CONTAINER_NAME="${CONTAINER_NAME:-auto_ros_${ROS_DISTRO}}"
IMAGE_NAME="${IMAGE_NAME:-auto_ros:${ROS_DISTRO}}"
DOCKERFILE="Dockerfile.${ROS_DISTRO}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
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
    image       Build the Docker image (Humble by default)
    build       Build the modular workspace (jetson_bot_*)
    auto        Install missing dependencies and build
    status      Show container and node health
    remote      Print URLs + SSH tunnel command for your PC browser
    watch       Auto-refresh status every 10s
    logs        Tail the live ROS launch log (/tmp/sim.log)
    shell       Enter container (or run command: ./robot.sh shell "ls")
    verify      Run EKF/fusion + basic health checks (inside Docker)
    clean       Delete build, install, and log folders
    help        Show this help

ENVIRONMENT:
    ROS_DISTRO  humble (default) or foxy
    Example:    ROS_DISTRO=foxy ./robot.sh image && ROS_DISTRO=foxy ./robot.sh sim

EXAMPLES:
    ./robot.sh image           # docker build -f Dockerfile.humble -t auto_ros:humble
    ./robot.sh sim
    ./robot.sh sim mode:=navigation
    ./robot.sh up
    ./robot.sh nav
    ./robot.sh map2nav          # save map + switch to nav (real hardware friendly)
    ./robot.sh sim x:=0.5 y:=0 z:=0.06 yaw:=0.0   # spawn pose (passed to launch)
EOH
}

# ── Container lifecycle ───────────────────────────────────────────────────────

build_image() {
    if [ ! -f "$DOCKERFILE" ]; then
        log_error "Missing $DOCKERFILE (ROS_DISTRO=${ROS_DISTRO})"
        exit 1
    fi
    log_info "Building ${IMAGE_NAME} from ${DOCKERFILE}..."
    docker build -f "$DOCKERFILE" -t "$IMAGE_NAME" .
    log_success "Image ${IMAGE_NAME} ready."
}

check_container() {
    if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
        log_error "Image $IMAGE_NAME not found. Run: ./robot.sh image"
        exit 1
    fi

    if ! docker ps -a | grep -q "$CONTAINER_NAME"; then
        log_warn "Container $CONTAINER_NAME does not exist. Attempting basic creation..."
        docker run -d \
            --name "$CONTAINER_NAME" \
            --network host \
            -v "$(pwd)":/autonomous_ROS \
            -e "ROS_DISTRO=${ROS_DISTRO}" \
            --privileged \
            "$IMAGE_NAME" \
            bash -c "while true; do sleep 3600; done" || {
                log_error "Create failed. Run: ./robot.sh image"
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
    log_info "Building modular workspace for ROS 2 ${ROS_DISTRO}..."
    if [ -f install/setup.bash ] && ! grep -q "/opt/ros/${ROS_DISTRO}" install/setup.bash 2>/dev/null; then
        log_warn "Existing install/ does not match ${ROS_DISTRO}. Run: ./robot.sh clean && ./robot.sh build"
    fi
    check_container
    docker exec "$CONTAINER_NAME" bash -c "
        cd /autonomous_ROS &&
        source ${ROS_SETUP} &&
        colcon build --symlink-install --continue-on-error
    "
}

# ── Dependencies ──────────────────────────────────────────────────────────────

install_deps() {
    log_info "Checking for missing dependencies..."
    check_container
    
    # 1. web_video_server
    if ! docker exec "$CONTAINER_NAME" bash -c "source ${ROS_SETUP} && ros2 pkg list 2>/dev/null" | grep -q web_video_server; then
        log_warn "web_video_server not found. Installing..."
        docker exec "$CONTAINER_NAME" apt-get update
        docker exec "$CONTAINER_NAME" apt-get install -y ros-${ROS_DISTRO}-web-video-server
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
            source ${ROS_SETUP} &&
            source install/setup.bash 2>/dev/null || true &&
            nohup ros2 launch jetson_bot_bringup main.launch.py ${EXTRA_ARGS} > /tmp/sim.log 2>&1
        "

    log_success "System is booting."
    print_remote_access
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

    # pkill -f PATTERN kills this stop script too (the pattern is in bash -c),
    # so leftover nodes survive and the next launch double-binds the serial port.
    docker exec -i "$CONTAINER_NAME" python3 - <<'PY' || true
import os, signal
from pathlib import Path
needles = (
    "ros2 launch jetson_bot_bringup",
    "jetson_bot_diffdrive",
    "jetson_bot_imu",
    "jetson_bot_gui",
    "imu_filter_madgwick",
    "rplidar_composition",
    "rplidar_node",
    "async_slam_toolbox",
    "slam_toolbox",
    "rosbridge_websocket",
    "web_video_server",
    "robot_state_publisher",
    "static_transform_publisher",
    "ekf_node",
    "controller_manager",
    "nav2_",
    "gzserver",
    "gzclient",
    "gazebo",
    "rviz2",
    "socat",
)
me = os.getpid()
for proc in Path("/proc").iterdir():
    if not proc.name.isdigit():
        continue
    pid = int(proc.name)
    if pid == me:
        continue
    try:
        cmd = (proc / "cmdline").read_bytes().replace(b"\x00", b" ").decode("utf-8", "ignore")
    except Exception:
        continue
    if any(needle in cmd for needle in needles):
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
PY
    
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
    if ! docker exec "$CONTAINER_NAME" bash -c "source ${ROS_SETUP} && ros2 node list 2>/dev/null | grep -q slam_toolbox" 2>/dev/null; then
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
        source ${ROS_SETUP} 2>/dev/null || true
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
        docker exec "$CONTAINER_NAME" bash -c "source ${ROS_SETUP} && source install/setup.bash 2>/dev/null || true && $*"
    else
        log_info "Entering container..."
        docker exec -it "$CONTAINER_NAME" bash -c "source ${ROS_SETUP} && source install/setup.bash 2>/dev/null || true && exec bash"
    fi
}

jetson_lan_ip() {
    local ip=""
    if [ -n "${SSH_CONNECTION:-}" ]; then
        ip=$(echo "$SSH_CONNECTION" | awk '{print $3}')
    fi
    if [ -z "$ip" ]; then
        ip=$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')
    fi
    echo "$ip"
}

print_remote_access() {
    local lan_ip
    lan_ip=$(jetson_lan_ip)
    echo "--- Access from this PC (SSH client) ---"
    echo -e "Do ${YELLOW}not${NC} open http://localhost:8000 on your laptop. That is the Jetson, not your PC."
    if [ -n "$lan_ip" ]; then
        echo -e "Same Wi-Fi:  ${GREEN}http://${lan_ip}:8000${NC}"
        echo -e "Rosbridge:   ws://${lan_ip}:9090"
        echo -e "Camera:      http://${lan_ip}:8080"
        echo -e "VNC:         ${lan_ip}:5900"
    fi
    echo "SSH tunnel (run on your PC, then browse http://localhost:8000):"
    if [ -n "$lan_ip" ]; then
        echo "  ssh -L 8000:127.0.0.1:8000 -L 9090:127.0.0.1:9090 -L 8080:127.0.0.1:8080 nvidia@${lan_ip}"
    else
        echo "  ssh -L 8000:127.0.0.1:8000 -L 9090:127.0.0.1:9090 -L 8080:127.0.0.1:8080 nvidia@<JETSON_IP>"
    fi
}

show_esp32_status() {
    echo "--- ESP32 ---"
    if ! docker ps --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
        echo -e "ESP32:     ${RED}container down${NC}"
        return
    fi

    python3 - <<'PY'
import os
GREEN, YELLOW, RED, NC = "\033[0;32m", "\033[1;33m", "\033[0;31m", "\033[0m"
by_id_root = "/dev/serial/by-id"
print("USB list:")
if os.path.isdir(by_id_root):
    for name in sorted(os.listdir(by_id_root)):
        path = os.path.realpath(os.path.join(by_id_root, name))
        tag = "ESP32?" if any(s in name.lower() for s in ("1a86", "qinheng", "wch", "esp", "acm")) and "cp210" not in name.lower() else \
              "lidar?" if "cp210" in name.lower() or "silicon" in name.lower() else "other"
        print(f"           {tag:7} {path}  ({name})")
else:
    print("           (no /dev/serial/by-id)")
skip = ("cp210", "silicon_labs", "rplidar")
prefer = ("1a86", "qinheng", "usb_single_serial", "wch", "esp32", "arduino")
esp = None
esp_id = None
if os.path.isdir(by_id_root):
    for name in sorted(os.listdir(by_id_root)):
        low = name.lower()
        path = os.path.realpath(os.path.join(by_id_root, name))
        if any(s in low for s in skip) or not os.path.exists(path):
            continue
        if any(s in low for s in prefer) or "ttyACM" in path:
            esp, esp_id = path, name
            if any(s in low for s in prefer):
                break
if not esp:
    for p in ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"):
        if os.path.exists(p):
            esp, esp_id = p, p
            break
if esp:
    print(f"USB:       {GREEN}OK{NC}  {esp}" + (f"  ({esp_id})" if esp_id != esp else ""))
else:
    print(f"USB:       {RED}FAIL{NC}  ESP32 not on USB (check cable / power)")
PY

    docker exec -i "$CONTAINER_NAME" python3 - <<'PY' || true
import glob, os
GREEN, YELLOW, RED, NC = "\033[0;32m", "\033[1;33m", "\033[0;31m", "\033[0m"
acm = [p for p in ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2") if os.path.exists(p)]
holders = []
for fd in glob.glob("/proc/[0-9]*/fd/*"):
    try:
        target = os.readlink(fd)
    except OSError:
        continue
    if not target.startswith("/dev/ttyACM"):
        continue
    pid = fd.split("/")[2]
    try:
        cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\x00", b" ").decode(errors="ignore")[:80]
    except OSError:
        cmd = ""
    item = f"{target} <- {cmd.strip() or pid}"
    if item not in holders:
        holders.append(item)
if holders:
    live = None
    by_id = "/dev/serial/by-id"
    if os.path.isdir(by_id):
        for name in os.listdir(by_id):
            low = name.lower()
            if any(s in low for s in ("1a86", "qinheng", "wch", "usb_single_serial")):
                live = os.path.realpath(os.path.join(by_id, name))
                break
    stale = live and all(live not in h and os.path.basename(live) not in h for h in holders)
    if stale:
        print(f"Lock:      {RED}STALE{NC}  bridge on old port, ESP32 is {live}")
        print("           " + holders[0])
    elif any("diffdrive" in h for h in holders):
        print(f"Lock:      {GREEN}OK{NC}  " + holders[0])
    else:
        print(f"Lock:      {YELLOW}WARN{NC}  " + holders[0])
    for extra in holders[1:]:
        print("           " + extra)
elif acm:
    print(f"Lock:      {YELLOW}WARN{NC}  {', '.join(acm)} present, not opened")
else:
    print(f"Lock:      {RED}FAIL{NC}  no ttyACM inside container (restart container after USB replug)")
PY

    docker exec -i "$CONTAINER_NAME" python3 - <<'PY' || echo -e "Bridge:     ${RED}FAIL${NC}  could not query ESP32"
import json, os, glob, time
GREEN, YELLOW, RED, NC = "\033[0;32m", "\033[1;33m", "\033[0;31m", "\033[0m"

bridge = False
for cmdline in glob.glob("/proc/[0-9]*/cmdline"):
    try:
        cmd = open(cmdline, "rb").read().replace(b"\x00", b" ").decode(errors="ignore")
    except OSError:
        continue
    if "diffdrive_node" in cmd:
        bridge = True
        break
print(("Bridge:     %sOK%s  diffdrive_bridge" % (GREEN, NC)) if bridge else
      ("Bridge:     %sFAIL%s  diffdrive_bridge not running" % (RED, NC)))

data = {}
try:
    # The bridge rewrites this every second. An older file is left over from a past run.
    if time.time() - os.path.getmtime("/tmp/esp32_status.json") < 5:
        with open("/tmp/esp32_status.json") as f:
            data = json.load(f)
except Exception:
    pass

if not data:
    print(f"Motors:     {YELLOW}WARN{NC}  waiting for /tmp/esp32_status.json (relaunch after build)")
    print(f"Encoders:   {YELLOW}WARN{NC}  need e <left> <right> lines from firmware")
    print(f"IMU serial: {YELLOW}WARN{NC}  need i ax ay az gx gy gz lines")
else:
    connected = data.get("connected")
    tx_age = float(data.get("tx_age_s", -1))
    enc_age = float(data.get("enc_age_s", -1))
    imu_age = float(data.get("imu_age_s", -1))
    rx_age = float(data.get("rx_age_s", -1))
    last_rx = data.get("last_rx") or "(none)"
    port = data.get("port", "?")
    tx_ok = data.get("tx_ok", 0)
    if connected and (tx_age < 0 or tx_age < 5):
        extra = f"tx {tx_ok} cmds" if tx_ok else "link open (drive in UI to count TX)"
        print(f"Motors:     {GREEN}OK{NC}  {port}  {extra}")
    elif connected:
        print(f"Motors:     {YELLOW}WARN{NC}  port open, last TX {tx_age}s ago")
    else:
        print(f"Motors:     {RED}FAIL{NC}  serial closed")
    enc = data.get("last_enc")
    imu = data.get("last_imu")
    enc_txt = f"L={enc[0]} R={enc[1]}" if isinstance(enc, list) and len(enc) >= 2 else ""
    if isinstance(imu, list) and len(imu) >= 6:
        imu_txt = f"acc={imu[0]},{imu[1]},{imu[2]}  gyro={imu[3]},{imu[4]},{imu[5]}"
    else:
        imu_txt = ""
    if 0 <= enc_age <= 1.5:
        print(f"Encoders:   {GREEN}OK{NC}  {enc_txt}  ({data.get('enc_ok', 0)} pkts)")
    elif 0 <= rx_age <= 1.5:
        print(f"Encoders:   {YELLOW}WARN{NC}  serial RX but not e-ticks  last: {last_rx}")
    else:
        print(f"Encoders:   {RED}FAIL{NC}  no e-lines (open-loop drive still possible)")
        print(f"            last RX: {last_rx}")
    if 0 <= imu_age <= 1.5:
        print(f"IMU serial: {GREEN}OK{NC}  {imu_txt}  ({data.get('imu_ok', 0)} pkts)")
    else:
        print(f"IMU serial: {YELLOW}WARN{NC}  no i-lines yet  last RX: {last_rx}")
PY

    if docker exec "$CONTAINER_NAME" [ -f /tmp/sim.log ]; then
        local last
        last=$(docker exec "$CONTAINER_NAME" grep -E 'Connected to ESP32|Serial Error|Serial write error|Serial read error|ESP32 Reboot|Motor Bridge' /tmp/sim.log 2>/dev/null | tail -3)
        if echo "$last" | grep -qE 'write error|Serial Error|Input/output'; then
            echo -e "Log:       ${RED}I/O errors${NC} — USB dropped or stale ttyACM. Run: ./robot.sh robot"
            echo "$last" | sed 's/^/           /'
        elif echo "$last" | grep -q 'Connected to ESP32'; then
            echo -e "Log:       ${GREEN}OK${NC}"
            echo "$last" | tail -1 | sed 's/^/           /'
        elif [ -n "$last" ]; then
            echo -e "Log:       ${YELLOW}check${NC}"
            echo "$last" | sed 's/^/           /'
        fi
    fi
}

show_status() {
    log_info "Status:"
    docker ps -f "name=$CONTAINER_NAME" --format "table {{.Names}}\t{{.Status}}"

    local lan_ip
    lan_ip=$(jetson_lan_ip)
    
    if curl -s --connect-timeout 2 --max-time 3 http://localhost:8000 >/dev/null 2>&1; then
        echo -e "Web UI:    ${GREEN}OK${NC}  http://localhost:8000  (Jetson only)"
        [ -n "$lan_ip" ] && echo -e "           ${GREEN}http://${lan_ip}:8000${NC}  (open this on your PC)"
    else
        echo -e "Web UI:    ${RED}Offline${NC}"
    fi

    if docker exec "$CONTAINER_NAME" pgrep x11vnc >/dev/null 2>&1; then
        echo -e "VNC:       ${GREEN}OK${NC}  localhost:5900"
        [ -n "$lan_ip" ] && echo -e "           ${lan_ip}:5900"
    else
        echo -e "VNC:       ${RED}Offline${NC}"
    fi

    echo "--- ROS Health ---"
    if ! docker exec "$CONTAINER_NAME" pgrep -f "main.launch.py" >/dev/null 2>&1; then
        echo -e "Launch:    ${RED}Crashed/Stop${NC}"
        return 0
    fi
    echo -e "Launch:    ${GREEN}Running${NC}"

    NODE_COUNT=$(docker exec "$CONTAINER_NAME" bash -c "source ${ROS_SETUP} && ros2 node list 2>/dev/null | wc -l" 2>/dev/null || echo "0")
    echo -e "Nodes:     ${GREEN}${NODE_COUNT}${NC}"

    USE_SIM=$(docker exec "$CONTAINER_NAME" python3 -c "import json,os; p='/autonomous_ROS/src/jetson_bot_gui/web/config.json'; print(json.load(open(p))['use_sim']) if os.path.exists(p) else print('unknown')" 2>/dev/null || echo "unknown")
    CURRENT_MODE=$(docker exec "$CONTAINER_NAME" python3 -c "import json,os; p='/autonomous_ROS/src/jetson_bot_gui/web/config.json'; print(json.load(open(p))['mode']) if os.path.exists(p) else print('unknown')" 2>/dev/null || echo "unknown")
    echo -e "Mode:      ${BLUE}${CURRENT_MODE}${NC} ($( [[ "$USE_SIM" == "False" ]] && echo "Hardware" || echo "Sim" ))"

    if [[ "$USE_SIM" == "False" ]]; then
        if docker exec "$CONTAINER_NAME" bash -c "source ${ROS_SETUP} && ros2 node list 2>/dev/null | grep -q diffdrive_bridge" 2>/dev/null; then
            echo -e "Control:   ${GREEN}OK (HW Bridge)${NC}"
        else
            echo -e "Control:   ${RED}Bridge Offline/Crashed${NC}"
        fi
    else
        if docker exec "$CONTAINER_NAME" bash -c "source ${ROS_SETUP} && ros2 node list 2>/dev/null | grep -q controller_manager" 2>/dev/null; then
            echo -e "Control:   ${GREEN}OK (Simulation)${NC}"
        else
            echo -e "Control:   ${YELLOW}Starting...${NC}"
        fi
    fi

    show_esp32_status

    echo "--- Hardware Audit (from logs) ---"
    if docker exec "$CONTAINER_NAME" [ -f /tmp/sim.log ]; then
        docker exec "$CONTAINER_NAME" awk '/🔍 HARDWARE AUDIT STARTING.../,/==================================================/' /tmp/sim.log | sed 's/🔍//g; s/❌//g; s/✅//g; s/🛑//g; s/🚀//g'
    else
        echo -e "${YELLOW}No audit data available yet.${NC}"
    fi
}

verify_fusion() {
    log_info "Running EKF/fusion + health verification inside Docker..."
    docker exec "$CONTAINER_NAME" bash -c "
        source ${ROS_SETUP} 2>/dev/null || true
        source /autonomous_ROS/install/setup.bash 2>/dev/null || true
        echo '=== Odom rates ==='
        timeout 5 ros2 topic hz /odom 2>/dev/null | head -3 || echo 'no /odom (start stack)'
        timeout 5 ros2 topic hz /odom_filtered 2>/dev/null | head -3 || echo 'no /odom_filtered (launch with EKF)'
        echo '=== TF / IMU link ==='
        ros2 topic echo /tf_static --once 2>/dev/null | grep -E 'imu_link|base_link' | head -3 || echo 'check rsp or launch'
        echo '=== Fusion nodes (if running) ==='
        ros2 node list 2>/dev/null | grep -E 'ekf|diffdrive|imu_filter' | head -5 || echo 'no fusion nodes active'
    "
}

# ── Main Dispatcher ───────────────────────────────────────────────────────────

case "${1:-help}" in
    up)              shift; stop_robot; _launch "$@" ;;
    sim)             shift; stop_robot; _launch "sim:=true viz:=true $@" ;;
    robot)           shift; stop_robot; _launch "sim:=false viz:=false $@" ;;
    nav)             shift; attempt_save_map || true; stop_robot; _launch "mode:=navigation $@" ;;
    map2nav)         shift; attempt_save_map || true; stop_robot; _launch "mode:=navigation $@" ;;
    image)           build_image ;;
    build)           build_workspace ;;
    auto)            install_deps; build_workspace ;;
    status)          show_status ;;
    remote)          print_remote_access ;;
    logs)            docker exec "$CONTAINER_NAME" tail -f /tmp/sim.log ;;
    verify)          verify_fusion ;;
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
