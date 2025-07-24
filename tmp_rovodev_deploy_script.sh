#!/bin/bash
# deploy_robot.sh - Automated Deployment Script for Jetson Robot
# Long-term solution for seamless deployment across environments
# Author: RovoDev Assistant

set -e  # Exit on any error

# =============================================================================
# CONFIGURATION VARIABLES
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_DIR="$WORKSPACE_ROOT/config"
LOG_DIR="/var/log/robot"
TEMP_DIR="/tmp/robot_deploy"

# Default values
DEPLOYMENT_MODE="real_hardware"  # real_hardware, simulation, development
ROBOT_NAME="jetson_bot"
ENABLE_SLAM="true"
ENABLE_NAVIGATION="false"
LOG_LEVEL="info"
DRY_RUN="false"
FORCE_REBUILD="false"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_dependencies() {
    log_info "Checking system dependencies..."
    
    # Check ROS2 installation
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2 not found. Please install ROS2 Foxy."
        exit 1
    fi
    
    # Check Python dependencies
    python3 -c "import yaml, serial" 2>/dev/null || {
        log_error "Missing Python dependencies. Installing..."
        pip3 install pyyaml pyserial
    }
    
    # Check hardware devices (for real hardware mode)
    if [ "$DEPLOYMENT_MODE" = "real_hardware" ]; then
        if [ ! -e "/dev/ttyACM0" ]; then
            log_warning "Arduino device /dev/ttyACM0 not found"
        fi
        
        if [ ! -e "/dev/ttyUSB0" ]; then
            log_warning "RPLidar device /dev/ttyUSB0 not found"
        fi
    fi
    
    log_success "Dependencies check completed"
}

setup_environment() {
    log_info "Setting up deployment environment..."
    
    # Create necessary directories
    mkdir -p "$LOG_DIR"
    mkdir -p "$TEMP_DIR"
    mkdir -p "$CONFIG_DIR/generated"
    
    # Set permissions
    sudo chown -R $USER:$USER "$LOG_DIR" 2>/dev/null || true
    
    # Source ROS2 environment
    source /opt/ros/foxy/setup.bash
    
    # Source workspace if built
    if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
        source "$WORKSPACE_ROOT/install/setup.bash"
    fi
    
    log_success "Environment setup completed"
}

validate_configuration() {
    log_info "Validating configuration..."
    
    # Check if unified config exists
    if [ ! -f "$CONFIG_DIR/unified_robot_config.yaml" ]; then
        log_error "Unified configuration file not found: $CONFIG_DIR/unified_robot_config.yaml"
        exit 1
    fi
    
    # Validate configuration syntax
    python3 -c "
import yaml
try:
    with open('$CONFIG_DIR/unified_robot_config.yaml', 'r') as f:
        yaml.safe_load(f)
    print('Configuration syntax valid')
except Exception as e:
    print(f'Configuration syntax error: {e}')
    exit(1)
    "
    
    log_success "Configuration validation completed"
}

build_workspace() {
    log_info "Building ROS2 workspace..."
    
    cd "$WORKSPACE_ROOT"
    
    if [ "$FORCE_REBUILD" = "true" ]; then
        log_info "Force rebuild requested, cleaning workspace..."
        rm -rf build/ install/ log/
    fi
    
    # Build with optimizations
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --parallel-workers $(nproc) \
        --event-handlers console_direct+
        
    if [ $? -eq 0 ]; then
        log_success "Workspace build completed successfully"
    else
        log_error "Workspace build failed"
        exit 1
    fi
    
    # Source the built workspace
    source install/setup.bash
}

generate_runtime_config() {
    log_info "Generating runtime configuration for $DEPLOYMENT_MODE mode..."
    
    # Use hardware interface to generate configs
    python3 "$SCRIPT_DIR/hardware_interface.py" \
        "$DEPLOYMENT_MODE" \
        "$CONFIG_DIR/unified_robot_config.yaml" \
        "$CONFIG_DIR/generated"
        
    if [ $? -eq 0 ]; then
        log_success "Runtime configuration generated"
    else
        log_error "Failed to generate runtime configuration"
        exit 1
    fi
}

setup_hardware_permissions() {
    if [ "$DEPLOYMENT_MODE" = "real_hardware" ]; then
        log_info "Setting up hardware permissions..."
        
        # Add user to dialout group for serial access
        sudo usermod -a -G dialout $USER
        
        # Set device permissions
        if [ -e "/dev/ttyACM0" ]; then
            sudo chmod 666 /dev/ttyACM0
        fi
        
        if [ -e "/dev/ttyUSB0" ]; then
            sudo chmod 666 /dev/ttyUSB0
        fi
        
        log_success "Hardware permissions configured"
    fi
}

start_robot_system() {
    log_info "Starting robot system in $DEPLOYMENT_MODE mode..."
    
    if [ "$DRY_RUN" = "true" ]; then
        log_info "DRY RUN: Would execute the following command:"
        echo "ros2 launch my_robot_launch unified_robot_launch.py \\"
        echo "  sim_mode:=$([ "$DEPLOYMENT_MODE" = "simulation" ] && echo "true" || echo "false") \\"
        echo "  robot_name:=$ROBOT_NAME \\"
        echo "  enable_slam:=$ENABLE_SLAM \\"
        echo "  enable_navigation:=$ENABLE_NAVIGATION \\"
        echo "  log_level:=$LOG_LEVEL"
        return 0
    fi
    
    # Determine sim_mode parameter
    SIM_MODE="false"
    if [ "$DEPLOYMENT_MODE" = "simulation" ]; then
        SIM_MODE="true"
    fi
    
    # Launch the unified system
    ros2 launch my_robot_launch unified_robot_launch.py \
        sim_mode:=$SIM_MODE \
        robot_name:=$ROBOT_NAME \
        enable_slam:=$ENABLE_SLAM \
        enable_navigation:=$ENABLE_NAVIGATION \
        log_level:=$LOG_LEVEL \
        2>&1 | tee "$LOG_DIR/robot_system_$(date +%Y%m%d_%H%M%S).log"
}

create_systemd_service() {
    log_info "Creating systemd service for automatic startup..."
    
    cat > /tmp/jetson-robot.service << EOF
[Unit]
Description=Jetson Autonomous Robot System
After=network.target
Wants=network.target

[Service]
Type=simple
User=$USER
Group=$USER
WorkingDirectory=$WORKSPACE_ROOT
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
ExecStart=/bin/bash $SCRIPT_DIR/deploy_robot.sh --mode=$DEPLOYMENT_MODE --robot-name=$ROBOT_NAME
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

    sudo mv /tmp/jetson-robot.service /etc/systemd/system/
    sudo systemctl daemon-reload
    
    log_success "Systemd service created. Enable with: sudo systemctl enable jetson-robot"
}

cleanup() {
    log_info "Cleaning up temporary files..."
    rm -rf "$TEMP_DIR"
    log_success "Cleanup completed"
}

show_usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Automated deployment script for Jetson Autonomous Robot

OPTIONS:
    -m, --mode MODE           Deployment mode: real_hardware, simulation, development
                             Default: real_hardware
    
    -n, --robot-name NAME     Robot name for multi-robot scenarios
                             Default: jetson_bot
    
    -s, --enable-slam         Enable SLAM functionality
                             Default: true
    
    -N, --enable-navigation   Enable navigation stack
                             Default: false
    
    -l, --log-level LEVEL     Logging level: debug, info, warn, error
                             Default: info
    
    -d, --dry-run            Show commands without executing
                             Default: false
    
    -f, --force-rebuild      Force rebuild of workspace
                             Default: false
    
    --create-service         Create systemd service for auto-startup
    
    -h, --help               Show this help message

EXAMPLES:
    # Deploy for real hardware with SLAM
    $0 --mode real_hardware --enable-slam
    
    # Deploy simulation environment
    $0 --mode simulation --robot-name sim_bot
    
    # Development mode with debug logging
    $0 --mode development --log-level debug
    
    # Dry run to see what would be executed
    $0 --mode real_hardware --dry-run
    
    # Create systemd service
    $0 --create-service

EOF
}

# =============================================================================
# MAIN EXECUTION
# =============================================================================

main() {
    log_info "Starting Jetson Robot Deployment Script"
    log_info "Deployment Mode: $DEPLOYMENT_MODE"
    log_info "Robot Name: $ROBOT_NAME"
    
    # Execute deployment steps
    check_dependencies
    setup_environment
    validate_configuration
    
    if [ "$FORCE_REBUILD" = "true" ] || [ ! -d "$WORKSPACE_ROOT/install" ]; then
        build_workspace
    fi
    
    generate_runtime_config
    setup_hardware_permissions
    start_robot_system
    
    log_success "Robot system deployment completed successfully!"
    log_info "System logs available in: $LOG_DIR"
    
    if [ "$DEPLOYMENT_MODE" = "real_hardware" ]; then
        log_info "Web interface available at: http://$(hostname -I | awk '{print $1}'):8000"
    fi
}

# =============================================================================
# ARGUMENT PARSING
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        -m|--mode)
            DEPLOYMENT_MODE="$2"
            shift 2
            ;;
        -n|--robot-name)
            ROBOT_NAME="$2"
            shift 2
            ;;
        -s|--enable-slam)
            ENABLE_SLAM="true"
            shift
            ;;
        -N|--enable-navigation)
            ENABLE_NAVIGATION="true"
            shift
            ;;
        -l|--log-level)
            LOG_LEVEL="$2"
            shift 2
            ;;
        -d|--dry-run)
            DRY_RUN="true"
            shift
            ;;
        -f|--force-rebuild)
            FORCE_REBUILD="true"
            shift
            ;;
        --create-service)
            create_systemd_service
            exit 0
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Validate deployment mode
case $DEPLOYMENT_MODE in
    real_hardware|simulation|development)
        ;;
    *)
        log_error "Invalid deployment mode: $DEPLOYMENT_MODE"
        log_error "Valid modes: real_hardware, simulation, development"
        exit 1
        ;;
esac

# Set up trap for cleanup
trap cleanup EXIT

# Execute main function
main