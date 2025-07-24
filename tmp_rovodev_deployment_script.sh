#!/bin/bash
# deployment_script.sh - Automated Deployment Script for ROS2 Robot
# Long-term solution for seamless deployment across environments
# Author: RovoDev Assistant

set -e  # Exit on any error

# =============================================================================
# CONFIGURATION VARIABLES
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_DIR="${WORKSPACE_DIR}/config"
LOG_DIR="${WORKSPACE_DIR}/logs"
BACKUP_DIR="${WORKSPACE_DIR}/backups"

# Default values
DEPLOYMENT_MODE="real"
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

show_usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Automated deployment script for ROS2 autonomous robot

OPTIONS:
    -m, --mode MODE         Deployment mode: real|sim|mock (default: real)
    -n, --name NAME         Robot name (default: jetson_bot)
    -s, --slam              Enable SLAM (default: true)
    -N, --navigation        Enable navigation (default: false)
    -l, --log-level LEVEL   Log level: debug|info|warn|error (default: info)
    -d, --dry-run           Show what would be done without executing
    -f, --force-rebuild     Force rebuild of workspace
    -h, --help              Show this help message

EXAMPLES:
    $0 --mode sim --slam --navigation
    $0 --mode real --name robot_01 --log-level debug
    $0 --dry-run --mode sim

EOF
}

check_dependencies() {
    log_info "Checking system dependencies..."
    
    local missing_deps=()
    
    # Check ROS2 installation
    if ! command -v ros2 &> /dev/null; then
        missing_deps+=("ros2")
    fi
    
    # Check colcon
    if ! command -v colcon &> /dev/null; then
        missing_deps+=("colcon")
    fi
    
    # Check Python dependencies
    if ! python3 -c "import yaml" &> /dev/null; then
        missing_deps+=("python3-yaml")
    fi
    
    # Mode-specific dependencies
    if [[ "$DEPLOYMENT_MODE" == "sim" ]]; then
        if ! command -v gazebo &> /dev/null; then
            missing_deps+=("gazebo")
        fi
    elif [[ "$DEPLOYMENT_MODE" == "real" ]]; then
        # Check hardware devices
        if [[ ! -e "/dev/ttyACM0" && ! -e "/dev/ttyUSB0" ]]; then
            log_warning "No Arduino/USB devices found. Hardware may not be connected."
        fi
    fi
    
    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        log_error "Missing dependencies: ${missing_deps[*]}"
        log_info "Please install missing dependencies and try again."
        return 1
    fi
    
    log_success "All dependencies satisfied"
    return 0
}

validate_configuration() {
    log_info "Validating configuration..."
    
    # Check workspace structure
    if [[ ! -d "${WORKSPACE_DIR}/src" ]]; then
        log_error "Invalid workspace structure. Missing src directory."
        return 1
    fi
    
    # Check required packages
    local required_packages=(
        "my_robot_launch"
        "web_gui_control"
        "slam_launch"
    )
    
    for package in "${required_packages[@]}"; do
        if [[ ! -d "${WORKSPACE_DIR}/src/${package}" ]]; then
            log_error "Missing required package: ${package}"
            return 1
        fi
    done
    
    # Validate mode-specific requirements
    if [[ "$DEPLOYMENT_MODE" == "real" ]]; then
        # Check diffdrive_arduino package
        if [[ ! -d "${WORKSPACE_DIR}/src/diffdrive_arduino" ]]; then
            log_warning "diffdrive_arduino package not found. Using fallback configuration."
        fi
        
        # Check hardware configuration
        if [[ ! -f "${CONFIG_DIR}/diffbot_hardware.yaml" ]]; then
            log_error "Hardware configuration file not found: ${CONFIG_DIR}/diffbot_hardware.yaml"
            return 1
        fi
    fi
    
    log_success "Configuration validation passed"
    return 0
}

setup_environment() {
    log_info "Setting up environment..."
    
    # Create necessary directories
    mkdir -p "${LOG_DIR}" "${BACKUP_DIR}" "${CONFIG_DIR}"
    
    # Set ROS2 environment
    if [[ -f "/opt/ros/foxy/setup.bash" ]]; then
        source "/opt/ros/foxy/setup.bash"
        log_info "Sourced ROS2 Foxy environment"
    elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
        source "/opt/ros/humble/setup.bash"
        log_info "Sourced ROS2 Humble environment"
    else
        log_error "No ROS2 installation found"
        return 1
    fi
    
    # Set workspace environment if built
    if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
        source "${WORKSPACE_DIR}/install/setup.bash"
        log_info "Sourced workspace environment"
    fi
    
    # Set environment variables
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    
    # Mode-specific environment setup
    if [[ "$DEPLOYMENT_MODE" == "sim" ]]; then
        export GAZEBO_MODEL_PATH="${WORKSPACE_DIR}/src/my_robot_launch/models:${GAZEBO_MODEL_PATH}"
        export GAZEBO_RESOURCE_PATH="${WORKSPACE_DIR}/src/my_robot_launch/worlds:${GAZEBO_RESOURCE_PATH}"
    fi
    
    log_success "Environment setup completed"
    return 0
}

build_workspace() {
    log_info "Building workspace..."
    
    cd "${WORKSPACE_DIR}"
    
    # Clean build if force rebuild requested
    if [[ "$FORCE_REBUILD" == "true" ]]; then
        log_info "Force rebuild requested. Cleaning workspace..."
        rm -rf build/ install/ log/
    fi
    
    # Build workspace
    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "DRY RUN: Would execute: colcon build --symlink-install"
    else
        log_info "Building workspace with colcon..."
        if colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; then
            log_success "Workspace built successfully"
        else
            log_error "Workspace build failed"
            return 1
        fi
        
        # Source the built workspace
        source "${WORKSPACE_DIR}/install/setup.bash"
    fi
    
    return 0
}

generate_launch_command() {
    local launch_cmd="ros2 launch my_robot_launch unified_robot_launch.py"
    
    # Add mode parameter
    if [[ "$DEPLOYMENT_MODE" == "sim" ]]; then
        launch_cmd+=" sim_mode:=true"
    else
        launch_cmd+=" sim_mode:=false"
    fi
    
    # Add robot name
    launch_cmd+=" robot_name:=${ROBOT_NAME}"
    
    # Add SLAM parameter
    launch_cmd+=" enable_slam:=${ENABLE_SLAM}"
    
    # Add navigation parameter
    launch_cmd+=" enable_navigation:=${ENABLE_NAVIGATION}"
    
    # Add log level
    launch_cmd+=" log_level:=${LOG_LEVEL}"
    
    echo "$launch_cmd"
}

backup_configuration() {
    log_info "Creating configuration backup..."
    
    local timestamp=$(date +"%Y%m%d_%H%M%S")
    local backup_file="${BACKUP_DIR}/config_backup_${timestamp}.tar.gz"
    
    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "DRY RUN: Would create backup: ${backup_file}"
    else
        tar -czf "${backup_file}" -C "${WORKSPACE_DIR}" config/ src/*/config/ 2>/dev/null || true
        log_success "Configuration backed up to: ${backup_file}"
    fi
}

deploy_robot() {
    log_info "Deploying robot in ${DEPLOYMENT_MODE} mode..."
    
    # Generate launch command
    local launch_cmd=$(generate_launch_command)
    
    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "DRY RUN: Would execute: ${launch_cmd}"
        return 0
    fi
    
    # Create log file
    local log_file="${LOG_DIR}/deployment_$(date +"%Y%m%d_%H%M%S").log"
    
    log_info "Starting robot deployment..."
    log_info "Launch command: ${launch_cmd}"
    log_info "Logs will be written to: ${log_file}"
    
    # Execute launch command
    exec ${launch_cmd} 2>&1 | tee "${log_file}"
}

cleanup_on_exit() {
    log_info "Cleaning up..."
    
    # Kill any remaining processes
    pkill -f "ros2 launch" || true
    pkill -f "gazebo" || true
    pkill -f "rosbridge" || true
    
    log_info "Cleanup completed"
}

# =============================================================================
# MAIN EXECUTION
# =============================================================================

main() {
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -m|--mode)
                DEPLOYMENT_MODE="$2"
                shift 2
                ;;
            -n|--name)
                ROBOT_NAME="$2"
                shift 2
                ;;
            -s|--slam)
                ENABLE_SLAM="true"
                shift
                ;;
            -N|--navigation)
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
    if [[ ! "$DEPLOYMENT_MODE" =~ ^(real|sim|mock)$ ]]; then
        log_error "Invalid deployment mode: $DEPLOYMENT_MODE"
        show_usage
        exit 1
    fi
    
    # Set up signal handlers
    trap cleanup_on_exit EXIT INT TERM
    
    log_info "Starting robot deployment script"
    log_info "Mode: ${DEPLOYMENT_MODE}"
    log_info "Robot Name: ${ROBOT_NAME}"
    log_info "SLAM Enabled: ${ENABLE_SLAM}"
    log_info "Navigation Enabled: ${ENABLE_NAVIGATION}"
    log_info "Log Level: ${LOG_LEVEL}"
    log_info "Dry Run: ${DRY_RUN}"
    
    # Execute deployment steps
    check_dependencies || exit 1
    validate_configuration || exit 1
    setup_environment || exit 1
    backup_configuration
    build_workspace || exit 1
    deploy_robot
}

# Execute main function if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi