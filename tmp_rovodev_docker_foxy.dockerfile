# Dockerfile.foxy-production - Production ready Foxy environment for Jetson
# Long-term solution for consistent deployment across development and production

FROM ros:foxy-ros-base-focal

# =============================================================================
# SYSTEM DEPENDENCIES & TOOLS
# =============================================================================

# Install essential tools and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    # Python dependencies
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-serial \
    python3-numpy \
    python3-opencv \
    # ROS2 Foxy packages
    ros-foxy-desktop \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-slam-toolbox \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-rosbridge-server \
    ros-foxy-xacro \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-rplidar-ros \
    ros-foxy-image-transport-plugins \
    ros-foxy-rqt-image-view \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-ros2-control \
    ros-foxy-ros2-controllers \
    ros-foxy-gazebo-ros2-control \
    ros-foxy-controller-manager \
    ros-foxy-diff-drive-controller \
    ros-foxy-joint-state-broadcaster \
    # Hardware interface dependencies
    ros-foxy-hardware-interface \
    ros-foxy-pluginlib \
    # Diagnostic tools
    ros-foxy-diagnostic-aggregator \
    ros-foxy-diagnostic-updater \
    # Network tools
    net-tools \
    iputils-ping \
    # Text editors
    nano \
    vim \
    # Cleanup
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# PYTHON DEPENDENCIES
# =============================================================================

# Install Python packages for robot functionality
RUN pip3 install --no-cache-dir \
    pyserial \
    smbus2 \
    opencv-python \
    numpy \
    scipy \
    matplotlib \
    pyyaml \
    jsonschema

# =============================================================================
# ROS2 WORKSPACE SETUP
# =============================================================================

# Create workspace directory
WORKDIR /autonomous_ros_ws

# Copy source code
COPY src/ src/
COPY *.md ./
COPY *.sh ./
COPY *.py ./

# =============================================================================
# DIFFDRIVE_ARDUINO INTEGRATION
# =============================================================================

# Clone and build diffdrive_arduino for Foxy
RUN cd src && \
    git clone -b foxy https://github.com/joshnewans/diffdrive_arduino.git && \
    cd ..

# =============================================================================
# BUILD WORKSPACE
# =============================================================================

# Initialize rosdep and install dependencies
RUN rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# =============================================================================
# ENVIRONMENT CONFIGURATION
# =============================================================================

# Setup environment variables
ENV ROS_DISTRO=foxy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV PYTHONPATH=/autonomous_ros_ws/install/lib/python3.8/site-packages:$PYTHONPATH

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS2 Foxy\n\
source /opt/ros/foxy/setup.bash\n\
\n\
# Source workspace\n\
if [ -f /autonomous_ros_ws/install/setup.bash ]; then\n\
    source /autonomous_ros_ws/install/setup.bash\n\
fi\n\
\n\
# Set up environment for Jetson\n\
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}\n\
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}\n\
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/fastdds.xml}\n\
\n\
# Hardware permissions\n\
if [ -c /dev/ttyACM0 ]; then\n\
    chmod 666 /dev/ttyACM0\n\
fi\n\
if [ -c /dev/ttyUSB0 ]; then\n\
    chmod 666 /dev/ttyUSB0\n\
fi\n\
\n\
# Execute command\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# =============================================================================
# FASTDDS CONFIGURATION FOR JETSON
# =============================================================================

# Create FastDDS configuration for better performance on Jetson
RUN echo '<?xml version="1.0" encoding="UTF-8" ?>\n\
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">\n\
    <participant profile_name="default_participant" is_default_profile="true">\n\
        <rtps>\n\
            <name>default_participant</name>\n\
            <builtin>\n\
                <discovery_config>\n\
                    <discoveryProtocol>SIMPLE</discoveryProtocol>\n\
                    <use_liveliness_qos>true</use_liveliness_qos>\n\
                </discovery_config>\n\
                <avoid_builtin_multicast>false</avoid_builtin_multicast>\n\
            </builtin>\n\
            <defaultUnicastLocatorList>\n\
                <locator>\n\
                    <udpv4>\n\
                        <address>0.0.0.0</address>\n\
                    </udpv4>\n\
                </locator>\n\
            </defaultUnicastLocatorList>\n\
        </rtps>\n\
    </participant>\n\
</profiles>' > /fastdds.xml

# =============================================================================
# RUNTIME CONFIGURATION
# =============================================================================

# Create runtime directories
RUN mkdir -p /var/log/robot && \
    mkdir -p /autonomous_ros_ws/maps && \
    mkdir -p /autonomous_ros_ws/config

# Set proper permissions
RUN chmod -R 755 /autonomous_ros_ws && \
    chmod 755 /entrypoint.sh

# =============================================================================
# HEALTH CHECK
# =============================================================================

# Add health check for container monitoring
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD ros2 node list || exit 1

# =============================================================================
# FINAL CONFIGURATION
# =============================================================================

# Expose ports for web interface and rosbridge
EXPOSE 8000 9090

# Set working directory
WORKDIR /autonomous_ros_ws

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command - can be overridden
CMD ["ros2", "launch", "my_robot_launch", "unified_robot_launch.py"]

# =============================================================================
# BUILD METADATA
# =============================================================================

# Add labels for container management
LABEL maintainer="RovoDev Assistant"
LABEL description="Autonomous Jetson Robot - ROS2 Foxy Production Environment"
LABEL version="2.0.0"
LABEL ros_distro="foxy"
LABEL target_platform="jetson_nano"