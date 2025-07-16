#!/bin/sh
xhost +local:docker
# Add a flag to rebuild the image if needed
REBUILD=false
DISTRO="foxy"
DOCKERFILE="Dockerfile.foxy"
IMAGE_N="auto_ros:$DISTRO"
CONTAINER_N="auto_ros_$DISTRO"
WORK_SPACE_N="autonomous_ROS"
NETWORK_N="ros2_network"

# Parse command-line arguments
for arg in "$@"; do
    if [ "$arg" = "--rebuild" ]; then
        REBUILD=true
    fi
done

# Create a custom Docker network with multicast enabled if it doesn’t exist
if ! docker network ls | grep -q "$NETWORK_N"; then
    echo "Creating custom Docker network '$NETWORK_N' with multicast support..."
    docker network create --driver bridge \
        --opt com.docker.network.bridge.enable_ip_multicast=true \
        "$NETWORK_N"
else
    echo "Docker network '$NETWORK_N' already exists."
fi

# Build the Docker image if it doesn’t exist or if --rebuild is specified
if [ "$REBUILD" = true ] || ! docker images --format '{{.Repository}}:{{.Tag}}' \
   | grep -q "^$IMAGE_N\$"; then
    echo "Building the Docker image..."
    docker build -t "$IMAGE_N" -f "$DOCKERFILE" .
fi

# Check if the container already exists
if ! docker ps -a | grep -q "$CONTAINER_N"; then
    echo "Creating and starting the Docker container..."
    docker run -it --privileged \
        --hostname $(hostname) \
        --network "$NETWORK_N" \
        --name "$CONTAINER_N" \
        -v "$(pwd)":/$WORK_SPACE_N \
        -e DISPLAY="$DISPLAY" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e ROS_DOMAIN_ID=0 \
        --device=/dev/ttyACM0:/dev/ttyACM0 \
        --device=/dev/ttyUSB0:/dev/ttyUSB0 \
        "$IMAGE_N" /bin/bash
else
    echo "Starting the existing Docker container..."
    docker start -ai "$CONTAINER_N"
fi

        # --gpus all 
        # -p 8001:8001 -p 7860:7860 \
        # --network=host \
# export ROS_DOMAIN_ID=0
# xhost +local:docker
# 