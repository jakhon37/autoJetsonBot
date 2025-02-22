#!/bin/sh

# Add a flag to rebuild the image if needed
REBUILD=false
IMAGE_N="autonomous_ros_project3:latest"
CONTAINER_N="auto_ros3"
WORK_SPACE_N="autonomous_ROS"

# Parse command-line arguments
for arg in "$@"; do
    if [ "$arg" = "--rebuild" ]; then
        REBUILD=true
    fi
done

# Build the Docker image if it doesn't exist or if --rebuild is specified
if [ "$REBUILD" = true ] || ! docker images --format '{{.Repository}}:{{.Tag}}' \
   | grep -q "^$IMAGE_N\$"; then
    echo "Building the Docker image..."
    docker build -t "$IMAGE_N" .
fi

# Check if the container already exists
if ! docker ps -a | grep -q "$CONTAINER_N"; then
    echo "Creating and starting the Docker container..."
    docker run -it --privileged \
        --hostname $(hostname) \
        --network host \
        --name "$CONTAINER_N" \
        -v "$(pwd)":/$WORK_SPACE_N \
        -e DISPLAY="$DISPLAY" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        "$IMAGE_N" /bin/bash
else
    echo "Starting the existing Docker container..."
    docker start -ai "$CONTAINER_N"
fi

        # --gpus all 
        # -p 8001:8001 -p 7860:7860 \
        # --network=host \
# 