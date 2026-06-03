#!/bin/sh
# runrosenv.sh — create/start the auto_ros_foxy container
# Port 5900 included for VNC (./robot.sh gui)

REBUILD=false
DISTRO="foxy"
DOCKERFILE="Dockerfile.foxy"
IMAGE_N="auto_ros:$DISTRO"
CONTAINER_N="auto_ros_$DISTRO"
WORK_SPACE_N="autonomous_ROS"
NETWORK_N="ros2_network"

for arg in "$@"; do
    [ "$arg" = "--rebuild" ] && REBUILD=true
done

# Create network with multicast if needed
if ! docker network ls | grep -q "$NETWORK_N"; then
    echo "Creating Docker network '$NETWORK_N'..."
    docker network create --driver bridge \
        --opt com.docker.network.bridge.enable_ip_multicast=true \
        "$NETWORK_N"
else
    echo "Network '$NETWORK_N' already exists."
fi

# Build image if needed or --rebuild requested
if [ "$REBUILD" = true ] || ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^$IMAGE_N\$"; then
    echo "Building image $IMAGE_N..."
    docker build -t "$IMAGE_N" -f "$DOCKERFILE" .
fi

# Create or start container
if ! docker ps -a | grep -q "$CONTAINER_N"; then
    echo "Creating container $CONTAINER_N..."
    docker run -it --privileged \
        -p 8000:8000 \
        -p 9090:9090 \
        -p 5900:5900 \
        --hostname "$(hostname)" \
        --network "$NETWORK_N" \
        --name "$CONTAINER_N" \
        -v "$(pwd):/$WORK_SPACE_N" \
        -e ROS_DOMAIN_ID=0 \
        "$IMAGE_N" /bin/bash
else
    echo "Starting existing container $CONTAINER_N..."
    docker start -ai "$CONTAINER_N"
fi