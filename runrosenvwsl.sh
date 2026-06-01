#!/bin/bash
xhost +local:docker

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

# Network
if ! docker network ls | grep -q "$NETWORK_N"; then
    echo "Creating Docker network '$NETWORK_N'..."
    docker network create --driver bridge \
        --opt com.docker.network.bridge.enable_ip_multicast=true \
        "$NETWORK_N"
else
    echo "Docker network '$NETWORK_N' already exists."
fi

# Build
if [ "$REBUILD" = true ] || ! docker images --format '{{.Repository}}:{{.Tag}}' \
   | grep -q "^$IMAGE_N\$"; then
    echo "Building Docker image..."
    docker build -t "$IMAGE_N" -f "$DOCKERFILE" . || { echo "Build failed!"; exit 1; }
fi

# Device args — faqat mavjud bo'lsa qo'sh
DEVICE_ARGS=""
[ -e /dev/ttyACM0 ] && DEVICE_ARGS="$DEVICE_ARGS --device=/dev/ttyACM0:/dev/ttyACM0"
[ -e /dev/ttyUSB0 ] && DEVICE_ARGS="$DEVICE_ARGS --device=/dev/ttyUSB0:/dev/ttyUSB0"

# Run or start
if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_N}$"; then
    echo "Creating and starting container..."
    docker run -it --privileged \
        -p 8000:8000 -p 9090:9090 \
        --hostname "$(hostname)" \
        --network "$NETWORK_N" \
        --name "$CONTAINER_N" \
        -v "$(pwd):/$WORK_SPACE_N" \
        -e DISPLAY="${DISPLAY:-:0}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e ROS_DOMAIN_ID=0 \
        $DEVICE_ARGS \
        "$IMAGE_N" /bin/bash
else
    echo "Starting existing container..."
    docker start -ai "$CONTAINER_N"
fi