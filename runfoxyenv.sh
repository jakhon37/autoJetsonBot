# !/bin/bash
xhost +local:docker
# DISTRO="foxy"
# IMAGE_N="osrf/ros:foxy-desktop"
# CONTAINER_N="ros_foxy"
# WORK_SPACE_N="autonomous_ROS"
# NETWORK_N="ros2_network"

# if ! docker network ls | grep -q "$NETWORK_N"; then
#     echo "Creating custom Docker network '$NETWORK_N' with multicast support..."
#     docker network create --driver bridge \
#         --opt com.docker.network.bridge.enable_ip_multicast=true \
#         "$NETWORK_N"
# else
#     echo "Docker network '$NETWORK_N' already exists."
# fi

# if ! docker ps -a | grep -q "$CONTAINER_N"; then
#     echo "Creating and starting the Docker container..."
#     docker run -it --privileged \
#         --hostname $(hostname) \
#         --network "$NETWORK_N" \
#         --name "$CONTAINER_N" \
#         -v "$(pwd)":/$WORK_SPACE_N \
#         -v /tmp/.X11-unix:/tmp/.X11-unix \
#         -v /dev:/dev \
#         -v /etc/localtime:/etc/localtime:ro \
#         -v $HOME/.Xauthority:/root/.Xauthority:rw \
#         -e DISPLAY=$DISPLAY \
#         -e ROS_DOMAIN_ID=0 \
#         -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
#         -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
#         -v ~/fastdds.xml:/fastdds.xml \
#         --device=/dev/ttyACM0:/dev/ttyACM0 \
#         --device=/dev/ttyUSB0:/dev/ttyUSB0 \
#         "$IMAGE_N" /bin/bash
# else
#     echo "Starting the existing Docker container..."
#     docker start -ai "$CONTAINER_N"
# fi



# docker run -it \
#     --net=host -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp --name ros_foxy osrf/ros:foxy-desktop bash
# docker run -it \
#     --net=host -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
#     --name ros_foxy \
#     -v /dev:/dev \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v /etc/localtime:/etc/localtime:ro \
#     -v $HOME/.Xauthority:/root/.Xauthority:rw \
#     osrf/ros:foxy-desktop bash


# docker run -it --net=host -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml -v ~/fastdds.xml:/fastdds.xml --name ros_foxy osrf/ros:foxy-desktop bash

# docker run -it \
#     --net=host -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
#     -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
#     -v ~/fastdds.xml:/fastdds.xml \
#     -v /dev:/dev \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v /etc/localtime:/etc/localtime:ro \
#     -v $HOME/.Xauthority:/root/.Xauthority:rw \
#     --name ros_foxy2 \
#     osrf/ros:foxy-desktop bash


#!/bin/bash
xhost +local:docker
DISTRO="foxy"
IMAGE_N="osrf/ros:foxy-desktop"
CONTAINER_N="ros_foxy"
WORK_SPACE_N="autonomous_ROS"

# Start or create the Docker container
if ! docker ps -a | grep -q "$CONTAINER_N"; then
    echo "Creating and starting the Docker container..."
    docker run -it --privileged \
        --hostname $(hostname) \
        --net=host \
        --name "$CONTAINER_N" \
        -v "$(pwd)":/$WORK_SPACE_N \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /dev:/dev \
        -v /etc/localtime:/etc/localtime:ro \
        -v $HOME/.Xauthority:/root/.Xauthority:rw \
        -e DISPLAY=$DISPLAY \
        -e ROS_DOMAIN_ID=0 \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
        -v ~/fastdds.xml:/fastdds.xml \
        --device=/dev/ttyACM0:/dev/ttyACM0 \
        --device=/dev/ttyUSB0:/dev/ttyUSB0 \
        "$IMAGE_N" /bin/bash
else
    echo "Starting the existing Docker container..."
    docker start -ai "$CONTAINER_N"
fi
