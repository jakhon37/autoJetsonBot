#!/bin/bash
# setup_jetson.sh - Environment setup for autoJetsonBot on native Ubuntu (Jetson Nano)
# Run this once to install all required dependencies.

set -e

echo "🚀 Starting autoJetsonBot Native Setup..."

# 1. Update System
sudo apt update

# 2. Install Build Tools & Python Utilities
sudo apt install -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-serial \
    socat \
    nano \
    curl

# 3. Install ROS 2 Foxy Core & Desktop (Assuming ROS2 Foxy repo is already added)
# If not, please follow: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
sudo apt install -y ros-foxy-desktop

# 4. Install Navigation, SLAM & Sensor Fusion Stack
sudo apt install -y \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-slam-toolbox \
    ros-foxy-robot-localization \
    ros-foxy-imu-filter-madgwick \
    ros-foxy-rosbridge-server \
    ros-foxy-xacro \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-rplidar-ros \
    ros-foxy-ros2-control \
    ros-foxy-ros2-controllers \
    ros-foxy-gazebo-ros2-control \
    ros-foxy-tf2-tools \
    ros-foxy-image-transport-plugins \
    ros-foxy-rqt-image-view

    # ros-foxy-web-video-server \

# 5. Install Infrastructure for Headless VNC (Optional but recommended)
sudo apt install -y \
    xvfb \
    x11vnc \
    openbox \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx

# 6. Install Python Dependencies for Web UI
echo "🐍 Installing Python dependencies..."
pip3 install pymongo tornado psutil pyserial

# 7. Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo "✅ Setup Complete!"
echo "Next: Run 'colcon build --symlink-install' in your workspace."
