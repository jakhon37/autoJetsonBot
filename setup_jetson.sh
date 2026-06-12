#!/bin/bash
# setup_jetson.sh - Environment setup for autoJetsonBot on native Ubuntu (Jetson Nano)
# Optimized for ARM64 (Jetson Nano) stability.

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

# 3. Install ROS 2 Foxy Core & Desktop
sudo apt install -y ros-foxy-desktop

# 4. Install CORE Navigation & Sensor Fusion
echo "📦 Installing Core ROS 2 packages..."
sudo apt install -y \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-nav2-msgs \
    ros-foxy-slam-toolbox \
    ros-foxy-robot-localization \
    ros-foxy-imu-filter-madgwick \
    ros-foxy-rosbridge-server \
    ros-foxy-xacro \
    ros-foxy-rplidar-ros \
    ros-foxy-ros2-control \
    ros-foxy-ros2-controllers \
    ros-foxy-tf2-tools

# 5. Install OPTIONAL UI/Camera packages
echo "📸 Attempting to install optional UI/Camera packages..."
sudo apt install -y \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-image-transport-plugins \
    ros-foxy-rqt-image-view || echo "⚠️ Some UI packages not found."

# 6. Hardware Permissions
echo "🔌 Configuring hardware permissions..."
sudo usermod -a -G dialout $USER
echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-lidar.rules
echo 'KERNEL=="ttyACM*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-esp32.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 7. Install VNC Stack
sudo apt install -y xvfb x11vnc openbox mesa-utils libgl1-mesa-dri libgl1-mesa-glx

# 8. Python Dependencies
pip3 install pymongo tornado psutil pyserial

# 9. rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo "✅ Setup Complete!"
