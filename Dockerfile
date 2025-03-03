FROM ros:humble-ros-core-jammy

# Install essential tools including add-apt-repository utility
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip\
    python3-pymongo\
 && rm -rf /var/lib/apt/lists/*

# Enable the Universe repository
RUN add-apt-repository universe && apt-get update

# Bootstrap rosdep
RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

# Setup colcon mixin and metadata
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
 && colcon mixin update \
 && colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
 && colcon metadata update

# Install ROS2 packages (desktop, navigation, Gazebo integration) and nano
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop* \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros-gz \
    ros-humble-rosbridge-server \
    nano \
 && rm -rf /var/lib/apt/lists/*

# Add ROS setup to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc


#sudo apt-get install ros-${ROS_DISTRO}-ros-gz
# ros-humble-xacro \
# pip install pyserial
