# Use an official ROS 2 Humble base image
FROM ros:humble

# Update and install necessary system packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    curl \
    git \
    python3-opencv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Use Bash as the default shell for subsequent RUN commands.
SHELL ["/bin/bash", "-c"]

# Install pip packages if needed (e.g., for RPi.GPIO)
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3 get-pip.py && \
    rm get-pip.py
RUN pip3 install RPi.GPIO

# Initialize rosdep (remove the default file first to avoid errors)
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update

# Install rosbridge packages using apt. Since pre-built packages are available,
# we install rosbridge-suite and rosbridge-server.
RUN apt-get update && apt-get install -y \
    ros-humble-rosbridge-suite \
    ros-humble-rosbridge-server

# Set the working directory for your ROS 2 workspace
WORKDIR /autonomous_ROS

# Copy your project files into the container
COPY . /autonomous_ROS

# Build your ROS 2 workspace (adjust or disable RViz plugin if needed)
RUN . /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DBUILD_RVIZ_PLUGIN=OFF

# Set the container's entrypoint to launch your unified launch file
CMD ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && . /autonomous_ROS/install/setup.bash && ros2 launch my_robot_launch launchpr.py"]
