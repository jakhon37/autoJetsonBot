
<!-- ```markdown -->
# Autonomous ROS Project

This project integrates multiple sensors and actuators on a mobile robot platform to achieve autonomous navigation. The system runs on a Raspberry Pi 4B (2GB) and combines motor control, encoder feedback, LIDAR scanning, SLAM, navigation, camera vision, and line tracking.
---
[<img src="assets/thubnl.png" width="50%">](https://youtu.be/JTg8ff2hSGM?si=UqfauM6vN_xyPFOV)
---
> **Status Overview:**  
> - **Completed:** Motor control node, encoder node, and web GUI for remote control  
> - **In Progress:** RP‑Lidar scan node, SLAM integration, navigation stack, camera & object detection, and line tracking

![project image](assets/thub.png)
---

## Table of Contents

- [Hardware Components](#hardware-components)
- [Circuit Connection / Wiring Details](#circuit-connection--wiring-details)
- [Software Overview and Nodes](#software-overview-and-nodes)
  - [Dual Motor Control Node](#motor-control-node)
  - [Encoder Node](#encoder-node)
  - [Web GUI Node](#web-gui-node)
  - [RP‑Lidar Scan Node (In Progress)](#rp-lidar-scan-node-in-progress)
  - [SLAM Node (In Progress)](#slam-node-in-progress)
  - [Navigation Node (In Progress)](#navigation-node-in-progress)
  - [Camera / Object Detection Node (In Progress)](#camera--object-detection-node-in-progress)
  - [Line Tracking Node (In Progress)](#line-tracking-node-in-progress)
- [Creating New Nodes](#creating-new-nodes)
- [To-Do List](#to-do-list)
- [Installation and Launch Instructions](#installation-and-launch-instructions)
- [License](#license)

---

## Hardware Components

The following devices and components are used in this project:

- **Raspberry Pi 4B (2GB)**  
  - Runs Ubuntu 22.04 and ROS 2 Humble
- **TB6612 Dual Motor Driver**  
  - Controls two DC motors via PWM and direction signals.
- **2 × JGA25-370 DC 6V Geared Motors with Encoders and Wheels**  
  - Provide propulsion and motion feedback.
- **7.2V 4500mAh Li-ion Battery**  
  - Powers the motors (connected to the motor driver’s VM and GND outputs).
- **5V 5000mAh Power Bank**  
  - Powers the Raspberry Pi via its Type-C connector.
- **RP‑Lidar A1**  
  - Provides 2D LaserScan data on the `/scan` topic for SLAM and obstacle detection.
- **Raspberry Pi Camera**  
  - For vision-based tasks such as object detection and line tracking.
- **Extra Balance Wheel**  
  - Used to help maintain stability (mechanical integration only).
- **XL6015 BUST CONVERTOR**
  - If your motor or motor driver could not handle your battary output (e.g., `12v battery`)
---

## Circuit Connection / Wiring Details

Below are the wiring details for connecting the components:

### Raspberry Pi 4B Wiring

- **Motor Driver (TB6612FNG) Connections:**
  - **PWMA (Left Motor PWM):** GPIO18 → TB6612FNG (PWMA)
  - **AI1 (Left Motor Direction):** GPIO23 → TB6612FNG (AI1)
  - **AI2 (Left Motor Direction):** GPIO24 → TB6612FNG (AI2)
  - **PWMB (Right Motor PWM):** GPIO13 → TB6612FNG (PWMB)
  - **BI1 (Right Motor Direction):** GPIO19 → TB6612FNG (BI1)
  - **BI2 (Right Motor Direction):** GPIO26 → TB6612FNG (BI2)
  - **STBY, VCC (Enable & Logic Power):** 5V → TB6612FNG (STBY, VCC)
  - **GND:** Connect to TB6612FNG (GND)

- **Motor with Encoder (JGA25-371dc) Connections:**
  - **Motor Power:**
    - **Motor power +:** Connected to TB6612FNG outputs (A01, B01)
    - **Motor power -:** Connected to TB6612FNG outputs (A02, B02)
  - **Encoder Power:**
    - **Encoder power +:** 3V3 → JGA25-371dc (encoder power +)
    - **Encoder power -:** GND → JGA25-371dc (encoder power -)
  - **Encoder Signals:**
    - **Channel 1 / A (C1/A):** Connected to Raspberry Pi (GPIO5 and/or GPIO17)
    - **Channel 2 / B (C2/B):** Connected to Raspberry Pi (GPIO6 and GPIO27)

- **Type-C Connector (Power):**
  - **Raspberry Pi Power Input:** Connected to the 5V, 5000mAh power bank.

### TB6612FNG Motor Driver Specific Wiring

- **Inputs from Raspberry Pi:**
  - **PWMA:** GPIO18 (Left motor speed)
  - **AI1:** GPIO23 (Left motor direction)
  - **AI2:** GPIO24 (Left motor direction)
  - **PWMB:** GPIO13 (Right motor speed)
  - **BI1:** GPIO19 (Right motor direction)
  - **BI2:** GPIO26 (Right motor direction)
- **Power Connections:**
  - **STBY, VCC:** 5V from Raspberry Pi (enables the driver)
  - **VM:** Connected to 7.2V 4500mAh Li-ion battery (motor power supply)
  - **GND:** Common ground with Raspberry Pi and battery
- **Outputs to Motors:**
  - **A01, B01:** Connect to the motor’s power + terminal
  - **A02, B02:** Connect to the motor’s power – terminal

---

## Software Overview and Nodes

### Motor Control Node

- **Status:** Complete  
<!-- /home/ubuntu/my_space/autonomous_ROS/src/dual_motor/dual_motor/dual_motor_controller.py -->
- **Location:** `src/dual_motor/dual_motor/dual_motor_controller.py`  
- **Description:**  
  Subscribes to a topic (e.g., `/cmd_vel`) to control motor speed and direction via PWM outputs to the TB6612FNG.
- **Key Topics:**
  - **Subscribed:** `/cmd_vel` (`std_msgs/Float32` or `geometry_msgs/Twist`)
  - **Published:** (Optional diagnostic messages)
- **Creation Steps:**
  1. Create package:  
     ```bash
     ros2 pkg create --build-type ament_python dual_motor --dependencies rclpy std_msgs
     ```
  2. Implement `dual_motor_controller.py` using RPi.GPIO.
  3. Update `package.xml` and `setup.py` if needed.

---

### Encoder Node

- **Status:** Complete  
- **Location:** `src/motor_encoder/motor_encoder/dual_encoder_node.py`  
- **Description:**  
  Reads encoder signals (using GPIO interrupts) from the motors and publishes RPM or position data.
- **Key Topics:**
  - **Published:** `/left_motor_rpm`, `right_motor_rpm` and (`std_msgs/Float32`) fir each side.
- **Creation Steps:**
  1. Create package:  
     ```bash
     ros2 pkg create --build-type ament_python motor_encoder --dependencies rclpy sensor_msgs std_msgs
     ```
  2. Implement `dual_encoder_node.py` using GPIO libraries.
  3. Update `package.xml` and `setup.py`.

---

### Web GUI Node

- **Status:** Complete  
- **Location:** `src/web_gui/index.html`  
- **Description:**  
  An HTML/JavaScript interface (using roslib.js) that allows remote control and monitoring by publishing motor commands and displaying sensor feedback.
- **Key Topics:**
  - **Published:** `/cmd_vel`.
  - **Subscribed:** Topics for sensor feedback (e.g., encoder RPM)
- **Creation Steps:**  
# sudo apt install ros-humble-rosbridge-server
  Install ``
  Place the HTML and JavaScript file (`roslib.min.js`) together; serve them via a local web server or use them directly.

---

### RP‑Lidar Scan Node (In Progress)

- **Status:** In Progress  
- **Location:** Typically provided by an external package (e.g., `ros-humble-rplidar-ros`).
- **Description:**  
  Reads LIDAR data from the RP‑Lidar A1 and publishes LaserScan messages on `/scan`.
- **Key Topics:**
  - **Published:** `/scan` (`sensor_msgs/LaserScan`)
- **Creation Steps:**  
  Install the driver package ` sudo apt install ros-humble-rplidar-ros` and create a node on launch file (e.g., `launch/autonomous_car_launch.py`).
     ```bash
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }]
        ),
     ```
---

### SLAM Node (In Progress)

- **Status:** In Progress  
- **Location:** `src/my_robot_slam/` (using slam_toolbox)  
- **Description:**  
  Uses `async_slam_toolbox_node` or `sync_slam_toolbox_node` to build a map from LIDAR data.
- **Key Topics:**
  - **Subscribed:** `/scan`
  - **Published:** `/map` (`nav_msgs/OccupancyGrid`), and relevant transforms.
- **Creation Steps:**  
  1. Create a configuration file (`slam_toolbox_config.yaml`) in `config/`.
  2. Create a node on launch file to start the SLAM node.
  3. Install SLAM package `sudo apt install ros-humble-slam-toolbox`

---

### Navigation Node (In Progress)

- **Status:** In Progress  
- **Location:** Typically in a package like `my_robot_nav` or using Nav2 (Navigation2 stack).
- **Description:**  
  Provides autonomous navigation using global and local planners, costmaps, and goal-setting.
- **Key Topics:**
  - **Subscribed:** `/map`, `/scan`, `/odom`
  - **Published:** `/cmd_vel` for motor commands.
- **Creation Steps:**  
  1. Create navigation parameters (e.g., `nav2_params.yaml`).
  2. Create a launch file for the Navigation2 stack.

---

### Camera / Object Detection Node (In Progress)

- **Status:** In Progress  
- **Location:** `src/object_detection/` or `src/camera_node/`  
- **Description:**  
  Captures images using the Raspberry Pi camera (or USB camera) with OpenCV, publishes on `/camera/image_raw`, and optionally performs object detection.
- **Key Topics:**
  - **Published:** `/camera/image_raw` (`sensor_msgs/Image`)
- **Creation Steps:**  
  1. Create a package with dependencies:  
     ```bash
     ros2 pkg create --build-type ament_python camera_node --dependencies rclpy sensor_msgs cv_bridge
     ```
  2. Implement the camera node using OpenCV and cv_bridge.

---

### Line Tracking Node (In Progress)

- **Status:** In Progress  
- **Location:** `src/line_tracking/line_tracking/line_tracking_node.py`  
- **Description:**  
  Uses camera input (or other sensors) to detect and follow a line on the ground.
- **Key Topics:**
  - **Subscribed:** `/camera/image_raw`
  - **Published:** `/cmd_vel` to control the robot.
- **Creation Steps:**  
  1. Create a package:  
     ```bash
     ros2 pkg create --build-type ament_python line_tracking --dependencies rclpy sensor_msgs cv_bridge geometry_msgs
     ```
  2. Implement image processing and motor command generation using OpenCV.

---

## Creating New Nodes

To add additional nodes:

1. **Create a New Package:**  
   ```bash
   ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy <other_dependencies>
   ```
2. **Write the Node Script:**  
   - Place your Python script (with a proper shebang, e.g., `#!/usr/bin/env python3`) in the `src/<package_name>/` directory.
   - Make it executable with:  
     ```bash
     chmod +x src/<package_name>/<node_script>.py
     ```
3. **Update `package.xml` and `setup.py`:**  
   - Ensure all dependencies are declared.
   - Include the node in the entry points if desired.
4. **Build the Package:**  
   ```bash
   colcon build --packages-select <package_name>
   source install/setup.bash
   ```
5. **(Optional) Create a Launch File:**  
   - Use Python launch files to start your node(s) if needed.

---

## To-Do List

- [x] Motor control node  
- [x] Encoder node  
- [x] Web GUI  
- [ ] RP‑Lidar scan node  
- [ ] SLAM integration (using slam_toolbox)  
- [ ] Navigation stack (Nav2)  
- [ ] Camera / Object detection node  
- [ ] Line tracking node  

---

## Installation and Launch Instructions

1. **Build the Workspace:**  
   From the workspace root (`autonomous_ROS/`):
   ```bash
   colcon build
   source install/setup.bash
   ```

2. **Launch All Nodes:**  
   Use the main launch file (e.g., `autonomous_car_launch.py` in `launch/`) to start the desired nodes. Example:
   ```bash
   ros2 launch my_robot_slam autonomous_car_launch.py
   ```
   This launch file includes (or will include) nodes for motor control, encoder, RP‑Lidar, SLAM, etc.

3. **Remote Visualization (Optional):**  
   If you want to visualize SLAM data or the map, run RViz on a separate machine and subscribe to the `/map` and `/scan` topics.



<!-- [![Watch the Video](<img src="/home/jakhon37/myspace/robotics/autonomous_ROS/assets/thubnl.png" width="50%">)](https://youtu.be/JTg8ff2hSGM?si=UqfauM6vN_xyPFOV) -->
<!-- [![Watch the Video](https://img.youtu.be/JTg8ff2hSGM?si=UqfauM6vN_xyPFOV/0.jpg)](https://youtu.be/JTg8ff2hSGM?si=UqfauM6vN_xyPFOV) -->

---

## Wiring and Circuit Connection Guide

**Overview:**  
The system integrates the Raspberry Pi 4B, TB6612 Dual Motor Driver, two DC geared motors with encoders, an RP‑Lidar A1, a Raspberry Pi camera, and extra hardware for balance. Use the following wiring details as a reference.

### Raspberry Pi 4B Connections

- **Motor Driver (TB6612FNG):**
  - **PWMA:** GPIO18  
  - **AI1:** GPIO23  
  - **AI2:** GPIO24  
  - **PWMB:** GPIO13  
  - **BI1:** GPIO19  
  - **BI2:** GPIO26  
  - **STBY & VCC:** 5V  
  - **GND:** Common ground

- **Motor with Encoder (JGA25-371dc):**
  - **Motor Power +:** Connect to TB6612FNG outputs A01/B01  
  - **Motor Power -:** Connect to TB6612FNG outputs A02/B02  
  - **Encoder Power +:** 3V3  
  - **Encoder Power -:** GND  
  - **Encoder Signal C1/A:** Connect to GPIO5 and/or GPIO17  
  - **Encoder Signal C2/B:** Connect to GPIO6 and/or GPIO27

- **Power:**
  - **Raspberry Pi:** Powered via a 5V 5000mAh power bank through the Type-C connector.
  - **Motor Power:** Provided by a 7.2V 4500mAh Li-ion battery connected to the TB6612FNG VM and GND outputs.

### TB6612FNG Motor Driver Connections

- **Inputs (from Raspberry Pi):**
  - **PWMA (Left Motor):** GPIO18  
  - **AI1 (Left Motor):** GPIO23  
  - **AI2 (Left Motor):** GPIO24  
  - **PWMB (Right Motor):** GPIO13  
  - **BI1 (Right Motor):** GPIO19  
  - **BI2 (Right Motor):** GPIO26
- **Power and Enable:**
  - **STBY and VCC:** Connected to 5V from the Raspberry Pi  
  - **VM:** Connected to the 7.2V Li-ion battery  
  - **GND:** Common ground (Raspberry Pi, motor driver, battery)
- **Outputs (to Motors):**
  - **A01, B01:** Motor power +  
  - **A02, B02:** Motor power -

### Additional Components

- **RP‑Lidar A1:**  
  - Connect via USB or serial (e.g., `/dev/ttyUSB0`), and configure in the driver launch file.
- **Raspberry Pi Camera:**  
  - Connect to the dedicated camera interface; configure using OpenCV in your camera node.
- **Balance Wheel:**  
  - Mount mechanically to help stabilize the robot.

---

## License

This project is provided under the MIT License. Feel free to modify, distribute, or use this code for personal, educational, or commercial purposes, as long as proper attribution is given.

---

*For any issues or contributions, please contact jakhon37@gmail.com*
```
