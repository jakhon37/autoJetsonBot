# autoJetsonBot

Autonomous ROS 2 Foxy platform for Jetson Nano B01. Optimized for simulation (Gazebo/RViz) and physical hardware integration. This project is an advanced continuation of the [Autonomous ROS](https://github.com/jakhon37/autonomous_ROS) architecture, adapted for the Jetson ecosystem with modular packages and REP 120 compliance.

---
[<img src="assets/thubnl.png" width="50%">](https://youtu.be/JTg8ff2hSGM?si=UqfauM6vN_xyPFOV)
---

## 📊 Project Status: Navigation Active 🚀
The system is currently mapping-ready and navigation-stable in simulation. It features a standardized TF tree (`map -> odom -> base_footprint -> base_link`) and a unified configuration management system.

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Standardized | REP 120 compliant tree with `base_footprint` root projection. |
| **Control** | ✅ Active | Remapped to standard `/cmd_vel` for Nav2 and Web UI parity. |
| **Navigation** | ✅ Active | AMCL auto-localization enabled via corrected array-based `initial_pose`. |
| **UI** | ✅ Hardened | Deep internal health checks via `./robot.sh status`. |

---

## 🏗️ Hardware Architecture & Components

The physical robot is designed for high-torque mobility and precise environment sensing.

### **1. Core Compute & Control**
*   **Main Brain:** NVIDIA Jetson Nano B01 (4GB) – Handles ROS 2 stack, SLAM, and Vision.
*   **Microcontroller:** ESP32 / Arduino – Acts as the real-time serial bridge between ROS 2 and the motor drivers.
*   **Motor Driver:** L298N / Cytron MD10C – High-current DC motor control with PWM support.

### **2. Sensors & Feedback**
*   **Lidar:** RPLidar A1/A2 – 360-degree laser scanning for SLAM and obstacle avoidance.
*   **IMU:** MPU6050 (I2C) – Provides 6-DOF acceleration and angular velocity for EKF fusion.
*   **Odometry:** Hall-effect Encoders (3436 counts/rev) – Precise wheel displacement feedback.
*   **Vision:** Raspberry Pi Camera v2 / USB Cam – Object detection and visual servoing.

### **3. Physical Chassis Specs**
*   **Wheel Separation:** 0.18 m
*   **Wheel Radius:** 0.035 m
*   **Drive Type:** Differential Drive with a rear caster for stability.
*   **Power:** 12V Li-ion Battery pack with buck converters for 5V (Jetson/Sensors).

---

## 🚀 Software Workflow (Host Machine)

Manage the entire lifecycle using the unified `./robot.sh` script.

| Command | Action |
| :--- | :--- |
| `./robot.sh build` | Build the workspace inside the container. |
| `./robot.sh sim` | Start Simulation (Mapping Mode) + Web UI. |
| `./robot.sh nav` | Start Navigation Mode (Map Loading + Nav2). |
| `./robot.sh status` | Show container, port, and ROS node health. |
| `./robot.sh stop` | Robustly kill all processes and free ports. |

---

## 🗺️ Mapping & Navigation Steps

### **Phase 1: Environment Mapping**
1.  Launch simulation: `./robot.sh sim`.
2.  Open RViz or Web UI and drive the robot to explore the area.
3.  Save the map dynamically:
    ```bash
    ./robot.sh shell "ros2 run nav2_map_server map_saver_cli -f /autonomous_ROS/maps/current_map"
    ```

### **Phase 2: Autonomous Navigation**
1.  Configure your map in `src/jetson_bot_bringup/config/unified_robot_config.yaml`.
2.  Launch navigation: `./robot.sh nav`.
3.  Use **"2D Nav Goal"** in RViz to send the robot to a destination. The robot will automatically calculate the optimal path using the DWB Local Planner.

---

## 🧩 Modular Package Overview
*   **`jetson_bot_bringup`**: Central orchestration and global configuration.
*   **`jetson_bot_description`**: URDF models, Gazebo plugins, and mesh resources.
*   **`jetson_bot_gui`**: Web-based dashboard (`localhost:8000`) and telemetry server.
*   **`jetson_bot_slam`**: SLAM Toolbox parameters for online asynchronous mapping.
*   **`jetson_bot_navigation`**: Nav2 planner, controller, and behavior tree configs.
*   **`jetson_bot_imu`**: Python-based MPU6050 driver for real-time IMU data.

---

## 📜 Technical Memory
Consult these files for deeper technical context:
*   **[GEMINI.md](./GEMINI.md)**: Mandates and standard workflows.
*   **[PROJECT_ANALYSIS.md](./PROJECT_ANALYSIS.md)**: Current roadmap and blocker tracking.
*   **[AGENTS.md](./AGENTS.md)**: Technical "Wisdom" and hard-won bug fixes.
*   **[SESSION_LOG.md](./SESSION_LOG.md)**: Detailed history of all work sessions.

---

## License
Licensed under the MIT License. Based on the [Autonomous ROS](https://github.com/jakhon37/autonomous_ROS) project.

*For issues or contributions, please contact jakhon37@gmail.com*
