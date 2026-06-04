# autoJetsonBot - Project Instructions

This project is a ROS 2 Foxy-based autonomous mobile robot platform designed for Jetson Nano B01. It features a dockerized development environment, simulation capabilities with Gazebo and RViz2 (via VNC), and support for real hardware integration.

## 🚀 Primary Workflows (Host Machine)

| Command | Action |
| :--- | :--- |
| `./robot.sh build` | Build the workspace inside the container. |
| `./robot.sh sim` | Start headless simulation + Web UI (`http://localhost:8000`). |
| `./robot.sh robot` | Start nodes for physical hardware integration. |
| `./robot.sh stop` | Robustly kill all processes and free ports (8000, 9090, 5900). |
| `./robot.sh status` | Show container, port, and ROS node health. |

## 🏗️ Modular Architecture (Active Packages)

| Package | Responsibility |
| :--- | :--- |
| `jetson_bot_bringup` | Master orchestration, launch files, and global lifecycle. |
| `jetson_bot_description` | URDF/XACRO physical models and mesh resources. |
| `jetson_bot_gui` | Web Dashboard and Python-based web server. |
| `jetson_bot_slam` | SLAM Toolbox configurations for mapping. |
| `jetson_bot_navigation` | Nav2 planner, controller, and behavior tree configs. |
| `jetson_bot_imu` | MPU6050 I2C driver (Python). |

## 📐 Standard Robot Specs
*   **Wheel Separation:** 0.18 m
*   **Wheel Radius:** 0.035 m
*   **Encoder Resolution:** 3436 counts/rev
*   **Serial Port:** `/dev/ttyACM0` @ 115200 baud
*   **Frame Root:** Always use `base_link` (No `base_footprint`).

## 🧠 Multi-Session Continuity
To maintain context efficiency across models and sessions, always consult:
1.  `PROJECT_ANALYSIS.md`: **The Hub.** Current status, active tasks, and roadmap.
2.  `AGENTS.md`: **The Wisdom.** Lessons learned, pitfalls, and technical "gotchas."
3.  `SESSION_LOG.md`: **The Timeline.** Chronological history of trials and successes.
