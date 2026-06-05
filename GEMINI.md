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
*   **Frame Root:** Always use `base_footprint` as the root of the robot tree (REP 120).

## 🛡️ Model Behavior & Safety Mandates
These rules are **MANDATORY** for all AI agents to prevent technical debt and history loss.

1.  **Historical Integrity:** NEVER delete or overwrite existing entries in `SESSION_LOG.md`. Always append or surgically update specific parts. The log is our primary defense against repeating past errors.
2.  **The "Rule of Two":** If a configuration change or fix fails twice, **STOP**. Do not attempt a third variation. Re-read the source code, check `AGENTS.md`, and verify your assumptions.
3.  **Active but Silent Paradox:** If a node is "Active" but topic frequency is 0, it is always a naming or interface mismatch. Use `ros2 param dump` to verify internal state.
4.  **Clock First, TF Second:** Never debug "Unknown Frame" or TF errors until you have verified that the simulation clock is ticking and `use_sim_time: true` is globally set.

## 🧠 Multi-Session Continuity
To maintain context efficiency across models and sessions, always consult:
1.  `PROJECT_ANALYSIS.md`: **The Hub.** Current status, active tasks, and roadmap.
2.  `AGENTS.md`: **The Wisdom.** Lessons learned, pitfalls, and technical "gotchas."
3.  `SESSION_LOG.md`: **The Timeline.** Chronological history of trials and successes.
