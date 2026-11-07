# autoJetsonBot - Project Instructions

This project is a ROS 2 autonomous mobile robot platform. It runs on a Jetson Xavier NX with Humble in Docker. It started on a Jetson Nano B01 with native Foxy, and those scripts are kept only for reference. It features a dockerized development environment, simulation capabilities with Gazebo and RViz2 (via VNC), and support for real hardware integration.

## 🚀 Primary Workflows (Host Machine)

| Command | Action |
| :--- | :--- |
| `./robot.sh build` | Build the workspace inside the container. |
| `./robot.sh sim` | Start the simulation (Gazebo and RViz over VNC) + Web UI (`http://localhost:8000`). |
| `./robot.sh robot` | Start nodes for physical hardware integration. |
| `./robot.sh stop` | Robustly kill all processes and free ports (8000, 9090, 5900, 8080). |
| `./robot.sh status` | Show container, port, and ROS node health. |

## 🏗️ Modular Architecture (Active Packages)

| Package | Responsibility |
| :--- | :--- |
| `jetson_bot_bringup` | Master orchestration, launch files, and global lifecycle. |
| `jetson_bot_description` | URDF/XACRO model built from boxes, cylinders and spheres (no meshes). |
| `jetson_bot_gui` | Web Dashboard and Python-based web server. |
| `jetson_bot_slam` | SLAM Toolbox configurations for mapping. |
| `jetson_bot_navigation` | Nav2 parameter files for Foxy and Humble. |
| `jetson_bot_imu` | Serial IMU bridge: turns the ESP32's `i` lines into `sensor_msgs/Imu`. Its I2C driver isn't launched. |
| `jetson_bot_diffdrive` | Python serial bridge to the ESP32-S3: wheel commands down, encoder and IMU lines up. |
| `jetson_bot_detection` | Object detection demo, not wired to a camera yet. |

## 📐 Standard Robot Specs
*   **Wheel Separation:** 0.212 m
*   **Wheel Radius:** 0.034 m
*   **Encoder Resolution:** 3436 counts/rev configured, but a hand test measured 344 to 387 per wheel turn, so recalibrate
*   **Serial Port:** ESP32 found by USB ID in `/dev/serial/by-id`, falling back to `/dev/ttyACM*`, @ 115200 baud
*   **Frame Root:** Always use `base_footprint` as the root of the robot tree (REP 120).

## 🛡️ Model Behavior & Safety Mandates
These rules are **MANDATORY** for all AI agents to prevent technical debt and history loss.

1.  **Historical Integrity:** NEVER delete or overwrite existing entries in `docs/SESSION_LOG.md`. Always append or surgically update specific parts. The log is our primary defense against repeating past errors.
2.  **The "Rule of Two":** If a configuration change or fix fails twice, **STOP**. Do not attempt a third variation. Re-read the source code, check `AGENTS.md`, and verify your assumptions.
3.  **Active but Silent Paradox:** If a node is "Active" but topic frequency is 0, it is always a naming or interface mismatch. Use `ros2 param dump` to verify internal state.
4.  **Clock First, TF Second:** Never debug "Unknown Frame" or TF errors until you have verified that the simulation clock is ticking and `use_sim_time: true` is set across the sim nodes. Hardware nodes run with `use_sim_time: false`.

## 🧠 Multi-Session Continuity
To maintain context efficiency across models and sessions, always consult:
1.  `docs/PROJECT_ANALYSIS.md`: **The Hub.** Current status, active tasks, and roadmap.
2.  `AGENTS.md`: **The Wisdom.** Lessons learned, pitfalls, and technical "gotchas."
3.  `docs/SESSION_LOG.md`: **The Timeline.** Chronological history of trials and successes.
