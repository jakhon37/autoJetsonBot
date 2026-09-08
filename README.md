# autoJetsonBot

ROS 2 Foxy mobile robot on an NVIDIA Jetson Nano B01. The same URDF, `slam_toolbox`, AMCL, and Nav2 stack run on the chassis and in Gazebo. Low-level control is an ESP32 serial bridge; sensing is RPLidar A1, wheel encoders, and an MPU6050.

Demo: [YouTube](https://youtu.be/JTg8ff2hSGM)

<p align="center">
  <a href="https://youtu.be/JTg8ff2hSGM?si=UqfauM6vN_xyPFOV">
    <img src="assets/gitpage.jpg" alt="Physical robot, Cirkit wiring, and Web UI" width="100%">
  </a>
  <br>
  <em>Physical chassis · Cirkit wiring (Jetson + ESP32 + motors/IMU/LiDAR) · Web dashboard</em>
</p>

---

## 🤖 Robot Structure

Grey 3D-printed chassis, RPLidar A1 on a raised mount, Jetson WiFi antenna, and a dual-motor undercarriage with a front caster.

<p align="center">
  <img src="assets/side_rb.jpg" alt="Side view — RPLidar mount and Jetson WiFi antenna" width="32%">
  <img src="assets/upfront_rb.jpg" alt="Front-three-quarter view — chassis and LiDAR" width="32%">
  <img src="assets/front_rb.jpg" alt="Undercarriage — dual geared motors, encoders, caster" width="32%">
</p>

| View | What it shows |
| :--- | :--- |
| **Side** | RPLidar A1 on standoffs, Jetson WiFi antenna, driven wheel |
| **Front 3/4** | Compact grey enclosure and top-mounted scanner |
| **Undercarriage** | Twin N20/GA12-N20 class geared motors, Hall encoders, front caster |

---

## Status

| System | Detail |
| :--- | :--- |
| **TF tree** | `map → odom → base_footprint → base_link` (REP 120) |
| **Hardware** | Python serial bridge (`diffdrive_node`) to the ESP32 |
| **Navigation** | AMCL + DWB local planner |
| **Simulation** | Gazebo model matched to measured chassis dimensions |

---

## 🏗️ System Architecture (Modular)

The project is decomposed into specialized ROS 2 packages within the `src/` directory to maximize maintainability and scalability.

| Package | Responsibility | Key Components |
| :--- | :--- | :--- |
| **`jetson_bot_bringup`** | Orchestration | Launch files, world files, global config (`unified_robot_config.yaml`). |
| **`jetson_bot_description`**| Physical Model | URDF/XACRO definitions, mesh resources, sensor placements. |
| **`jetson_bot_gui`** | Web Dashboard | Telemetry server, roslibjs bridge, and real-time UI. |
| **`jetson_bot_slam`** | Environment Mapping| `slam_toolbox` configurations for asynchronous mapping. |
| **`jetson_bot_navigation`**| Path Planning | Nav2 parameters, Behavior Trees, and Local/Global planners. |
| **`jetson_bot_imu`** | Sensor Driver | Python driver for MPU6050 I2C communication. |
| **`jetson_bot_diffdrive`**| HW Interface | **Python Serial Bridge** for ESP32/Arduino communication. |
| **`jetson_bot_detection`** | Computer Vision | Image processing node for real-time detection. |

### Project Directory Layout
```text
autoJetsonBot/
├── README.md               # You are here
├── robot.sh                # Unified CLI Control Script
├── unified_robot_config.yaml # Global Source of Truth
├── Dockerfile.foxy         # Foxy/Ubuntu 20.04 Container Definition
├── src/                    # ROS 2 Source Workspace
│   ├── jetson_bot_bringup/  # Master Launch & Configs
│   ├── jetson_bot_description/ # URDF & Physical Model
│   ├── jetson_bot_gui/      # Web UI & Telemetry
│   ├── jetson_bot_navigation/ # Nav2 Stack Configs
│   ├── jetson_bot_slam/     # SLAM Toolbox Configs
│   ├── jetson_bot_imu/      # MPU6050 Driver
│   └── jetson_bot_detection/ # Vision Processing
├── maps/                   # Saved Occupancy Grid Maps (.yaml, .pgm)
├── assets/                 # Project images and diagrams
└── test_suite/             # Integration & Hardware Verification
```

---

## 🏎️ Hardware Specification (The "Digital Twin" Standard)

The platform is meticulously calibrated to a **1:1 Digital Twin standard**. Every parameter in the simulation exactly matches the physical hardware audit conducted on **2026-06-07**.

<p align="center">
  <img src="assets/circuit_image.png" alt="Cirkit Designer wiring — Jetson Nano, ESP32, motors, IMU, LiDAR, power" width="80%">
  <br>
  <em>Wiring: Jetson Nano B01 (brain) · ESP32 (motor/IMU bridge) · dual Hall-encoder motors · MPU6050 · RPLidar A1 · isolated 5V rails</em>
</p>

### 1. Physical Metrics & Kinematics
*   **Wheel Separation:** 0.212m (Center-to-center)
*   **Wheel Radius:** 0.034m (3.4cm)
*   **Ground Clearance:** 0.056m (Floor to chassis floor)
*   **Chassis Dimensions:** 19.4cm (L) x 17.8cm (W) x 9.7cm (H)
*   **Total Weight:** 1.4kg (Physics-accurate inertial distribution)

### 2. Compute & Intelligence
*   **Main Brain:** NVIDIA Jetson Nano B01 (4GB) – Executes the full ROS 2 stack and vision processing.
*   **Low-Level Controller:** ESP32 / Arduino – Real-time PID bridge for motors and encoders.
*   **HW Interface:** `jetson_bot_diffdrive` – A high-performance Python bridge using a human-readable serial protocol (`m v_l v_r\r`).

### 3. Sensing & Perception
*   **Lidar:** RPLidar A1 – Positioned at `x=0.064` (forward of axle) and `z=0.161` (from floor).
*   **IMU:** MPU6050 – Fuses 6-DOF data for improved odometry via EKF (`/odom_filtered`).
*   **Encoders:** Hall-effect sensors (11 PPR × gear reduction) – High-resolution position feedback.
*   **Vision:** Raspberry Pi Camera v2 – Serves MJPEG stream on port 8080 via `web_video_server`.

<p align="center">
  <img src="assets/rplidar.jpg" alt="RPLidar A1 pinout (A1M8-R6)" width="48%">
  <img src="assets/encoder.jpg" alt="Hall encoder connection and wiring" width="48%">
</p>

<p align="center">
  <img src="assets/motor.png" alt="Encoder wiring sequence (C1/C2 Hall phases)" width="50%">
</p>

### 4. Power Management
*   **Battery:** 12V Li-ion (3S)
*   **Regulation:** Dual buck converters for isolated 5V (Jetson) and 5V/3.3V (Sensors) rails.
*   **Failsafe:** Software-defined Emergency Stop integrated into the Nav2 action stack.

---

## 🚀 Development Workflow (The `robot.sh` CLI)

We utilize a unified control script to manage the dockerized environment efficiently.

### Core Commands
| Command | Result |
| :--- | :--- |
| `./robot.sh build` | Rebuilds the modular workspace inside the container. |
| `./robot.sh auto` | Auto-installs missing dependencies and builds. |
| `./robot.sh sim` | Launches Gazebo simulation + Web UI in **Mapping Mode**. |
| `./robot.sh nav` | Launches simulation + Nav2 in **Navigation Mode**. |
| `./robot.sh map2nav` | Saves the current map (if mapping) and switches to Nav2. |
| `./robot.sh status` | Deep-inspects ROS nodes, topics, and port health. |
| `./robot.sh stop` | Robustly terminates all processes (including Gazebo/VNC). |

### Web Dashboard

Teleop, live LiDAR, wheel RPM, and system metrics at [http://localhost:8000](http://localhost:8000).

<p align="center">
  <img src="assets/web_ui.png" alt="Web UI — lidar view, teleop pad, metrics" width="90%">
</p>

---

## 🗺️ Mapping & Navigation Steps

### Phase 1: Creating a Map
1.  Run `./robot.sh sim`.
2.  Use the Web UI or a joystick to drive the robot around the environment.
3.  Monitor mapping progress in RViz (via VNC at `localhost:5900`).
4.  Save the map dynamically:
    ```bash
    ./robot.sh shell "ros2 run nav2_map_server map_saver_cli -f /autonomous_ROS/maps/current_map"
    ```

<p align="center">
  <img src="assets/gaz_rviz.png" alt="Gazebo lab world and RViz laser scan" width="90%">
  <br>
  <em>Gazebo lab world (left) + RViz scan (right) during mapping</em>
</p>

### Phase 2: Autonomous Navigation
1.  Verify the map name in `src/jetson_bot_bringup/config/unified_robot_config.yaml`.
2.  Run `./robot.sh nav` (or `./robot.sh map2nav` after a mapping run).
3.  The robot will auto-localize at its starting point. Use **"2D Nav Goal"** in RViz to set a destination.
4.  The system uses the **DWB Local Planner** for smooth obstacle avoidance.

<p align="center">
  <img src="assets/gaz_rviz_costmap.jpg" alt="Gazebo and Nav2 costmap in RViz via VNC" width="90%">
  <br>
  <em>Nav2 costmap (inflated walls + local/global layers) over VNC</em>
</p>

---

## 🧠 Project Continuity & Memory
To maintain long-term technical health, consult these primary documentation nodes:

1.  **[PROJECT_ANALYSIS.md](./docs/PROJECT_ANALYSIS.md)**: Current roadmap, active blockers, and system status.
2.  **[AGENTS.md](./AGENTS.md)**: Repository of technical wisdom, bug fixes, and "hard-won" lessons.
3.  **[SESSION_LOG.md](./docs/SESSION_LOG.md)**: Detailed chronological history of all development sessions.
4.  **[GEMINI.md](./GEMINI.md)**: Core mandates, architectural conventions, and standard workflows.

---

## 📜 License
Licensed under the MIT License. Based on the [Autonomous ROS](https://github.com/jakhon37/autonomous_ROS) project.

*For issues or contributions, please contact jakhon37@gmail.com*
