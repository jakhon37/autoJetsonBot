# Optimization Guide - autoJetsonBot

This document outlines strategies for running the ROS 2 Navigation stack and Gazebo simulation on resource-constrained hardware (e.g., Jetson Nano, Docker-on-Mac).

## 📊 Resource Analysis (Baseline)
During high-activity tasks (Navigation + Gazebo), the following bottlenecks were identified:
*   **Gazebo (`gzserver`):** ~137% CPU usage. Primary drain on resources due to real-time physics and sensor simulation.
*   **Nav2 Servers:** ~150% combined CPU usage. Constant path recalculation and costmap updates are expensive.
*   **Total Container Load:** ~460% CPU usage.

---

## 🚀 Optimization Strategies

### 1. Level 1: Physics & Configuration (Highest Impact)
*   **Physics Step Size:** 
    *   *File:* `src/jetson_bot_bringup/worlds/*.world`
    *   *Change:* Increase `max_step_size` from `0.001` to `0.005` or `0.01`.
    *   *Effect:* Reduces physics resolution, significantly lowering CPU usage.
*   **Lidar Update Rate:**
    *   *File:* `src/jetson_bot_description/urdf/lidar.xacro`
    *   *Change:* Reduce `<update_rate>` from `10` to `5` or `3`.
    *   *Effect:* Lowers the frequency of point cloud processing in the Navigation stack.
*   **Lidar Resolution:**
    *   *File:* `src/jetson_bot_description/urdf/lidar.xacro`
    *   *Change:* Reduce the number of samples (e.g., from 360 to 180).
    *   *Effect:* Halves the data volume for every scan message.

### 2. Level 2: Navigation Stack Tuning
*   **Costmap Resolution:**
    *   *File:* `src/jetson_bot_navigation/config/nav2_params.yaml`
    *   *Change:* Increase `resolution` from `0.05` to `0.1`.
    *   *Effect:* Drastically reduces memory and CPU needed for occupancy grid calculations.
*   **Update Frequencies:**
    *   *File:* `src/jetson_bot_navigation/config/nav2_params.yaml`
    *   *Change:* Lower `expected_planner_frequency` and `controller_frequency`.
    *   *Effect:* Less aggressive path recalculation.

### 3. Level 3: Infrastructure & Workflow
*   **Headless Mode:**
    *   *Action:* Always set `headless: true` in `unified_robot_config.yaml` for testing.
    *   *Effect:* Saves ~80% of graphical resources by not rendering the Gazebo client window.
*   **Web Services:**
    *   *Action:* Disable `web_video_server` if camera streams are not actively needed.
    *   *Effect:* Prevents heavy video encoding CPU usage.
*   **Externalize Physics:**
    *   *Strategy:* Run Gazebo on the host machine (if it has a dedicated GPU) and ROS nodes in the container.
    *   *Effect:* Offloads heavy 3D rendering to host hardware.

---

## 🛠️ Implementation Checklist
- [ ] Increase physics step size to 0.005 in active world files.
- [ ] Reduce Lidar samples to 180 in URDF.
- [ ] Set costmap resolution to 0.1 in Nav2 params.
- [ ] Disable camera streaming by default.
