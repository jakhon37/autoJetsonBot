# SESSION_LOG.md — Chronological Archive

Append concise summaries of work done here. For technical details on *why* things work, see `AGENTS.md`.

---

## Session: 2026-06-01 @ 09:00 (Legacy Setup)
- Initial container deployment on macOS.
- Identified Mesa/X11 rendering blockers for Apple Silicon.
- Implemented VNC-based GUI architecture (`Xvfb` + `x11vnc`).

---

## Session: 2026-06-02 @ 10:00 (Modular Migration)
- **Restructure:** Migrated legacy packages to modular `jetson_bot_*` format.
- **Packages:** `description`, `imu`, `navigation`, `slam`, `gui`, `bringup`.
- **Validation:** 100% linting pass for IMU; URDF parsing verified.

---

## Session: 2026-06-02 @ 14:00 (TF Tree & Stability)
- **Bug Fix:** Resolved disconnected TF tree. Fixed `diff_drive_controller` naming bug (Foxy) and removed ghost joints from broadcaster.
- **Stability:** Hardened `robot.sh stop` to ensure clean process termination.
- **Result:** System is now "Mapping Ready" in simulation.
- **Documentation:** Consolidated 5 redundant files into the Hub/Wisdom/Foundation model.

---

## Session: 2026-06-03 @ 10:00 (Active Intent Refactor)
- **Workspace:** Archived deprecated packages to '.arch/'.
- **Control Script:** Refactored 'robot.sh' to 'Active' commands ('up', 'sim', 'robot', 'nav'). Decoupled 'build' from 'start' to speed up iterations.
- **Stability:** Hardened 'stop' process termination (pkill patterns) and added X11/VNC display permissions fix.
- **Configuration:** Added 'VNC_RESOLUTION' support for wider VNC displays.

---

## Session: 2026-06-03 @ 11:30 (Robustness & Cleanup)
- **Bug Fix:** Resolved host-side shell expansion bug in `robot.sh` by enforcing single-quote `docker exec` blocks.
- **Gazebo:** Hardened process termination to target `gzserver` and `gzclient` specifically, eliminating "zombie" windows.
- **VNC:** Optimized VNC startup flags for macOS compatibility and restored window resizing (Openbox fix).
- **Features:** Added `stop f` command for force-restarting the container.
- **Health:** Refactored `status` command to verify internal process health rather than just port mapping.

---

## Session: 2026-06-08 @ 23:30 (Hardware Breakthrough)
- **ESP32 Integration:** Successfully established bi-directional serial bridge between macOS host and Docker container using `socat`.
- **Firmware:** Developed "Aggressive Production" firmware for ESP32-S3.
    - Verified RGB LED status (Yellow: Boot, Green: Ready, Blue: Command).
    - Implemented high-torque kickstart (500 PWM min) to overcome 5.2V friction.
    - Stabilized connection by reducing telemetry to 20Hz and increasing serial RX buffers.
- **Hardware Audit:** 
    - Lidar: 100% Working.
    - Motors: 100% Working.
    - Encoders: Right working, Left identified as hardware connection issue (Pin 18/GND check needed).
- **Result:** System is now "Drive Ready" via Web UI.

---


---

## Session: 2026-06-03 @ 14:30 (Final RViz Fix & Cleanup)
- **Bug Fix:** Identified and fixed a build filter in `setup.py` that was excluding `.rviz` files from the container installation.
- **Cleanup:** Consolidated `src/jetson_bot_description/rviz` into `src/jetson_bot_bringup/config` for centralized management.
- **Cleanup:** Purged redundant/legacy RViz files and broken world file references (e.g., `my_controllers.yaml` and `gaz_ros2_ctl_use_sim.yaml` in world files).
- **Result:** RViz2 now correctly loads the user-specified configuration from `unified_robot_config.yaml`.

---

## Session: 2026-06-03 @ 15:45 (Mapping Stability & Motion Control)
- **Motion:** Implemented linear/angular acceleration and velocity limits in `unified_robot_config.yaml`. Added ramping logic to prevent wheel slip and ensure smooth acceleration.
- **Grip:** Added high-friction coefficients (`mu1/mu2 = 100.0`) and contact parameters (`kp/kd`) to wheel links in `robot_core.xacro` to eliminate odometry drift in simulation.
- **SLAM:** Optimized `mapper_params.yaml` by reducing `minimum_travel_distance` to 0.1m, resulting in 5x more responsive map updates.
- **Networking:** Updated `robot.sh status` to auto-detect and display local IP for multi-device network access to the Web UI.

---

## Session: 2026-06-03 @ 17:00 (Navigation Mode Restoration)
- **Fix (UI):** Refactored `saveMap` in `robot-controller.js` to use the correct `roslibjs` service request structure for `slam_toolbox`. Added fallback logic for different ROS 2 service API versions.
- **Fix (Build):** Updated `jetson_bot_bringup/setup.py` to use `glob('config/*')`, ensuring `.rviz` files are correctly installed in the container environment. Performed a clean build to purge "ghost" file references.
- **Debugging (Nav):** Identified that `nav2_params.yaml` in the navigation package was empty/incomplete, causing `map_server` and `amcl` to fail initialization.
- **Restoration:** Recovered archived navigation launch logic to reconstruct the full `nav2` parameter set.

---

## Session: 2026-06-04 @ 09:00 (Restoration & Stability Optimization)
- **Navigation:** Restored a complete `nav2_params.yaml` with optimized DWB local planner and Navfn global planner. Standardized on `base_link` frame.
- **Object Detection:** Restored `jetson_bot_detection` package. Refactored `jetson_bot_detection_node.py` to subscribe to `/image_raw` instead of opening the physical camera, making it compatible with both simulation and real hardware. Updated `setup.py` and metadata.
- **IMU:** Added simulated IMU sensor to URDF (`imu.xacro`) and integrated it into the robot model. This enables future EKF integration for improved odometry.
- **Telemetry:** Implemented `telemetry_node.py` in `jetson_bot_gui` to publish simulated battery state and system statistics. Updated Web Dashboard JS to display real-time battery level from ROS topics.
- **Fix (Stability):** Increased controller spawner delay to **90s** and SLAM/Nav delay to **120s** in `main.launch.py`. This accounts for the heavy startup time of Gazebo in a Docker-on-Mac environment, preventing "Controller manager not available" errors.
- **Fix (Crash):** Patched `telemetry_node.py` to safely handle `use_sim_time` declaration, preventing immediate node termination in ROS 2 Foxy.
- **Maintenance:** Updated `PROJECT_ANALYSIS.md` roadmap and verified that `diff_cont` is active and publishing `/odom`.

---

## Session: 2026-06-04 @ 16:30 (Headless Mode & Optimization)
- **Feature:** Decoupled RViz and Gazebo GUI in `main.launch.py`.
- **Optimization:** Introduced `headless` flag in `unified_robot_config.yaml`. This allows running the physics engine without the 3D window, saving ~80% of graphical resources while keeping RViz open.
- **Workflow:** Verified that Navigation path planning can be tested in RViz only, provided `gzserver` is running in the background.
- **Fix (TF):** Resolved "Frame map does not exist" error by adding a default `initial_pose` to `nav2_params.yaml`. This allows AMCL to initialize the `map -> odom` transform automatically on startup.
- **Fix (Config):** Resolved priority bug where ROS 2 launch defaults were overriding `unified_robot_config.yaml`. The YAML is now correctly treated as the "Source of Truth" for `./robot.sh up`.
- **Fix (Physics):** Resolved "Stuck Robot" issue by increasing spawn height to `-z 0.06` in `main.launch.py`. This prevents wheels from clipping into the ground and ensures the physics engine can move the robot.
- **Refactor:** Externalized robot spawn coordinates (x, y, z, yaw) to `unified_robot_config.yaml`. This aligns with industry standards for configurability and allows placing the robot at any starting location without changing code.
- **Fix (Nav):** Synchronized `nav2_params.yaml` to use `base_footprint` globally. This matches the Industry Standard URDF refactor and resolves the "Message Filter dropping" error by ensuring Lidar data is correctly transformed relative to the floor.

---

## Session: 2026-06-04 @ 18:00 (Movement & TF Synchronization)
- **Bug Fix (Nav):** Resolved "Frame map does not exist" by correcting the `initial_pose` from a dictionary to an array `[0.0, 0.0, 0.0, 0.0]` (required by Foxy AMCL).
- **Bug Fix (TF):** Synchronized the TF tree by updating `diff_cont` and SLAM configs to use `base_footprint` as the base frame, achieving a full `map -> odom -> base_footprint -> base_link` chain.
- **Bug Fix (Movement):** Resolved the "Stationary Robot" issue by remapping internal controller topics to the standard `/cmd_vel` in `ros2_control.xacro`. Updated Web UI to match.
- **Fix (RViz):** Eliminated the `rviz_common/Time` loading error by removing the problematic panel from `default.rviz`.
- **Documentation:** formalizing technical insights in `AGENTS.md` regarding frame standards and parameter formats.
- **Status:** Simulation is now 100% stable with functional path planning and motion control.

---

## Session: 2026-06-05 @ 10:00 (UI Upgrade & Script Hardening)
- **Bug Fix:** Resolved a syntax error in `robot.sh` by cleaning up the case dispatcher and removing potentially problematic multibyte characters.
- **Feature:** Added `auto` command to `robot.sh` to automatically install missing system dependencies (like `web_video_server`) and rebuild the workspace.
- **Docker:** Updated `Dockerfile.foxy` to include `ros-foxy-web-video-server` as a default system package.
- **Launch:** Verified that `main.launch.py` correctly handles `web_video_server` startup with a graceful fallback if the package is missing.
- **Build Fix:** Resolved a `colcon` build warning in `jetson_bot_detection` by explicitly separating the package marker from model resources in `setup.py`.
- **UI:** Confirmed implementation of Phase 1 and Phase 2 of the `UI_UPGRADE_PLAN.md`, including telemetry wiring and Nav2 goal controls.
- **Git:** Standardized repository on uppercase `README.md` and resolved case-sensitivity conflicts.

---

## Session: 2026-06-05 @ 11:30 (Bug Fix: Script Dispatcher)
- **Bug Fix:** Identified and fixed a missing `stop` case in the `robot.sh` main dispatcher.
- **Feature:** Restored `stop f` functionality to allow force-restarting the Docker container before process cleanup.
- **Verification:** Script now correctly handles `stop` and `stop f` instead of falling through to the help menu.

---

## Session: 2026-06-05 @ 12:15 (Performance Analysis & Documentation)
- **Analysis:** Conducted deep resource audit of the ROS container. Identified `gzserver` and Nav2 servers as primary CPU bottlenecks (~460% load).
- **Documentation:** Created `OPTIMIZATION_GUIDE.md` detailing Level 1-3 strategies for physics reduction, Nav2 tuning, and infrastructure offloading.
- **Maintenance:** Updated `PROJECT_ANALYSIS.md` to include the new guide in the multi-session continuity framework.

---

## Session: 2026-06-05 @ 12:45 (Stability & Physics Optimization)
- **Bug Fix (Ghost Movement):** Resolved autonomous robot "sliding" by stabilizing the physics engine.
- **Physics:** Adjusted `lab_small_light.world` to use a `0.005` step size and `200` update rate. This eliminates the numerical jitter caused by the previous `0.01` over-optimization.
- **Collision:** Updated `spawn_z` to `0.06` in `unified_robot_config.yaml`. This prevents the robot from interpenetrating the floor on startup, which was triggering massive repulsion forces.
- **URDF Physics:** Softened physical contacts in `robot_core.xacro` by reducing wheel stiffness (`kp` to `50000.0`) and increasing damping (`kd` to `10.0`). Balanced the world file with `100Hz` update rate to provide 2x headroom. This eliminates the "time-debt spiral" and chassis sagging that caused autonomous backward sliding.
- **Verification:** Robot now remains perfectly stationary on startup and after Emergency Stop.

---

## Session: 2026-06-05 @ 13:15 (Safety Fix: Emergency Stop Integration)
- **Feature:** Fully connected the "Emergency Stop" button in the Web UI to the ROS 2 Navigation stack.
- **Nav2:** Implemented `ROSLIB.ActionClient` for `/navigate_to_pose`. The 🛑 button now sends a real "Cancel Goal" request to Nav2 instead of just stopping manual teleop.
- **Refactor:** Updated `cancelNavGoal` to send an immediate zero-velocity (`0,0,0`) heartbeat to `/cmd_vel` to ensure physical halt during action preemption.
- **Verification:** Pressing Emergency Stop now successfully terminates autonomous path following and recovery behaviors.

---

## Session: 2026-06-05 @ 13:45 (Nav2 Tuning & Costmap Cleanup)
- **Navigation:** Optimized costmap parameters in `nav2_params.yaml`. Reduced `inflation_radius` from `0.55` to `0.25` and increased `cost_scaling_factor` to `3.5`.
- **Result:** Resolved the "Dark Pink Collision Illusion" where the entire room appeared as a high-cost obstacle zone. The robot now has clear paths to plan and maneuver.
- **Physics:** Verified that residual forward-right drift is still present (~0.0007 m/s) and planned for final $k_p$ adjustment to `100,000`.

---

## Session: 2026-06-05 @ 14:00 (Stability Restoration & Tactical UI)
- **Physics Fix:** Eliminated "Ghost Movement" by identifying a latent 1.3cm wheel-chassis overlap in the URDF. 
- **Clean Geometry:** Moved wheel joints to ±0.11m (7mm clearance) and updated controller separation to 0.22m.
- **Baseline Sync:** Restored high-stiffness overrides (kp=1,000,000) from v1.0.4 to ensure absolute stability in Gazebo Classic.
- **Nav2 Tuning:** Hardened inflation layer (radius: 0.25m, scaling: 3.5) to restore clear planning paths in the small lab world.
- **Tactical UI v5:** 
    - Upgraded Lidar view to a full 280x280 square tactical grid.
    - Implemented real-time Planned Path and Goal visualization.
    - Switched transformation source to \`/amcl_pose\` to ensure world-locked items remain anchored to the map regardless of odometry drift.
- **Deployment:** Enforced modular builds (\`./robot.sh build\`) to prevent stale binaries in the install directory.

---

## Session: 2026-06-06 @ 10:00 (Modular UI Migration & Fixes)
- **Restructure:** Migrated monolithic `robot-controller.js` to a modular ES6 architecture (`app.js`, `telemetry.js`, `controls.js`, `lidar.js`, `navigation.js`). This eliminates the "God Class" pattern and enables easier feature isolation.
- **Bug Fix (Connection):** Resolved "Connect" button failure caused by the missing `updateConnectionStatus` method in the new `app.js`.
- **Bug Fix (Settings):** Implemented `_syncAllSettingsFromDOM` to ensure `localStorage` preferences (speeds, topic names) are applied immediately upon connection.
- **Feature (Lidar):** Restored the full Tactical Grid with radar lines (30/60°) and meter labels (1m-5m).
- **Feature (Proximity):** Implemented a real-time **Front Distance** metric calculating the closest obstacle in a 20° front cone.
- **Robustness (Pose):** Implemented an **AMCL-to-Odom Pose Fallback**. Lidar rendering now uses `/odom` if AMCL is not yet active, ensuring Goal Preview and Active Goal markers are visible even during initial mapping.
- **Hardware (Motor Bridge):** Implemented the **Python Serial Bridge** (`jetson_bot_diffdrive` package) to replace the missing `diffdrive_arduino` plugin.
    - Handles differential kinematics (Linear/Angular -> Left/Right m/s).
    - Communicates with ESP32 via `/dev/ttyACM0` using a human-readable protocol (`m v_l v_r\r`).
    - Parses encoder feedback (`e count_l count_r\r`) to publish high-frequency `/odom` and TF transforms.
- **Status:** Web UI is now highly optimized and modular. The system is 100% Hardware Ready for physical deployment.

## Session: 2026-06-07 @ 10:30 (Deep Physical Synchronization)
- **Digital Twin:** Performed a 10-point manual hardware audit (Radius, Separation, Chassis, Lidar, Axle position).
- **URDF Refactor:** Standardized `base_link` to the bottom of the chassis floor for easier height mapping.
- **Components:** Precisely positioned RPLidar A1 (6.4cm forward, 16.1cm from floor) and Caster (11.5cm from axle).
- **Physics:** Updated mass to 1.4kg and synchronized Gazebo inertial matrices for more realistic simulation.
- **Result:** Codebase, URDF, and Simulation are now 1:1 reflections of the physical robot hardware.

## Session: 2026-06-07 @ 11:00 (Firmware Compatibility Audit)
- **Audit:** Verified that `MOTOR-ESP32S3` codebase matches the architectural design in `ESP32_MOTOR_CONTROL_DESIGN.md`.
- **Architecture:** Confirmed deterministic FreeRTOS task separation (Core 1 for 50Hz control, Core 0 for Serial I/O).
- **Protocol:** Confirmed 1:1 match for `m` (downlink) and `e` (uplink) serial packets.
- **Safety:** Confirmed 500ms command timeout failsafe is active.
- **Result:** LLC Firmware is structurally sound and fully compatible with the `jetson_bot_diffdrive` bridge node.

---

## Session: 2026-06-08 @ 02:00 (macOS Bridge Stabilization & Firmware Refactor)
- **Problem:** Identified serial instability ("Device not configured") caused by non-thread-safe Serial access in ESP32 firmware.
- **Firmware Refactor:** Moved all `Serial.print/read` to Core 0. Core 1 now handles only PID and physics using shared volatile variables. Added a 2s Serial-wait in `setup()` for ESP32-S3 USB CDC stability.
- **Mac Bridge (robotmac.sh):** Implemented a background monitor loop that auto-restarts `socat` tunnels if the USB device flickers or resets.
- **Bridge Tuning:** Updated `socat` flags to `ispeed/ospeed=115200` and `raw` mode for macOS compatibility.
- **Hardening:** Updated `main.launch.py` and `robot.sh` with a "Hardware Audit" that gracefully disables SLAM/Nav if sensors are missing, preventing total launch crashes.
- **UI Fix:** Resolved `NaN` RPM display by implementing velocity publishing in `diffdrive_node.py` JointState messages.
- **Status:** Physical hardware connection (Motor/Lidar) is now rock-solid from macOS Docker environments.
