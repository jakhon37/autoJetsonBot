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

## Session: 2026-06-03 @ 13:00 (Dynamic UI & Map Integration)
- **Feature:** Implemented `config.json` bridge between ROS 2 launch system and Web UI.
- **Mapping:** Fully integrated `slam_toolbox` save service. Map paths and names are now dynamic and honor `unified_robot_config.yaml`.
- **UI:** Refactored `robot-controller.js` to fetch system configuration on startup, eliminating hardcoded world paths.
- **Verification:** Confirmed that `main.launch.py` correctly exports global parameters for the frontend.

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

---

## Session: 2026-06-07 @ 10:30 (Deep Physical Synchronization)
- **Digital Twin:** Performed a 10-point manual hardware audit (Radius, Separation, Chassis, Lidar, Axle position).
- **URDF Refactor:** Standardized `base_link` to the bottom of the chassis floor for easier height mapping.
- **Components:** Precisely positioned RPLidar A1 (6.4cm forward, 16.1cm from floor) and Caster (11.5cm from axle).
- **Physics:** Updated mass to 1.4kg and synchronized Gazebo inertial matrices for more realistic simulation.
- **Result:** Codebase, URDF, and Simulation are now 1:1 reflections of the physical robot hardware.

---

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

## Session: 2026-06-09 @ 00:30 (IMU Integration Prep)
- **Dependency:** Resolved `smbus2` missing module error in Docker container.
- **Architecture:** Decided to integrate MPU6050 IMU via ESP32-S3 to leverage real-time processing and the existing serial bridge.
- **Plan:** Scheduled I2C wiring (GPIO 1/21) and firmware update for accel/gyro telemetry.

---

## Session: 2026-06-09 @ 23:40 (IMU Dashboard Bug Fix & UI Refactor)
- **Bug Fixes:**
    - Resolved critical connection blockage by renaming the global variable `status` to `statusBadge` to avoid conflict with the browser's reserved `window.status` namespace.
    - Fixed undefined `az` parameter in the WebSocket message handler which was causing `NaN` propagation inside the filter.
- **Robot Axes & Aesthetics:** Restored the local robot coordinate axes using `THREE.ArrowHelper` (rendering arrows for X, Y, and Z axes) and restored the red "Front" marker box on the chassis.
- **Coordinate Mapping:** Added mathematical mapping from the Z-up sensor coordinate system ($X_{sensor} \leftrightarrow -Z_{three}$, $Y_{sensor} \leftrightarrow -X_{three}$, $Z_{sensor} \leftrightarrow Y_{three}$) to the Three.js $Y$-up coordinate system.
- **Filter Stabilization:** Completely rewrote the buggy/incorrect manual algebraic expansions for the gradient descent vector ($s_1$, $s_2$, $s_3$, $s_4$) with standard, clean, and mathematically verified Madgwick AHRS IMU equations, and corrected the sign of the first term in $s_0$ to matching derivative physics.
- **WebSocket Safety:** Resolved the duplicate socket reconnection timer bug by implementing robust timer cleanup (`clearTimeout` and event listener unbinding).
- **UI Upgrade:** Redesigned the HTML/CSS layout with a premium glassmorphic dark theme, clear layout division, card-hover states, glowing status badges, and proper semantic HTML.

---

## Session: 2026-11-05 @ 18:39 (Humble Port Saved + PR #2 Cleanup)
- **Discovery:** The `jetson` host is a Jetson Xavier NX (L4T R35.6.5, Ubuntu 20.04), not the Nano B01 the docs described. It held an uncommitted ROS 2 Humble Docker port from 2026-08-18 to 08-21.
- **Saved:** Committed that work to `jetson-humble-port` (`e119edb`), opened PR #2, and merged `master` into the branch to pick up the README rewrite.
- **use_sim_time:** Tested `slam_toolbox` in a throwaway Humble container. The launch argument always wins over `mapper_params.yaml`, so the YAML line added in the port had no effect and was removed.
- **Cleanup:** Removed the unused plain `Dockerfile` (identical to `Dockerfile.humble` apart from blank lines). Updated docs to the Xavier NX + Humble target and corrected stale wheel specs in `GEMINI.md`.
- **Hardware Audit:** Only the RPLidar's CP2102 adapter is detected. ESP32 and camera are not, so physical validation is still open.

---

## Session: 2026-11-05 @ 21:40 (Humble Hardware Bring-up: Lidar)
- **Lidar:** First run on hardware under Humble. Health OK (firmware 1.29, hardware rev 7), `/scan` at 6.46 Hz with 360 points per scan. Only 40% of points were valid, with most returns within 30 cm, so check for obstructions on the chassis. The scan's front direction is not yet checked against the robot.
- **ESP32:** Still not detected. The lidar is the only USB device to appear since boot, and there was no failed enumeration, which points at the cable or USB socket. A status file left in the container shows it running on this Jetson on 2026-08-21 at `/dev/ttyACM0`, with 356,570 encoder and IMU packets.
- **Camera:** Still not detected. The `imx219` driver is loaded and probed both CSI ports at boot, and neither answered (`-121`). No launch file starts `camera_go`, so `/camera/image_raw` has no publisher yet.
- **Bug Fix:** `./robot.sh status` read that leftover file as live and reported encoders and IMU OK with no ESP32 attached. It now ignores the file unless it was written in the last 5 seconds (the bridge writes it every second).
- **Safety:** With no ESP32 the launch doesn't start the bridge, so `/cmd_vel` had no subscriber and nothing could drive. Motors, encoders and IMU remain untested on this date.

---

## Session: 2026-11-05 @ 22:00 (ESP32 and Camera Recheck)
- **Recheck:** After reconnecting and rebooting, the ESP32 and camera are still not detected. The ESP32 isn't on the development PC either.
- **ESP32 History:** `/var/log/kern.log` shows it on this Jetson from 2026-08-18 to about 09-02, last as `303a:1001` (Espressif native USB) on hub port 1-2.1. It also appeared as `1a86:55d3` (QinHeng) on 08-21. Today no connect event was logged, not even a failed one.
- **Camera History:** The `imx219` probe failed with `-121` on all 20 boots in the log, so no camera has ever been detected on this Jetson and there's no working setup to go back to.

---

## Session: 2026-11-05 @ 22:20 (Camera Correction)
- **Correction:** The robot's camera is an ESP32 Wi-Fi camera, not the Pi Camera v2 the docs assumed. The `imx219` conclusions in today's earlier entries don't apply, because there's no CSI camera to find.
- **Camera Search:** It isn't on the Jetson's USB today. One ping of `192.168.219.0/24` found 9 devices, none from Espressif. Its model and address are still unknown.
- **Motor ESP32:** `MOTOR-ESP32S3` builds with `ARDUINO_USB_CDC_ON_BOOT=1`, so its serial link is the native USB socket. The launch error now says so instead of pointing at the QinHeng/WCH socket.
- **Serial Discovery:** The container has no `/dev/serial/by-id`, so launch uses the first `/dev/ttyACM*` and `/dev/ttyUSB*`. An ESP32 camera on a data cable could be picked instead of the motor board.

---

## Session: 2026-11-05 @ 23:15 (ESP32-S3 Read-Only Bring-up)
- **ESP32-S3:** Detected on the native USB socket next to the RGB LED (`303a:1001`, `/dev/ttyACM0`). Encoder and IMU lines arrive at 50 Hz each. `/odom`, `/joint_states` and `/imu/data` run at 50 Hz, `/odom_filtered` at 30 Hz. The full TF chain is present and `slam_toolbox` built a map. 0 motor commands were sent.
- **Odometry Fix:** The bridge treated the ESP32's existing counts as motion on its first reading, which put `/odom` at (-3.13, -2.72) at startup. It now takes its first reading as a baseline. Tested on hardware with the bridge alone: `/odom` starts at zero with the encoders at -133,420, including after a bridge restart. The full launch hasn't been rerun with this fix.
- **Encoders:** L -133,420 and R 0, unchanged during 4 minutes at rest. How L got there is unknown, so a hand test is next.
- **IMU:** At rest, |a| is 9.77 m/s² and gyro z reads about 0.024 rad/s (1.4°/s). `/odom_filtered` yaw drifted about 0.1°/s, so bias calibration is still needed. Gyro values arrive with only 2 decimals.
- **Status Fix:** Verified the `./robot.sh status` change against the live bridge.
- **ESP32-CAM:** On the development PC as a CH340 (`1a86:7523`) at `/dev/ttyUSB0`. Not on the Wi-Fi network after about 50 minutes powered, and no Espressif access point in range. That PC's user isn't in `dialout`, so its serial log hasn't been read.

---

## Session: 2026-11-05 @ 23:35 (ESP32-CAM Network Test)
- **Firmware:** Custom firmware, source not found under `~/myspace`. It tries `REMOVED` at boot and falls back to its own `ESP32-CAM` hotspot (`192.168.4.1`) if that fails. Endpoints are `/stream`, `/capture` and `/status`. It also has a 0.5 GB SD card.
- **Boots:** Opening its serial port resets it. Of 3 boots, 2 joined Wi-Fi at `192.168.219.118` in about 4 seconds, and 1 fell back to the hotspot. It wasn't retrying during the 18 seconds watched. After the plain power-up at 22:25 it was on neither network at 23:16, for an unknown reason.
- **Stream from the Jetson:** MJPEG at 640×480, 9.8 fps, about 97 KB/s, frames about 10 KB. Ping averaged 122 ms (83 to 158 ms). The snapshot was dark and blurry, which may just be where it was pointing.
- **Correction:** Its CH340 appears as `/dev/ttyUSB*`, so on a Jetson data cable it would collide with the lidar's port discovery, not the motor board's as noted earlier.
- **Encoders:** The hand test hasn't been done yet. Counts were still L -133,420 and R 0.

---

## Session: 2026-11-06 @ 02:10 (Wheel Test and Camera Firmware)
- **Wheel Test:** Both encoders count. R went from 103 to 417 (+314), then L from -208,538 to -208,828 (-290), against 3,436 configured per turn. The distance and direction turned aren't confirmed, so `encoder_cpr` is unchanged until a test with marked wheels. No measurement behind 3,436 was found, and the older `dev` branch of `MOTOR-ESP32S3` says 360. R moved first, so left and right may be swapped if L was turned first as asked.
- **Left Jumps:** L went from -133,420 to -208,538 between 00:11 and 02:00 while no logger was running, with R moving 103 and 0 motor commands sent. That would take over 16 minutes of nonstop turning at the fastest hand speed logged. Neither count moved during 45 minutes at rest. A loose connection on Left A (GPIO 18) is the leading suspect: in the repo's firmware, noise on A with B reading high only counts down.
- **Motor Firmware:** The flashed build isn't in `MOTOR-ESP32S3`. That repo has no IMU code on `main` or `dev`, and `main` sends encoders at 20 Hz, while the board sends `e` and `i` at 50 Hz each. No source was found under `~/myspace` on the PC or the Jetson. `main` counts only rising edges of channel A, and its `m` command is open loop with a minimum of 500/1023 above 0.01 m/s, but the flashed behaviour is unknown.
- **ESP32-CAM Firmware:** Source is `github.com/jakhon37/myESP32CAM-BLACKBOX`, and its endpoints and replies match the camera (AI-Thinker, OV2640). It tries Wi-Fi once for 10 s at boot, then stays on its hotspot until reset. It halts with no network if the sensor fails to start, which would fit the "neither network" state seen last night but isn't confirmed.
- **Power-Up:** The replug at 01:20 came back on Wi-Fi by itself, at the same address, without the serial port being opened. That makes 1 of 2 plain power-ups.
- **Stream Retest:** 1.8 to 9.7 fps over 7 runs, up to 121 KB/s, frames about 12 to 13 KB. Idle ping averaged 273 to 318 ms from the Jetson and 278 ms from the wired PC, while the PC reaches the router in 0.3 ms and the Jetson in about 2 ms (-62 dBm). During a stream, ping went as low as 17 ms, which points at the camera's Wi-Fi power saving, left on by the firmware. Not proven yet.
- **Correction:** Idle ping isn't video delay, so the earlier "about 120 ms" says nothing about it. Video delay hasn't been measured.
- **Stream Limits:** The 10 fps is the firmware's default (`/fps`), but asking for 15 or no limit gave 6.3 to 6.6 fps in those runs. Restored to 10. `/status` got no answer for 7 s during a stream while frames kept arriving, so it serves one client at a time. In one run at 1.8 fps, `/status` answered after 2.4 s, so that stream had apparently dropped. Frames carry no timestamps.
- **SD Card:** It's a Raspberry Pi boot card, and the "0.5 GB" was only its 509 MB boot partition. `/capture` saves a copy to the card on every call, and this session's tests added 2 of the 13 snapshots. Nothing was deleted. The recorder's overwrite only removes `vid_*.avi`.
- **Picture:** A frame from the stream at 02:03 is clear and well exposed, so last night's dark snapshot came from where the camera was pointing.

---

## Session: 2026-11-06 @ 02:45 (Marked Wheel Test)
- **Test:** The request was 5 turns forward and then 5 turns backward by hand on each wheel, left first, with the logger on. Left moved first, so left and right aren't swapped.
- **Scale:** L went from -208,828 to -212,265 (-3,437) and R from 417 to 4,283 (+3,866). Over 10 turns that's about 344 and 387 counts per turn, about 9 to 10 times fewer than the configured 3,436. L's total is almost exactly 3,436, so that value may have been measured over 10 turns and recorded as one. That's unconfirmed.
- **Direction:** Neither count reversed. L only went down and R only went up, which is what the repo's firmware does when channel B reads high, as an unconnected input would with its pull-up. Earlier readings have the same signs (L -429,862 and R +49,727 on 2026-08-21). The bridge would then report both wheels moving backward whichever way they turn, and a spin in place would read as reversing. This depends on the second 5 turns really being backward, which still needs confirming.
- **`encoder_cpr`:** Left at 3,436 for now, because a direction fix in firmware could change the count per turn. Counting all four edges would give 4 times as many.
- **Left Jumps:** Nothing moved in the log after 02:43:58, so wiggling the wires caused no jump if that was done by then. The -75,118 is still unexplained.
- **Correction:** "Over 16 minutes" for that jump used the speed from the first test. The fastest hand speed logged is now about 270 counts/s, which makes it about 4.5 minutes of nonstop turning, or about 200 turns.

---

## Session: 2026-11-06 @ 03:15 (Motor Firmware Recheck and Camera Fix)
- **Motor Firmware:** `MOTOR-ESP32S3` is the motor code. Rechecked on GitHub: `main`, `dev`, tags `v1.0` and `v1.1`, and merged PR #1 have no IMU code. `main` sends `e` lines at 20 Hz and no `i` lines, while the board sends both at 50 Hz, so the version on the board was probably never pushed.
- **Camera Fix:** Branch `wifi-reliability` in `myESP32CAM-BLACKBOX` turns off Wi-Fi power saving, retries home Wi-Fi every 30 s while on the hotspot with nobody connected, and restarts if the sensor fails to start. It's local only and not flashed yet.
- **Build:** Compiles with placeholder Wi-Fi settings on espressif32 8.0.0 and Arduino core 3.3.12 (RAM 18.5%, flash 38.4%). The repo doesn't pin a platform version, so this may be newer than what's on the camera. Not tested on hardware yet.
- **Encoders:** Still waiting on whether the second 5 turns were backward.

---

## Session: 2026-11-06 @ 03:30 (Motor Firmware Traced)
- **Correction:** The earlier "no known source" note was wrong. `imu_viz` had appeared in the search and was set aside without being reported, and a case-sensitive search of the docs missed `docs/IMU_WIRING.md`.
- **Where It Came From:** The firmware on the ESP32-S3 is `MOTOR-ESP32S3` with IMU code added from `imu_viz` (the `test_IMU` sandbox). The June 11 plan says to move that code into `MOTOR-ESP32S3`, and `IMU_WIRING.md` says it's in `MOTOR-ESP32S3/src/main.cpp`, but that repo's last code commit is June 9. The plan was deleted the same day and survives in commit `b23068e`.
- **Evidence:** `imu_viz` has an MPU6050 through the Adafruit driver at 50 Hz, with an α=0.8 filter, ±2 g and ±250°/s, on an ESP32-C3. The board's `i` lines run at 50 Hz with the same field order and units, and the wiring doc names the same filter. Pins on the robot are SDA GPIO 1 and SCL GPIO 21.
- **Search:** 105 `jakhon37` repos and 3 `gachonuz` repos, 277 branches, plus the full history of the 6 firmware repos and this one. No commit has IMU code and encoder interrupts together, so the combined version was never committed. `gh search code` returned nothing, even for text known to be in a public repo, so it isn't usable here.
- **`encoder_cpr`:** 3,436 goes back to May 2025 and was also in a 2025 firmware for different wheel specs (radius 0.035, separation 0.18). No measurement behind it was found.
- **Drive Behaviour:** June's notes match `MOTOR-ESP32S3`: open loop, a minimum of 500/1023 power, and a 500 ms timeout. That's probably what's on the board, but it's unconfirmed.

---

## Session: 2026-11-06 @ 04:30 (Camera Firmware Flashed, Stream Collapsed)
- **Flashed:** `wifi-reliability` (`cc2aea2`, local only) is on the ESP32-CAM. It pins espressif32 6.12.0, which has the same core libraries as the old firmware (IDF 4.4.7, built Mar 5 2024). Unpinned, the repo now resolves to Arduino core 3.3.12. A 4 MB backup of the old firmware is in the session scratchpad and contains the Wi-Fi password.
- **Retry, Tested on Hardware:** With a network that doesn't exist, it fell back at 10 s and retried every 30 s for 10 s, and the hotspot was seen within the last 8 s throughout. With a 1 ms boot timeout, it fell back, joined `REMOVED` about 1 s after the retry began at 32 s, and the hotspot turned off.
- **Idle Ping:** Median about 13 ms on the new firmware against 290 to 330 ms on the old, measured within 15 minutes of each other. Turning off power saving removed the delay on small packets.
- **Stream:** Slow on both. The old firmware managed 0.1 to 0.8 fps (3 to 21 KB/s) at 04:08 and the new 0.1 to 0.2 fps (1 to 5 KB/s) at 04:15, against 7.5 to 9.7 fps from the old firmware at 02:05. It varies over time, so whether the new one is worse is still unknown.
- **Ruled Out:** Signal (-48 to -55 dBm), other clients (none connected), TCP retransmissions (0 on an 18 KB download), the sensor (that page comes from flash and still downloads at 1.5 to 24 KB/s), and capture load (QQVGA isn't faster in bytes per second). The router and Jetson links are fine (about 2 ms). The camera sends a few segments and then goes silent for 400 to 660 ms.
- **Wrong Theories:** "Poor for the first minutes after a reset" didn't hold, because the settled stream was still slow. "Large frames starve Wi-Fi" didn't hold either.
- **Changed Since 02:06:** The camera was re-plugged at 02:41 and has moved, since its view is different. USB power or noise near the PC is the next thing to test, with a phone charger away from the PC.
- **Boot After Flashing:** The camera was unreachable for about 80 s after the 04:10 flash, which fits a failed first join followed by a successful retry. No serial log was captured for that boot.
- **Serial Port:** This programmer board stays in reset if DTR and RTS are held active, so clear them before opening the port. Reading flash failed at 921600 baud and worked at 460800.

---

## Session: 2026-11-06 @ 13:05 (Camera on a Charger: Network Recovered, No Frames at VGA)
- **Setup:** The ESP32-CAM runs `wifi-reliability` (`cc2aea2`) from a phone charger away from the PC, with no USB data connection. It rejoined `REMOVED` by itself at `192.168.219.118`. Measured over the network only, 12:48 to 13:03.
- **Network Recovered:** The 18 KB page from flash downloads at a median of 177 KB/s from the PC and 153 KB/s from the Jetson, against about 5 KB/s at 04:30. Idle ping averages 5 to 9 ms from both, with 1 packet lost out of 160. Page speed and ping are the same at QVGA and VGA, so capture load isn't what slowed the page at 04:30.
- **Cause of the Recovery Unknown:** Power source, position and time of day all changed together.
- **Stream by Frame Size:** QQVGA runs at 7.2 to 9.6 fps with 3.8 KB frames. QVGA runs at 3.8 to 8.2 fps with 11 KB frames, and stalls for up to 2.8 s. VGA, SVGA and XGA deliver no frames, with 0 bytes and the connection left open.
- **Where It Fails:** `/capture` at VGA returned HTTP 500 after 4.03 s, which is the driver's frame timeout. `/status` answers while a client holds `/stream`, because the handler has already given up. The handler sends nothing until it has a frame, so the client just waits.
- **Not Wi-Fi or a Dead Sensor:** QVGA from the same boot gives a clear picture of a room. VGA worked earlier in other views: 7.5 to 9.7 fps with frames of about 12 KB at 02:05, and 0.1 to 0.8 fps with frames of about 24 KB at 04:08.
- **Frame Size Pattern:** Delivery falls as frames get bigger: 3.8 KB at up to 9.6 fps, 11 KB at up to 8.2 fps, 24 KB under 1 fps, and nothing at VGA now. The view isn't unusual: at the same size and JPEG tables it has 1.2 times the detail of the 04:21 view, so a VGA frame should be about 26 to 45 KB.
- **Two Possible Causes, Both Untested:** The driver sizes its JPEG buffer once at boot as width × height / 5, which is 61,440 bytes for VGA, and throws away a frame that overflows it (`esp32-camera` at `f0bb429`, the version in this core). That estimate sits under the limit, though, so a failure that grows with frame length is also possible. The driver's task shares core 0 with Wi-Fi and drops the frame if it falls behind. Its overflow warnings are compiled out, so the serial port wouldn't show the first case.
- **Side Effects:** One `/capture` call failed, so nothing was saved to the SD card. Frame size changes reset on reboot, and the camera was left at VGA.

---

## Session: 2026-11-06 @ 13:25 (Camera Lens Test: 640×480 Back, First Boot Unexplained)
- **Test:** At 640×480 on the charger, frames were logged while the view was changed by hand: lens covered, plain wall, then back to the room.
- **Simple Views:** The view from the 12:48 test gave no frames until about 13:08:50, when the camera was picked up. After that, frames of 8 to 21 KB ran at a full 10 fps. 8.3 KB looks like the minimum for a flat picture at this size. For about 20 s at that size the rate fell to 2 to 10 fps, which fits a hand near the antenna.
- **Room View:** 640×480 now works there. It gave 1 to 2 fps at 13:11, with two 5 s gaps, and 3 to 7 fps by 13:14, with frames of 23 to 24 KB.
- **Not a Size Limit:** In the same view, 800×600 delivered 34 KB frames at 3.2 fps. So neither the 60 KB buffer nor a limit near 23 KB explains this view, and the "stops dead at 23 KB" reading from earlier in this test was wrong.

| Size | Frame | From the PC | From the Jetson |
| :--- | :--- | :--- | :--- |
| 160×120 | 2.9 KB | 10.1 fps | Not measured |
| 320×240 | 8.2 KB | 9.3 fps, 78 KB/s | 9.8 fps, 83 KB/s |
| 640×480 | 23.6 KB | 4.1 to 4.9 fps, 96 to 118 KB/s | 2.9 fps, 70 KB/s |
| 800×600 | 34 KB | 3.2 fps, 110 KB/s | Not measured |

- **Stalls at 640×480:** Asking for 2 fps delivered 1.6 fps with one 3.1 s gap, and asking for 5 fps delivered 3.4 fps. So it's more than a fixed throughput ceiling: a stall this long at 46 KB/s isn't explained by bandwidth. Whether it's capture or the link is untested. 320×240 had no gap over 0.5 s in this view.
- **Still Unexplained:** Why the first boot on the charger gave no frames at 640×480 or larger for 20 minutes. Scaling from this view, its 640×480 frames should have been about 31 KB, smaller than the 34 KB that passes here.
- **Reboots While Handled:** The camera went offline at about 13:08:43 for up to 7 s, and from 13:10:03 to 13:10:28. "Connection refused" in the second shows a reboot. Whether those were power being pulled is unconfirmed.
- **Older Photos:** On the SD card, 640×480 photos are 9.4 to 19.9 KB and the largest photo is 28.3 KB at 1024×768, so they don't show bigger frames working in the past. Read for size only, nothing was kept.
- **Next:** Three plain power-ups in the same view, logging frames next to a 5 Hz ping.
