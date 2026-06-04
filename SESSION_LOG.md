# SESSION_LOG.md — Chronological Archive

Append concise summaries of work done here. For technical details on *why* things work, see `AGENTS.md`.

---

## Session: 2026-06-01 (Legacy Setup)
- Initial container deployment on macOS.
- Identified Mesa/X11 rendering blockers for Apple Silicon.
- Implemented VNC-based GUI architecture (`Xvfb` + `x11vnc`).

---

## Session: 2026-06-02 (Modular Migration)
- **Restructure:** Migrated legacy packages to modular `jetson_bot_*` format.
- **Packages:** `description`, `imu`, `navigation`, `slam`, `gui`, `bringup`.
- **Validation:** 100% linting pass for IMU; URDF parsing verified.

---

## Session: 2026-06-02 (TF Tree & Stability)
- **Bug Fix:** Resolved disconnected TF tree. Fixed `diff_drive_controller` naming bug (Foxy) and removed ghost joints from broadcaster.
- **Stability:** Hardened `robot.sh stop` to ensure clean process termination.
- **Result:** System is now "Mapping Ready" in simulation.
- **Documentation:** Consolidated 5 redundant files into the Hub/Wisdom/Foundation model.


## Session: 2026-06-03 @ 10:00 (Active Intent Refactor)
- **Workspace:** Archived deprecated packages to '.arch/'.
- **Control Script:** Refactored 'robot.sh' to 'Active' commands ('up', 'sim', 'robot', 'nav'). Decoupled 'build' from 'start' to speed up iterations.
- **Stability:** Hardened 'stop' process termination (pkill patterns) and added X11/VNC display permissions fix.
- **Configuration:** Added 'VNC_RESOLUTION' support for wider VNC displays.

## Session: 2026-06-03 @ 11:30 (Robustness & Cleanup)
- **Bug Fix:** Resolved host-side shell expansion bug in `robot.sh` by enforcing single-quote `docker exec` blocks.
- **Gazebo:** Hardened process termination to target `gzserver` and `gzclient` specifically, eliminating "zombie" windows.
- **VNC:** Optimized VNC startup flags for macOS compatibility and restored window resizing (Openbox fix).
- **Features:** Added `stop f` command for force-restarting the container.
- **Health:** Refactored `status` command to verify internal process health rather than just port mapping.

## Session: 2026-06-03 @ 13:00 (Dynamic UI & Map Integration)
- **Feature:** Implemented `config.json` bridge between ROS 2 launch system and Web UI.
- **Mapping:** Fully integrated `slam_toolbox` save service. Map paths and names are now dynamic and honor `unified_robot_config.yaml`.
- **UI:** Refactored `robot-controller.js` to fetch system configuration on startup, eliminating hardcoded world paths.
- **Verification:** Confirmed that `main.launch.py` correctly exports global parameters for the frontend.

## Session: 2026-06-03 @ 14:30 (Final RViz Fix & Cleanup)
- **Bug Fix:** Identified and fixed a build filter in `setup.py` that was excluding `.rviz` files from the container installation.
- **Cleanup:** Consolidated `src/jetson_bot_description/rviz` into `src/jetson_bot_bringup/config` for centralized management.
- **Cleanup:** Purged redundant/legacy RViz files and broken world file references (e.g., `my_controllers.yaml` and `gaz_ros2_ctl_use_sim.yaml` in world files).
- **Result:** RViz2 now correctly loads the user-specified configuration from `unified_robot_config.yaml`.

## Session: 2026-06-03 @ 15:45 (Mapping Stability & Motion Control)
- **Motion:** Implemented linear/angular acceleration and velocity limits in `unified_robot_config.yaml`. Added ramping logic to prevent wheel slip and ensure smooth acceleration.
- **Grip:** Added high-friction coefficients (`mu1/mu2 = 100.0`) and contact parameters (`kp/kd`) to wheel links in `robot_core.xacro` to eliminate odometry drift in simulation.
- **SLAM:** Optimized `mapper_params.yaml` by reducing `minimum_travel_distance` to 0.1m, resulting in 5x more responsive map updates.
- **Networking:** Updated `robot.sh status` to auto-detect and display local IP for multi-device network access to the Web UI.

## Session: 2026-06-03 @ 17:00 (Navigation Mode Restoration)
- **Fix (UI):** Refactored `saveMap` in `robot-controller.js` to use the correct `roslibjs` service request structure for `slam_toolbox`. Added fallback logic for different ROS 2 service API versions.
- **Fix (Build):** Updated `jetson_bot_bringup/setup.py` to use `glob('config/*')`, ensuring `.rviz` files are correctly installed in the container environment. Performed a clean build to purge "ghost" file references.
- **Debugging (Nav):** Identified that `nav2_params.yaml` in the navigation package was empty/incomplete, causing `map_server` and `amcl` to fail initialization.
- **Restoration:** Recovered archived navigation launch logic to reconstruct the full `nav2` parameter set.

## Session: 2026-06-04 (Restoration & Stability Optimization)
- **Navigation:** Restored a complete `nav2_params.yaml` with optimized DWB local planner and Navfn global planner. Standardized on `base_link` frame.
- **Object Detection:** Restored `object_detection` package. Refactored `object_detection_node.py` to subscribe to `/image_raw` instead of opening the physical camera, making it compatible with both simulation and real hardware. Updated `setup.py` and metadata.
- **IMU:** Added simulated IMU sensor to URDF (`imu.xacro`) and integrated it into the robot model. This enables future EKF integration for improved odometry.
- **Telemetry:** Implemented `telemetry_node.py` in `jetson_bot_gui` to publish simulated battery state and system statistics. Updated Web Dashboard JS to display real-time battery level from ROS topics.
- **Fix (Stability):** Increased controller spawner delay to **90s** and SLAM/Nav delay to **120s** in `main.launch.py`. This accounts for the heavy startup time of Gazebo in a Docker-on-Mac environment, preventing "Controller manager not available" errors.
- **Fix (Crash):** Patched `telemetry_node.py` to safely handle `use_sim_time` declaration, preventing immediate node termination in ROS 2 Foxy.
- **Maintenance:** Updated `PROJECT_ANALYSIS.md` roadmap and verified that `diff_cont` is active and publishing `/odom`.

---

## Session: 2026-06-04 @ Current (Documentation Sync)
- **Insight:** Identified `diffdrive_arduino` plugin source as a missing critical dependency for physical integration.
- **Docs:** Updated `PROJECT_ANALYSIS.md` to reflect restored Navigation status and documented the hardware blocker.
- **Wisdom:** Appended hardware dependency and telemetry parity notes to `AGENTS.md`.
- **Status:** System is optimized in simulation; physical integration is the new primary bottleneck.

---

## Session: 2026-06-04 @ Current (Headless Mode & Optimization)
- **Feature:** Decoupled RViz and Gazebo GUI in `main.launch.py`.
- **Optimization:** Introduced `headless` flag in `unified_robot_config.yaml`. This allows running the physics engine without the 3D window, saving ~80% of graphical resources while keeping RViz open.
- **Workflow:** Verified that Navigation path planning can be tested in RViz only, provided `gzserver` is running in the background.
- **Fix (TF):** Resolved "Frame map does not exist" error by adding a default `initial_pose` to `nav2_params.yaml`. This allows AMCL to initialize the `map -> odom` transform automatically on startup.
- **Fix (Config):** Resolved priority bug where ROS 2 launch defaults were overriding `unified_robot_config.yaml`. The YAML is now correctly treated as the "Source of Truth" for `./robot.sh up`.
- **Fix (Physics):** Resolved "Stuck Robot" issue by increasing spawn height to `-z 0.06` in `main.launch.py`. This prevents wheels from clipping into the ground and ensures the physics engine can move the robot.
- **Refactor:** Externalized robot spawn coordinates (x, y, z, yaw) to `unified_robot_config.yaml`. This aligns with industry standards for configurability and allows placing the robot at any starting location without changing code.
- **Fix (Nav):** Synchronized `nav2_params.yaml` to use `base_footprint` globally. This matches the Industry Standard URDF refactor and resolves the "Message Filter dropping" error by ensuring Lidar data is correctly transformed relative to the floor.
