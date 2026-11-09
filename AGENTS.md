# AGENTS.md — Technical Wisdom & Lessons Learned

This file captures the "Hard-Won" knowledge of the project. Consult this before debugging "Active but Silent" nodes or TF disconnection issues.

## ⚠️ Known Pitfalls & Anti-Patterns

### 1. Controller Naming (ROS 2 Foxy Bug)
The `diff_drive_controller` in ROS 2 Foxy is extremely sensitive. 
*   ❌ **Don't:** Use singular joint names like `left_wheel_name: "name"`. This results in "Wheel names parameters are empty" errors.
*   ✅ **Do:** Use plural list format: `left_wheel_names: ["joint_name"]`. Even for a single wheel.

### 2. The "Ghost Joint" (Broadcaster Hang)
*   ❌ **Don't:** Include `fixed` joints (like the `caster_wheel_joint`) in the `joint_state_broadcaster` list.
*   ✅ **Do:** Only list motorized joints. Broadcasters wait for a state interface that fixed joints don't have, which can cause the entire node to hang silently.

### 3. Simulation Time Drift
*   ❌ **Don't:** Let Gazebo run on its own clock while ROS nodes use system time. This creates "disconnected islands" in the TF tree.
*   ✅ **Do:** Ensure `use_sim_time: true` is set globally in the controller YAML and passed to nodes like RViz2.

### 4. Frame Source of Truth (REP 120)
*   ❌ **Don't:** Use `base_link` as the odometry root.
*   ✅ **Do:** Use `base_footprint`. 
*   **Context:** The system is standardized to `map -> odom -> base_footprint -> base_link`. `base_footprint` is the floor projection, and `base_link` sits on the axle line at chassis-floor height. Standardizing to `base_footprint` ensures Lidar data is correctly projected to the Map.

### 5. Wait for Physics (Gazebo Startup)
*   ❌ **Don't:** Launch controller spawners or SLAM/Navigation nodes immediately after starting Gazebo.
*   ✅ **Do:** On Docker-on-Mac, Gazebo takes 25-35s to initialize its physics engine. Set a delay of at least **90s** for controller spawners and **120s** for higher-level stacks like AMCL/Nav2.

### 6. Foxy Parameter Redeclaration
*   ❌ **Don't:** Blindly use `declare_parameter('use_sim_time')` in Python nodes.
*   ✅ **Do:** ROS 2 Foxy sometimes pre-declares this via `ros-args`. Use `if not self.has_parameter('use_sim_time'):` to prevent crashes.

### 7. Configuration Priority (YAML vs CLI)
*   **Discovery:** ROS 2 Launch arguments with defaults were overriding the `unified_robot_config.yaml` even when not passed.
*   ✅ **Fix Applied:** Refactored `main.launch.py` to use empty defaults for CLI arguments. 
*   **Behavior:** The system now correctly treats the YAML as the "Source of Truth" for `./robot.sh up`. CLI arguments (like `./robot.sh nav`) still act as temporary overrides.

## 🚀 Performance & UI Optimization

### 8. Fast Testing: Headless Mode
*   **Discovery:** Gazebo rendering (`gzclient`) is the main CPU bottleneck.
*   ✅ **Do:** Set `headless: true` in `unified_robot_config.yaml` to run physics in the background.
*   **RViz Synergy:** You can keep `viz: true` while `headless: true`. This gives you the RViz interface for path planning verification without the heavy Gazebo overhead.

### 9. Navigation: The "Missing Map Frame" Startup Bug
*   **Discovery:** RViz often reports "Frame [map] does not exist" on startup, even if the map is loaded.
*   **Cause:** AMCL waits for an `initial_pose` before it publishes the `map -> odom` transform.
*   ⚠️ **Foxy Warning:** AMCL in ROS 2 Foxy requires the array format: `initial_pose: [0.0, 0.0, 0.0, 0.0]`. Using a dictionary (`x: 0, y: 0...`) will fail silently. Unverified: the Humble file (`nav2_params_humble.yaml`) uses `initial_pose.x/.y/.z/.yaw`, and both files hold only zeros, which are AMCL's defaults, so neither format has really been tested.
*   ✅ **Fix Applied:** Updated `nav2_params.yaml` with the correct array structure.

### 10. Movement & Topic Remapping
*   **Discovery:** Navigation was active and planning paths, but the robot wouldn't move.
*   **Cause:** Nav2 talks on `/cmd_vel`, but `ros2_control` listens on `/diff_cont/cmd_vel_unstamped` by default.
*   ✅ **Fix:** Added remapping in `ros2_control.xacro` and updated Web UI to use the industry standard `/cmd_vel`.

### 11. RViz Panel Stability
*   **Discovery:** RViz failed to load specific configurations with an error regarding `rviz_common/Time`.
*   **Fix:** Remove the `Time` panel from `.rviz` config files. In containerized environments (Xvfb), non-essential panels can cause plugin loading crashes. It's still in `lab_slam.rviz` and `default_nav.rviz`, so remove it there if RViz fails to load.

## 🛡️ Efficient Debugging & Loop Prevention
These rules are mirrored as foundational mandates in **`GEMINI.md`**.

1.  **The "Rule of Two":** If a configuration change or fix fails twice, **STOP**. Do not attempt a third variation. Re-read the source code and verify your assumptions.
2.  **Verify the "Active but Silent" Paradox:** If topic hz is 0 but status is Active, it's a naming/interface mismatch. Use `ros2 param dump`.
3.  **Clock First, TF Second:** Never debug "Unknown Frame" errors until you have verified that the simulation clock is ticking and `use_sim_time` is correctly propagated.

## 🔍 Technical Implementation Notes

### VNC / GUI Architecture
Since macOS Docker Desktop cannot forward X11 correctly, we use:
*   `Xvfb :99` (Virtual Framebuffer) inside the container.
*   `x11vnc` to expose the display on Port 5900.
*   `openbox` as the window manager.
*   **Env Vars:** `QT_X11_NO_MITSHM=1`, `LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=softpipe`.

### Process Lifecycle & "Zombies"
If `robot.sh stop` fails to terminate Gazebo, it is usually because `gzserver` is still holding the socket. Use `pkill -9 -f gzserver` to clear the slate. On the old Mac setup with port forwarding, an open port 5900 only meant Docker was listening. With host networking, which the robot uses, it means x11vnc itself is listening.

## 🐳 ROS 2 Humble Docker (Added 2026-08-18)

*   **Default image is Humble.** `./robot.sh image` builds `Dockerfile.humble` → `auto_ros:humble`. Foxy is still available: `ROS_DISTRO=foxy ./robot.sh image`.
*   ❌ **Don't:** Install `ros-humble-ros-gz` in this image. That is Gazebo Fortress. This workspace uses Gazebo Classic plugins (`libgazebo_ros_*.so`, `libgazebo_ros2_control.so`).
*   ❌ **Don't:** Reuse a Foxy `install/` tree inside the Humble container. Run `./robot.sh clean && ./robot.sh build` after switching distro.
*   ✅ **Do:** Install `gazebo-ros-pkgs` + `gazebo-ros2-control` only when apt has them (Humble amd64). On arm64 those debs are unpublished because jammy has no `gazebo11`/`libgazebo-dev`. Hardware mode and SLAM run. Nav2 hasn't been tested on the hardware yet.
*   Launch already switches `spawner.py`→`spawner`, slam `params_file`→`slam_params_file`, and Nav2 to `nav2_params_humble.yaml`.
*   **Current target:** Jetson Xavier NX (L4T R35, Ubuntu 20.04 host) running the Ubuntu 22.04 Humble image through Docker. The robot no longer uses a Jetson Nano. Its native Foxy scripts (`setup_jetson.sh`, `robot_nano.sh`) are kept only for reference.
*   **`use_sim_time` for SLAM:** `online_async_launch.py` passes the launch argument after the params file, so it always beats any value in `mapper_params.yaml`. Tested both ways on 2026-11-05, so don't set it in that YAML.
*   **Serial Ports:** Launch looks for the ESP32 and lidar by USB ID (`/dev/serial/by-id`) because `ttyACM` numbers change after an ESP32 reset. The CP2102 adapter is the lidar. ⚠️ The current container has no `/dev/serial/by-id`, so launch falls back to the first `/dev/ttyACM*` and `/dev/ttyUSB*` it finds, and a second USB serial device (like an ESP32 camera on a data cable) could be picked by mistake. The ESP32-S3's serial link is its native USB socket, the one next to the RGB LED (`303a:1001`). The ESP32-CAM's CH340 (`1a86:7523`) appears as `/dev/ttyUSB*`, so on a data cable it can be taken for the lidar, and opening its port resets the camera. Power it without a data connection on the robot. Restart the container after plugging in USB devices, or it won't see them.
*   **ESP32-CAM Firmware:** Source is `github.com/jakhon37/myESP32CAM-BLACKBOX`. It serves one request at a time, so `/status` hangs while anyone holds `/stream`. `/capture` saves a copy to the SD card on every call, so read frames from `/stream`. Frames carry no timestamps. `/fps?val=N` changes the rate limit until the next reboot. Idle ping doesn't measure video delay. Its programmer board stays in reset if DTR and RTS are held active, so clear them before opening the serial port. Pin the PlatformIO platform, because unpinned it now jumps to Arduino core 3.x.
*   **Motor Firmware:** The build on the ESP32-S3 is `MOTOR-ESP32S3` with IMU code added from `imu_viz` (MPU6050, SDA GPIO 1, SCL GPIO 21), and that combination was never committed. Reflashing from `MOTOR-ESP32S3` as it stands would lose the IMU stream.

## 🧩 Hardware & Dependency Insights

### 1. Removed Hardware Plugin (`diffdrive_arduino`)
*   **Context:** The physical robot once relied on a custom `diffdrive_arduino` hardware interface plugin for `ros2_control`, and its source was missing from the workspace.
*   ✅ **Fix Applied:** The plugin is gone from the non-sim xacro. On hardware, the Python bridge `jetson_bot_diffdrive` talks to the ESP32, so the hardware path no longer needs `libdiffdrive_arduino.so` or `controller_manager`.

### 2. Gazebo Physics: Ground Clipping
*   **Discovery:** The robot appeared "stuck" even with active nodes. RViz showed wheels underground and chassis touching the floor.
*   **Cause:** Mismatch between URDF origin and the floor plane.
*   ✅ **Permanent Fix:** Refactored URDF so `base_footprint` is at the floor level ($z=0$) with `base_link` offset 56 mm above it. 
*   ✅ **Spawn Fix:** Updated `main.launch.py` to spawn with `-z 0.06`. This ensures the wheels clear the ground and the physics engine can apply traction.

## 🛡️ Physics & UI Lessons (Added 2026-06-05/06)

*   **Finalized Digital Twin (2026-06-07):** The URDF's dimensions (wheel separation, radius, clearance, chassis box) match manual measurements of the hardware. Encoder scale, IMU pose and mass were not part of that check.
    *   **Wheel Separation:** 0.212m (Center-to-center)
    *   **Wheel Radius:** 0.034m (3.4cm)
    *   **Ground Clearance:** 0.056m (5.6cm floor-to-chassis)
    *   **Total Weight:** 1.4 kg recorded. The URDF's link masses add up to 1.7 kg.
*   **URDF Structural Standard:** 
    *   `base_footprint` is at floor level ($z=0$).
    *   `base_link` is on the axle line at the height of the chassis floor ($z=0.056$). The chassis box is centred 6.1 cm ahead of it.
    *   Children of `base_link` or `chassis` are placed by their height above the chassis floor, so don't add the ground clearance again.
*   **Lidar Offset Logic:** The RPLidar A1 is positioned at `x=0.064` (forward of axle) and `z=0.161` above the chassis floor, which is 0.217 m above the ground. This accounts for the 9.7cm chassis height plus the 6.4cm mount height.
*   **Clearance Check:** With `wheel_separation: 0.212m` and a 17.8cm chassis, each chassis side is **1.7cm** from the wheel's centre plane, leaving a 0.4 cm gap to the inner face of the 2.6 cm wide wheel. This eliminates numerical jitter and "explosive" repulsion in Gazebo.
*   **The Overlap Paradox:** Gazebo Classic can handle internal geometry overlaps IF stiffness (`kp`) is extremely high (1M+). If you lower `kp` to "optimize" performance, the collision energy is unlocked, causing sliding. Fix geometry FIRST before softening physics.
*   **AMCL-Centric UI:** When visualizing paths or goals in a robot-relative UI, always transform from the Map frame using `/amcl_pose`, not `/odom`. Odometry drift will cause the path to "float" away from the Lidar walls; AMCL keeps them anchored.
*   **Build Synchronization:** New files in `src/` aren't visible until a modular build (`./robot.sh build`). The workspace uses `--symlink-install`, so edits to existing `.yaml`, `.xacro` or `.js` files often apply on the next launch, but rebuild when in doubt.
*   **Inflation Scaling:** Lower `cost_scaling_factor` values make walls look thicker in the costmap, and higher values give sharper boundaries. Both Nav2 configs use 3.5.
*   **Modular Architecture (ES6):** The Web UI now uses native ES6 modules. 
    *   ❌ **Don't:** Mix `import` statements with legacy script tags in `index.html`. Use `type="module"`.
    *   ✅ **Do:** Maintain the `app.js` as the central orchestrator to prevent circular dependencies between sub-managers.
*   **The Pose Fallback Rule:** UI components that transform Map coordinates (like Goals or Paths) must have a fallback. If `/amcl_pose` is empty, the UI should use `/odom` relative to the map origin to prevent "Zero-coordinate jumping."
*   **Initial Settings Sync:** Always sync UI input values to the internal JS `config` object during `init()`. Relying solely on `change` events means the first connection will use incorrect defaults if the user doesn't touch the inputs.
*   **Lidar Front Sector:** For safety metrics, calculate "Front Distance" using a narrow sector (±10°) rather than the whole scan. This keeps peripheral walls from skewing the reading in hallways. In the Web UI it only colours a label and logs; nothing stops automatically.
*   **Python Serial Bridge:** The bridge reads with non-blocking `in_waiting` reads (10 ms timeout) and its own `\r`/`\n` splitter, so it never blocks the ROS thread. Ensure the ESP32 protocol is human-readable (`m v_l v_r\r`) for easy debugging with tools like `minicom` or `screen`.
*   **Odometry Calculation:** On hardware the bridge doesn't broadcast TF (`publish_tf: False`). The EKF publishes `odom -> base_footprint` at 30 Hz while `/odom` runs at about 50 Hz. A bridge that does broadcast TF should keep it in step with its odometry, or the Web UI and RViz will shake.

## 🍎 macOS & ESP32-S3 Stability (Added 2026-06-08)

*   **Serial Thread-Safety:** On ESP32-S3, never call `Serial.print` from a high-priority FreeRTOS task on a different core than the main loop. This causes race conditions that make the USB device "drop" from the Mac host, leading to "Device not configured" errors.
*   **Mac Socat Flags:** For stable serial-to-network bridging on Mac, use `ispeed=115200,ospeed=115200` and `raw` mode. Avoid the legacy `b115200` flag.
*   **Bridge Auto-Healing:** Physical USB ports on Mac often reset after a firmware flash. Always run bridges in a monitor loop (`while true; do socat... done`) to ensure the Docker container doesn't lose connectivity permanently.
*   **The "Nuclear" Clean:** When changing package installation paths (via `setup.cfg`), a standard `colcon build` is not enough. You must run `./robot.sh clean` to delete the old `install/` symlinks, or ROS will try to execute non-existent binaries and crash the whole launch.
*   **JointState Velocities:** Web-based RPM displays usually depend on the `velocity` array in `/joint_states`. If your hardware bridge only publishes `position`, the UI will show `NaN`. Always populate both arrays.
*   **Hardware Audit Guard:** To prevent "Total Launch Failure" on machines without sensors, wrap hardware node launches in `os.path.exists()` checks and disable downstream stacks (SLAM/Nav) if critical topics won't be available.

## 🧪 Gazebo Check Lessons (Added 2026-11-09)

*   **Name Every Sensor Frame:** Gazebo merges links on fixed joints into `base_footprint`, so a sensor plugin without `<frame_name>` stamps its data `base_footprint`. The IMU is mounted upside down (roll π), so its unnamed data turned the EKF's heading the wrong way once `diff_cont` stopped publishing TF. ROS 1's `<frameName>` is ignored too, which left the simulated camera's images stamped `base_footprint`.
*   **rosbridge Throttling Drops Messages:** Inside a subscription's `throttle_rate` window, rosbridge 2.0.8 discards messages rather than delaying them, unless `queue_length` is set. Don't throttle topics that publish only on a change, like Nav2's goal status, which lost CANCELED that way.
*   **Who Publishes a Transform:** Humble's `diff_drive_controller` keeps a `/tf` publisher even with `enable_odom_tf: false`, and ROS 2's `tf2_monitor` doesn't name broadcasters. To show the EKF is the only source, compare each `odom -> base_footprint` sample with `/odom_filtered` at the same stamp.
*   **Sim Clock Rate:** Gazebo publishes `/clock` at 10 Hz, so in simulation the EKF publishes `/odom_filtered` and its transform at 10 Hz, not 30.
*   **Keep a PC's Simulation off the Robot:** With host networking and the default domain, a simulation on a PC shares DDS discovery with the robot on the same LAN, so a test's `/cmd_vel` could reach the real robot. Run the PC's container with `ROS_LOCALHOST_ONLY=1`, which `robot.sh` doesn't set.
