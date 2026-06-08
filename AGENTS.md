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
*   **Context:** The system is standardized to `map -> odom -> base_footprint -> base_link`. `base_footprint` is the floor projection, and `base_link` is the physical chassis center. Standardizing to `base_footprint` ensures Lidar data is correctly projected to the Map.

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
*   ⚠️ **Foxy Warning:** AMCL in ROS 2 Foxy requires the array format: `initial_pose: [0.0, 0.0, 0.0, 0.0]`. Using a dictionary (`x: 0, y: 0...`) will fail silently.
*   ✅ **Fix Applied:** Updated `nav2_params.yaml` with the correct array structure.

### 10. Movement & Topic Remapping
*   **Discovery:** Navigation was active and planning paths, but the robot wouldn't move.
*   **Cause:** Nav2 talks on `/cmd_vel`, but `ros2_control` listens on `/diff_cont/cmd_vel_unstamped` by default.
*   ✅ **Fix:** Added remapping in `ros2_control.xacro` and updated Web UI to use the industry standard `/cmd_vel`.

### 11. RViz Panel Stability
*   **Discovery:** RViz failed to load specific configurations with an error regarding `rviz_common/Time`.
*   **Fix:** Surgically removed the `Time` panel from `.rviz` config files. In containerized environments (Xvfb), non-essential panels can cause plugin loading crashes.

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
If `robot.sh stop` fails to terminate Gazebo, it is usually because `gzserver` is still holding the socket. Use `pkill -9 -f gzserver` to clear the slate. A port being open on the host (5900) only means Docker is listening; it doesn't guarantee the internal service is alive.

## 🧩 Hardware & Dependency Insights

### 1. Missing Hardware Plugin (`diffdrive_arduino`)
*   **Context:** The physical robot relies on a custom `diffdrive_arduino` hardware interface plugin for `ros2_control`. 
*   ⚠️ **Critical Discovery:** The plugin source is currently missing from the workspace. Without `libdiffdrive_arduino.so`, the `controller_manager` will fail to spawn the `diff_cont` in `mode:=robot`. 
*   **Mitigation:** Sourcing the original repository or migrating to a standard `ros2_control` hardware interface for the ESP32 is required before physical testing.

### 2. Gazebo Physics: Ground Clipping
*   **Discovery:** The robot appeared "stuck" even with active nodes. RViz showed wheels underground and chassis touching the floor.
*   **Cause:** Mismatch between URDF origin and the floor plane.
*   ✅ **Permanent Fix:** Refactored URDF so `base_footprint` is at the floor level ($z=0$) with `base_link` offset 55mm above it. 
*   ✅ **Spawn Fix:** Updated `main.launch.py` to spawn with `-z 0.06`. This ensures the wheels clear the ground and the physics engine can apply traction.

## 🛡️ Physics & UI Lessons (Added 2026-06-05/06)

*   **Finalized Digital Twin (2026-06-07):** The system is now 100% synchronized with manual measurements of the physical hardware.
    *   **Wheel Separation:** 0.212m (Center-to-center)
    *   **Wheel Radius:** 0.034m (3.4cm)
    *   **Ground Clearance:** 0.056m (5.6cm floor-to-chassis)
    *   **Total Weight:** 1.4kg (Physics-accurate inertial values)
*   **URDF Structural Standard:** 
    *   `base_footprint` is at floor level ($z=0$).
    *   `base_link` is at the **bottom-center of the chassis floor** ($z=0.056$).
    *   This standard simplifies height calculations: component $Z$ origin = $Ground\_Clearance + Height\_from\_floor$.
*   **Lidar Offset Logic:** The RPLidar A1 is positioned at `x=0.064` (forward of axle) and `z=0.161` (16.1cm from floor). This accounts for the 9.7cm chassis height plus the 6.4cm mount height.
*   **Clearance Check:** With `wheel_separation: 0.212m` and a 17.8cm chassis, the per-side clearance is **1.7cm**. This eliminates numerical jitter and "explosive" repulsion in Gazebo.
*   **The Overlap Paradox:** Gazebo Classic can handle internal geometry overlaps IF stiffness (`kp`) is extremely high (1M+). If you lower `kp` to "optimize" performance, the collision energy is unlocked, causing sliding. Fix geometry FIRST before softening physics.
*   **AMCL-Centric UI:** When visualizing paths or goals in a robot-relative UI, always transform from the Map frame using `/amcl_pose`, not `/odom`. Odometry drift will cause the path to "float" away from the Lidar walls; AMCL keeps them anchored.
*   **Build Synchronization:** Changes to `.yaml`, `.xacro`, or `.js` files in the `src/` directory are NOT visible to the simulation until a modular build (`./robot.sh build`) is executed. Always rebuild after changing parameters.
*   **Inflation Scaling:** A `cost_scaling_factor` below 5.0 results in thick, "bloated" walls. For precise navigation in narrow spaces, use a value around 3.5 for smooth gradients or 10.0 for sharp boundaries.
*   **Modular Architecture (ES6):** The Web UI now uses native ES6 modules. 
    *   ❌ **Don't:** Mix `import` statements with legacy script tags in `index.html`. Use `type="module"`.
    *   ✅ **Do:** Maintain the `app.js` as the central orchestrator to prevent circular dependencies between sub-managers.
*   **The Pose Fallback Rule:** UI components that transform Map coordinates (like Goals or Paths) must have a fallback. If `/amcl_pose` is empty, the UI should use `/odom` relative to the map origin to prevent "Zero-coordinate jumping."
*   **Initial Settings Sync:** Always sync UI input values to the internal JS `config` object during `init()`. Relying solely on `change` events means the first connection will use incorrect defaults if the user doesn't touch the inputs.
*   **Lidar Front Sector:** For safety metrics, calculate "Front Distance" using a narrow sector (±10°) rather than the whole scan. This prevents peripheral walls from triggering false emergency stops during hallway navigation.
*   **Python Serial Bridge:** Use `serial.readline()` with a reasonable timeout (0.1s) to prevent blocking the main ROS execution thread. Ensure the ESP32 protocol is human-readable (`m v_l v_r\r`) for easy debugging with tools like `minicom` or `screen`.
*   **Odometry Calculation:** When implementing a bridge, ensure TF (`odom -> base_footprint`) is broadcast at the same frequency as the Odometry message. Jitter between these two will cause "shaking" visualizations in the Web UI or RViz.

## 🍎 macOS & ESP32-S3 Stability (Added 2026-06-08)

*   **Serial Thread-Safety:** On ESP32-S3, never call `Serial.print` from a high-priority FreeRTOS task on a different core than the main loop. This causes race conditions that make the USB device "drop" from the Mac host, leading to "Device not configured" errors.
*   **Mac Socat Flags:** For stable serial-to-network bridging on Mac, use `ispeed=115200,ospeed=115200` and `raw` mode. Avoid the legacy `b115200` flag.
*   **Bridge Auto-Healing:** Physical USB ports on Mac often reset after a firmware flash. Always run bridges in a monitor loop (`while true; do socat... done`) to ensure the Docker container doesn't lose connectivity permanently.
*   **The "Nuclear" Clean:** When changing package installation paths (via `setup.cfg`), a standard `colcon build` is not enough. You must run `./robot.sh clean` to delete the old `install/` symlinks, or ROS will try to execute non-existent binaries and crash the whole launch.
*   **JointState Velocities:** Web-based RPM displays usually depend on the `velocity` array in `/joint_states`. If your hardware bridge only publishes `position`, the UI will show `NaN`. Always populate both arrays.
*   **Hardware Audit Guard:** To prevent "Total Launch Failure" on machines without sensors, wrap hardware node launches in `os.path.exists()` checks and disable downstream stacks (SLAM/Nav) if critical topics won't be available.
