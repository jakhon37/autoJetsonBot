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
