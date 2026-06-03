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

### 4. Frame Source of Truth
*   ❌ **Don't:** Use `base_footprint` as a root link.
*   ✅ **Do:** Use `base_link`. All SLAM and Nav2 configs are now standardized to this root to prevent "split-tree" errors.

## 🛡️ Efficient Debugging & Loop Prevention

To avoid getting stuck in "trial-and-error" loops, any agent working on this robot **must** follow this protocol:

1.  **The "Rule of Two":** If a configuration change or fix fails twice, **STOP**. Do not try a third variation. Re-read the source code of the controller and the full log output (not just the last 10 lines).
2.  **Verify the "Active but Silent" Paradox:** If `ros2 control list_controllers` says **Active** but `ros2 topic hz` shows **0Hz**:
    *   It is always a naming or interface mismatch (e.g., plural vs singular names).
    *   Use `ros2 param dump /<node_name>` to see what the node *actually* loaded.
3.  **Clock First, TF Second:** Never debug "Unknown Frame" errors until you have verified that the Gazebo clock is ticking (`ros2 topic hz /clock`).
4.  **Grepping for Truth:** Use specific grep patterns: `docker exec auto_ros_foxy grep -iE "error|fail|exception|empty" /tmp/sim.log`.
5.  **Non-Interactive Verification:** Use the fixed `robot.sh shell "command"` to run quick checks without hanging the terminal.

## 🔍 Technical Implementation Notes

### VNC / GUI Architecture
Since macOS Docker Desktop cannot forward X11 correctly (Mesa driver failure), we use:
*   `Xvfb :99` (Virtual Framebuffer) inside the container.
*   `x11vnc` to expose the display on Port 5900.
*   `openbox` as the window manager.
*   **Env Vars:** `QT_X11_NO_MITSHM=1`, `LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=softpipe`.

### Process Lifecycle (`robot.sh`)
*   The script uses `pkill -9` on specific regex patterns to ensure "Zombie" Gazebo or ROS nodes don't survive.
*   Port cleanup is handled via `netstat` analysis to find PIDs holding 8000, 9090, or 5900.
