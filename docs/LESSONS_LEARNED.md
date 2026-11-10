# Lessons Learned

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

## 🛡️ Debugging Habits

1.  **The "Rule of Two":** If a configuration change or fix fails twice, **STOP**. Don't try a third variation. Re-read the source code and check your assumptions.
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
*   **Motor Firmware:** Since 2026-11-10 the ESP32-S3 runs `MOTOR-ESP32S3`'s `quadrature-speed-control` branch, IMU included, and since that evening `gyro-offset-learning`, which adds gyro offsets learned at rest. The old build it replaced was never committed and is kept as a 16 MB image outside git. That repo's `tools/wheel_test.py` and `docs/FIRST_DRIVE.md` drive and measure the wheels without ROS.

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
*   **Lidar Offset Logic:** The RPLidar A1 is positioned at `x=0.064` (forward of axle) and `z=0.161` above the chassis floor, which is 0.217 m above the ground. This accounts for the 9.7cm chassis height plus the 6.4cm mount height. It's turned so its 0° points to the robot's right, which `laser_joint` gives as yaw −90° (2026-11-10).
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

## 🔧 First Drive Lessons (Added 2026-11-10)

*   **Silent ESP32-S3:** If the bridge connects but gets nothing, reset the board and read what it prints. `boot:0x2 (DOWNLOAD(USB/UART0))` and `waiting for download` mean it started in ROM download mode. On 2026-11-09 every reset through the USB control lines did that, until the USB cable was unplugged for 5 s. esptool still works in that state, which made it a good moment to back up the old firmware.
*   **No Motor Power Looks Like a Dead Wheel:** The ESP32-S3 runs from the Jetson's USB, so it answers and reports a PWM even when the driver's VM has no battery. A non-zero PWM with no motion and `no_counts` means check the motor supply first.
*   **A Dead B Channel Zeroes a Quadrature Count:** Counting all 4 edges, each A pulse adds a count and then takes it away when B is stuck, so the total stays at 0 while A toggles. Counting one edge of A instead hides the same fault as a count that never reverses. Check each channel's activity on its own, with the firmware's `d` pin report, before blaming the counter.
*   **Jumper Wires Fail at the Crimp:** Encoder signals carry about a milliamp, so thin wire is fine, but push-on jumpers break inside the crimp and loosen with vibration. The left encoder's green (B) wire had broken that way. Secure them, or use crimped JST-PH cables. Only the motor and battery wires need to be thick (20 to 22 AWG).
*   **Mirrored Motors:** The left motor is mounted mirrored, so its encoder counted backwards and its sign had to flip. Only a direction test, with someone watching which way each wheel turns, settles it.
*   **Encoder Resolution in Tests:** At 1,497 counts per turn and 50 Hz, one count is about 7 mm/s of wheel speed, so speed tolerances in odometry tests must allow a count of rounding.
*   **Madgwick Without a Magnetometer Has No Heading:** In `stateless` mode with `use_mag: false`, its orientation comes from gravity alone, so a flat turn doesn't change its yaw. The EKF fused that yaw and kept its heading within 6° through a 90° hand turn. Fuse the IMU's yaw rate only.
*   **Test the Heading by Hand, Flat:** With the wheels lifted, turning the robot by hand shows what the IMU and EKF do while the wheels say nothing moved. Keep it flat and measure only between two key presses: lifting or tilting it changes what the accelerometer sees, and the filter's output swings with it.

## 🧭 Floor and EKF Lessons (Added 2026-11-10)

*   **Fuse Each Thing Once:** Fusing the wheels' x and y along with their speeds, plus an uncalibrated accelerometer, gave the EKF conflicts it settled by turning its heading, so straight drives read as 15° and 47° turns. For a differential drive, fuse the wheels' vx, vy (always 0, which rules out sideways slides) and vyaw, and the gyro's vyaw.
*   **Try EKF Settings Side by Side:** A second `ekf_node` with `publish_tf: false` and its output remapped, for example to `/odom_filtered_test`, sees the same data as the real one without touching TF or SLAM, so one drive compares both.
*   **Send Enough Decimals:** In steps of 0.01 rad/s, the gyro's offsets of 0.01 to 0.06 rad/s couldn't be measured. The `i` line now sends 4 decimals.
*   **Gyro Offsets Move With Temperature:** Offsets measured after a flash were off by 0.003 rad/s (0.17°/s) about 20 minutes into a ROS run, so fixed constants won't hold them. The firmware now re-learns them whenever the robot stands still.
*   **A Loose Encoder Wire Shows as `no_counts`:** The board then drives that wheel on feed-forward alone. Reseat the connector and rerun `wheel_test.py turns` before trusting the counts again.
*   **The Floor Needs Its Own Feed-Forward:** On the floor the right wheel needed about 46 PWM more than in the air at every speed. Set the values from `hold --on-floor` trims, as in the firmware's first-drive checklist.

## 📡 Robot Check Lessons (Added 2026-11-10)

*   **Check Which Way the Lidar Faces:** Put an object straight ahead of the robot and find it in the scan, comparing the median of several scans before and after. On this robot it showed at +101°, so the lidar's 0° points right. SLAM still matched turns, which look the same either way, but read a 31 cm straight drive as 22 cm and a 12.8° turn.
*   **One Frame, One Value:** `main.launch.py` publishes the URDF's fixed frames again as static transforms, and TF keeps whichever copy arrives last, so a yaw changed in one place showed after one start and not the next. Change both. `test_configs.py` checks that they match.
*   **No ": " in the URDF:** gazebo_ros2_control passes the whole URDF as a `--param` override, which rcl reads as YAML. A colon and a space, even in a comment, kept `diff_cont` from loading, so the simulated robot had no `/odom`.
*   **A Half-Dead Encoder Curves the Robot:** When one channel of a quadrature encoder drops out, the counts mostly cancel instead of stopping, so the board sees a slow wheel and drives it harder. On the floor the robot curved right while `/odom` claimed a left curve, and the gyro was right. The `d` line's pin counts show it: the dead channel toggles far less than the other.
*   **Wait for Odometry After a Start:** Right after `./robot.sh robot`, a new ROS process took more than 2 s to get `/odom`. A check that drives should wait for odometry and SLAM's pose, and refuse to drive without them.
*   **Stop Before Relaunching:** `./robot.sh sim` and `./robot.sh robot` stop a running stack first and, in mapping mode, save its map over `lab_map_v2`. When that map isn't wanted, stop or restart the container before relaunching.
*   **Learn Offsets Only When Truly Still:** Stopped motors aren't enough, since a robot pushed by hand or turned slowly reads like an offset. The firmware also waits for still wheel counts, a steady window and a mean near the measured offsets before it learns.
*   **Check Learned Values Against a Direct Reading:** A minute of the old firmware's gyro at rest, read just before flashing, gives the offsets the new firmware should learn. They matched within 0.0002 rad/s.
