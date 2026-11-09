# autoJetsonBot - Project Hub

This is the primary status board. Consult this file to understand the current operational health and the immediate roadmap.

## 📊 Current Status (Updated 2026-11-10)

The stack follows **REP 120** and uses a Python Serial Bridge + mature Web UI. Code phases are complete (Docker + sim verified on Foxy). On the Humble stack, the lidar and the ESP32-S3's encoder and IMU streams have been checked on hardware, and on 2026-11-10 both motors ran with the wheels lifted under a new closed-loop motor firmware. Driving through ROS and on the floor are next. All work happens inside Docker (`./robot.sh`). The current target is a Jetson Xavier NX running ROS 2 Humble in Docker; Foxy stays available with `ROS_DISTRO=foxy`.

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Standardized | `map → odom → base_footprint → base_link` (REP 120 everywhere). |
| **Motor Driver** | ⚠️ Wheels lifted only | Since 2026-11-10 the ESP32-S3 runs closed-loop speed control (`quadrature-speed-control`). With the wheels lifted, both hold 0.02 to 0.25 m/s within 1 to 2% and stop 0.75 s after commands stop. Not yet driven through `/cmd_vel` or on the floor. |
| **Web UI + Global Map** | ✅ Mature | Modular ES6, real-time map/pose/goal overlay. |
| **EKF Fusion** | ⚠️ Configured | Madgwick, the EKF and `/odom_filtered` run, but aren't validated on hardware. See Known Issues. |
| **Simulation** | ✅ Functional | Full when launched. Under Humble, needs an amd64 host. On 2026-11-09 the code review fixes ran in Gazebo on a PC, which found and fixed three more bugs, see Known Issues. |
| **Hardware** | ⚠️ Partial | The RPLidar and ESP32-S3 pass checks under Humble (`/scan` at about 7 Hz, encoders and IMU at 50 Hz, SLAM building a map). On 2026-11-09/10 the encoders were fixed and measured at 1,497 counts per turn, and both motors ran with the wheels lifted. Not driven on the floor yet. The ESP32 camera streams to the Jetson over Wi-Fi but isn't connected to ROS yet. |

**Primary Focus:** Phase 4 physical validation — mapping + Nav2 on the Jetson car.
Optimizations applied (2026-06):
- EKF /odom_filtered enforced for mapping (physical stability)
- Smart mode switching: `./robot.sh map2nav` + auto-save on `nav` / stop (when in mapping)
- Hardware-friendly default `map_dir: /autonomous_ROS/maps` + auto-mkdir
- use_sim_time cleaned in nav2_params for real hardware
- Physical lidar/EKF tuned defaults in mapper_params (min travel, timeout)
- Web UI guidance + save reminders for mode switches
- Launch prints hardware mapping tips
See [LESSONS_LEARNED.md](LESSONS_LEARNED.md) for serial/EKF/physical tuning and robot.sh help.

## 🚧 Critical Blockers
* **Not driven through ROS or on the floor yet (2026-11-10).** The motors have only run from `MOTOR-ESP32S3`'s `wheel_test.py`, with the wheels lifted. Next: drive through `/cmd_vel` with the wheels still lifted and check `/odom`, then the floor step of that repo's `docs/FIRST_DRIVE.md`, which raises the feed-forward for the floor. The ESP32-S3 is on its native USB socket, the one next to the RGB LED (`303a:1001` at `/dev/ttyACM0`).
* **Motor firmware branch not pushed (2026-11-10).** The board ran a build that was never committed, so it was backed up first, as a verified 16 MB image kept out of git because it may hold Wi-Fi details. It now runs `MOTOR-ESP32S3`'s `quadrature-speed-control` branch at `0051e9a`, which adds four commits from the first drive: the left encoder's B on GPIO 14, the left encoder's sign, `ENCODER_CPR` 1,497 and the measured feed-forward. The branch exists only on the PC, so push it before building the firmware anywhere else.
* **ESP32 camera startup fix isn't merged (2026-11-06).** The camera is an AI-Thinker ESP32-CAM (OV2640) on Wi-Fi, not the Pi Camera v2 the docs assumed, so the kernel's `imx219` errors and the missing `/dev/video*` are expected. Its firmware, [`myESP32CAM-BLACKBOX`](https://github.com/jakhon37/myESP32CAM-BLACKBOX), used to try Wi-Fi once at boot and stay on its hotspot until reset, and to halt if the sensor failed to start. That repo's `wifi-reliability` branch, flashed but local only, retries Wi-Fi every 30 s and restarts on a sensor failure. The retry passed on hardware.
* **ESP32 camera delivery at 640×480 is unreliable (2026-11-06).** On a phone charger away from the PC, one boot delivered nothing at 640×480 or larger for 20 minutes, with `/capture` returning HTTP 500 after 4 s, while 320×240 and 160×120 worked. The cause is unknown. After the camera was moved and rebooted, `http://192.168.219.118/stream` to the Jetson runs at 9.8 fps at 320×240 and about 3 fps at 640×480 with 24 KB frames, with stalls of up to 3 s at 640×480. The slowdown seen after 04:00, when an 18 KB page from flash downloaded at about 5 KB/s, is gone: the page now downloads at about 150 to 180 KB/s and idle ping is under 10 ms. Power source, position and time of day all changed together, so the cause of that recovery isn't known either. Video delay hasn't been measured.
* **Nothing publishes camera images.** No launch file starts a camera node, and `camera_go` only opens a local video device, so `/camera/image_raw` has no source. The ESP32 camera needs a node that reads `/stream`. It must be the stream's only client because the firmware serves one request at a time, reconnect if the stream drops, and stamp frames on arrival because they carry no timestamps. It shouldn't poll `/capture`, since each call saves a file to the SD card.
* **IMU bias not calibrated:** at rest the gyro's z axis reads about 0.024 rad/s (1.4°/s), and `/odom_filtered` yaw drifts about 0.1°/s. Phase 2a in the roadmap puts the bias in firmware. The new motor firmware doesn't do that yet, and it still sends gyro values with only 2 decimals.

## 🔎 Known Issues (Code Review, 2026-11-07)

Found by reading the code while checking the docs. Seven were fixed on 2026-11-07, each in its own commit with a test where the repo allows one. On 2026-11-09 they ran in Gazebo under Humble on an amd64 PC, which found three more bugs, all fixed the same day. The bridge's changes still need the robot.

**Checked in simulation (2026-11-09):**

* **`./robot.sh watch`:** It was defined twice, so its refresh loop never ran. It now reruns `status` every 10 s, and the fusion checks stay under `verify`. `python3 run_tests.py --unit` tests it with stub commands. With the simulation up, it redrew the status about every 15 s, the 10 s pause plus the time `status` takes.
* **Simulation TF:** `diff_cont` and the EKF both published `odom → base_footprint`. `diff_cont` now has `enable_odom_tf: false` in `unified_robot_config.yaml`, so the EKF is the only publisher, as on the robot. In Gazebo the parameter reads false, all 90 transform samples from a 9 s drive and stop equal an `/odom_filtered` pose with the same stamp, and the largest step between samples is 1.5 cm. Humble's `diff_cont` still creates a `/tf` publisher with its transform off, so the check compares content rather than publishers.
* **slam_toolbox odometry:** `mapper_params.yaml` set `odom_topic`, which slam_toolbox doesn't read. It's gone, and the comments now say slam_toolbox takes odometry from the EKF's `odom → base_footprint` transform. No behaviour changes. With the IMU frame fix below, SLAM in Gazebo draws the room's walls straight from that transform.
* **Docker test:** `test_docker.py` checked `docker port`, which prints nothing under host networking. It now connects to ports 8000 and 9090 on localhost while the stack runs, and skips otherwise. With the simulation up on the PC, `python3 run_tests.py --docker` passed 10 of 10 with none skipped. It hasn't run on the Jetson yet.
* **Web UI goals:** Goal and cancel used `ROSLIB.ActionClient` from the bundled roslib 1.4.1, a ROS 1 API, so they probably never reached Nav2. Goals now go to `/goal_pose` as a `geometry_msgs/PoseStamped`, which bt_navigator turns into a NavigateToPose goal as it does for RViz, and cancel calls `/navigate_to_pose/_action/cancel_goal`. The badge now reads status 3 as Canceling, not Recovering. In Gazebo, dashboard goals 1 m and 1.7 m away succeeded, and cancel stopped the robot within 0.6 s of the click. The badge then stuck on Canceling, fixed below.

**Found in simulation and fixed (2026-11-09):**

* **Simulated IMU frame:** `imu.xacro` mounts `imu_link` upside down (roll π), and its Gazebo plugin named no frame, so once Gazebo merged the fixed joints the data was stamped `base_footprint`. The EKF fused the flipped readings without rotating them back and turned the wrong way: in a 4 s counter-clockwise spin `/odom` measured +100° and the EKF −86° to −95°. `diff_cont`'s transform hid this until the TF fix above turned it off. SLAM then drew the room rotated with doubled walls, and Nav2 weaved on a straight goal. The plugin now sets `<frame_name>imu_link</frame_name>`, `test_urdf.py` checks it, and in the same spin the EKF reads +95°. SLAM draws straight walls, and Nav2 drives the straight goal straight.
* **Dashboard badge stuck at Canceling:** The dashboard subscribed to Nav2's goal status with `throttle_rate: 500`. rosbridge drops messages inside that window rather than delaying them, and Nav2 publishes status only on a change, so CANCELED, 20 ms after CANCELING, never arrived. The subscription is no longer throttled, and after a cancel the badge goes from Canceling to Idle within 30 ms.
* **Simulated camera frame:** `camera.xacro` named its frame with ROS 1's `<frameName>`, which the ROS 2 plugin ignores, so `/camera/image_raw` was stamped `base_footprint`. It now uses `<frame_name>`, and Gazebo stamps the images `camera_optical_link`.

**Still to check on the robot:**

* **Odometry spikes:** The bridge took dt from processing time, so the second of two `e` lines handled in one 20 ms tick got a near-zero dt and a speed spike. Now a tick's encoder lines share the time since the last tick that had any, and none counts as shorter than one 50 Hz period (`encoder_rate`). A late timer tick can still make one reading low, never high. `test_odometry.py` simulates bunched, late and lost lines. On the robot, `/odom` speeds should stay steady while driving straight.
* **`/odom` covariance:** The bridge sent all zeros, which the EKF treats as near-perfect. It now sends variances of 0.001 for x, y and vx, 0.1 for yaw and 0.05 for the yaw rate, so the gyro (0.01) leads on rotation as the roadmap plans. They're the `pose_covariance_diagonal` and `twist_covariance_diagonal` parameters, and `diff_cont` in simulation, which also sent zeros, now uses the same values, as Gazebo's `/odom` confirmed. Until the gyro bias is calibrated, its 0.024 rad/s at rest can now show in `/odom_filtered`'s yaw rate.

**Open:**

* **EKF inputs:** Madgwick runs `stateless` without a magnetometer, so its orientation has no heading, yet `ekf.yaml` fuses its yaw.
* **Nav2 footprint:** `robot_radius` is 0.15 m, but the chassis corners are 0.181 m from `base_footprint`.
* **IMU frame:** `imu_link` is upside down (roll π) in the URDF. At rest on 2026-11-09 the real MPU6050 read about −10.2 m/s² on z, so its z axis points down too. The yaw rate's sign is still to check, by turning the robot by hand. The robot's bridge, and since 2026-11-09 the simulation, stamp IMU data `imu_link`, so the EKF applies that roll on both.
* **ESP32-S3 download mode:** On 2026-11-09, after the Jetson rebooted, every reset through the board's USB control lines started the ESP32-S3 in its ROM download mode (`boot:0x2`, GPIO 0 read low), silent with the motors off, until its USB cable was unplugged for 5 s. Later flashes restarted it normally. If the board goes silent, power-cycle it first.
* **Left encoder jumps:** The left count jumped by tens of thousands with nothing moving in June and on 2026-11-06, under the old firmware. The broken left encoder wire found on 2026-11-09 may explain them. Watch the counts at rest for a few minutes to confirm they're gone.
* **Simulation clock:** Gazebo publishes `/clock` at 10 Hz, so in simulation the EKF publishes at 10 Hz, not 30.
* **Simulation map:** `./robot.sh sim mode:=navigation` loads `maps/lab_map_v2`, a map from the robot that doesn't cover the simulated room. Load `src/jetson_bot_bringup/worlds/lab_small_light.yaml` through the `/map_server/load_map` service, or point `map_dir` and `map_name` at it in the YAML.
* **`robot.sh status` in simulation:** It runs the ESP32 checks there too, and they fail.
* **License metadata:** `LICENSE` is MIT, but every `package.xml` and `setup.py` says Apache-2.0.
* **RViz:** The `Time` panel LESSONS_LEARNED.md says was removed is still in `lab_slam.rviz` and `default_nav.rviz`.
* **Unused files:** `esp32_hardware.xml`, `gazebo_control.xacro`, `face.xacro`, `localization_params.yaml`, `main.rviz` and `default_nav.rviz` aren't referenced by any launch file.

## 🗺️ Roadmap & Active Tasks — Perfect Phases

Phases are accurate as of 2026-11-05. See `docs/plans/imu_nav2_roadmap_v2.md` for the detailed EKF technical plan with verification criteria.

### Phase 1: Industry Standard (✅ Complete)
- [x] **URDF Refactor:** `base_footprint` root + measured lidar offset. The `imu_link` pose isn't measured.
- [x] **Frame Sync:** AMCL, costmaps, slam, ekf, diffdrive all on `base_footprint`.
- [x] **Physics & Grounding:** Correct wheel sep/radius, collision, spawn height.

**Verification:** `ros2 run tf2_tools view_frames`

### Phase 2: Feature Restoration + UI (✅ Complete)
- [x] **Navigation:** DWB + tuned costmaps + recovery.
- [x] **UI:** Full modular ES6 (map, lidar, nav, controls, telemetry).
- [x] **Telemetry & Ops:** RPM, battery (simulated), stop button (zero velocity, and a Nav2 cancel through the action's cancel service, checked against Nav2 in simulation), save map, global map overlay.

**Verification:** Web UI at :8000 shows everything when stack is up.

### Phase 3: Hardware Bridge + Fusion (✅ Complete)
- [x] Python Motor Bridge (`jetson_bot_diffdrive`).
- [x] IMU bridge + Madgwick in hardware launch.
- [x] EKF + `/odom_filtered` (controller, bt_navigator, costmaps aligned via TF) + `publish_tf=false` on bridge.
- [x] IMU TF via URDF (base_link → imu_link) + EKF lever-arm ready.
- [x] Nav2 fully using filtered odometry.
- [x] URDF dimensions match the 2026-06-07 measurements.
- [x] Spawn pose driven from config + CLI support (`x:= y:= z:= yaw:=`).
- [x] Dead `diffdrive_arduino` plugin removed from non-sim xacro.
- [x] `use_sim_time` aligned in yaml + launch.
- [x] Docker diagnostics + verification support added.

**Verification (Docker):**
```bash
./robot.sh shell "ros2 topic hz /odom_filtered"
./robot.sh shell "ros2 topic echo /tf_static --once | grep imu_link"
# EKF + Nav2 using fused odom; run with ./robot.sh sim or robot for live check
```

### Phase 4: Physical Validation (✅ Complete for Docker/Sim; 🚧 Hardware pending)
- [x] Mapping + autonomous goals verified in sim with fused odometry.
- [x] Jetson-native Docker setup (mac dev uses Xvfb/VNC; native path documented).
- [x] Tests run with the live stack. The Docker port test, rewritten on 2026-11-07, passed against the simulation on 2026-11-09.
- [x] Calibration docs + procedures in roadmap + LESSONS_LEARNED.md.
- [x] RPLidar on Humble: health OK, `/scan` at 6.5 Hz (2026-11-05).
- [ ] Check that the front of the scan matches the front of the robot.
- [x] ESP32-S3 on Humble: encoders and IMU streaming at 50 Hz, `./robot.sh status` all OK (2026-11-05).
- [x] Test each wheel's encoder by hand, then drive the motors with someone present (2026-11-10, wheels lifted: 1,497 counts per turn, speeds within 1 to 2%).
- [ ] Drive through `/cmd_vel` with the wheels lifted, then on the floor.
- [ ] Bring the ESP32 camera's stream into ROS.
- [ ] Run `./robot.sh robot` on the Humble stack and do the bias calibration (Phase 2a in roadmap).
- [ ] Complete a hardware mapping run and a Nav2 goal.

## 🛠️ Active Context (2026-11-10)
The Humble Docker port was made on the Jetson in 2026-08 and committed on 2026-11-05 (PR #2). The Jetson's checkout follows `master` since 2026-11-09. Docker-only dev. Container idle until `./robot.sh sim/robot/nav`.

**Next for physical:** Drive through `/cmd_vel` with the wheels lifted, now that the bridge uses 1,497 counts per turn, then on the floor, then the bias calibration (Phase 2a in roadmap). Start the container only after USB devices are plugged in. Use `./robot.sh` for everything.
