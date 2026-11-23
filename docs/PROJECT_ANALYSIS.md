# autoJetsonBot - Project Hub

This is the primary status board. Consult this file to understand the current operational health and the immediate roadmap.

## 📊 Current Status (Updated 2026-11-23)

The stack follows **REP 120** and uses a Python Serial Bridge + mature Web UI. Code phases are complete (Docker + sim verified on Foxy). On the Humble stack, the lidar and the ESP32-S3's encoder and IMU streams have been checked on hardware, and on 2026-11-10 the motors were tuned on the floor under a new closed-loop motor firmware, the gyro's resting offsets went into that firmware, and the robot drove through ROS on the floor. The EKF's settings, fixed after those drives, checked out on the robot the same day, and that evening the firmware began re-learning the gyro's offsets whenever the robot stands still, so the heading now holds at rest. The checks also showed the lidar turned 90° from what the URDF said, now corrected but not yet measured exactly, and the left encoder's B wire dropping out under power. On 2026-11-23 that wire was redone and held under power, and the recheck found and fixed reversed encoder power, the motor driver's logic supply on a dead pin, and both motor plugs reversed, the last in the motor firmware. A slow floor check comes next. All work happens inside Docker (`./robot.sh`). The current target is a Jetson Xavier NX running ROS 2 Humble in Docker; Foxy stays available with `ROS_DISTRO=foxy`.

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Standardized | `map → odom → base_footprint → base_link` (REP 120 everywhere). |
| **Motor Driver** | ⚠️ Floor re-check | Since 2026-11-10 the ESP32-S3 runs closed-loop speed control (`quadrature-speed-control`, and since that evening `gyro-offset-learning`, which adds gyro offsets learned at rest), with feed-forward set on the floor: at 0.03 and 0.15 m/s both wheels came within 0.003 m/s of the request, with trims within 11 PWM. They stop 0.75 s after commands stop. Through `/cmd_vel` on the floor, 30 cm asked read 30.9 and 31.3 cm on `/odom`. Since 2026-11-23 the firmware (`reversed-motor-wiring`) also reverses both motors and encoders for the reversed motor plugs. Lifted, both wheels then held 0.100 m/s with no faults, but each motor now runs on its other polarity for forward, so the floor feed-forward needs checking again. |
| **Web UI + Global Map** | ✅ Mature | Modular ES6, real-time map/pose/goal overlay. |
| **EKF Fusion** | ✅ Checked on robot | On the floor on 2026-11-10 the EKF turned its heading −15° and +47° on 30 cm straight drives, because it fused the wheels' x and y and the uncalibrated accelerometer as well as speeds. It now fuses the wheels' speeds, sideways included, and the gyro's yaw rate only. On the robot that read +2.4° on a 30 cm drive, with the wheels at +2.7° and the gyro at +2.3°, and the gyro's 79.1° through a turn. At rest it follows the gyro, whose offsets drift as the robot warms up, so the firmware re-learns them whenever the robot stands still: the heading at rest moved −0.11°/min, where it had crept +4.8°/min. |
| **Simulation** | ✅ Functional | Full when launched. Under Humble, needs an amd64 host. On 2026-11-09 the code review fixes ran in Gazebo on a PC, which found and fixed three more bugs, see Known Issues. |
| **Hardware** | ⚠️ Partial | The RPLidar and ESP32-S3 pass checks under Humble (`/scan` at about 7 Hz, encoders and IMU at 50 Hz, SLAM building a map). On 2026-11-09/10 the encoders were fixed and measured at 1,497 counts per turn, and the motors were tuned and driven on the floor. The lidar is mounted with its 0° to the robot's right, which `laser_frame` says since 2026-11-10. The left encoder's B wire, which dropped out under power, was redone on 2026-11-23 and now changes with A under power. The camera is the CSI camera below. |
| **Camera** | ✅ In ROS | Since 2026-11-23 an IMX219 CSI camera on the Jetson's CAM0 connector, read on the host by `tools/csi_camera_server.py` and published by `jetson_bot_camera` on `/camera/image_raw` and `/camera/image_raw/compressed` at 15 fps (820×616), 14 ms after capture. The Web UI shows it. It replaced the Wi-Fi ESP32-CAM. |

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
* **Floor check after the rewiring (2026-11-23).** On 2026-11-10 the left encoder's B wire (GPIO 14) dropped out under power, toggling 20 times to A's 122 in one drive, and the robot curved right and fell. The wire was redone, and lifted it now changes with A under power, 923 times to 939 in 20 s. The recheck also found the encoders' power reversed, the motor driver's logic supply on the ESP32's 5V pin, which this board leaves unpowered on USB, and both motor plugs reversed. The first two were rewired, the driver now running on 3.3 V. The plugs are made up for in the motor firmware (`reversed-motor-wiring`, `7f52d48`), so if they're ever turned round, that commit must be reverted. Lifted, both wheels now turn forward and hold 0.100 m/s with no faults (experiment 16). Before mapping, drive slowly on the floor and check that `/odom` agrees with the gyro, and check the floor feed-forward again, since each motor now runs on its other polarity for forward.
* **Lidar's exact angle not measured (2026-11-10).** The lidar's 0° points to the robot's right: objects placed around the robot showed at +101° (straight ahead) and at +115° and +145° (front-left) in the scan. SLAM matched turns but read a 31 cm straight drive as 22 cm and a 12.8° turn. `laser_joint` in `lidar.xacro`, the static `laser_frame_broadcaster` in `main.launch.py` and the dashboard's lidar view now all turn the scan −90°, and a config test keeps them equal. In Gazebo SLAM then matched the EKF. On the robot, with the encoder fixed, measure the exact angle on a straight drive (the scan's change fits the direction of travel to about 3°) before a mapping run.
* **Motor firmware branches not merged (2026-11-10).** The board ran a build that was never committed, so it was backed up first, as a verified 16 MB image kept out of git because it may hold Wi-Fi details. Since 2026-11-23 it runs `MOTOR-ESP32S3`'s [`reversed-motor-wiring`](https://github.com/jakhon37/MOTOR-ESP32S3/tree/reversed-motor-wiring) branch at `7f52d48`: [`gyro-offset-learning`](https://github.com/jakhon37/MOTOR-ESP32S3/tree/gyro-offset-learning) (`e23d577`) plus one commit that reverses both motors and encoders for the reversed motor plugs. `gyro-offset-learning` is [`quadrature-speed-control`](https://github.com/jakhon37/MOTOR-ESP32S3/tree/quadrature-speed-control) plus one commit that re-learns the gyro's offsets whenever the robot stands still. `quadrature-speed-control` holds, from the first drive, the left encoder's B on GPIO 14, the left encoder's sign and `ENCODER_CPR` 1,497, then feed-forward set on the floor, gyro values with 4 decimals, the gyro's resting offsets, and the native unit tests' own tuning. All three branches are on GitHub but not merged into `main`, so build the firmware from `reversed-motor-wiring`, where all 43 native unit tests pass.
* **Camera lens and colour (2026-11-23).** The CSI camera replaced the ESP32-CAM, so that camera's open items no longer block the robot. The new camera's lens had been turned out of focus when the camera was fitted, and was refocused by hand to be sharp from about 0.5 m to the far wall. It isn't locked yet, so vibration can turn it again: lock it with its ring or a dab of glue. With the Jetson's default tuning the picture had a magenta cast that turned pink toward the edges. The camera server now corrects it from a measurement on white paper (`./robot.sh camera-color`), so white paper comes out neutral grey to within about 3%.
* **Motor board can come up silent after a power cut (2026-11-23).** After the Jetson lost power and restarted, the ESP32-S3 sat in its ROM download mode (`boot:0x2`, "waiting for download"), so no `e` or `i` lines came. Resets through the USB control lines left it there. esptool's watchdog reset brought it back without unplugging; see [LESSONS_LEARNED.md](LESSONS_LEARNED.md). From 2026-11-11 to 2026-11-23 the board was also on its COM socket (a CH343, `1a86:55d3`), where the firmware doesn't talk, so the 2026-11-17 run got no wheel or IMU data. It's back on the native socket.

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
* **`/odom` covariance:** The bridge sent all zeros, which the EKF treats as near-perfect. It now sends variances of 0.001 for x, y and vx, 0.1 for yaw and 0.05 for the yaw rate, so the gyro (0.01) leads on rotation as the roadmap plans. They're the `pose_covariance_diagonal` and `twist_covariance_diagonal` parameters, and `diff_cont` in simulation, which also sent zeros, now uses the same values, as Gazebo's `/odom` confirmed. A gyro offset at rest would show in `/odom_filtered`'s yaw rate, so the firmware removes the offsets and re-learns them whenever the robot stands still.

**Open:**

* **Nav2 footprint:** `robot_radius` is 0.15 m, but the chassis corners are 0.181 m from `base_footprint`.
* **ESP32-S3 download mode:** On 2026-11-09, after the Jetson rebooted, every reset through the board's USB control lines started the ESP32-S3 in its ROM download mode (`boot:0x2`, GPIO 0 read low), silent with the motors off, until its USB cable was unplugged for 5 s. Later flashes restarted it normally. If the board goes silent, power-cycle it first.
* **Left encoder jumps:** The left count jumped by tens of thousands with nothing moving in June and on 2026-11-06, under the old firmware. The broken left encoder wire found on 2026-11-09 may explain them. Watch the counts at rest for a few minutes to confirm they're gone. On 2026-11-10 a floor run lost the left encoder to a loose wire mid-run (`no_counts`). It was reseated, and the hand test passed after. Later that day its B wire dropped out under power again, see Critical Blockers.
* **Accelerometer not calibrated:** At rest it reads 10.33 m/s², with 1.27 m/s² on x. The EKF fuses only the gyro's yaw rate, so odometry doesn't use it.
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
- [x] Check that the front of the scan matches the front of the robot (2026-11-10: it didn't, since the lidar's 0° points right, and the frame now says −90°).
- [x] ESP32-S3 on Humble: encoders and IMU streaming at 50 Hz, `./robot.sh status` all OK (2026-11-05).
- [x] Test each wheel's encoder by hand, then drive the motors with someone present (2026-11-10, wheels lifted: 1,497 counts per turn, speeds within 1 to 2%).
- [x] Drive through `/cmd_vel` with the wheels lifted (2026-11-10: 0.308 m for 0.30 m asked, +89.9° for +86°).
- [x] Drive on the floor (2026-11-10: feed-forward set for the floor; 30.9 and 31.3 cm on `/odom` for 30 cm through `/cmd_vel`).
- [x] Bring the camera into ROS (2026-11-23: the CSI camera on `/camera/image_raw` at 15 fps, shown in the Web UI).
- [x] Run `./robot.sh robot` on the Humble stack and do the bias calibration (2026-11-10: gyro offsets in firmware, re-learned whenever the robot stands still; the heading moved −0.11°/min at rest).
- [x] Check the EKF's heading on the robot with the new `ekf.yaml` (2026-11-10: +2.4° on a 30 cm drive, wheels +2.7°, gyro +2.3°).
- [x] Fix the left encoder's B wire for good (2026-11-23: lifted, A and B changed 939 and 923 times in 20 s at 0.1 m/s).
- [ ] Drive slowly on the floor after the rewiring, checking `/odom` against the gyro and the floor feed-forward, then measure the lidar's exact angle on a straight drive.
- [ ] Complete a hardware mapping run and a Nav2 goal.

## 🛠️ Active Context (2026-11-10)
The Humble Docker port was made on the Jetson in 2026-08 and committed on 2026-11-05 (PR #2). The Jetson's checkout follows `master` since 2026-11-09. Docker-only dev. Container idle until `./robot.sh sim/robot/nav`.

**Next for physical:** A slow floor drive after the 2026-11-23 rewiring, checking `/odom` against the gyro and the floor feed-forward, then the lidar's exact angle on a straight drive, then a mapping run. Start the container only after USB devices are plugged in. Use `./robot.sh` for everything.
