# autoJetsonBot - Project Hub

This is the primary status board. Consult this file to understand the current operational health and the immediate roadmap.

## 📊 Current Status (Updated 2026-11-26)

The stack follows **REP 120** and uses a Python Serial Bridge + mature Web UI. Code phases are complete (Docker + sim verified on Foxy). On the Humble stack, the lidar and the ESP32-S3's encoder and IMU streams have been checked on hardware, and on 2026-11-10 the motors were tuned on the floor under a new closed-loop motor firmware, the gyro's resting offsets went into that firmware, and the robot drove through ROS on the floor. The EKF's settings, fixed after those drives, checked out on the robot the same day, and that evening the firmware began re-learning the gyro's offsets whenever the robot stands still, so the heading now holds at rest. The checks also showed the lidar turned 90° from what the URDF said, and the left encoder's B wire dropping out under power. On 2026-11-23 that wire was redone and held under power, and the recheck found and fixed reversed encoder power and the motor driver's logic supply on a dead pin. That evening's floor check found the two motor plugs swapped left for right, now made up for in the motor firmware, set the floor feed-forward again, and measured the remounted lidar at 0°. With those, the wheels, the gyro, the EKF and SLAM agreed on the floor. On 2026-11-24 the robot mapped its room (`lab_map_v3`) and reached its first Nav2 goal, stopping 6.5 cm short of a goal 1.0 m ahead, inside Nav2's 10 cm tolerance. That run also showed the EKF's distance falling 8.5% behind the wheels under load, because of the bridge's wheel speeds, and two faults in how maps are saved, now fixed and checked in simulation and on the robot. Since the motor board sends the time it read each encoder line's counts, and the bridge times the speeds by it, the EKF's distance matched the wheels within 0.1% on the same goal under heavier load. All work happens inside Docker (`./robot.sh`). The current target is a Jetson Xavier NX running ROS 2 Humble in Docker; Foxy stays available with `ROS_DISTRO=foxy`.

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Standardized | `map → odom → base_footprint → base_link` (REP 120 everywhere). |
| **Motor Driver** | ✅ Floor-tuned | Since 2026-11-10 the ESP32-S3 runs closed-loop speed control, with gyro offsets learned at rest since that evening. Since 2026-11-23 its firmware is `e05cc20`, on `MOTOR-ESP32S3`'s `main` as `1d423bc`, which swaps the left and right pin sets for the robot's swapped motor plugs and holds feed-forward set on the floor that day: at 0.03 and 0.15 m/s both wheels' trims ended within 5 PWM. Since 2026-11-24 it runs `9833afd`, the same with the time the counts were read added to each encoder line. The wheels stop 0.75 s after commands stop. Through `/cmd_vel` on the floor, 30 cm asked read 31.1 cm on `/odom` and an 86° turn +85.5°, with the gyro within 1° of the wheels. |
| **Web UI + Global Map** | ✅ Mature | Modular ES6, real-time map/pose/goal overlay. |
| **EKF Fusion** | ✅ Follows the wheels under load | On the floor on 2026-11-10 the EKF turned its heading −15° and +47° on 30 cm straight drives, because it fused the wheels' x and y and the uncalibrated accelerometer as well as speeds. It now fuses the wheels' speeds, sideways included, and the gyro's yaw rate only. On the robot that read +2.4° on a 30 cm drive, with the wheels at +2.7° and the gyro at +2.3°, and the gyro's 79.1° through a turn. At rest it follows the gyro, whose offsets drift as the robot warms up, so the firmware re-learns them whenever the robot stands still: the heading at rest moved −0.11°/min, where it had crept +4.8°/min. On 2026-11-24 its distance read 8.5% short of the wheels on a Nav2 goal and 2.1% short while mapping, because the bridge's wheel speeds read low when the Jetson is busy. With the speeds timed by the motor board's clock it matched the wheels within 0.1% on the same goal, at a load average of 11 to 15 (Known Issues). |
| **Simulation** | ✅ Functional | Full when launched. Under Humble, needs an amd64 host. On 2026-11-09 the code review fixes ran in Gazebo on a PC, which found and fixed three more bugs, see Known Issues. |
| **Hardware** | ⚠️ Partial | The RPLidar and ESP32-S3 pass checks under Humble (`/scan` at about 7 Hz, encoders and IMU at 50 Hz, SLAM building a map). On 2026-11-09/10 the encoders were fixed and measured at 1,497 counts per turn, and the motors were tuned and driven on the floor. The lidar's 0° points straight ahead since it was remounted on 2026-11-23, within 1° on four straight drives, and `laser_frame` says so. Before, it pointed to the robot's right. The left encoder's B wire, which dropped out under power, was redone on 2026-11-23 and now changes with A under power and on the floor. The camera is the CSI camera below. The robot's inside was rebuilt on 2026-11-15 to 17: a tray for the Jetson and the motor board, a battery each for the Jetson and the motors with their own switches and USB-C charging, a DC jack for the Jetson's adapter, a swivel caster and vents. Since 2026-11-26 the URDF and [HARDWARE.md](architecture/HARDWARE.md) describe that body. |
| **Mapping + Nav2** | ✅ First goal on the robot | On 2026-11-24 the robot mapped its room as `lab_map_v3`, whose walls lie within 2 to 4 cm of straight lines on 5 cm cells. Nav2's first goal on the robot, 1.0 m ahead, succeeded in 8.9 s with no recoveries and stopped 6.5 cm short by tape, inside its 10 cm tolerance. Nav2 now treats the robot as a 0.19 m circle and drives at up to 0.15 m/s and 0.5 rad/s. |
| **Camera** | ✅ In ROS | Since 2026-11-23 an IMX219 CSI camera on the Jetson's CAM0 connector, read on the host by `tools/csi_camera_server.py` and published by `jetson_bot_camera` on `/camera/image_raw` and `/camera/image_raw/compressed` at 15 fps (820×616). The Web UI shows it. It replaced the Wi-Fi ESP32-CAM. Since 2026-11-24 the Jetson's JPEG engine encodes the frames, so the server takes a third of a core instead of three quarters, and with the whole stack running frames reach the topic a median 21 ms after capture instead of 36 ms. The same day its lens was locked and calibrated, and the calibration is published on `/camera/camera_info`. A second calibration, with views across the picture, replaced the first, provisional one, whose optical centre was 41 px off. The URDF has the camera's measured place, 3.75 cm left of the centre line on the front face, looking 2° down. |

**Primary Focus:** Phase 4 physical validation. The first map and Nav2 goal on the robot are done (2026-11-24). Next are longer goals.
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
* **Motor plugs swapped, made up for in firmware (2026-11-23).** Since the rewiring the two motor plugs are swapped left for right, each with its encoder: the driver's A side and encoder pins 18 and 14 now go to the right wheel. The motor firmware on `MOTOR-ESP32S3`'s `main` (`1d423bc`) swaps the pin sets to match, so if the plugs are ever put back, swap the pin sets back in the firmware and check which wheel turns when one is driven. An earlier fix for the same symptom, `reversed-motor-wiring`, reversed each motor instead: straight drives went forward, turns went the wrong way, and the robot hit a wall once (experiment 17).
* **Camera calibration and colour (2026-11-24).** The CSI camera replaced the ESP32-CAM, so that camera's open items no longer block the robot. The new camera's lens was refocused by hand on 2026-11-23 and locked on 2026-11-24 without losing focus. Its calibration's views reached all of the picture but its four corners, where it is extrapolated and bearings may be several degrees off. Add views in the corners (`tools/camera_intrinsics.py`). With the Jetson's default tuning the picture had a magenta cast that turned pink toward the edges. The camera server corrects it from a measurement on white paper under the room's light (`./robot.sh camera-color`), so white paper comes out neutral grey to within about 3%. That hasn't been checked in daylight. The camera's yaw on the robot isn't measured.
* **Motor board can come up silent after a power cut (2026-11-23).** After the Jetson lost power and restarted, the ESP32-S3 sat in its ROM download mode (`boot:0x2`, "waiting for download"), so no `e` or `i` lines came. Resets through the USB control lines left it there. esptool's watchdog reset brought it back without unplugging; see [LESSONS_LEARNED.md](LESSONS_LEARNED.md). From 2026-11-11 to 2026-11-23 the board was also on its COM socket (a CH343, `1a86:55d3`), where the firmware doesn't talk, so the 2026-11-17 run got no wheel or IMU data. It's back on the native socket, and launch and `./robot.sh status` now warn when the COM socket is all they find.

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

* **`/odom` covariance:** The bridge sent all zeros, which the EKF treats as near-perfect. It now sends variances of 0.001 for x, y and vx, 0.1 for yaw and 0.05 for the yaw rate, so the gyro (0.01) leads on rotation as the roadmap plans. They're the `pose_covariance_diagonal` and `twist_covariance_diagonal` parameters, and `diff_cont` in simulation, which also sent zeros, now uses the same values, as Gazebo's `/odom` confirmed. A gyro offset at rest would show in `/odom_filtered`'s yaw rate, so the firmware removes the offsets and re-learns them whenever the robot stands still.

**Fixed (2026-11-24):**

* **Dashboard reconnect loops:** A dashboard tab could reconnect to rosbridge forever. `connect()` closed the previous connection with its handlers still attached, so that close scheduled another `connect()`, and a failed attempt scheduled two reconnects, one of which could no longer be cancelled. Clicking Connect while a reconnect was pending started a loop that reconnected every 2 s and subscribed to every topic again, `/map` included, and overlapping loops closed each other's connections before they opened. On 2026-11-24 a tab left open reconnected 24 times a second for 12 minutes, and rosbridge took 59% of a core. A replaced connection is now closed without its handlers, and only one reconnect can be pending. In a browser against rosbridge on the PC, the same clicks and outages now end in one connection. An open dashboard tab gets the fix when reloaded.
* **Wheel speeds read low under load:** Timing each `e` line by when the bridge handled it gave the second of two lines read together a near-zero dt and a speed spike. Sharing a tick's time between its lines, at least 1/50 s each, stopped the spikes, but a late tick then gave its line a lower speed and nothing gave one a higher, so the speeds read low, more so the busier the Jetson. The EKF fuses only those speeds, so its distance read 8.5% short of the wheels on a Nav2 goal and 2.1% short while mapping. The motor firmware now sends `e <left> <right> <us>`, the microseconds at which its control task read the counts, and the bridge divides the change in counts by the change in that time. A line without it counts as one 50 Hz period (`encoder_rate`): right on average, but the line after a lost one reads double. `test_odometry.py` simulates bunched, late, stalled and lost lines, and a minute of a busy timer, on which the old rule read 7% low. On the same 1 m goal on the robot, at a load average of 11 to 15, the EKF's distance matched the wheels within 0.1%, and `/odom` held 0.142 m/s within 0.007 m/s while cruising, about one count per line.
* **Nav2 footprint:** `robot_radius` was 0.15 m, but the chassis' front corners are 0.181 m from `base_footprint`. Both Nav2 files now use a 0.19 m circle with 0.30 m inflation, and `test_configs.py` checks that the circle holds the URDF's chassis.
* **Map saving:** slam_toolbox's `save_map` started `map_saver_cli` for each save, which waits 2 s for `/map`, and on the busy Jetson a new node took about 3 s to find the others, so a save failed. The Web UI's Save Map reported success whatever the service returned, and `robot.sh`'s auto-save didn't check. Mapping mode now runs Nav2's map saver beside slam_toolbox, with a 15 s wait. The Web UI and `robot.sh` (`stop`, `nav`, `map2nav` and the new `save-map`) call it and report a failed save as failed, and `nav` and `map2nav` stay in mapping mode if theirs fails. The dashboard saves over a connection of its own, since on its teleop connection a slow save held back drive and stop commands. Checked in Gazebo and on the robot, see [RESULTS.md](experiments/RESULTS.md#23-map-saving-and-teleop-speeds).
* **Unknown cells loaded as free:** `map_saver_cli` wrote `free_thresh: 0.25`, and an unknown pixel (205) reads as 0.196, so Nav2 loaded every unknown cell as free. The map saver now writes 0.196, every committed map YAML has it, and tests check both.
* **Fallback map name:** `robot.sh` and the Web UI saved as `lab_map_v2` when `config.json` was missing. `robot.sh` now takes the map from `unified_robot_config.yaml`, and the Web UI doesn't save without `config.json`.
* **Web UI teleop speed:** The dashboard sends Max Linear Velocity and Max Angular Velocity times its sliders. Their defaults, 1.0 m/s and 2.0 rad/s times 0.50, sent 0.5 m/s and 1 rad/s while the room was mapped. They're now 0.3 and 1.0, which sends the 0.15 m/s and 0.5 rad/s the robot was checked at and Nav2 uses. The page no longer keeps them in the browser, where old pages had stored 1.0 and 2.0, and a test ties them to Nav2's limits.

**Open:**

* **ESP32-S3 download mode:** On 2026-11-09, after the Jetson rebooted, every reset through the board's USB control lines started the ESP32-S3 in its ROM download mode (`boot:0x2`, GPIO 0 read low), silent with the motors off, until its USB cable was unplugged for 5 s. Later flashes restarted it normally. If the board goes silent, power-cycle it first.
* **Left encoder jumps:** The left count jumped by tens of thousands with nothing moving in June and on 2026-11-06, under the old firmware. The broken left encoder wire found on 2026-11-09 may explain them. Watch the counts at rest for a few minutes to confirm they're gone. On 2026-11-10 a floor run lost the left encoder to a loose wire mid-run (`no_counts`). It was reseated, and the hand test passed after. Later that day its B wire dropped out under power again, see Critical Blockers.
* **Accelerometer not calibrated:** At rest it reads 10.33 m/s², with 1.27 m/s² on x. The EKF fuses only the gyro's yaw rate, so odometry doesn't use it.
* **Simulation clock:** Gazebo publishes `/clock` at 10 Hz, so in simulation the EKF publishes at 10 Hz, not 30.
* **Simulation map:** `./robot.sh sim mode:=navigation` loads `maps/lab_map_v3`, a map from the robot that doesn't cover the simulated room. Load `src/jetson_bot_bringup/worlds/lab_small_light.yaml` through the `/map_server/load_map` service, or point `map_dir` and `map_name` at it in the YAML.
* **`robot.sh status` in simulation:** It runs the ESP32 checks there too, and they fail.
* **License metadata:** `LICENSE` is MIT, but every `package.xml` and `setup.py` says Apache-2.0.
* **RViz:** The `Time` panel LESSONS_LEARNED.md says was removed is still in `lab_slam.rviz` and `default_nav.rviz`.
* **Unused files:** `esp32_hardware.xml`, `gazebo_control.xacro`, `face.xacro`, `localization_params.yaml`, `main.rviz` and `default_nav.rviz` aren't referenced by any launch file.
* **Rebuilt body:** The robot hasn't been weighed since the rebuild of 2026-11-15 to 17, so the URDF's masses stand at 1.7 kg. The IMU's place and the caster's size are read from photos, not measured with a ruler. The front cover, which carries the camera, was off on 2026-11-26, after the camera's tilt was measured on 2026-11-24. The Jetson battery's type, the rating of the fuse under the top and a measured motor supply voltage aren't recorded. Nothing measures either battery, and the dashboard's battery value is simulated.
* **Simulated turns:** In Gazebo, `/odom` read a half turn in place 5.3 to 5.5% larger than Gazebo's own pose of the model, with the earlier 68 mm caster ball and with the rebuilt body's caster alike (2026-11-26, one run each). Straight drives matched within 0.1 mm. The cause hasn't been looked into.

## 🗺️ Roadmap & Active Tasks — Perfect Phases

Phases are accurate as of 2026-11-05. See `docs/plans/imu_nav2_roadmap_v2.md` for the detailed EKF technical plan with verification criteria.

### Phase 1: Industry Standard (✅ Complete)
- [x] **URDF Refactor:** `base_footprint` root + measured lidar offset. The `imu_link` pose was read from photos of the rebuilt body on 2026-11-26, to about a centimetre, not measured.
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
- [x] URDF dimensions match the 2026-06-07 measurements and, since 2026-11-26, the rebuilt body's caster, lidar height and IMU place as read from photos.
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
- [x] Check that the front of the scan matches the front of the robot (2026-11-10: it didn't, since the lidar's 0° pointed right, and the frame then said −90°. Remounted by 2026-11-23, it points straight ahead, and the frame says 0°).
- [x] ESP32-S3 on Humble: encoders and IMU streaming at 50 Hz, `./robot.sh status` all OK (2026-11-05).
- [x] Test each wheel's encoder by hand, then drive the motors with someone present (2026-11-10, wheels lifted: 1,497 counts per turn, speeds within 1 to 2%).
- [x] Drive through `/cmd_vel` with the wheels lifted (2026-11-10: 0.308 m for 0.30 m asked, +89.9° for +86°).
- [x] Drive on the floor (2026-11-10: feed-forward set for the floor; 30.9 and 31.3 cm on `/odom` for 30 cm through `/cmd_vel`).
- [x] Bring the camera into ROS (2026-11-23: the CSI camera on `/camera/image_raw` at 15 fps, shown in the Web UI).
- [x] Run `./robot.sh robot` on the Humble stack and do the bias calibration (2026-11-10: gyro offsets in firmware, re-learned whenever the robot stands still; the heading moved −0.11°/min at rest).
- [x] Check the EKF's heading on the robot with the new `ekf.yaml` (2026-11-10: +2.4° on a 30 cm drive, wheels +2.7°, gyro +2.3°).
- [x] Fix the left encoder's B wire for good (2026-11-23: lifted, A and B changed 939 and 923 times in 20 s at 0.1 m/s).
- [x] Drive slowly on the floor after the rewiring, checking `/odom` against the gyro and the floor feed-forward, then measure the lidar's exact angle on a straight drive (2026-11-23: motor plugs found swapped and made up for in firmware, trims within 5 PWM, 31.1 cm and +85.5° on `/odom` for 30 cm and 86° with the gyro within 1°, the lidar at 0° within 1°, and SLAM at 30.8 cm).
- [x] Complete a hardware mapping run and a Nav2 goal (2026-11-24: the room mapped as `lab_map_v3`, and a goal 1.0 m ahead reached in 8.9 s, 6.5 cm short by tape, inside the 10 cm tolerance).
- [ ] Make the bridge's wheel speeds unbiased, so the EKF's distance follows the wheels under load.
- [ ] Goals with turns and across the room.
- [x] Get the camera ready for camera-based docking (2026-11-24: JPEG on the Jetson's encoder, the lens locked and calibrated provisionally, `camera_info` published, and the camera's position and tilt in the URDF).
- [x] Calibrate the camera again with views across the picture (2026-11-24: 16 views reaching all but its corners. The optical centre moved 41 px, and the camera's measured pitch from 7.0° to 2.1°).
- [ ] Add calibration views in the picture's corners, check the camera's colour in daylight, and measure its yaw.

## 🛠️ Active Context (2026-11-24)
The Humble Docker port was made on the Jetson in 2026-08 and committed on 2026-11-05 (PR #2). The Jetson's checkout follows `master` since 2026-11-09. Docker-only dev. Container idle until `./robot.sh sim/robot/nav`.

**Next for physical:** Goals with turns across the room. Navigation loads `lab_map_v3`. In mapping mode stop with `./robot.sh stop f`, since plain `stop`, `nav` and `map2nav` save over the map the YAML names. Keep the robot on its wheels while SLAM runs, or restart SLAM after carrying or turning it by hand: the wheels don't see that, and SLAM lost its track. Start the container only after USB devices are plugged in. Use `./robot.sh` for everything.
