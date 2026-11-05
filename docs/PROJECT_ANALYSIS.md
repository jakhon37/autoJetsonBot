# autoJetsonBot - Project Hub

This is the primary status board. Consult this file to understand the current operational health and the immediate roadmap.

## 📊 Current Status (Updated 2026-11-06)

The stack follows **REP 120** and uses a Python Serial Bridge + mature Web UI. Code phases are complete (Docker + sim verified on Foxy). On the Humble stack, the lidar and the ESP32-S3's encoder and IMU streams have been checked on hardware, but the motors haven't been driven yet. All work happens inside Docker (`./robot.sh`). The current target is a Jetson Xavier NX running ROS 2 Humble in Docker; Foxy stays available with `ROS_DISTRO=foxy`.

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Standardized | `map → odom → base_footprint → base_link` (REP 120 everywhere). |
| **Motor Driver** | ✅ Active | `jetson_bot_diffdrive` Python bridge. |
| **Web UI + Global Map** | ✅ Mature | Modular ES6, real-time map/pose/goal overlay. |
| **EKF Fusion** | ✅ Complete | Pipeline + Madgwick + filtered odom + diagnostics ready. |
| **Simulation** | ✅ Functional | Full when launched. Under Humble, needs an amd64 host. |
| **Hardware** | ⚠️ Partial | On 2026-11-05 the RPLidar and ESP32-S3 passed read-only checks under Humble (`/scan` at 6.9 Hz, encoders and IMU at 50 Hz, SLAM building a map). On 2026-11-06 both encoders counted by hand, but far fewer per turn than configured and apparently in only one direction. Motors haven't been driven. The ESP32 camera streams to the Jetson over Wi-Fi but isn't connected to ROS yet. |

**Primary Focus:** Phase 4 physical validation — mapping + Nav2 on the Jetson car.
Optimizations applied (2026-06):
- EKF /odom_filtered enforced for mapping (physical stability)
- Smart mode switching: `./robot.sh map2nav` + auto-save on `nav` / stop (when in mapping)
- Hardware-friendly default `map_dir: /autonomous_ROS/maps` + auto-mkdir
- use_sim_time cleaned in nav2_params for real hardware
- Physical lidar/EKF tuned defaults in mapper_params (min travel, timeout)
- Web UI guidance + save reminders for mode switches
- Launch prints hardware mapping tips
See AGENTS.md for serial/EKF/physical tuning and robot.sh help.

## 🚧 Critical Blockers
* **Motors not driven yet under Humble (2026-11-05).** The ESP32-S3 works on its native USB socket, the one next to the RGB LED (`303a:1001` at `/dev/ttyACM0`). Only read-only checks have run, with 0 motor commands sent.
* **Flashed motor firmware was never committed (2026-11-06).** It's `MOTOR-ESP32S3` with IMU code added from `imu_viz` (MPU6050 on SDA GPIO 1 and SCL GPIO 21, see `docs/IMU_WIRING.md`). No branch, tag or commit in any repo has the combined version, and `MOTOR-ESP32S3` alone sends encoder lines at 20 Hz with no IMU lines. Don't reflash from that repo as it stands, or the IMU stream goes away. It can be rebuilt from those two repos. June's notes and that repo suggest any drive command starts at about half power with a 500 ms timeout, but that's unconfirmed on the board, so keep the wheels off the ground for the first drive.
* **ESP32 camera startup fix isn't merged (2026-11-06).** The camera is an AI-Thinker ESP32-CAM (OV2640) on Wi-Fi, not the Pi Camera v2 the docs assumed, so the kernel's `imx219` errors and the missing `/dev/video*` are expected. Its firmware, [`myESP32CAM-BLACKBOX`](https://github.com/jakhon37/myESP32CAM-BLACKBOX), used to try Wi-Fi once at boot and stay on its hotspot until reset, and to halt if the sensor failed to start. That repo's `wifi-reliability` branch, flashed but local only, retries Wi-Fi every 30 s and restarts on a sensor failure. The retry passed on hardware.
* **ESP32 camera stream collapsed (2026-11-06).** At `http://192.168.219.118/stream` (640×480) it ran at 7.5 to 9.7 fps at 02:05, but only 0.1 to 0.8 fps after 04:00, on both the old and new firmware. Even an 18 KB page from flash downloads at about 5 KB/s, in bursts with 400 to 660 ms gaps and no retransmissions. Signal, other clients and capture load are ruled out. The camera was re-plugged and moved around 02:41, so test it on a phone charger away from the PC next. Turning off power saving cut idle ping from about 300 ms to 13 ms. Video delay hasn't been measured.
* **Nothing publishes camera images.** No launch file starts a camera node, and `camera_go` only opens a local video device, so `/camera/image_raw` has no source. The ESP32 camera needs a node that reads `/stream`. It must be the stream's only client because the firmware serves one request at a time, reconnect if the stream drops, and stamp frames on arrival because they carry no timestamps. It shouldn't poll `/capture`, since each call saves a file to the SD card.
* **Encoders don't appear to report direction or the right scale (2026-11-06).** In a hand test of 5 turns forward and 5 back on each wheel, L moved -3,437 and R +3,866. Neither count reversed, and that's about 344 to 387 counts per turn against 3,436 configured. If the backward turns are confirmed, both B channels are suspect. Until this is fixed, odometry would report about 1/10 of real distance, always as backward, and a spin in place would read as reversing. Left and right aren't swapped. No measurement behind 3,436 was found: it goes back to May 2025, and the firmware's older `dev` branch says 360. L also jumped by -75,118 with no commands sent and R moving 103, after an earlier unexplained -133,420, and it was flagged as a connection problem on 2026-06-08 (Pin 18/GND). Don't change `encoder_cpr` until direction is fixed, because a firmware fix could change the count per turn.
* **IMU bias not calibrated:** at rest the gyro's z axis reads about 0.024 rad/s (1.4°/s), and `/odom_filtered` yaw drifts about 0.1°/s. Phase 2a in the roadmap puts the bias in firmware, which needs the rebuild above, and gyro values currently arrive with only 2 decimals.

## 🗺️ Roadmap & Active Tasks — Perfect Phases

Phases are accurate as of 2026-11-05. See `docs/imu_nav2_roadmap_v2.md` for the detailed EKF technical plan with verification criteria.

### Phase 1: Industry Standard (✅ Complete)
- [x] **URDF Refactor:** `base_footprint` root + measured offsets (lidar, imu_link).
- [x] **Frame Sync:** AMCL, costmaps, slam, ekf, diffdrive all on `base_footprint`.
- [x] **Physics & Grounding:** Correct wheel sep/radius, collision, spawn height.

**Verification:** `ros2 run tf2_tools view_frames`

### Phase 2: Feature Restoration + UI (✅ Complete)
- [x] **Navigation:** DWB + tuned costmaps + recovery.
- [x] **UI:** Full modular ES6 (map, lidar, nav, controls, telemetry).
- [x] **Telemetry & Ops:** RPM, battery, E-stop (Nav2 + zero vel), save map, global map overlay.

**Verification:** Web UI at :8000 shows everything when stack is up.

### Phase 3: Hardware Bridge + Fusion (✅ Complete)
- [x] Python Motor Bridge (`jetson_bot_diffdrive`).
- [x] IMU bridge + Madgwick in hardware launch.
- [x] EKF + `/odom_filtered` (controller, bt_navigator, costmaps aligned via TF) + `publish_tf=false` on bridge.
- [x] IMU TF via URDF (base_link → imu_link) + EKF lever-arm ready.
- [x] Nav2 fully using filtered odometry.
- [x] Digital twin synced.
- [x] Spawn pose driven from config + CLI support (`x:= y:= z:= yaw:=`).
- [x] Dead `diffdrive_arduino` plugin removed from non-sim xacro.
- [x] `use_sim_time` aligned in yaml + launch.
- [x] Docker diagnostics + verification support added.

**Verification (Docker):**
```bash
./robot.sh shell "ros2 topic hz /odom_filtered"
./robot.sh shell "ros2 topic echo /tf | grep imu_link"
# EKF + Nav2 using fused odom; run with ./robot.sh sim or robot for live check
```

### Phase 4: Physical Validation (✅ Complete for Docker/Sim; 🚧 Hardware pending)
- [x] Mapping + autonomous goals verified in sim with fused odometry.
- [x] Jetson-native Docker setup (mac dev uses Xvfb/VNC; native path documented).
- [x] Tests pass with live stack (Docker-aware).
- [x] Calibration docs + procedures in roadmap + AGENTS.md.
- [x] RPLidar on Humble: health OK, `/scan` at 6.5 Hz (2026-11-05).
- [ ] Check that the front of the scan matches the front of the robot.
- [x] ESP32-S3 on Humble: encoders and IMU streaming at 50 Hz, `./robot.sh status` all OK (2026-11-05).
- [ ] Test each wheel's encoder by hand, then drive the motors with someone present.
- [ ] Bring the ESP32 camera's stream into ROS.
- [ ] Run `./robot.sh robot` on the Humble stack and do the bias calibration (Phase 2a in roadmap).
- [ ] Complete a hardware mapping run and a Nav2 goal.

## 🛠️ Active Context (2026-11-05)
The Humble Docker port was made on the Jetson in 2026-08 and committed on 2026-11-05 (PR #2). Docker-only dev. Container idle until `./robot.sh sim/robot/nav`.

**Next for physical:** Check both encoders' B wires (see Critical Blockers), start the container only after USB devices are plugged in, then run bias calibration (Phase 2a in roadmap) and validate on hardware. Use `./robot.sh` for everything.
