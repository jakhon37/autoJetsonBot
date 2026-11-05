# autoJetsonBot - Project Hub

This is the primary status board. Consult this file to understand the current operational health and the immediate roadmap.

## 📊 Current Status (Updated 2026-11-05)

The stack follows **REP 120** and uses a Python Serial Bridge + mature Web UI. Code phases are complete (Docker + sim verified on Foxy). On the Humble stack, the lidar and the ESP32-S3's encoder and IMU streams have been checked on hardware, but the motors haven't been driven yet. All work happens inside Docker (`./robot.sh`). The current target is a Jetson Xavier NX running ROS 2 Humble in Docker; Foxy stays available with `ROS_DISTRO=foxy`.

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Standardized | `map → odom → base_footprint → base_link` (REP 120 everywhere). |
| **Motor Driver** | ✅ Active | `jetson_bot_diffdrive` Python bridge. |
| **Web UI + Global Map** | ✅ Mature | Modular ES6, real-time map/pose/goal overlay. |
| **EKF Fusion** | ✅ Complete | Pipeline + Madgwick + filtered odom + diagnostics ready. |
| **Simulation** | ✅ Functional | Full when launched. Under Humble, needs an amd64 host. |
| **Hardware** | ⚠️ Partial | On 2026-11-05 the RPLidar and ESP32-S3 passed read-only checks under Humble (`/scan` at 6.9 Hz, encoders and IMU at 50 Hz, SLAM building a map). Motors haven't been driven, and the ESP32 camera hasn't been reached yet. |

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
* **ESP32 camera not reached yet (2026-11-05).** The camera is an ESP32 Wi-Fi camera, not the Pi Camera v2 the docs assumed, so the kernel's `imx219` errors and the missing `/dev/video*` are expected. It shows up on the development PC's USB through a CH340 adapter (`1a86:7523`), but no Espressif device answered a ping of `192.168.219.0/24` and no Espressif access point was in range. Its serial boot log or address is still needed.
* **Nothing publishes camera images.** No launch file starts a camera node, and `camera_go` only opens a local video device, so `/camera/image_raw` has no source. The ESP32 camera needs a node that reads its network stream.
* **Encoders unverified:** the left was flagged as a connection problem on 2026-06-08 (Pin 18/GND), with no recorded fix. It had counted about 8.6 times as much as the right on 2026-08-21 (429,862 vs 49,727). On 2026-11-05 it read -133,420 with the right at exactly 0, and neither changed during 4 minutes at rest. Test each wheel by hand.
* **IMU bias not calibrated:** at rest the gyro's z axis reads about 0.024 rad/s (1.4°/s), and `/odom_filtered` yaw drifts about 0.1°/s.

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

**Next for physical:** Test both encoders by hand (see Critical Blockers), start the container only after USB devices are plugged in, then run bias calibration (Phase 2a in roadmap) and validate on hardware. Use `./robot.sh` for everything.
