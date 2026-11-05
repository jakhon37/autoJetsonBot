# autoJetsonBot - Project Hub

This is the primary status board. Consult this file to understand the current operational health and the immediate roadmap.

## 📊 Current Status (Updated 2026-11-05)

The stack follows **REP 120** and uses a Python Serial Bridge + mature Web UI. Code phases are complete (Docker + sim verified on Foxy). On the Humble stack, only the lidar has been validated on hardware so far. All work happens inside Docker (`./robot.sh`). The current target is a Jetson Xavier NX running ROS 2 Humble in Docker; Foxy stays available with `ROS_DISTRO=foxy`.

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Standardized | `map → odom → base_footprint → base_link` (REP 120 everywhere). |
| **Motor Driver** | ✅ Active | `jetson_bot_diffdrive` Python bridge. |
| **Web UI + Global Map** | ✅ Mature | Modular ES6, real-time map/pose/goal overlay. |
| **EKF Fusion** | ✅ Complete | Pipeline + Madgwick + filtered odom + diagnostics ready. |
| **Simulation** | ✅ Functional | Full when launched. Under Humble, needs an amd64 host. |
| **Hardware** | ⚠️ Partial | On 2026-11-05 the RPLidar passed under Humble (health OK, `/scan` at 6.5 Hz). The Jetson doesn't detect the ESP32 or camera. |

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
* **ESP32 not detected on the Jetson (2026-11-05).** The kernel logged no USB event for it, even after a reboot, so check the cable first. `/var/log/kern.log` last shows it around 2026-09-02 as `303a:1001` (Espressif native USB) on hub port 1-2.1. All four hub ports worked in 2026-08. A status file left in the container shows it running here on 2026-08-21 at `/dev/ttyACM0`, with encoder and IMU lines arriving. Motors, encoders and IMU can't be tested until it's back.
* **Camera not detected (2026-11-05).** The `imx219` driver probes both CSI ports at boot and neither answers (`-121`). This happened on all 20 boots in `/var/log/kern.log`, including one after reseating the ribbon, so no camera has ever been detected on this Jetson. Check the ribbon direction (contacts toward the heatsink), the small connector on the camera board, and the camera model.
* **Nothing publishes camera images.** No launch file starts `camera_go`, so `/camera/image_raw` has no source even with a detected camera.
* **Left encoder:** flagged as a connection problem on 2026-06-08 (Pin 18/GND), with no recorded fix. In the 2026-08-21 status file it had counted about 8.6 times as much as the right encoder (429,862 vs 49,727), so test each wheel by hand.

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
- [ ] Connect the ESP32 and camera, then confirm with `./robot.sh status`.
- [ ] Run `./robot.sh robot` on the Humble stack and do the bias calibration (Phase 2a in roadmap).
- [ ] Complete a hardware mapping run and a Nav2 goal.

## 🛠️ Active Context (2026-11-05)
The Humble Docker port was made on the Jetson in 2026-08 and committed on 2026-11-05 (PR #2). Docker-only dev. Container idle until `./robot.sh sim/robot/nav`.

**Next for physical:** Get the ESP32 and camera detected (see Critical Blockers), start the container only after they're plugged in, then run bias calibration (Phase 2a in roadmap) and validate on hardware. Use `./robot.sh` for everything.
