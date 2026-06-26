# autoJetsonBot - Project Hub

This is the primary status board. Consult this file to understand the current operational health and the immediate roadmap.

## 📊 Current Status (Updated 2026-06-26)

The stack follows **REP 120** and uses a Python Serial Bridge + mature Web UI. All development phases are complete (Docker + sim verified). All work happens inside Docker (`./robot.sh`).

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Standardized | `map → odom → base_footprint → base_link` (REP 120 everywhere). |
| **Motor Driver** | ✅ Active | `jetson_bot_diffdrive` Python bridge. |
| **Web UI + Global Map** | ✅ Mature | Modular ES6, real-time map/pose/goal overlay. |
| **EKF Fusion** | ✅ Complete | Pipeline + Madgwick + filtered odom + diagnostics ready. |
| **Simulation** | ✅ Functional | Full when launched. |

**Primary Focus:** Phase 4 physical validation — mapping + Nav2 on Jetson Nano car.
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
* None. All core phases completed in code + Docker workflow.

## 🗺️ Roadmap & Active Tasks — Perfect Phases

Phases are accurate as of 2026-06-26. See `docs/imu_nav2_roadmap_v2.md` for the detailed EKF technical plan with verification criteria.

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

### Phase 4: Physical Validation (✅ Complete for Docker/Sim; Ready for Jetson)
- [x] Mapping + autonomous goals verified in sim with fused odometry.
- [x] Jetson-native Docker setup (mac dev uses Xvfb/VNC; native path documented).
- [x] Tests pass with live stack (Docker-aware).
- [x] Calibration docs + procedures in roadmap + AGENTS.md.

## 🛠️ Active Context (2026-06-26)
All phases completed in this session. Docker-only dev. Container idle until `./robot.sh sim/robot/nav`.

**Next for physical:** Deploy to Jetson Nano, run bias calibration (Phase 2a in roadmap), validate on hardware. Use `./robot.sh` for everything.
