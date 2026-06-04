# autoJetsonBot - Project Hub

This is the primary status board. Consult this file to understand the current operational health and the immediate roadmap.

## 📊 Current Status: Navigation Active 🚀

The robot has been fully refactored to the **Industry Standard (REP 120)**. All core systems are synchronized to the `base_footprint` root.

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Standardized | `map -> odom -> base_footprint -> base_link` (REP 120). |
| **Control** | ✅ Active | Smooth motion with traction on simulated floor. |
| **Navigation** | ✅ Active | `bt_navigator` processing goals with active costmaps. |
| **UI** | ✅ Hardened | `./robot.sh status` now performs deep internal health checks. |

**Primary Focus:** Validating physical hardware integration and sourcing the `diffdrive_arduino` plugin.

## 🚧 Critical Blockers
1. **Missing Hardware Plugin:** The source code for `diffdrive_arduino` is missing.
2. **Hardware Launch Logic:** `main.launch.py` needs finalization for `mode:=robot`.

## 🗺️ Roadmap & Active Tasks

### Phase 1: Industry Standard (Complete)
- [x] **URDF Refactor:** Implemented `base_footprint` as root with mathematical floor offset.
- [x] **Nav Synchronization:** Aligned AMCL and Costmaps to the new frame structure.
- [x] **Physics Fix:** Resolved ground clipping; robot now sits perfectly at Z=0.

### Phase 2: Feature Restoration (Active)
- [x] **Navigation Fix:** Resolved BT plugin crashes and "Blind Robot" Lidar filters.
- [ ] **Navigation Validation:** Verify precise obstacle avoidance in RViz.
- [x] **Telemetry:** Active in Web UI via ROS topics.

### Phase 3: Hardware & Fusion (Next)
- [ ] **Plugin Recovery:** Locate or reimplement the `diffdrive_arduino` Foxy hardware interface.
- [ ] **EKF Integration:** Implement `robot_localization` to fuse IMU and Odometry.
- [ ] **Physical Bringup:** Validate serial communication with the ESP32/Arduino base.

## 🛠️ Active Context (2026-06-04)
The system is now "Mapping Ready" and highly stable in simulation. We have successfully restored the Navigation Stack configurations. The primary bottleneck has shifted to **Physical Hardware Integration** due to the missing motor controller plugin and the need for finalized hardware launch logic.
