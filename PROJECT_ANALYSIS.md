# autoJetsonBot - Project Hub

This is the primary status board. Consult this file to understand the current operational health and the immediate roadmap.

## 📊 Current Status: Mapping Ready ✅

The robot is functionally stable in simulation. All core systems (Control, TF, Sensors, UI) are operational and synchronized.

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Connected | `map -> odom -> base_link -> laser_frame` fully resolvable. |
| **Control** | ✅ Active | `diff_cont` publishing `/odom` and responding to `/cmd_vel`. |
| **Sensors** | ✅ Active | Lidar (`/scan`) and Camera (`/image_raw`) publishing. |
| **Slam** | ✅ Mapping | `slam_toolbox` actively generating occupancy grid. |
| **UI** | ✅ Stable | Web Dashboard (8000) and VNC (5900) functional. |

## 🗺️ Roadmap & Active Tasks

### Phase 1: Verification (Current)
- [ ] **Test Drive:** Execute a complete circuit in `lab.world` and verify map consistency.
- [ ] **Map Persistence:** Verify map saving/loading from the `/maps` directory.

### Phase 2: Feature Restoration
- [ ] **Object Detection:** Restore the `object_detection` package.
    - [ ] Download MobileNetSSD model files.
    - [ ] Update node paths to point to correct resources.
- [ ] **IMU Validation:** Confirm simulated IMU data integration for EKF.

### Phase 3: Cleanup & Refactor
- [ ] **Delete Deprecated:** Remove all `*.deprecated` folders in `src/`.
- [ ] **Telemetry:** Implement a simple battery/latency node for the dashboard.

## 🛠️ Active Context (2026-06-02)
We just completed a major migration to modular `jetson_bot_*` packages and resolved a critical "Active but Silent" controller bug caused by Foxy-specific naming constraints. The system is now baseline-stable.

**Primary Focus:** Verification of autonomy stack in simulation.
