# autoJetsonBot - Project Hub

This is the primary status board. Consult this file to understand the current operational health and the immediate roadmap.

## 📊 Current Status: Hardware Ready 🛡️🚀

The robot has been fully refactored to the **Industry Standard (REP 120)** and is now **100% Hardware Ready**. The missing C++ plugin has been replaced by a high-performance Python Serial Bridge.

### System Health
| System | Status | Note |
| :--- | :--- | :--- |
| **TF Tree** | ✅ Standardized | `map -> odom -> base_footprint -> base_link` (REP 120). |
| **Motor Driver**| ✅ Active | Python Serial Bridge handles ESP32 communication. |
| **Navigation** | ✅ Active | Web UI Ground Station replaces RViz/Gazebo for monitoring. |
| **Global Map** | ✅ Active | Real-time Occupancy Map overlay in browser. |

**Primary Focus:** Physical deployment and field testing.

## 🚧 Critical Blockers
*   *None.* All software architectural blockers have been resolved.

## 🗺️ Roadmap & Active Tasks

### Phase 1: Industry Standard (Complete)
- [x] **URDF Refactor:** Implemented `base_footprint` as root with mathematical floor offset.
- [x] **Nav Synchronization:** Aligned AMCL and Costmaps to the new frame structure.
- [x] **Physics Fix:** Resolved ground clipping; robot now sits perfectly at Z=0.

### Phase 2: Feature Restoration (Complete)
- [x] **Navigation Fix:** Resolved BT plugin crashes and "Blind Robot" Lidar filters.
- [x] **UI Modularization:** Split monolith into maintainable ES6 modules.
- [x] **Telemetry:** Real-time RPM, Battery, CPU/Mem, and Front Proximity active.

### Phase 3: Hardware & Fusion (Complete)
- [x] **Global Map Overlay:** Integrate `/map` topic into Web UI for global situational awareness.
- [x] **Motor Bridge:** Implemented `jetson_bot_diffdrive` Python Serial Bridge.
- [x] **Physical Sync:** Verified "Digital Twin" (0.212m/0.034m) with manual hardware audit.
- [x] **macOS Bridge:** Created `robotmac.sh` with self-healing `socat` tunnels for Mac dev.
- [ ] **EKF Integration:** Implement `robot_localization` to fuse IMU and Odometry (Optional).
- [x] **Physical Bringup:** Verified bi-directional serial communication (m-commands and e-telemetry).

## 🛠️ Active Context (2026-06-06)
The system is now "Mapping Ready" and highly stable in simulation. We have successfully modularized the Web UI and restored all tactical telemetry. The next critical step is implementing the Global Map Overlay to achieve 100% parity with RViz monitoring capabilities. The primary bottleneck remains **Physical Hardware Integration**.
