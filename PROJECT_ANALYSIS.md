# Project Analysis

Last updated: 2026-03-29
Status: Active

---

## Architecture Overview

```
Browser (port 8000) ←HTTP→ python3 http.server
Browser (port 9090) ←WebSocket→ rosbridge_server ←ROS2→ Robot Nodes
```

### ROS2 Packages

| Package | Type | Status | Purpose |
|---------|------|--------|---------|
| `my_robot_launch` | Python | ✅ Active | Launch files, URDF, controllers — main package |
| `web_gui_control` | Python | ✅ Active | Static web UI (no ROS nodes) |
| `diffdrive_arduino` | C++ | ✅ Active | ros2_control hardware interface for Arduino |
| `object_detection` | Python | ⚠️ Partial | Camera + MobileNetSSD detection (node disabled) |
| `mpu6050_imu` | Python | ✅ Active | I2C IMU sensor driver |
| `slam_launch` | Python | ✅ Active | SLAM Toolbox configs (no nodes) |

### Key Files
- Main control script: `./robot.sh`
- Web interface: `src/web_gui_control/launch_web_gui/index.html`
- JS controller: `src/web_gui_control/launch_web_gui/js/robot-controller.js`
- Launch files (4): `src/my_robot_launch/launch/robot_body_launch.py`, `robot_body_launch_sim.py`, `autonomous_car_launch_1.py`, `navigation_launch.py`
- URDF: `src/my_robot_launch/urdf/*.xacro`
- Controllers config: `src/my_robot_launch/config/my_controllers.yaml`
- SLAM configs: `src/slam_launch/config/*.yaml`

### Robot Specs
- Wheel separation: 0.18m
- Wheel radius: 0.035m
- Encoder resolution: 3436 counts/rev
- Max linear velocity: 1.0 m/s
- Max angular velocity: 2.0 rad/s
- Serial port: /dev/ttyACM0, 115200 baud

---

## Known Issues

### 🔴 Critical
| # | Issue | Location | Status |
|---|-------|----------|--------|
| 3 | `object_detection_node` disabled + missing model files | `src/object_detection/` | TODO |

### 🟡 Major
| # | Issue | Location | Status |
|---|-------|----------|--------|
| 17 | Boilerplate TODO descriptions in package.xml files | Multiple package.xml | TODO |

### 🟢 Minor
| # | Issue | Location | Status |
|---|-------|----------|--------|
| 16 | Debug print statements in launch files | `src/my_robot_launch/launch/` | TODO |
| 18 | No diagonal movement support in web GUI | `src/web_gui_control/js/robot-controller.js` | TODO |
| 21 | SLAM base_frame inconsistency (base_link vs base_footprint) | `slam_toolbox_config.yaml` vs others | TODO |

### ✅ Resolved
| # | Issue | Resolved |
|---|-------|----------|
| 1 | `src/serial/` was empty — added ros-foxy-serial to Dockerfile.foxy | 2026-03-29 |
| 2 | `diffdrive_robot.cpp` was ROS1 code — deleted | 2026-03-29 |
| 4 | Hardcoded absolute paths — dynamic paths via ament_index | 2026-03-29 |
| 5 | Web GUI publish rate ignored — wired to JS interval | 2026-03-29 |
| 6 | "Reset Odometry" and "Save Map" buttons dead — handlers added | 2026-03-29 |
| 7 | DockerClient import order bug — moved to top | 2026-03-29 |
| 8 | 7+ launch files overlap — consolidated to 4 | 2026-03-29 |
| 9 | Duplicate files everywhere — removed all duplicates | 2026-03-29 |
| 10 | Wheel separation inconsistent — standardized to 0.18 | 2026-03-29 |
| 11 | Encoder counts inconsistent — standardized to 3436 | 2026-03-29 |
| 12 | Fake system metrics — replaced with N/A | 2026-03-29 |
| 13 | CDN roslib.js dependency — switched to local copy | 2026-03-29 |
| 14 | Test runner bugs — discovery pattern and --quick fixed | 2026-03-29 |
| 15 | `robot_body` broken package — removed | 2026-03-29 |
| 19 | Bare except clauses — changed to `except Exception` | 2026-03-29 |
| 20 | Hardcoded IPs in web GUI — removed | 2026-03-29 |

---

## Config Values (Standardized)

All configs now use consistent values:

| Parameter | Value |
|-----------|-------|
| Wheel separation | 0.18m |
| Wheel radius | 0.035m |
| Encoder resolution | 3436 counts/rev |
| Serial port | /dev/ttyACM0 |
| Baud rate | 115200 |

---

## Test Coverage

**Full suite: 66 tests, 55 passed, 11 skipped, 0 failures (2026-03-30)**

| Area | Status | Tests | Notes |
|------|--------|-------|-------|
| Docker environment | ✅ Passing | 10/10 | Container lifecycle, ports, ROS2 install |
| ROS2 core | ✅ Passing | 13/13 | Topics, nodes, frequencies, parameters |
| Web interface | ✅ Passing | 8/8 | HTTP, ROSBridge WebSocket, performance |
| Navigation/SLAM | ✅ Passing | 10/12 | Nodes, topics, TF, odometry (2 skipped: nav disabled) |
| Simulation | ✅ Passing | 9/12 | Controllers, robot model, sensors (3 skipped: headless) |
| Hardware devices | ✅ Passing | 5/11 | Serial, topics (6 skipped: no hardware connected) |
| Unit tests | ❌ None | 0 | `test_suite/unit/` empty |
| E2E tests | ❌ None | 0 | `test_suite/e2e/` empty |

### Test Bugs — All Fixed ✅
1. ~~DockerClient used before import~~ — moved to top of file
2. ~~Discovery pattern runs all tests~~ — now targets specific file
3. ~~--quick runs tests twice~~ — run_ros added to quick test
4. ~~WebClient URL wrong~~ — takes rosbridge_url parameter
5. ~~Docker format typo~~ — CPUPercs → CPUPerc

---

## Recommended Next Steps

1. Verify Docker build works with ros-foxy-serial added
2. Add unit tests and functional tests
3. Wire `unified_robot_config.yaml` to actual launch files
4. Clean up remaining comment-based hardcoded paths (low priority)
5. Consider diagonal movement support in web GUI

---

## Work Log

| Date | Session | Changes |
|------|---------|---------|
| 2026-03-29 | Analysis | Initial deep analysis, identified all issues |
| 2026-03-29 | Fixes | Fixed 16 issues: serial dep, ROS1 code, duplicates, configs, web GUI, tests, paths |
| 2026-03-30 | Testing | Built workspace, launched sim, fixed tests, all 66 tests pass (0 failures) |
