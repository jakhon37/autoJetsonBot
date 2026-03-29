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
| `diffdrive_arduino` | C++ | ⚠️ Broken | ros2_control hardware interface for Arduino |
| `object_detection` | Python | ⚠️ Partial | Camera + MobileNetSSD detection (node disabled) |
| `mpu6050_imu` | Python | ✅ Active | I2C IMU sensor driver |
| `slam_launch` | Python | ✅ Active | SLAM Toolbox configs (no nodes) |
| `robot_body` | — | ❌ Broken | Not a valid ROS2 package (no package.xml) |

### Key Files
- Main control script: `./robot.sh`
- Web interface: `src/web_gui_control/index.html`
- JS controller: `src/web_gui_control/js/robot-controller.js`
- Launch files: `src/my_robot_launch/launch/*.py`
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
| 1 | `src/serial/` is empty — diffdrive_arduino build fails | `src/serial/` | TODO |
| 2 | `diffdrive_robot.cpp` is ROS1 code in ROS2 package | `src/diffdrive_arduino/diffdrive_robot.cpp` | TODO |
| 3 | `object_detection_node` disabled + hardcoded paths + missing model files | `src/object_detection/` | TODO |
| 4 | Hardcoded absolute paths scattered across packages | Multiple files | TODO |
| 5 | Web GUI publish rate setting is ignored (JS hardcodes 100ms) | `src/web_gui_control/js/robot-controller.js` | TODO |
| 6 | "Reset Odometry" and "Save Map" buttons do nothing | `src/web_gui_control/index.html` | TODO |
| 7 | Test bug: DockerClient used before import | `test_suite/hardware/test_hardware.py:28` | TODO |

### 🟡 Major
| # | Issue | Location | Status |
|---|-------|----------|--------|
| 8 | 7+ launch files with 80%+ overlap → consolidate to 1-2 | `src/my_robot_launch/launch/` | TODO |
| 9 | Duplicate files: index.html=index_modern.html, SLAM configs, xacro copies | Multiple | TODO |
| 10 | Wheel separation inconsistent: 0.15 vs 0.18 vs 0.3 | Various configs | TODO |
| 11 | Encoder counts inconsistent: 1920 vs 3436 | config.h vs ros2_control.xacro | TODO |
| 12 | System metrics in GUI are fake (Math.random) | `src/web_gui_control/js/robot-controller.js` | TODO |
| 13 | CDN dependency for roslib.js despite local copy | `src/web_gui_control/index.html` | TODO |
| 14 | Test runner bugs: --quick runs twice, discovery pattern | `run_tests.py` | TODO |
| 15 | `robot_body` is not a valid ROS2 package | `src/robot_body/` | TODO |

### 🟢 Minor
| # | Issue | Location | Status |
|---|-------|----------|--------|
| 16 | Debug print statements in launch files | `src/my_robot_launch/launch/` | TODO |
| 17 | Boilerplate TODO descriptions in package.xml files | Multiple package.xml | TODO |
| 18 | No diagonal movement support in web GUI | `src/web_gui_control/js/robot-controller.js` | TODO |
| 19 | Bare `except:` clauses in test utils | `test_suite/utils.py` | TODO |
| 20 | Hardcoded IPs in web GUI files | `src/web_gui_control/` | TODO |

---

## Config Inconsistencies

| Parameter | Location A | Value A | Location B | Value B |
|-----------|-----------|---------|------------|---------|
| Wheel separation | gazebo_control.xacro | 0.15 | ros2_control.xacro | 0.18 |
| Wheel separation | my_controllers.yaml | 0.18 | robot_controller_example.yaml | 0.3 |
| Encoder counts | config.h | 1920 | ros2_control.xacro | 3436 |
| Encoder counts | diffbot_hardware.yaml | 3436 | — | — |
| Serial port | config.h | /dev/ttyUSB0 | ros2_control.xacro | /dev/ttyACM0 |
| Baud rate | config.h | 57600 | ros2_control.xacro | 115200 |
| SLAM base_frame | slam_toolbox_config.yaml | base_link | all other SLAM configs | base_footprint |

---

## Test Coverage

| Area | Status | Notes |
|------|--------|-------|
| Docker environment | ✅ Good | Container lifecycle, ports, ROS2 install |
| ROS2 topics/nodes | ✅ Good | Topic existence, publishers, frequencies |
| Web interface | ⚠️ Moderate | HTTP availability, WebSocket connection |
| Navigation/SLAM | ⚠️ Moderate | Node/topic existence, costmaps, TF |
| Simulation | ⚠️ Moderate | Gazebo, controllers, sensors |
| Hardware devices | ⚠️ Moderate | Device nodes, topic presence |
| Unit tests | ❌ None | `test_suite/unit/` empty |
| E2E tests | ❌ None | `test_suite/e2e/` empty |
| Functional tests | ❌ None | No cmd_vel→movement verification |

### Test Bugs
1. `DockerClient` used before import in `test_hardware.py:28`
2. `run_tests.py` discovery finds all tests in directory, not specific file
3. `--quick` flag runs docker tests + ROS2 tests separately (duplication)
4. `WebClient.connect_rosbridge()` constructs wrong URL
5. Docker format typo `{{.CPUPercs}}` should be `{{.CPUPerc}}`

---

## Recommended Priority Order

1. Fix `serial` dependency for `diffdrive_arduino`
2. Consolidate launch files → 1-2 parameterized
3. Remove duplicate files and backup copies
4. Standardize config values (wheel separation, encoder counts, baud rate)
5. Fix web GUI bugs (publish rate, dead buttons, offline roslib)
6. Fix test bugs (DockerClient import, discovery pattern)
7. Add unit tests and functional tests
8. Make `robot_body` a valid package or remove it
9. Remove hardcoded absolute paths
10. Wire `unified_robot_config.yaml` to actual launch files

---

## Work Log

| Date | Session | Changes |
|------|---------|---------|
| 2026-03-29 | Analysis | Initial deep analysis, identified all issues |
