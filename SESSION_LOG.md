# Session Log

Track work done across sessions. Updated at end of each session.

---

## Session: 2026-03-29 (Part 2 - Fixes)

### Work Done
- Fixed `serial` dependency: added `ros-foxy-serial` to Dockerfile.foxy, removed empty `src/serial/`
- Removed ROS1 code: deleted `diffdrive_arduino/src/diffdrive_robot.cpp`
- Removed duplicate files:
  - `index_modern.html`, `index1.html`, `index_dynamic.html` (web GUI)
  - `robot_core copy.xacro`, `robot_core copy 2.xacro`, `lidar1.xacro`, `robot_dimension.txt` (URDF backups)
  - `mapper_params_online_async-sim.yaml` (identical SLAM config)
  - `autonomous_car_launch_1.py.backup`
  - `.arch/` directory (old web GUI versions)
- Updated 5 launch files to use `mapper_params_online_async.yaml` instead of deleted sim variant
- Removed 5 unused launch files (robot_body_launch_robot, autonomous_car_launch_unified/modern, unified_robot_launch)
- Removed broken `src/robot_body/` directory
- Standardized config values:
  - `gazebo_control.xacro`: wheel_separation 0.15 → 0.18
  - `config.h`: /dev/ttyUSB0→/dev/ttyACM0, 57600→115200, 1920→3436
  - `robot_controller_example.yaml`: wheel_separation 0.3→0.18, radius 0.05→0.035
- Fixed web GUI:
  - Publish rate from UI now wired to JS interval
  - Max velocity settings now read from UI inputs
  - Removed hardcoded IPs (192.168.219.x)
  - Replaced fake system metrics (Math.random) with N/A
  - Added Reset Odometry and Save Map button handlers
  - LaserScan processing optimized (sample every 10th point, throttle warnings)
  - Switched from CDN roslib.js to local copy
- Fixed test bugs:
  - Moved DockerClient import to top of test_hardware.py
  - Fixed discovery pattern in run_tests.py (specific file, not all in directory)
  - Fixed --quick duplication (added run_ros to quick test)
  - Fixed Docker format typo CPUPercs → CPUPerc
  - Fixed bare `except:` clauses → `except Exception:`
  - Fixed WebClient.connect_rosbridge() wrong URL
  - Made PROJECT_ROOT dynamic in test_suite/config.py
- Fixed hardcoded absolute paths:
  - `rviz_body.py`: uses ament_index_python now
  - `object_detection_node.py`: uses ament_index_python now
  - `test_suite/config.py`: uses dynamic path now

### Files Modified
- `Dockerfile.foxy` — added ros-foxy-serial
- `src/web_gui_control/setup.py` — removed deleted HTML refs
- `src/web_gui_control/launch_web_gui/js/robot-controller.js` — multiple bug fixes
- `src/web_gui_control/launch_web_gui/index.html` — local roslib.js
- `src/my_robot_launch/urdf/gazebo_control.xacro` — wheel_separation fix
- `src/diffdrive_arduino/include/diffdrive_arduino/config.h` — serial defaults fix
- `src/diffdrive_arduino/controllers/robot_controller_example.yaml` — wheel params fix
- `src/my_robot_launch/launch/*.py` (5 files) — SLAM config reference update
- `src/my_robot_launch/my_robot_launch/rviz_body.py` — dynamic path
- `src/object_detection/object_detection/object_detection_node.py` — dynamic path
- `test_suite/hardware/test_hardware.py` — import fix
- `test_suite/integration/test_docker.py` — typo fix
- `test_suite/utils.py` — except clauses, URL fix
- `test_suite/config.py` — dynamic PROJECT_ROOT
- `run_tests.py` — discovery pattern, --quick fix

### Files Deleted
- `src/serial/` (empty directory)
- `src/robot_body/` (broken package)
- `src/diffdrive_arduino/src/diffdrive_robot.cpp` (ROS1 code)
- `src/web_gui_control/.arch/` (old GUI versions)
- `src/web_gui_control/launch_web_gui/index_modern.html` (duplicate)
- `src/web_gui_control/launch_web_gui/index1.html` (old version)
- `src/web_gui_control/launch_web_gui/index_dynamic.html` (old version)
- `src/my_robot_launch/urdf/robot_core copy.xacro` (backup)
- `src/my_robot_launch/urdf/robot_core copy 2.xacro` (backup)
- `src/my_robot_launch/urdf/lidar1.xacro` (backup)
- `src/my_robot_launch/urdf/robot_dimension.txt` (old doc)
- `src/slam_launch/config/mapper_params_online_async-sim.yaml` (duplicate)
- `src/my_robot_launch/launch/autonomous_car_launch_1.py.backup`
- `src/my_robot_launch/launch/robot_body_launch_robot.py` (unused)
- `src/my_robot_launch/launch/autonomous_car_launch_unified.py` (unused)
- `src/my_robot_launch/launch/autonomous_car_launch_modern.py` (unused)
- `src/my_robot_launch/launch/unified_robot_launch.py` (unused)

### Next Steps
- Test that the Docker build still works with ros-foxy-serial added
- Consider adding unit tests and functional tests
- Clean up remaining comment-based hardcoded paths (low priority)

---
