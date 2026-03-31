# Agent Instructions

## Session Startup (MUST DO)
1. Read `SESSION_LOG.md` — understand previous work and context
2. Read `PROJECT_ANALYSIS.md` — know all known issues and their status
3. Check `git log --oneline -5` — see recent commits
4. Review the task at hand

## Session Shutdown (MUST DO)
1. Update `SESSION_LOG.md` — add entry with work done, files changed, next steps
2. Update `PROJECT_ANALYSIS.md` — if any issues were fixed, update status to ✅ Done
3. If new issues discovered, add them to `PROJECT_ANALYSIS.md`

## Project Overview
- Autonomous Jetson Robot — ROS2 Foxy with web GUI, SLAM, object detection
- Main control script: `./robot.sh` (sim, robot, web, shell, build, stop, status, logs, clean)
- Web interface: http://localhost:8000
- ROSBridge WebSocket: ws://localhost:9090

## Build & Test
- Build: `./robot.sh build` (builds inside Docker container `auto_ros_foxy`)
- Test: `./run_tests.py --all`
- Test categories: `--docker`, `--ros`, `--web`, `--nav`, `--sim`, `--hardware`
- Tests run from HOST, use `docker exec` for ROS2 commands
- wjwwood serial library is installed manually in container at `/opt/ros/foxy/{include,lib}/`

## Project Structure
```
src/
├── my_robot_launch/    # Launch files, URDF, controllers (main package)
├── web_gui_control/    # Static web interface
├── diffdrive_arduino/  # C++ ros2_control hardware interface
├── object_detection/   # Camera + MobileNetSSD
├── mpu6050_imu/        # I2C IMU driver
└── slam_launch/        # SLAM Toolbox configs
config/                 # unified_robot_config.yaml
test/                   # Legacy hardware diagnostic scripts
test_suite/             # Structured test framework
```

## Conventions
- ROS2 Foxy (ament_python for Python, ament_cmake for C++)
- Launch files in `src/my_robot_launch/launch/`
- URDF/XACRO in `src/my_robot_launch/urdf/`
- Web interface in `src/web_gui_control/`
- Controllers config in `src/my_robot_launch/config/my_controllers.yaml`
- Robot specs: wheel_sep=0.18m, radius=0.035m, enc=3436, serial=/dev/ttyACM0, 115200

## When Making Changes
1. Check `PROJECT_ANALYSIS.md` for related known issues
2. After fixing an issue, mark it ✅ Done in `PROJECT_ANALYSIS.md`
3. Add a Work Log entry in `PROJECT_ANALYSIS.md`
4. Update `SESSION_LOG.md` with summary
5. Follow existing code style and conventions
