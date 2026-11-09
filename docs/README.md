# Documentation

| Section | Document | What's in it |
| :--- | :--- | :--- |
| **Architecture** | [SYSTEM_ARCHITECTURE.md](architecture/SYSTEM_ARCHITECTURE.md) | The main idea, ROS 2 node graph, TF tree, serial protocol and launch modes |
| | [HARDWARE.md](architecture/HARDWARE.md) | Parts, photos, connections, pin map, physical parameters |
| | [ESP32_MOTOR_CONTROL_DESIGN.md](architecture/ESP32_MOTOR_CONTROL_DESIGN.md) | Original PID design for the motor firmware, which differs from what's deployed |
| | [IMU_WIRING.md](architecture/IMU_WIRING.md) | MPU6050 wiring to the ESP32-S3 and its status LED |
| **Guides** | [GETTING_STARTED.md](guides/GETTING_STARTED.md) | Docker setup, `robot.sh` commands, configuration |
| | [MAPPING_AND_NAVIGATION.md](guides/MAPPING_AND_NAVIGATION.md) | Building a map, then navigating with AMCL and Nav2 |
| | [HARDWARE_BRINGUP.md](guides/HARDWARE_BRINGUP.md) | Connecting and checking the real robot, and what to do before the first drive |
| | [OPTIMIZATION_GUIDE.md](guides/OPTIMIZATION_GUIDE.md) | Running the simulation faster |
| **Results** | [RESULTS.md](experiments/RESULTS.md) | Every experiment in simulation and on the robot, with figures |
| **Plans** | [imu_nav2_roadmap_v2.md](plans/imu_nav2_roadmap_v2.md) | IMU and EKF roadmap with verification criteria |
| | [UI_UPGRADE_PLAN.md](plans/UI_UPGRADE_PLAN.md) | Web dashboard upgrade plan |
| **Project records** | [PROJECT_ANALYSIS.md](PROJECT_ANALYSIS.md) | Status board: system health, blockers, roadmap |
| | [SESSION_LOG.md](SESSION_LOG.md) | Dated log of every development session |
| | [LESSONS_LEARNED.md](LESSONS_LEARNED.md) | Lessons learned and known pitfalls |
| | [test_suite/README.md](../test_suite/README.md) | Test categories and how to run them |
