# Robot Test Suite

Smoke and integration checks for autoJetsonBot. Most tests check that the container, nodes, topics and devices exist, and skip when something is absent.

Known issues: the navigation test expects Foxy's `recoveries_server` (Humble has `behavior_server`), the Gazebo checks look for a node named `gzserver` (it's `/gazebo`), and no test drives the motors.

## Quick Start

```bash
# Run all tests
python3 run_tests.py --all

# Run specific test categories
python3 run_tests.py --docker      # Docker environment tests
python3 run_tests.py --ros         # ROS2 core tests  
python3 run_tests.py --web         # Web interface tests
python3 run_tests.py --nav         # Navigation tests
python3 run_tests.py --sim         # Simulation tests
python3 run_tests.py --hardware    # Hardware tests
python3 run_tests.py --unit        # Unit tests, no Docker or ROS needed

# Quick smoke test
python3 run_tests.py --quick
```

## Test Structure

```
test_suite/
├── config.py              # Test configuration and constants
├── utils.py               # Test utilities (ROS2, Web, Docker clients)
├── integration/
│   ├── test_docker.py     # Docker container tests
│   ├── test_ros2_core.py  # ROS2 core (topics, nodes, params)
│   ├── test_web_interface.py  # Web GUI tests
│   ├── test_navigation.py # Nav2 and SLAM tests
│   └── test_simulation.py # Gazebo simulation tests
├── hardware/
│   └── test_hardware.py   # Real hardware tests
└── unit/
    └── test_robot_sh.py   # robot.sh commands, run with stub commands
```

## Prerequisites

Install test dependencies:
```bash
pip install websocket-client requests
```

## Running Tests

### Prerequisites

1. Start Docker container:
   ```bash
   docker start auto_ros_humble
   ```

2. For full tests, start the simulation (under Humble this needs an amd64 host):
   ```bash
   ./robot.sh sim
   ```

### Test Categories

| Category | Description | Requires Simulation |
|----------|-------------|---------------------|
| Docker | Container setup and workspace. Ports 8000 and 9090 once the stack is up, skipped before | No |
| ROS2 Core | Topics, nodes, parameters | Yes |
| Web | Web interface, ROSBridge | Yes |
| Navigation | Nav2, SLAM, TF | Yes |
| Simulation | Gazebo, controllers | Yes |
| Hardware | Device and topic presence (real robot) | Real hardware |
| Unit | `robot.sh` commands, with stub `docker` | No |

## Test Output

unittest prints `ok`, `FAIL` or `skipped` for each test, and the summary line shows ✓ or ✗. Skips are normal when the simulation isn't running.

## Exit Codes

- 0: All tests passed
- 1: Some tests failed
