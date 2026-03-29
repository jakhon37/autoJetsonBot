# Robot Test Suite

Production-level test suite for Autonomous Jetson Robot project.

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
    └── (placeholder for unit tests)
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
   docker start auto_ros_foxy
   ```

2. For full tests, start the simulation:
   ```bash
   ./robot.sh sim
   ```

### Test Categories

| Category | Description | Requires Simulation |
|----------|-------------|---------------------|
| Docker | Container setup, ports, workspace | No |
| ROS2 Core | Topics, nodes, parameters | Yes |
| Web | Web interface, ROSBridge | Yes |
| Navigation | Nav2, SLAM, TF | Yes |
| Simulation | Gazebo, controllers | Yes |
| Hardware | Motors, sensors (real robot) | Real hardware |

## Test Output

Tests will show:
- ✓ Passed tests (green)
- ✗ Failed tests (red)  
- ⊘ Skipped tests (yellow) - normal when simulation not running

## Exit Codes

- 0: All tests passed
- 1: Some tests failed
