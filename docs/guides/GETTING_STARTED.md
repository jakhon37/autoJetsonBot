# Getting Started

Everything runs in Docker and is driven by one script, `robot.sh`. The same workspace runs the Gazebo simulation on a PC and the hardware stack on the robot's Jetson Xavier NX.

## Requirements

| Use | Host | Notes |
| :--- | :--- | :--- |
| Simulation | amd64 Linux PC with Docker | Gazebo Classic has no Humble packages for arm64. On macOS, Docker Desktop works with the built-in VNC display |
| Real robot | Jetson Xavier NX (L4T R35, Ubuntu 20.04) with Docker | ROS 2 Humble runs in an Ubuntu 22.04 container, so the host OS stays as shipped |

## First Run

```bash
git clone https://github.com/jakhon37/autoJetsonBot.git
cd autoJetsonBot
./robot.sh image   # build auto_ros:humble from Dockerfile.humble
./robot.sh build   # build the jetson_bot_* packages inside the container
./robot.sh sim     # Gazebo + SLAM + Web UI
```

Then open:

| Interface | Address |
| :--- | :--- |
| Web dashboard | <http://localhost:8000> |
| rosbridge (used by the dashboard) | `ws://localhost:9090` |
| RViz and Gazebo windows over VNC | `localhost:5900` |
| Camera stream from `web_video_server` | <http://localhost:8080> |

On the robot, `./robot.sh remote` prints the Jetson's addresses and an SSH tunnel command for viewing the dashboard from another PC.

## Commands

| Command | What it does |
| :--- | :--- |
| `./robot.sh image` | Build the Docker image (Humble by default) |
| `./robot.sh build` | Build the workspace inside the container |
| `./robot.sh auto` | Install `web_video_server` if it's missing, then build |
| `./robot.sh up` | Start with the defaults in `unified_robot_config.yaml` |
| `./robot.sh sim` | Simulation with RViz (`sim:=true viz:=true`), in the YAML's mode, mapping by default |
| `./robot.sh robot` | Real hardware, no RViz or Gazebo (`sim:=false viz:=false`) |
| `./robot.sh nav` | Navigation mode on the configured target, saving the map first if it was mapping |
| `./robot.sh map2nav` | Same as `nav`, the recommended way to switch on hardware |
| `./robot.sh status` | Web UI, VNC, launch process, node count, mode, bridge and ESP32 link |
| `./robot.sh logs` | Follow the launch log |
| `./robot.sh verify` | EKF and fusion health checks |
| `./robot.sh shell` | Open a shell in the container, or run one command: `./robot.sh shell "ros2 topic list"` |
| `./robot.sh remote` | Addresses and SSH tunnel command for another PC |
| `./robot.sh stop` | Stop all robot processes and free the ports |
| `./robot.sh clean` | Delete `build/`, `install/` and `log/` |

The committed YAML has `use_sim: false`, so `up`, `nav` and `map2nav` start the hardware stack. For navigation in simulation, use `./robot.sh sim mode:=navigation`.

Launch arguments pass straight through, for example a spawn pose in simulation:

```bash
./robot.sh sim x:=0.5 y:=0 z:=0.06 yaw:=0.0
./robot.sh sim mode:=navigation
```

## Configuration

`src/jetson_bot_bringup/config/unified_robot_config.yaml` is the source of truth for mode, simulation, visualization, map name and spawn pose. Launch arguments on the command line override it for that run only.

Run `./robot.sh build` after changing anything under `src/`. The workspace is built with `--symlink-install`, so edits to existing files often apply on the next launch, but new files always need a build.

## Maps

Every command that restarts or stops the stack (`up`, `sim`, `robot`, `nav`, `map2nav` and `stop`) first saves the current map if the robot was mapping. Copy any map you want to keep before switching. Maps live in `maps/` as `.pgm` + `.yaml` pairs.

## ROS 2 Foxy

The original Foxy image is still available:

```bash
ROS_DISTRO=foxy ./robot.sh image
ROS_DISTRO=foxy ./robot.sh sim
```

Run `./robot.sh clean && ./robot.sh build` after switching distros, because a Foxy `install/` tree won't run under Humble.

## Tests

```bash
python3 run_tests.py --quick      # smoke test
python3 run_tests.py --all        # everything
python3 run_tests.py --hardware   # needs the robot
```

These are smoke and presence checks rather than full tests. The Docker port test currently fails under host networking, see the status board. [test_suite/README.md](../../test_suite/README.md) lists the categories.

## Legacy: Jetson Nano

The robot started on a Jetson Nano B01 with native ROS 2 Foxy. `setup_jetson.sh` and `robot_nano.sh` are kept for reference but are no longer used.
