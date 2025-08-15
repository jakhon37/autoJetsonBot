# 🤖 Autonomous Jetson Robot

A modern ROS2 Foxy autonomous robot system with web-based control interface, designed for Jetson Nano deployment.

## ✨ Features

- 🎮 **Gazebo Simulation** - Full robot simulation environment
- 🌐 **Modern Web Interface** - Professional control dashboard
- 🎯 **ROS2 Control Integration** - Industry-standard robot control
- 📱 **Mobile-Friendly** - Responsive design for all devices
- ⌨️ **Keyboard Control** - WASD + Arrow key support
- 📊 **Real-time Monitoring** - Live metrics and system status
- 🔧 **Easy Configuration** - Simple setup and deployment

## 🚀 Quick Start

### Prerequisites
- Docker installed and running
- Container `auto_ros_foxy` available

### Simple Commands

```bash
# Start simulation with web interface
./robot.sh sim

# Start real robot
./robot.sh robot

# Open web control interface
./robot.sh web

# Enter robot container for debugging
./robot.sh shell

# Show robot status
./robot.sh status

# Stop all robot processes
./robot.sh stop
```

## 🌐 Web Interface

Once running, access the robot control interface:

- **Control Dashboard**: http://localhost:8000
- **ROSBridge WebSocket**: ws://localhost:9090

### Controls
- **WASD Keys** or **Arrow Keys**: Move robot
- **Spacebar**: Emergency stop
- **Mouse/Touch**: Use on-screen D-pad
- **Sliders**: Adjust speed limits

## 📁 Project Structure

```
autonomous_jetson_robot/
├── robot.sh                    # Main control script
├── src/
│   ├── my_robot_launch/        # Robot launch files & URDF
│   ├── web_gui_control/        # Modern web interface
│   ├── object_detection/       # Camera & object detection
│   ├── mpu6050_imu/           # IMU sensor integration
│   └── slam_launch/           # SLAM configuration
├── config/                     # Configuration files
└── README.md                  # This file
```

## 🎮 Usage Examples

### Start Simulation
```bash
./robot.sh sim
# Opens Gazebo + Web interface
# Control robot at http://localhost:8000
```

### Control Real Robot
```bash
./robot.sh robot
# Starts real hardware interface
# Control via web interface
```

### Debug & Development
```bash
./robot.sh shell
# Enter container for ROS development
# Full ROS2 environment available
```

## 🔧 Configuration

### Robot Settings
- **Max Linear Velocity**: 1.0 m/s
- **Max Angular Velocity**: 2.0 rad/s
- **Wheel Separation**: 0.18m
- **Wheel Radius**: 0.035m
- **Encoder Resolution**: 3436 counts/rev

### Network Settings
- **Web Server**: Port 8000
- **ROSBridge**: Port 9090
- **Auto IP Detection**: Supports multiple network interfaces

## 📊 Monitoring

The web interface provides real-time monitoring:

- **Wheel RPM**: Left/Right wheel speeds
- **Velocities**: Linear and angular motion
- **System Health**: CPU, memory, battery status
- **Activity Log**: Real-time event logging
- **Connection Status**: Visual connection indicator

## 🛠️ Development

### Build Workspace
```bash
./robot.sh build
```

### View Logs
```bash
./robot.sh logs
```

### Clean Build Files
```bash
./robot.sh clean
```

## 🎯 Key Components

### ROS2 Packages
- **my_robot_launch**: Main robot launch system
- **web_gui_control**: Modern web interface
- **object_detection**: MobileNetSSD object detection
- **mpu6050_imu**: IMU sensor integration
- **slam_launch**: SLAM toolbox configuration

### Hardware Support
- **Motor Control**: TB6612FNG dual motor driver
- **Sensors**: RPLidar A1, MPU6050 IMU, Camera
- **Communication**: Micro-ROS, ROSBridge WebSocket
- **Platform**: Jetson Nano, ROS2 Foxy

## 🚀 Deployment

### Jetson Nano
1. Copy project to Jetson Nano
2. Run `./robot.sh robot` for real hardware
3. Access web interface from any device on network

### Development
1. Use `./robot.sh sim` for simulation testing
2. Develop and test features safely
3. Deploy to real hardware when ready

## 📝 License

MIT License - Feel free to use and modify for your projects.

## 🤝 Contributing

Contributions welcome! The modular architecture makes it easy to add new features:

- **New Sensors**: Add to hardware interface
- **Web Features**: Extend the modern web GUI
- **AI/ML**: Integrate with object detection
- **Navigation**: Add path planning capabilities

---

**Happy Robot Building! 🤖✨**