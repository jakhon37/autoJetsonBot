# Migration Guide: ROS2 Control Integration with diffdrive_arduino
## Long-term Development Plan Implementation for Jetson Nano (Foxy)

### Overview
This guide provides step-by-step instructions for migrating from the current fragmented control system to a unified ROS2 control architecture using diffdrive_arduino, specifically designed for ROS2 Foxy deployment on Jetson Nano.

---

## Phase 1: Immediate Migration (Week 1-2)

### 1.1 Current State Assessment
**Existing Issues:**
- Multiple control paradigms (micro_ros, legacy GPIO, ros2_control)
- Hardcoded launch parameters
- Inconsistent encoder configurations
- Topic name conflicts

**Current Architecture:**
```
autonomous_car_launch_1.py (micro_ros) ❌
├── motor_usbserial_node
├── rplidar_node  
└── slam_toolbox

robot_body_launch_robot.py (ros2_control) ❌
├── controller_manager
├── diff_cont spawner
└── joint_broad spawner
```

### 1.2 Migration Steps

#### Step 1: Backup Current Configuration
```bash
# Create backup of current working system
cp -r src/my_robot_launch src/my_robot_launch_backup
cp autonomous_car_launch_1.py autonomous_car_launch_1.py.backup
```

#### Step 2: Install diffdrive_arduino for Foxy
```bash
# Clone diffdrive_arduino (Foxy branch)
cd src/
git clone -b foxy https://github.com/joshnewans/diffdrive_arduino.git
cd ..

# Build workspace
colcon build --packages-select diffdrive_arduino
source install/setup.bash
```

#### Step 3: Update Hardware Configuration
Replace `src/my_robot_launch/config/diffbot_hardware.yaml`:
```yaml
diffdrive_arduino:
  ros__parameters:
    left_wheel_name: left_wheel_joint
    right_wheel_name: right_wheel_joint
    loop_rate: 60.0
    device: /dev/ttyACM0
    baud_rate: 115200
    timeout: 1000
    enc_counts_per_rev: 3436  # STANDARDIZED
```

#### Step 4: Implement Unified Launch File
Copy `tmp_rovodev_unified_launch.py` to `src/my_robot_launch/launch/unified_robot_launch.py`

---

## Phase 2: Core Integration (Week 3-4)

### 2.1 Arduino Firmware Update

#### Required Arduino Code Structure:
```cpp
// Arduino code for diffdrive_arduino compatibility
#include <micro_ros_arduino.h>

// Motor control pins (TB6612FNG)
#define MOTOR_A_PWM 32
#define MOTOR_A_IN1 16
#define MOTOR_A_IN2 18
#define MOTOR_B_PWM 33
#define MOTOR_B_IN1 22
#define MOTOR_B_IN2 24

// Encoder pins
#define ENCODER_A_A 17
#define ENCODER_A_B 27
#define ENCODER_B_A 5
#define ENCODER_B_B 6

// Global variables
volatile long encoder_a_count = 0;
volatile long encoder_b_count = 0;
float target_vel_left = 0.0;
float target_vel_right = 0.0;

void setup() {
  // Initialize micro-ROS
  set_microros_transports();
  
  // Initialize motors and encoders
  setupMotors();
  setupEncoders();
  
  // Initialize micro-ROS node
  initMicroROS();
}

void loop() {
  // Handle micro-ROS communication
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
  // Update motor control
  updateMotors();
  
  // Publish encoder data
  publishEncoderData();
  
  delay(10); // 100Hz loop
}
```

### 2.2 ROS2 Control Integration

#### Update ros2_control.xacro:
```xml
<ros2_control name="RealRobot" type="system">
  <hardware>
    <plugin>diffdrive_arduino/DiffDriveArduino</plugin>
    <param name="left_wheel_name">left_wheel_joint</param>
    <param name="right_wheel_name">right_wheel_joint</param>
    <param name="loop_rate">60</param>
    <param name="device">/dev/ttyACM0</param>
    <param name="baud_rate">115200</param>
    <param name="timeout">1000</param>
    <param name="enc_counts_per_rev">3436</param>
  </hardware>
  <!-- Joint definitions remain the same -->
</ros2_control>
```

---

## Phase 3: Docker Deployment (Week 5-6)

### 3.1 Docker Environment Setup

#### Build Production Image:
```bash
# Copy Dockerfile
cp tmp_rovodev_docker_foxy.dockerfile Dockerfile.foxy-production

# Build image
docker build -f Dockerfile.foxy-production -t jetson_robot:foxy-production .
```

#### Deploy with Script:
```bash
# Make deployment script executable
chmod +x tmp_rovodev_deploy_script.sh

# Deploy for real hardware
./tmp_rovodev_deploy_script.sh deploy

# Or deploy for simulation
./tmp_rovodev_deploy_script.sh sim
```

### 3.2 Jetson Nano Specific Optimizations

#### Memory Optimization:
```bash
# Add to /etc/systemd/system/robot.service
[Unit]
Description=Autonomous Robot Service
After=docker.service
Requires=docker.service

[Service]
Type=forking
ExecStart=/home/jetson/autonomous_ROS/tmp_rovodev_deploy_script.sh start
ExecStop=/home/jetson/autonomous_ROS/tmp_rovodev_deploy_script.sh stop
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

#### Performance Tuning:
```bash
# Jetson Nano performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Docker resource limits
docker update --memory=3g --cpus=3 jetson_robot_foxy
```

---

## Phase 4: Testing & Validation (Week 7-8)

### 4.1 Functional Testing

#### Test Scenarios:
1. **Hardware Communication Test:**
```bash
# Test Arduino communication
ros2 topic echo /joint_states

# Test motor control
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

2. **Simulation Parity Test:**
```bash
# Start simulation
./tmp_rovodev_deploy_script.sh sim

# Compare topics and behavior
ros2 topic list
ros2 topic echo /joint_states
```

3. **Web Interface Test:**
```bash
# Access web interface
curl http://localhost:8000

# Test ROSBridge
wscat -c ws://localhost:9090
```

### 4.2 Performance Validation

#### Metrics to Monitor:
- Control loop frequency: >50Hz
- Command latency: <50ms
- Memory usage: <2GB
- CPU usage: <80%

#### Monitoring Commands:
```bash
# Container resource usage
docker stats jetson_robot_foxy

# ROS2 performance
ros2 topic hz /joint_states
ros2 topic delay /diff_cont/cmd_vel_unstamped
```

---

## Phase 5: Production Deployment (Week 9-10)

### 5.1 System Integration

#### Auto-start Configuration:
```bash
# Enable robot service
sudo systemctl enable robot.service
sudo systemctl start robot.service

# Check status
sudo systemctl status robot.service
```

#### Network Configuration:
```bash
# Configure static IP for Jetson
sudo nano /etc/netplan/01-network-manager-all.yaml

# Add:
network:
  version: 2
  ethernets:
    eth0:
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8]
```

### 5.2 Monitoring & Maintenance

#### Log Management:
```bash
# Setup log rotation
sudo nano /etc/logrotate.d/robot

# Add:
/var/log/robot/*.log {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
    create 644 root root
}
```

#### Health Monitoring:
```bash
# Create health check script
cat > /home/jetson/health_check.sh << 'EOF'
#!/bin/bash
# Check if robot container is running
if ! docker ps | grep -q jetson_robot_foxy; then
    echo "Robot container not running, restarting..."
    /home/jetson/autonomous_ROS/tmp_rovodev_deploy_script.sh restart
fi

# Check critical topics
timeout 5 ros2 topic echo /joint_states --once > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "Joint states not publishing, restarting robot..."
    /home/jetson/autonomous_ROS/tmp_rovodev_deploy_script.sh restart
fi
EOF

chmod +x /home/jetson/health_check.sh

# Add to crontab
echo "*/5 * * * * /home/jetson/health_check.sh" | crontab -
```

---

## Troubleshooting Guide

### Common Issues & Solutions

#### 1. Arduino Communication Failure
```bash
# Check device permissions
ls -la /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0

# Check if device is in use
lsof /dev/ttyACM0

# Reset Arduino
echo "1" > /sys/class/gpio/gpio18/value
sleep 1
echo "0" > /sys/class/gpio/gpio18/value
```

#### 2. Controller Spawning Failure
```bash
# Check controller manager
ros2 control list_controllers

# Manually spawn controllers
ros2 run controller_manager spawner diff_cont
ros2 run controller_manager spawner joint_broad
```

#### 3. Topic Remapping Issues
```bash
# Check topic list
ros2 topic list

# Check topic info
ros2 topic info /diff_cont/cmd_vel_unstamped
ros2 topic info /cmd_vel

# Manual remapping test
ros2 run topic_tools relay /cmd_vel /diff_cont/cmd_vel_unstamped
```

#### 4. Docker Container Issues
```bash
# Check container logs
docker logs jetson_robot_foxy

# Enter container for debugging
docker exec -it jetson_robot_foxy /bin/bash

# Restart container
docker restart jetson_robot_foxy
```

---

## Success Criteria

### Technical Metrics
- ✅ Single command deployment (`./deploy_script.sh deploy`)
- ✅ Seamless sim-to-real switching
- ✅ Control loop frequency >50Hz
- ✅ Memory usage <2GB on Jetson Nano
- ✅ Zero configuration conflicts
- ✅ Automated error recovery

### Operational Metrics
- ✅ 99% uptime over 24 hours
- ✅ <5 minute deployment time
- ✅ Remote monitoring capability
- ✅ Automated health checks
- ✅ Log management and rotation

### Development Metrics
- ✅ Unified codebase for sim/real
- ✅ Comprehensive documentation
- ✅ Automated testing pipeline
- ✅ Version control integration
- ✅ Rollback capability

---

## Next Steps

1. **Immediate (Week 1):** Implement unified launch file
2. **Short-term (Week 2-4):** Complete diffdrive_arduino integration
3. **Medium-term (Week 5-8):** Docker deployment and testing
4. **Long-term (Week 9-10):** Production deployment and monitoring

This migration plan ensures a smooth transition to a production-ready, maintainable system while preserving existing functionality and adding robust deployment capabilities for Jetson Nano environments.