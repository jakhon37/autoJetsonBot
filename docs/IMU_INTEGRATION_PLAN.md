# IMU Integration Strategy: ESP32-S3 Bridge

## 1. Verified Core Architecture (from test_IMU)
Based on experimental results in the `test_IMU` sandbox, the following architectural standards are confirmed for the main robot integration:

- **Telemetry Frequency:** 50Hz stable via WebSocket/Serial.
- **Signal Processing:** Digital Low-Pass Filter (LPF) with $\alpha=0.8$ for real-time responsiveness with suppressed jitter.
- **Sensor Range:** Accelerometer $\pm 2G$, Gyro $\pm 250^\circ/s$ for maximum precision.

## 2. Navigation Integration Benefits
Integrating the MPU6050 via the ESP32-S3 bridge provides three primary advantages to the `autoJetsonBot` navigation stack:

### A. Heading Stability (Yaw Correction)
- **Problem:** Wheel slip causes odometry drift during turns.
- **Solution:** Fuse Gyroscope `gz` data with wheel encoders via `robot_localization` (EKF). This prevents "map rotation" errors during rapid maneuvers.

### B. Terrain & Safety Monitoring
- **Pitch/Roll Feedback:** Real-time detection of floor inclines or chassis tipping.
- **Failsafe:** Automatic motor cutoff if `pitch > 15°`, preventing hardware damage.

### C. SLAM Map Sharpening
- **Predictive Mapping:** The 50Hz IMU data acts as a "motion predictor" for the 10Hz Lidar. This eliminates "ghost walls" in `slam_toolbox` by providing high-frequency pose updates between laser scans.

## 3. Implementation Roadmap
1.  **Firmware Update:** Migrate Alpha-filter and I2C logic to the `MOTOR-ESP32S3` main firmware.
2.  **Bridge Expansion:** Update `diffdrive_node.py` to publish `sensor_msgs/Imu` on the `/imu/data_raw` topic.
3.  **EKF Fusion:** Configure `robot_localization` to merge `/odom` (encoders) and `/imu/data_raw` into a stable `/odom_filtered` transform.

## 4. Verification & Testing
- **Firmware Test**: Use `test_esp32_verbose.py` to see raw `i` lines.
- **ROS Test**: Verify topic frequency and values in Docker.
- **Web UI**: (Optional) Add IMU visualization to the dashboard.
