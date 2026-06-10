# Roadmap: IMU-Lidar Sensor Fusion & Advanced Navigation

## 1. Executive Summary
This document outlines the final phase of the `autoJetsonBot` navigation stack: the fusion of high-frequency IMU data with wheel odometry and Lidar-based SLAM. The goal is to eliminate odometry drift and "map smearing" during rapid rotations.

## 2. Current Status & Baseline
- **Lidar:** RPLidar A1 installed in the chassis center, publishing at 10Hz.
- **Wheels:** Differential drive with encoders in the rear, providing basic `/odom`.
- **IMU:** MPU6050 verified at 50Hz in sandbox; ready for ESP32-S3 integration.
- **Problem:** When the robot rotates, the Lidar "swings" (due to being forward of the pivot point), and encoders slip, causing map misalignment.

## 3. The Fusion Architecture (EKF)
We will implement an **Extended Kalman Filter (EKF)** using the `robot_localization` package to create a stable, fused world-view.

### A. Data Inputs
1.  **Enconders (Wheel Odom):** Provides $x, y$ position and $\theta$ (low speed, high distance accuracy).
2.  **IMU (MPU6050):** Provides $\dot{\theta}$ (Yaw velocity) and $\ddot{x}, \ddot{y}$ (acceleration). High frequency (50Hz), zero-slip.

### B. The Fused Topic
The EKF node will output a new topic: `/odom_filtered`. This will be the **Source of Truth** for the robot's position, used by both SLAM and Nav2.

## 4. Integration Phases

### Phase 1: URDF & Transform Synchronization
- **Pivot Point:** Define `base_link` exactly between the rear wheels.
- **Lidar Offset:** Maintain the precise $X$ offset in URDF so `slam_toolbox` can mathematically cancel the "swing" effect during turns.
- **IMU Placement:** Mount the MPU6050 as close to the rear axle center as possible.

### Phase 2: Firmware & Bridge Expansion
- Migrate the `test_IMU` Alpha-filter ($\alpha=0.8$) to the `MOTOR-ESP32S3` firmware.
- Update `diffdrive_node.py` to parse the new serial packet `i ax ay az gx gy gz` and publish to `/imu/data_raw`.

### Phase 3: SLAM Optimization (The "Magic")
- Configure `slam_toolbox` to use the IMU as a "motion predictor."
- **Benefit:** When the robot spins, the IMU tells the SLAM engine exactly how much it rotated *before* the next Lidar scan arrives, keeping the map perfectly sharp.

## 5. Installation & Calibration Steps
1.  **Static Calibration:** With the robot stationary, calculate the IMU gyro bias and update the firmware.
2.  **Dynamic Alignment:** Perform a "1-meter drive" and "360-degree spin" test to verify that the encoders and IMU agree on the distance and angle.
3.  **Covariance Tuning:** Adjust the EKF covariance matrices to "trust" the IMU for rotations and the Encoders for straight lines.

## 6. Expected Outcome
- **Zero-Drift Turns:** The robot can spin in place without the map shifting.
- **Obstacle Stability:** Improved costmap reliability; obstacles won't "ghost" or move when the robot rotates.
- **Terrain Robustness:** Ability to detect and adapt to floor unevenness.
