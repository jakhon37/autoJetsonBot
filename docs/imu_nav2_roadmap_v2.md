# autoJetsonBot — IMU Sensor Fusion & Nav2 Integration
### Refined Technical Roadmap v2.0
*ROS2 Foxy · ESP32-S3 · MPU6050 · RPLidar A1 · Jetson Nano*

---

## 1. Critical Analysis of the Original Plan

| Gap / Risk | Severity | Problem | Fix in v2.0 |
|---|---|---|---|
| No Madgwick pre-filter | 🔴 HIGH | Raw MPU6050 accel+gyro fed directly into EKF. EKF cannot compute orientation from raw data alone — it needs a quaternion input, or will silently degrade. | Add `imu_filter_madgwick` as Phase 2c. It fuses accel+gyro into an orientation quaternion on `/imu/data`. EKF consumes this, not `/imu/data_raw`. |
| No covariance values | 🔴 HIGH | EKF covariance matrices left as "tune later". `robot_localization` with zero or identity covariances either ignores sensors entirely or diverges immediately on first run. | Concrete starting covariance values provided for both odom and IMU in Phase 3. Tuning procedure defined as a dedicated calibration step. |
| Gyro drift not addressed | 🔴 HIGH | MPU6050 raw gyro has 1–3°/s zero-offset bias. At 50 Hz, an uncorrected bias will visibly rotate the SLAM map in under 60 seconds. | Static bias calibration added as the very first step of Phase 2a firmware work. Bias constants burned into firmware, not computed at runtime. |
| No IMU static TF | 🔴 HIGH | `robot_localization` requires a static transform `base_link → imu_link`. Without it, EKF silently ignores all IMU messages — no error, no warning. | URDF gains a dedicated `<imu_link>` joint. Static TF publisher added to launch file. TF tree verification step added before EKF starts. |
| Nav2 still reads raw `/odom` | 🔴 HIGH | The plan fuses data into `/odom_filtered` but never updates `nav2_params.yaml`. Nav2 keeps using raw drifting odometry — the entire fusion effort is invisible to navigation. | Phase 4 explicitly replaces `odom_topic` in `nav2_params.yaml` and updates the costmap source. |
| `diffdrive_node.py` overloaded | 🟡 MED | Adding IMU serial parsing into `diffdrive_node.py` mixes motor control, encoder reading, and IMU parsing in one node — impossible to debug independently. | New `imu_bridge_node.py` is a standalone node with one responsibility: read serial `i` lines and publish `sensor_msgs/Imu`. Fully testable in isolation. |
| No timestamp sync | 🟡 MED | ESP32 hardware clock drifts vs ROS time. If IMU message stamps diverge by >0.1s, EKF rejects them silently — a frequent source of "IMU not contributing" bugs. | `imu_bridge_node.py` stamps messages with `rospy.Time.now()` at receive time, not ESP32 time. `transform_time_offset` set in EKF config. |
| No diagnostic layer | 🟡 MED | Silent EKF failure is the #1 debugging trap. The system can appear to be running while the IMU contributes nothing, with no way to detect it. | Phase 5 adds a full diagnostics checklist: `rqt_plot` covariance traces, `ros2 topic hz` verification, and a spin-in-place map stability test as the acceptance criterion. |
| Lidar offset calibration vague | 🟢 LOW | The plan mentions the lidar X-offset in URDF but does not specify how to measure it. A 5mm error creates a systematic arc distortion in every scan during turns. | Physical measurement procedure added to Phase 1: measure from rear axle midpoint to lidar center with calipers. URDF value must be accurate to ±2mm. |
| Phase order inefficiency | 🟢 LOW | Original plan serializes all phases. URDF work and firmware work are fully independent and can run in parallel, saving significant wall-clock time. | Phases 1 and 2a explicitly marked as parallelizable, each with independent deliverables and separate verification steps. |

---

## 2. Revised Full-Stack Architecture

### 2.1 Hardware Layer (ESP32-S3)

The ESP32-S3 runs two concurrent RTOS tasks on the same UART, using prefix characters as packet type discriminators:

- **Motor task:** reads encoder ticks, executes PID velocity control, outputs `m <left_ticks> <right_ticks>`
- **IMU task:** polls MPU6050 at 100 Hz over I2C, applies Alpha LPF (α=0.8), subtracts calibrated gyro bias, outputs `i <ax> <ay> <az> <gx> <gy> <gz>`

No mutex required — UART writes are atomic at the operating baud rate.

### 2.2 ROS2 Bridge Layer

| Node | Subscribes | Publishes | Role |
|---|---|---|---|
| `diffdrive_node.py` (existing) | `cmd_vel` | `/odom`, `/tf` (base_link→odom) | Motor control + encoder odometry |
| `imu_bridge_node.py` **(NEW)** | serial port (read-only) | `/imu/data_raw` | IMU serial → ROS2 bridge |
| `imu_filter_madgwick` **(NEW)** | `/imu/data_raw` | `/imu/data` | Fuses accel+gyro into orientation quaternion |
| `ekf_node` (robot_localization) | `/odom` + `/imu/data` | `/odom_filtered`, `/tf` (fused) | EKF sensor fusion |
| `static_tf_publisher` **(NEW)** | — | `/tf_static` (base_link→imu_link) | Required for EKF to locate IMU in robot frame |

### 2.3 SLAM & Navigation Layer

- **slam_toolbox:** consumes `/scan` + `/odom_filtered` + `/tf` tree. The IMU's 50 Hz pose updates fill the 100ms gaps between 10 Hz lidar scans, eliminating map smear during rotation.
- **Nav2:** `odom_topic` explicitly set to `/odom_filtered` in `nav2_params.yaml`. This is the single most important config change that makes fusion visible to navigation.
- **costmap_2d:** both local and global costmaps receive `/odom_filtered` as their odometry source.

### 2.4 Node Startup Order

```
static_tf_publisher → imu_bridge_node → imu_filter_madgwick → ekf_node → slam_toolbox → nav2
```

> **Why the Madgwick filter is not optional**
>
> The EKF in `robot_localization` expects `sensor_msgs/Imu` messages with a valid orientation quaternion (or explicit `-1` diagonal to mark it unknown). If raw accel+gyro are passed without the Madgwick pre-filter, the EKF orientation estimate degrades silently. The Madgwick filter runs at effectively zero CPU cost on the Jetson Nano and must be treated as required infrastructure, not an optional enhancement.

---

## 3. Phase-by-Phase Implementation Plan

> Phases 1 and 2a are **independent and can run in parallel**. All other phases are sequential.

---

### Phase 1 — URDF & Transform Precision `[Parallel with Phase 2a]`

**Goal:** Establish a millimeter-accurate transform tree before any sensor fusion code is written. Fusion built on wrong transforms produces wrong results silently.

**Tasks:**

1. **Physical measurement:** use calipers to measure the distance along X and Z from the rear axle midpoint to the lidar center. Target accuracy: ±2mm. Record both values.
2. **URDF update — lidar joint:** set `<origin xyz="X_measured 0 Z_measured">` on `laser_joint`. Verify with `ros2 run tf2_tools view_frames` — the output must show `base_link → laser` with the correct offset.
3. **URDF update — IMU link (NEW):** add a dedicated `<imu_link>` joint. Mount position should be as close to the rear axle center as possible. Measure and record X/Y/Z offsets. This joint is required for `robot_localization` to apply the lever-arm correction.
4. **Verify TF tree completeness:** the tree must show `odom → base_link → laser` and `odom → base_link → imu_link` before proceeding. A missing `imu_link` branch means EKF will ignore all IMU data.

**Deliverable:** Updated URDF. Screenshot of `view_frames` PDF showing both `laser` and `imu_link` branches. All physical measurements recorded in a calibration log.

---

### Phase 2a — Firmware: Gyro Bias Calibration & IMU Packet `[Parallel with Phase 1]`

**Goal:** Produce clean, bias-corrected IMU data from the ESP32-S3 before any ROS node touches it. Calibrating in firmware is more robust than calibrating in ROS because it applies at the source.

**Tasks:**

1. With the robot completely stationary on a flat surface, collect 1000 samples of `gz` via `test_esp32_verbose.py`. Compute the mean — this is the yaw gyro bias.
2. Repeat for all six axes: `gx`, `gy`, `gz`, `ax`, `ay`, `az`.
3. Burn bias constants into `MOTOR-ESP32S3` firmware as compile-time constants (`GZ_BIAS`, `GX_BIAS`, etc.). Subtract from raw readings **before** the Alpha filter is applied.
4. Migrate Alpha LPF (α=0.8) from `test_IMU` sandbox into the main `MOTOR-ESP32S3` firmware. Filter applies after bias subtraction.
5. Extend serial output to the full 6-axis format: `i <ax> <ay> <az> <gx> <gy> <gz>`. Verify with `test_esp32_verbose.py`. Confirm `gz ≈ 0.0` when stationary.

**Acceptance criterion:** `gz` reading must be **< 0.05 rad/s** when the robot is completely stationary. If this is not met, the map will visibly rotate during navigation regardless of how well the rest of the stack is configured.

---

### Phase 2b — ROS2 Bridge: `imu_bridge_node.py`

**Goal:** A minimal, single-responsibility node that translates serial IMU packets into standard ROS2 messages. Deliberately kept separate from `diffdrive_node.py`.

**Design constraints:**

- **One node, one job:** serial read → `sensor_msgs/Imu` publish. No motor control, no encoder logic.
- **Timestamps:** use `rospy.Time.now()` at the moment of serial receipt. Do not use ESP32 hardware timestamps — they drift and will cause EKF message rejection.
- **Frame ID:** `header.frame_id = 'imu_link'` — must match the URDF joint from Phase 1.
- **Orientation:** set `orientation_covariance[0] = -1` to signal "orientation unknown". The Madgwick filter in Phase 2c provides this.
- **Angular velocity covariance:** diagonal = `[0.01, 0.01, 0.01]` (rad/s)²
- **Linear acceleration covariance:** diagonal = `[0.1, 0.1, 0.1]` (m/s²)²

**Verification:**

```bash
ros2 topic hz /imu/data_raw          # must show ~50 Hz
ros2 topic echo /imu/data_raw        # angular_velocity.z must be ~0.0 stationary
# manually rotate robot: gz must respond clearly, return to ~0.0 when stopped
```

---

### Phase 2c — Madgwick Filter (`imu_filter_madgwick`)

**Goal:** Convert raw IMU data into a proper orientation estimate before it reaches the EKF. This is the step that was missing from the original plan.

**Configuration:**

| Parameter | Value | Reason |
|---|---|---|
| `frequency` | `50.0` | Match IMU publish rate |
| `gain` | `0.1` | Conservative for indoor flat floor. Higher = faster convergence but more vibration sensitivity. |
| `zeta` | `0.0` | No gyro drift compensation at filter level — handled by firmware bias subtraction in Phase 2a. |
| `publish_tf` | `false` | `robot_localization` manages all TF. Never let two nodes fight over the same transform. |
| `world_frame` | `enu` | Standard ROS2 convention. Must match EKF `world_frame` setting. |

**Output:** publishes `/imu/data` with a valid orientation quaternion. The EKF consumes this topic, not `/imu/data_raw`.

---

### Phase 3 — EKF Configuration (`robot_localization`)

**Goal:** Configure the Extended Kalman Filter to fuse wheel odometry and IMU into a single stable, drift-resistant pose estimate.

#### 3.1 State Vector & Source Assignment

`robot_localization` tracks a 15-DOF state vector. The table below defines which sensor controls which state element.

| State | Source | Reason |
|---|---|---|
| x, y position | Wheel odom only | Encoders are accurate for straight-line distance on flat floor. IMU double-integration of acceleration is too noisy for position. |
| yaw (θ) | IMU (primary) | Zero-slip gyro integration is more accurate than wheel slip during turns. Encoders remain as secondary for slow drift correction. |
| vyaw (yaw rate) | IMU `gz` (primary) | 50 Hz vs encoder-derived yaw rate at 10–20 Hz. Most critical IMU contribution for SLAM quality. |
| vx (forward vel) | Wheel odom | Reliable on flat floor at low speed. IMU `ax` used as cross-check only. |
| ax, ay accel | IMU | Useful for terrain change detection. Set high covariance — MPU6050 accel is noisy. |

#### 3.2 Starting Covariance Values

These are concrete starting values, not "tune later". They encode the relative trust between sensors.

**Odometry process noise (`odom0_config` diagonal):**
```
x:0.05, y:0.05, z:0 | roll:0, pitch:0, yaw:0.1 | vx:0.1, vy:0, vz:0 | vyaw:0.05 | ax:0, ay:0
```
*Trust encoders for X/Y position (0.05 m²), distrust yaw from encoders (0.1 rad²) since wheel slip is the primary failure mode.*

**IMU process noise (`imu0_config` diagonal):**
```
x:0, y:0, z:0 | roll:0.05, pitch:0.05, yaw:0.02 | vx:0, vy:0, vz:0 | vyaw:0.01 | ax:0.5, ay:0.5
```
*Trust IMU heavily for yaw (0.02 rad²) and yaw rate (0.01 rad²/s²). Distrust IMU acceleration (0.5 m²/s⁴) due to MPU6050 vibration noise.*

#### 3.3 Critical EKF Parameters

| Parameter | Value | Notes |
|---|---|---|
| `world_frame` | `odom` | Provides continuous (non-jumping) pose. Do not use `map` here. |
| `odom_frame` | `odom` | |
| `base_link_frame` | `base_link` | |
| `transform_time_offset` | `0.0` | Increase to `0.05` if EKF logs "transform not available" warnings. |
| `smooth_lagged_data` | `true` | Handles timing jitter between 50 Hz IMU and 10–20 Hz encoder updates. |
| `history_length` | `0.3` | 3-frame buffer. Required when `smooth_lagged_data` is true. |

---

### Phase 4 — Nav2 Integration

**Goal:** Make Nav2 consume `/odom_filtered` instead of raw `/odom`. This is a configuration change, not a code change, but it is the step that makes the entire fusion effort visible to navigation.

**`nav2_params.yaml` changes:**

| Parameter | Old Value | New Value |
|---|---|---|
| `bt_navigator → odom_topic` | `/odom` | `/odom_filtered` |
| `controller_server → odom_topic` | `/odom` | `/odom_filtered` |
| `amcl → odom_model_type` | `diff` | `diff` (verify unchanged) |

**SLAM Toolbox:** no topic changes needed. `slam_toolbox` uses `/tf`, not `/odom` directly. Once EKF publishes the `odom→base_link` transform, slam_toolbox automatically benefits.

---

### Phase 5 — Calibration, Verification & Acceptance Testing

#### 5.1 Static Calibration (one-time, ~30 min)

1. Place robot on a flat, level surface.
2. Run `imu_bridge_node.py` only. Record 5 minutes of `/imu/data_raw`.
3. Compute mean of `angular_velocity.z`. If `|mean| > 0.05 rad/s`, return to Phase 2a and re-calibrate firmware bias.
4. Verify `/imu/data` (Madgwick output) shows a stable orientation quaternion with `quaternion.w ≈ 1.0` when flat.

#### 5.2 Dynamic Alignment Test ("1m + 360°" Protocol)

1. Drive robot forward exactly 1 metre (mark floor with tape).
2. Compare `/odom` vs `/odom_filtered` x-position. Difference must be < 5%.
3. Rotate robot exactly 360° in place using a known-angle fixture.
4. Compare `/odom` vs `/odom_filtered` yaw accumulation. Encoder-only odom will over- or under-count due to wheel slip. EKF-fused yaw must be within 5° of true 360°.
5. If encoder-only yaw error > 15°: reduce `imu0` yaw covariance to increase IMU trust.

#### 5.3 EKF Health Diagnostics

Silent EKF failure is the #1 debugging trap. These checks confirm the EKF is actually using IMU data.

| Check | Command | Expected |
|---|---|---|
| IMU publishing at rate | `ros2 topic hz /imu/data_raw` | ~50 Hz ±5 Hz |
| Madgwick output active | `ros2 topic hz /imu/data` | ~50 Hz |
| EKF output active | `ros2 topic hz /odom_filtered` | ~30–50 Hz |
| IMU frame in TF tree | `ros2 run tf2_tools view_frames` | `imu_link` branch present |
| EKF covariance shrinking | `rqt_plot /odom_filtered/pose/covariance[35]` | Yaw covariance decreases after motion starts |
| EKF not ignoring IMU | `ros2 topic echo /diagnostics` | No "transform unavailable" for `imu_link` |

#### 5.4 Navigation Acceptance Test (Final Gate)

> **Spin-in-Place Map Stability Test**
>
> 1. Build a map of the test area with SLAM Toolbox.
> 2. Command the robot to spin in place 720° (two full rotations).
> 3. Observe the map in RViz2.
>
> **PASS:** Static walls remain sharp and stationary. No rotation artifacts.
> **FAIL:** Walls smear or rotate. Return to Phase 5.2 and re-tune EKF yaw covariance.

---

## 4. Execution Summary

| Phase | Key Deliverable | Parallelizable? | Acceptance Criterion |
|---|---|---|---|
| 1 | URDF with `imu_link` + calibrated lidar offset | ✅ Yes (with 2a) | `view_frames` shows `base_link → imu_link` |
| 2a | Firmware with bias calibration + 6-axis serial output | ✅ Yes (with 1) | `gz < 0.05 rad/s` stationary |
| 2b | `imu_bridge_node.py` publishing `/imu/data_raw` | No | `/imu/data_raw` at 50 Hz in ROS |
| 2c | Madgwick filter publishing `/imu/data` with valid quaternion | No | Stable quaternion on `/imu/data` |
| 3 | EKF running, `/odom_filtered` published, TF tree complete | No | `/odom_filtered` active, yaw covariance decreasing |
| 4 | Nav2 consuming `/odom_filtered`, `nav2_params.yaml` updated | No | Nav2 navigation goal succeeds |
| 5 | All calibration and acceptance tests passed | No | Spin-in-place map stability **PASS** |

---

## 5. Out-of-Box Enhancements (Post-v2.0, Optional)

- **Dual EKF architecture:** one local EKF (continuous odom frame) + one global EKF (map frame with AMCL corrections). Provides both smooth local control and globally consistent pose.
- **IMU-assisted recovery behavior:** if Nav2 detects localization failure (costmap divergence), trigger a controlled spin using IMU yaw tracking to re-align before re-running particle filter recovery.
- **Terrain-adaptive speed controller:** read pitch/roll from `/imu/data` in the Nav2 controller and automatically reduce `max_vel_x` when `pitch > 5°`. Protects hardware on ramps.
- **Vibration signature monitoring:** a separate diagnostic node subscribes to `/imu/data_raw` and computes RMS of `ax/ay`. Sustained high RMS = loose wheel or damaged bearing. Publish to `/diagnostics`.
- **Gyro-only SLAM mode:** for very slow mapping sessions on slippery floors (tile, etc.), temporarily disable encoder fusion and rely exclusively on IMU yaw + lidar scan matching.

---

*autoJetsonBot — IMU Integration Roadmap v2.0*
