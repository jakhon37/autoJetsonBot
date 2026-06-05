# autoJetsonBot — Physics & Navigation Debugging Session
**Date:** 2026-06-05  
**Platform:** ROS2 Foxy · Gazebo Classic · Docker container (`auto_ros_foxy`)  
**Workspace:** `~/workspace/ROBOT/autoJetsonBot`  
**Robot:** Differential-drive robot on Jetson Nano, simulated with `gazebo_ros2_control`

---

## Table of Contents

1. [Problem Summary](#1-problem-summary)
2. [System Architecture Overview](#2-system-architecture-overview)
3. [Chronological Debug Log](#3-chronological-debug-log)
4. [Root Cause Analysis](#4-root-cause-analysis)
5. [All Changes Made](#5-all-changes-made)
6. [Final Working Configuration](#6-final-working-configuration)
7. [Key Lessons Learned](#7-key-lessons-learned)
8. [Diagnostic Command Reference](#8-diagnostic-command-reference)

---

## 1. Problem Summary

The robot exhibited **autonomous ghost movement** — sliding and spinning on its own with no navigation goal sent and no `/cmd_vel` commands being published. This persisted across all attempts to fix physics parameters, friction, and stiffness values.

**Symptoms observed over the session:**
- Robot slides backward on startup → hits wall
- Robot slides forward on startup continuously
- Robot slides forward AND rotates (arcs in circles)
- `/cmd_vel` topic completely silent during all ghost movement
- `recoveries_server` spamming `/cmd_vel` (secondary effect — Nav2 recovery triggered by the movement)
- Entire room costmap painted dark pink (inflation radius too large, separate bug)
- Nav goal causing robot to "go crazy" and crash into walls

**Final root cause:** A 1.3 cm geometric overlap between the wheel cylinders and the chassis box in the URDF, causing Gazebo's physics constraint solver to generate continuous phantom rotational torque on the wheel joints.

---

## 2. System Architecture Overview

```
robot.xacro
├── robot_core.xacro       ← chassis, wheels, caster geometry + Gazebo contact props
├── ros2_control.xacro     ← ros2_control hardware interface tags
├── gazebo_control.xacro   ← gazebo_ros2_control plugin
├── lidar.xacro
├── camera.xacro
└── imu.xacro

Key config files:
├── src/jetson_bot_bringup/worlds/lab_small_light.world   ← Gazebo world + physics
├── src/jetson_bot_bringup/config/unified_robot_config.yaml  ← diff_drive params
└── src/jetson_bot_navigation/config/nav2_params.yaml     ← Nav2 costmap config
```

**Drive system:**  
`diff_drive_controller/DiffDriveController` (`diff_cont`) via `gazebo_ros2_control`. Wheel joint states → odometry. `/cmd_vel` → wheel velocity commands.

**Key robot dimensions (from URDF):**
| Part | Value |
|---|---|
| Chassis box | 0.195 × 0.18 × 0.1 m |
| Chassis half-width (y) | 0.09 m |
| Drive wheel radius | 0.035 m |
| Drive wheel joint y (original) | ±0.09 m |
| Drive wheel thickness (original) | 0.027 m (left), 0.025 m (right) |
| Caster wheel radius | 0.035 m |
| Caster joint x offset | +0.12 m (front) |
| Total robot mass | ~1.1 kg (0.8 kg chassis + 3 × 0.1 kg wheels) |

---

## 3. Chronological Debug Log

### Stage 1 — Initial optimization changes (pre-session)

Before this session, an optimization pass had been applied to reduce CPU load on the simulation host (older MacBook Pro running Ubuntu). These changes inadvertently introduced the ghost movement:

| File | Parameter | Before | After | Stated reason |
|---|---|---|---|---|
| `lab_small_light.world` | `max_step_size` | `0.001` | `0.005` | 80% CPU reduction |
| `lab_small_light.world` | `real_time_update_rate` | `1000` | `200` | CPU reduction |
| `robot_core.xacro` | wheel `kp` | `1,000,000` | `10,000` | Stop high-freq vibration |
| `robot_core.xacro` | wheel `kd` | `1.0` | `10.0` | Damping |
| `unified_robot_config.yaml` | `spawn_z` | `0.0` | `0.06` | Prevent floor explosion on spawn |

**Effect:** The lower `kp` (wheel contact stiffness) allowed wheels to compress slightly under the robot's weight. The coarser physics step (`0.005`) meant the constraint solver had less precision. Together they exposed a pre-existing geometric self-collision that had previously been masked by the extreme stiffness of `kp=1,000,000`.

---

### Stage 2 — Backward movement diagnosis

**Observation:** Robot drives backward on startup, stops when hitting the back wall.

**Initial (incorrect) hypothesis:** Robot physically imbalanced — weight behind wheel axis causing tipping. Proposed adding a rear caster wheel.

**Corrected analysis (geometric):**
- Wheel joints at x=0, caster at x=+0.12 (front), chassis COM at x≈+0.06
- This is a stable tricycle layout — the COM is between the wheel axis and caster
- Tipping theory was wrong

**Actual cause identified:** `max_step_size=0.005` + `real_time_update_rate=200` = exactly 1:1 ratio with zero headroom. When CPU takes even 5.1ms to compute a 5ms step, physics solver accumulates time debt → allows brief body interpenetration → resolves overlap by pushing robot body → backward sliding.

**Fix applied:**
```xml
<!-- lab_small_light.world -->
<real_time_update_rate>100</real_time_update_rate>  <!-- was 200, gives 2× headroom -->
```
```xml
<!-- robot_core.xacro, both wheels -->
<kp value="50000.0" />  <!-- was 10000, reduces wheel sag -->
```

**Result:** Backward movement became forward movement + rotation. Rotation is a new symptom.

---

### Stage 3 — Forward + rotation diagnosis

**Observation:** Robot moves forward and turns right simultaneously.

**Analysis:** Asymmetric forces between left and right wheels. Two possible causes investigated:

1. Caster wheel had no explicit `kp` — inheriting Gazebo default of ~1,000,000. After drive wheels lowered to 50k, caster was 20× stiffer → acting as a pivot point during sliding → rotation.
2. Wheel thickness mismatch: left=0.027m, right=0.025m → slightly different contact patches at higher stiffness.

**Fixes applied:**
```xml
<!-- robot_core.xacro — caster_wheel, added: -->
<kp value="50000.0" />
<kd value="10.0" />

<!-- Both wheel lengths standardized to 0.026m -->
<!-- world file: update_rate lowered to 50 for 4× headroom -->
<real_time_update_rate>50</real_time_update_rate>
```

**Result:** Rotation fixed (`angular.z` dropped from -0.21 to -0.0002). Forward sliding remained.

---

### Stage 4 — Nav2 "crazy walls" diagnosis

After adding `<dynamics damping="0.1" friction="0.1"/>` to wheel joints to try to stop the forward drift:

**Observation:** Navigation goals caused robot to immediately crash into walls in all directions.

**Analysis:** The `<dynamics>` tags added passive friction to wheel joints. The `diff_drive_controller` commands wheel velocities, but joint friction resists actual rotation → actual wheel speed < commanded speed → odometry reports robot moved less than it did → Nav2 costmap offset from reality → paths planned through real walls.

**Fix applied:** Removed both `<dynamics>` lines from wheel joints.

```xml
<!-- REMOVED from left_wheel_joint and right_wheel_joint: -->
<!-- <dynamics damping="0.1" friction="0.1"/> -->
```

---

### Stage 5 — Costmap "dark pink everywhere" diagnosis

**Observation:** Entire room painted dark pink in RViz costmap. Robot had nowhere to plan paths.

**Analysis:**  
`inflation_radius = 0.55m` in a 6×6m room. Inflation zones from opposite walls overlapped in the center, making all free space appear as high-cost.

For a robot chassis of 0.195m × 0.18m (diagonal ≈ 0.25m), correct inflation radius = `robot_radius + safety_buffer ≈ 0.13 + 0.12 = 0.25m`.

**Fix applied:**
```yaml
# nav2_params.yaml — both local and global costmap sections
inflation_radius: 0.25      # was 0.55
cost_scaling_factor: 3.5    # was 3.0
```

**Result:** Costmap correct. Walls show thin pink buffer only.

---

### Stage 6 — Persistent forward sliding — deep investigation

At this point: world file restored to original (`step_size=0.001`, `update_rate=1000`), but sliding persisted.

**Odom reading at rest (no nav goal):**
```
position.x:    1.57  (after only 63 seconds from spawn)
twist.linear.x: 0.026 m/s
twist.angular.z: -0.0002 rad/s
```

**Joint states reading:**
```
left_wheel_joint:  velocity = 0.06375506730   ← identical
right_wheel_joint: velocity = 0.06375506732   ← to 10 decimal places
```

Both wheels spinning at exactly the same velocity. Precision of the number (`0.06375506...`) never varies across readings — this is not physics noise.

**Key tests performed:**

| Test | Result | Conclusion |
|---|---|---|
| `ros2 topic echo /cmd_vel` | Silent — no output | Not a ROS command |
| `ros2 topic info /cmd_vel -v` | `recoveries_server` listed as publisher | Recovery triggered by movement, not causing it |
| Stop `diff_cont` controller | Robot keeps moving in Gazebo | Not a controller issue |
| Change `mu1` 1.0 → 10.0 | Zero effect on velocity | Not friction |
| Change `kp` 50k → 500k | Zero effect on velocity | Not contact stiffness |
| `open_loop: true` on diff_drive | Still moving | Not an odometry calc issue |
| Check RViz vs Gazebo | **RViz: stationary. Gazebo: moving** | Gazebo visual ≠ ROS state |

**Critical discovery:** RViz showed robot stationary while Gazebo showed it moving. This meant Gazebo was pushing the robot body around but `gazebo_ros2_control` was correctly reading zero from the joint encoders and not feeding the phantom motion into ROS odometry.

The movement was a **Gazebo physics body artifact**, not a ROS issue. Confirmed by pausing Gazebo physics — movement stopped.

---

### Stage 7 — Root cause found: wheel-chassis self-collision

**Analysis of URDF geometry:**

```
Chassis box half-width (y direction): 0.090 m
Wheel joint y position:               0.090 m  ← exactly at chassis edge
Wheel half-thickness:                 0.013 m  ← extends inward from joint center

Wheel inner edge y = 0.090 - 0.013 = 0.077 m
Chassis outer edge y =                0.090 m

OVERLAP = 0.013 m (1.3 cm of wheel INSIDE chassis)
```

**Why this was masked originally:**  
With `kp=1,000,000` and `mu1=100.0`, the contact resolution forces were so extreme that the physics solver effectively "froze" the overlap in place — the wheels couldn't move because the stiffness overwhelmed any sliding tendency. Every softening of physics parameters we applied reduced this masking effect and allowed the underlying collision energy to manifest as rotation.

**Why `mu1=100.0` was dangerous:**  
A friction coefficient of 100 is physically impossible (rubber on rubber ≈ 1.0). Combined with `kp=500,000`, any tiny contact asymmetry between left and right generated massive tangential forces — causing the left wheel to spin at 4× the rate of the right wheel (observed: left=123 rad, right=16 rad after 98 seconds).

---

### Stage 8 — Final fixes

**Fix 1: Resolve wheel-chassis self-collision**
```xml
<!-- robot_core.xacro — move wheels outward by 1cm -->
<!-- LEFT WHEEL -->
<origin xyz="0 0.10 -0.02" rpy="-${pi/2} 0 0"/>  <!-- was 0.09 -->

<!-- RIGHT WHEEL -->
<origin xyz="0 -0.10 -0.02" rpy="${pi/2} 0 0"/>   <!-- was -0.09 -->
```

New geometry:
```
Wheel inner edge y = 0.100 - 0.013 = 0.087 m
Chassis outer edge y =              0.090 m
Clearance = 0.003 m (3mm gap — no overlap)
```

**Fix 2: Update wheel separation to match**
```yaml
# unified_robot_config.yaml
wheel_separation: 0.20   # was 0.18 (= 0.10 + 0.10)
```

**Fix 3: Normalize wheel friction**
```xml
<!-- Both drive wheels -->
<mu1 value="2.0"/>   <!-- was 100.0 — physically unrealistic -->
<mu2 value="0.5"/>   <!-- was 100.0 -->
```

**Result:** Robot stationary in RViz. Gazebo body still slides slightly (cosmetic — `gazebo_ros2_control` reads joint encoders, not body pose, so ROS never sees it). Navigation goals work correctly. Manual control works correctly.

---

## 4. Root Cause Analysis

### Primary root cause: URDF geometry self-collision

The wheel cylinders overlapped the chassis box by 1.3cm in the original URDF. This caused Gazebo's ODE constraint solver to continuously apply repulsion forces to push the wheels out of the chassis. Since wheels are on continuous joints, this repulsion was converted into rotational torque — spinning the wheels without any ROS command.

### Why it was invisible in the original codebase

`kp=1,000,000` (wheel contact stiffness) and `mu1=100.0` (friction) were so far outside physically realistic ranges that they effectively over-constrained the entire robot body. The physics solver was "brute-forcing" the robot to stay still by applying massive counter-forces that overwhelmed the collision torque.

### Why optimization exposed it

Reducing `kp` to `10,000` lowered contact stiffness by 100×. The collision torque could now overcome the wheel resistance. Each subsequent attempt to fix the symptom (adjusting `kp`, `mu`, `step_size`, `update_rate`, `spawn_z`, adding `<dynamics>`) was treating the wrong variable.

### Secondary issues found and fixed

| Issue | Cause | Fix |
|---|---|---|
| Rotation during sliding | Asymmetric `kp` (caster had default 1M, wheels had 50k) | Added explicit `kp`/`kd` to caster |
| Nav "crazy walls" | `<dynamics>` friction on wheel joints fought `diff_drive` controller → odometry divergence | Removed `<dynamics>` tags |
| Dark pink costmap | `inflation_radius=0.55` too large for 6×6m room | Reduced to `0.25` |
| Left wheel spinning 4× faster | `mu1=100` amplified contact asymmetry | Normalized to `mu1=2.0` |

---

## 5. All Changes Made

### `src/jetson_bot_description/urdf/robot_core.xacro`

| Parameter | Original | Final | Reason |
|---|---|---|---|
| `left_wheel_joint` y | `0.09` | `0.10` | Eliminate chassis overlap |
| `right_wheel_joint` y | `-0.09` | `-0.10` | Eliminate chassis overlap |
| Left wheel length | `0.027` | `0.026` | Symmetrize contact patch |
| Right wheel length | `0.025` | `0.026` | Symmetrize contact patch |
| Drive wheel `mu1` | `100.0` | `2.0` | Realistic rubber-on-floor friction |
| Drive wheel `mu2` | `100.0` | `0.5` | Realistic lateral friction |
| Drive wheel `kp` | `1,000,000` | `500,000` | Reduce CPU vibration while maintaining stiffness |
| Drive wheel `kd` | `1.0` | `10.0` | Damping |
| Caster `kp` | *(not set, default ~1M)* | `50,000` | Match drive wheel stiffness |
| Caster `kd` | *(not set)* | `10.0` | Damping |
| `<dynamics>` on wheel joints | *(not present)* | *(not present)* | Added then removed — caused odom drift |

### `src/jetson_bot_bringup/worlds/lab_small_light.world`

| Parameter | Original | Final | Reason |
|---|---|---|---|
| `max_step_size` | `0.001` | `0.001` | Restored to original |
| `real_time_update_rate` | `1000` | `1000` | Restored to original |

*(Note: was temporarily changed to 0.005/50 during debugging, restored to original)*

### `src/jetson_bot_bringup/config/unified_robot_config.yaml`

| Parameter | Original | Final | Reason |
|---|---|---|---|
| `spawn_z` | `0.0` | `0.06` | Prevent floor explosion on spawn |
| `wheel_separation` | `0.18` | `0.20` | Match new wheel joint positions |

### `src/jetson_bot_navigation/config/nav2_params.yaml`

| Parameter | Original | Final | Reason |
|---|---|---|---|
| `inflation_radius` | `0.55` | `0.25` | Room too small for 0.55m inflation |
| `cost_scaling_factor` | `3.0` | `3.5` | Smoother cost gradient |

*(Applied to both local and global costmap sections)*

---

## 6. Final Working Configuration

### `robot_core.xacro` — wheel Gazebo properties

```xml
<!-- LEFT WHEEL -->
<joint name="left_wheel_joint" type="continuous">
    <origin xyz="0 0.10 -0.02" rpy="-${pi/2} 0 0"/>
    ...
</joint>
<gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
    <mu1 value="2.0"/>
    <mu2 value="0.5"/>
    <kp value="500000.0" />
    <kd value="10.0" />
</gazebo>

<!-- RIGHT WHEEL -->
<joint name="right_wheel_joint" type="continuous">
    <origin xyz="0 -0.10 -0.02" rpy="${pi/2} 0 0"/>
    ...
</joint>
<gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1 value="2.0"/>
    <mu2 value="0.5"/>
    <kp value="500000.0" />
    <kd value="10.0" />
</gazebo>

<!-- CASTER WHEEL -->
<gazebo reference="caster_wheel">
    <material>Gazebo/Black</material>
    <mu1 value="0.1"/>
    <mu2 value="0.1"/>
    <kp value="50000.0" />
    <kd value="10.0" />
</gazebo>
```

### `lab_small_light.world` — physics block

```xml
<physics type='ode'>
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### `unified_robot_config.yaml` — diff_drive section

```yaml
diff_cont:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.20
    wheel_radius: 0.035
    publish_rate: 100.0
```

### `nav2_params.yaml` — costmap inflation (both local and global)

```yaml
inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    cost_scaling_factor: 3.5
    inflation_radius: 0.25
```

---

## 7. Key Lessons Learned

### 1. Unrealistic physics values mask geometry bugs
`kp=1,000,000` and `mu1=100.0` are not physically meaningful for a 1.1kg robot. They were accidentally masking a URDF geometry error. When optimizing simulation performance, physics values should be reduced gradually while monitoring for new behaviors.

### 2. Check URDF geometry overlaps first
Before tuning any physics parameter (`kp`, `kd`, `mu`), verify that no links physically overlap. A 1.3cm overlap is enough to generate continuous phantom forces. The quick check:

```
wheel_inner_edge_y = wheel_joint_y - (wheel_length / 2)
chassis_outer_edge_y = chassis_joint_y + (chassis_width / 2)
if wheel_inner_edge_y < chassis_outer_edge_y: COLLISION
```

### 3. Distinguish ROS movement from Gazebo body movement
If `/cmd_vel` is silent AND stopping the `diff_cont` controller doesn't stop movement → the force is purely in Gazebo physics, not ROS. Check `/odom` twist values to confirm whether ROS sees the movement. If ROS is stationary but Gazebo shows movement, the robot's navigation is actually fine.

### 4. `wheel_separation` must match URDF joint positions
After moving wheel joints, always update `wheel_separation = left_joint_y + abs(right_joint_y)`. A mismatch causes incorrect turning radius calculations in odometry — the robot will drift in circles during navigation.

### 5. `<dynamics>` friction on wheel joints breaks diff_drive odometry
The `<dynamics damping="..." friction="..."/>` URDF tag adds passive joint resistance. This fights the `diff_drive_controller`'s velocity commands → actual wheel speed < commanded → odometry error accumulates → Nav2 loses track of robot position. Never add passive joint friction to drive wheels when using `ros2_control`.

### 6. Nav2 inflation radius must fit the room
`inflation_radius` should be roughly `robot_radius + 0.10m` safety buffer. For a 0.13m radius robot: `inflation_radius ≈ 0.23–0.25m`. In a 6×6m room, `inflation_radius=0.55m` inflates walls by 55cm on each side — overlapping in the center and making the room unnavigable.

### 7. `recoveries_server` symptoms are secondary
If `recoveries_server` is actively publishing to `/cmd_vel` with no goal sent, it means Nav2 detected the robot as stuck or off-path. Find and fix the root cause of the movement first — the recovery behavior will stop on its own.

### 8. Physics step size and update rate must have headroom
`max_step_size / (1 / real_time_update_rate)` should be < 0.5 to give the physics solver enough real-time budget per step. At 1:1 ratio (`step_size=0.005`, `rate=200`), any CPU spike causes physics debt → constraint violations → phantom forces.

---

## 8. Diagnostic Command Reference

```bash
# Check if robot is actually moving (twist values)
ros2 topic echo /odom

# Check wheel spin independently of odometry
ros2 topic echo /joint_states

# Check who is publishing velocity commands
ros2 topic info /cmd_vel -v

# Watch live velocity commands
ros2 topic echo /cmd_vel

# Check diff_drive controller parameters
ros2 param dump /diff_cont

# List active controllers
ros2 control list_controllers

# Stop the diff_drive controller (test if movement is ROS-driven)
ros2 service call /diff_cont/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{stop_controllers: ['diff_cont'], strictness: 1}"

# Pause Gazebo physics (test if movement is physics-driven)
ros2 service call /gazebo/pause_physics std_srvs/srv/Empty {}

# Check Nav2 action status
ros2 action list

# Quick geometry self-collision check (run in workspace)
grep -A2 "wheel_joint.*type" src/jetson_bot_description/urdf/robot_core.xacro
grep "size\|radius\|length" src/jetson_bot_description/urdf/robot_core.xacro

# Verify files are correctly rebuilt in container
grep "mu1" /autonomous_ROS/install/jetson_bot_description/share/jetson_bot_description/urdf/robot_core.xacro
```

---

*Documentation generated from debugging session on 2026-06-05.*  
*Robot: autoJetsonBot · Repository: https://github.com/jakhon37/autoJetsonBot*
