
  ## Meticulous Plan to Reach a Stable Solution

  ### Diagnosis Confirmed from Live Data

  The geometry check shows 7mm clearance between wheel and chassis — no overlap. So the phantom torque is not from
  wheel-chassis self-collision anymore. The current sliding at 0.024 m/s with both wheels spinning identically at 0.044
   rad/s with zero /cmd_vel is coming from a different source. The robot exits the map in ~3 minutes from spawn, which
  matches the "few goals then lost" symptom exactly.

  The real culprits, ranked:

    1. chassis_joint z=0.02 offset — raises the chassis 2cm above base_link. Combined with spawn_z=0.06, the robot
       spawns with its chassis floor at 0.055 (base_footprint→base_link) + 0.02 (chassis_joint z) = 0.075m above
ground. The
       caster sphere radius is 0.035m, so it doesn't reach the floor. The robot is balancing on two drive wheels only —
 any
       asymmetry in contact generates torque.
    2. open_loop: false — faithfully integrates phantom wheel spin into odometry, poisoning the map frame.
    3. Lidar at 5Hz / 180 samples — AMCL can't correct the drift fast enough.
    4. AMCL thresholds too tight — triggers on degraded data.


----------------------------------------------------------------------------------------------------------------------

  ### The Plan — 4 Stages, Ordered by Risk

  #### Stage 1 — Stop the Odometry Poisoning (Immediate, Zero Risk)

  One parameter change. No geometry touched.

  Change open_loop: false → open_loop: true in unified_robot_config.yaml.

    * Why: With open_loop: true, diff_drive_controller computes odometry from commanded velocity, not measured wheel
      velocity. Phantom wheel spin stops poisoning /odom. The robot stays on the map long enough for AMCL to localize.
    * Trade-off: Odometry is less accurate during real movement (no encoder feedback). Acceptable for simulation where
      AMCL corrects it.
    * Risk: Zero. Single parameter, fully reversible.
    * Expected result: Robot stays near spawn. Nav goals work. Sliding in Gazebo continues cosmetically but ROS doesn't
      see it.

  #### Stage 2 — Fix the Caster Ground Contact (Root Cause of Torque)

  One geometry change in robot_core.xacro.

  Revert chassis_joint z offset from 0.02 back to 0.0.

    * Why: With chassis_joint z=0.02, the chassis sits 2cm higher than designed. The caster wheel joint is at xyz="0.12
       0 -0.02" relative to base_link — so caster center is at z = 0.055 (footprint→base_link) + (-0.02) = 0.035m above

      ground. Caster radius = 0.035m. So caster just barely touches ground at z=0. But with spawn_z=0.06, the robot
drops
      6cm — the caster hits first and the drive wheels may not be fully loaded. This creates asymmetric normal forces →

      asymmetric friction → torque.
    * Correct geometry: chassis_joint z=0.0 (v2 baseline). Drive wheels at z = 0.055 - 0.02 = 0.035m above ground =
      exactly wheel radius. Caster at z = 0.055 - 0.02 = 0.035m = exactly caster radius. All three contact points touch

      simultaneously at spawn.
    * Risk: Low. Requires rebuild + restart. Fully reversible.
    * Expected result: Symmetric ground contact → no phantom torque → wheels stop spinning at rest.

  #### Stage 3 — Restore Lidar and AMCL to v2 Baseline (Localization Fix)

  Two file changes.

  In lidar.xacro:

    * update_rate: 5 → 10
    * samples: 180 → 360

  In nav2_params.yaml:

    * update_min_a: 0.1 → 0.2
    * update_min_d: 0.1 → 0.25
    * Local costmap resolution: 0.1 → 0.05
    * Why: These are the optimization changes that were never reverted. At 5Hz/180 samples, AMCL particle filter
      diverges during turns. Restoring to v2 values gives AMCL the data density it needs to maintain localization
across
      multiple goals.
    * Risk: Low. CPU load increases (back to pre-optimization baseline). On Docker-on-Mac this is acceptable — the
      AGENTS.md says headless mode saves 80% GPU, which compensates.
    * Expected result: AMCL holds localization through 5+ consecutive goals.

  #### Stage 4 — Normalize Ground Plane Friction (Cleanup)

  One world file change.

  In lab_small_light.world, change ground plane <mu>100</mu><mu2>50</mu2> → <mu>1.0</mu><mu2>1.0</mu2> to match
  lab_small_lightorg.world.

    * Why: The active world (lab_small_light.world) has unrealistic ground friction. The "org" world (which is actually
      loaded per config) already has mu=1.0. This is a cleanup to make lab_small_light.world consistent in case it's
ever
      used.
    * Risk: Minimal. The active world is lab_small_lightorg.world which already has correct values.
    * Expected result: Consistent physics across world files.


----------------------------------------------------------------------------------------------------------------------

  ### Execution Order and Verification Gates

  Stage 1 → restart → verify: odom.x stays < 1.0 after 5 min
      ↓ (pass)
  Stage 2 → rebuild → restart → verify: joint_states velocity ≈ 0.0 at rest
      ↓ (pass)
  Stage 3 → restart → verify: nav goal completes 3× in a row without AMCL loss
      ↓ (pass)
  Stage 4 → cleanup only, no verification needed

  Stop condition (Rule of Two from AGENTS.md): If Stage 2 does not stop phantom wheel spin after 2 attempts with
  different z values, do not try a third. Instead, keep open_loop: true from Stage 1 as the permanent mitigation and
  accept cosmetic Gazebo sliding.


----------------------------------------------------------------------------------------------------------------------

  ### What NOT to Touch

    * wheel_joint_y = ±0.11 — geometry is correct, 7mm clearance confirmed, do not change
    * wheel_separation: 0.22 — matches URDF, do not change
    * kp=500000, mu1=2.0 — correct values from debug session, do not change
    * spawn_z: 0.06 — keep this, it prevents floor explosion on spawn
    * inflation_radius: 0.25 — correct fix from debug session, do not change


----------------------------------------------------------------------------------------------------------------------

  ### Summary Table

  ┌──────────────────────┬──────────────────────┬──────────────────────┬──────────────────────┬──────────────────────┐
  │ Stage                │ File                 │ Change               │ Risk                 │ Reversible           │
  ├──────────────────────┼──────────────────────┼──────────────────────┼──────────────────────┼──────────────────────┤
  │ 1                    │ unified_robot_confi… │ open_loop: false →   │ Zero                 │ Yes                  │
  │                      │                      │ true                 │                      │                      │
  ├──────────────────────┼──────────────────────┼──────────────────────┼──────────────────────┼──────────────────────┤
  │ 2                    │ robot_core.xacro     │ chassis_joint z:     │ Low                  │ Yes                  │
  │                      │                      │ 0.02 → 0.0           │                      │                      │
  ├──────────────────────┼──────────────────────┼──────────────────────┼──────────────────────┼──────────────────────┤
  │ 3a                   │ lidar.xacro          │ rate 5→10, samples   │ Low                  │ Yes                  │
  │                      │                      │ 180→360              │                      │                      │
  ├──────────────────────┼──────────────────────┼──────────────────────┼──────────────────────┼──────────────────────┤
  │ 3b                   │ nav2_params.yaml     │ AMCL thresholds +    │ Low                  │ Yes                  │
  │                      │                      │ local costmap res    │                      │                      │
  ├──────────────────────┼──────────────────────┼──────────────────────┼──────────────────────┼──────────────────────┤
  │ 4                    │ lab_small_light.wor… │ ground mu 100/50 →   │ Minimal              │ Yes                  │
  │                      │                      │ 1.0/1.0              │                      │                      │
  └──────────────────────┴──────────────────────┴──────────────────────┴──────────────────────┴──────────────────────┘

  Shall I proceed with Stage 1 now?

> doc this to md file and then go                                                                                      

> go                                                                                                                   

● Starting with Stage 1 — single parameter, no rebuild needed.

┃ → Read src/jetson_bot_bringup/config/unified_robot_config.yaml:56–65

┃ ← Edit src/jetson_bot_bringup/config/unified_robot_config.yaml
┃            @@ -48,7 +48,7 @@                                                                                         
┃   48  48       odom_frame_id: "odom"                                                                                 
┃   49  49       enable_odom_tf: true                                                                                  
┃   50  50       use_stamped_vel: false                                                                                
┃   51     -     open_loop: false                                                                                      
┃       51 +     open_loop: true  # Stage1 fix: prevent phantom wheel spin from poisoning odometry                     
┃   52  52                                                                                                             
┃   53  53       # 🎢 Velocity & Acceleration Limits (Smooth Ramping)                                                  
┃   54  54       linear:                                                                                               

● Now Stage 2 — fix chassis_joint z offset:

┃ ← Edit src/jetson_bot_description/urdf/robot_core.xacro
┃            @@ -56,7 +56,7 @@                                                                                         
┃   56  56                                                                                                             
┃   57  57       <!-- ROBOT CHASSIS LINK-->                                                                            
┃   58  58       <joint name="chassis_joint" type="fixed">                                                             
┃   59     -         <origin xyz="-0.035 0 0.02" rpy="0 0 0"/>                                                         
┃       59 +         <origin xyz="-0.035 0 0.0" rpy="0 0 0"/>                                                          
┃   60  60           <parent link="base_link"/>                                                                        
┃   61  