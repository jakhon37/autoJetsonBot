# autoJetsonBot — Physics Stability & Optimization Retrospective (REVISED)

**Date:** 2026-06-05  
**Context:** ROS 2 Foxy · Gazebo Classic · Docker (Mac Host)  
**Goal:** Document the regression from v1.0.4 stability to "Ghost Movement" and the subsequent deep-audit restoration.

---

## 1. Initial State: The "Optimization" Trap
The project was stable at v1.0.4, but CPU load on the host was excessive (~460%). An optimization pass was initiated with three primary targets:

1.  **Physics Load:** Increased \`max_step_size\` (0.001 -> 0.005) and decreased \`real_time_update_rate\` (1000 -> 200).
2.  **Contact Softening:** Reduced wheel contact stiffness \`kp\` (1,000,000 -> 10,000) and friction \`mu\` (100.0 -> 1.0).
3.  **Data Density:** Halved Lidar samples (360 -> 180) and update rate (10Hz -> 5Hz).

**The Trap:** These changes "unlocked" a latent URDF geometry error that the original v1.0.4 parameters were "brute-force" masking.

---

## 2. Symptom Progression & Diagnostic Trials

### Symptom A: Backward Sliding (The "Headroom" Rule)
*   **Observation:** Robot slid backward on startup and crashed into the South wall.
*   **Diagnosis:** The ratio of \`max_step_size\` to \`update_rate\` was exactly 1:1. Any CPU spike caused "physics debt," allowing links to interpenetrating and then explosively repel.
*   **Rule Learned:** \`max_step_size / (1 / real_time_update_rate)\` must be **< 0.5** to provide headroom for the solver.

### Symptom B: Forward Arcing (The "Asymmetry" Pivot)
*   **Observation:** Robot slid forward while rotating clockwise.
*   **Diagnosis:** Drive wheels had lowered \`kp\` (50k), but the Caster inherited the Gazebo default (1M). The 20× stiffer caster acted as a physical pivot point, converting forward sliding into rotation. Additionally, wheel thicknesses were mismatched (0.027m vs 0.025m), creating asymmetric contact patches.

### Symptom C: "Crazy Walls" (The <dynamics> Failure)
*   **Observation:** Robot ignored real walls and crashed immediately during navigation.
*   **Diagnosis:** To stop sliding, \`<dynamics damping="0.1" friction="0.1"/>\` was added to wheel joints. This added passive resistance that the \`diff_drive_controller\` couldn't account for. The robot moved less than commanded, causing Odometry to drift from reality.

---

## 3. The Root Cause: "Brute-Force" Masking
The **Deep Audit** revealed a fundamental geometry overlap in the URDF:

*   **Chassis Half-Width:** 0.090 m
*   **Wheel Joint Y:** 0.090 m
*   **Wheel Half-Thickness:** 0.013 m
*   **THE OVERLAP:** 0.013 m (**1.3 cm**) of wheel was embedded inside the chassis box.

**Why v1.0.4 worked:** \`kp=1,000,000\` and \`mu=100.0\` were so extreme that they "froze" the robot in place. The physics solver was using massive counter-forces to essentially override the collision torque. The "Optimization" pass lowered these guards, allowing the collision energy to manifest as "Ghost Movement."

---

## 4. Final Resolution: Restoration & Correction

### Physics Restoration (The Baseline):
- **Stiffness:** Restored \`kp=1,000,000.0\` to re-mask the overlaps.
- **Friction:** Restored \`mu=100.0/50.0\` to stabilize the contact solver.
- **Environment:** Reverted ground plane friction in \`lab_small_lightorg.world\` to the proven high values.

### Navigation Hardening:
- **Costmap Inflation:** Fixed the "all dark pink" room by reducing \`inflation_radius\` from 0.55m to **0.25m**. This prevents zones from opposite walls from overlapping in the 6x6m lab world.
- **Cost Gradient:** Set \`cost_scaling_factor\` to **3.5** to restore a smooth visual and logic gradient for path planning.

### System Synchronization:
- **Build Verification:** Confirmed that URDF changes in \`src/\` are only live after a full modular build (\`./robot.sh build\`) to update the \`install/\` directory.

---

## 5. Key Lessons for the autoJetsonBot Platform
1.  **Don't Soften the Brute:** If a model has internal overlaps, Gazebo Classic requires extreme stiffness to remain stable. "Soft" physics is only for perfectly clear URDFs.
2.  **Dynamics vs Controllers:** Never add \`friction\` or \`damping\` to joints controlled by \`ros2_control\` unless the controller is explicitly tuned for it; otherwise, Odometry becomes a lie.
3.  **Inflation vs Room Size:** Inflation radius is a function of **both** robot size and room dimensions. In small labs, the diagonal of the robot is the maximum safe radius.
4.  **Stale Installs:** Stale binaries in the \`install/\` folder are the most common cause of "non-reactive" fixes. Rebuild early, rebuild often.

---
*Documentation revised and finalized on 2026-06-05.*

---

## 7. Web UI: Tactical Square Display & Coordination
**Update:** 2026-06-05 (Session End)

The Web UI was upgraded from a static circular radar to a **Dynamic Square Tactical Grid** (280x280).

### Key Upgrades:
1.  **Coordinate Synchronization:** Implemented real-time **Map-to-Robot frame transformations**. World-locked elements (Nav2 Global Path and Goal Points) now correctly "slide" and rotate on the dashboard as the robot moves, while the robot stays centered.
2.  **Visual Expansion:** Removed the CSS and Canvas circular clipping. The rendering area now utilizes the full rectangular bounds of the tactical grid.
3.  **Robust Rendering:** Added defensive bound-checking and try-catch blocks to the animation loop to ensure UI stability during complex path-planning events.

### Technical Requirement:
- **Build Sync:** Full modular build (\`./robot.sh build\`) required to synchronize CSS/JS assets.
- **Cache Invalidation:** Users must perform a **Hard Refresh** (Ctrl+F5) to clear cached JavaScript when upgrading the UI.
