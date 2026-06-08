# ESP32-S3 Motor Control Firmware — Architectural Design

**Project:** autoJetsonBot  
**Version:** 1.0.0  
**Target:** ESP32-S3 (PlatformIO / Arduino Core)  
**Status:** Design Phase

---

## 1. Executive Summary
The ESP32-S3 serves as the **Low-Level Controller (LLC)** for the autoJetsonBot. Its primary role is to bridge the gap between high-level ROS 2 velocity commands (m/s) and physical motor voltage (PWM), while providing precise odometry feedback via wheel encoders. This design prioritizes **deterministic timing**, **safety failsafes**, and **modular C++ organization**.

---

## 2. Software Architecture

The firmware follows a **non-blocking, task-based architecture** utilizing FreeRTOS (built into ESP32). This ensures that heavy serial parsing does not interfere with the high-priority motor control loops.

### 2.1 Core Modules (Object-Oriented Design)

| Module | Responsibility |
| :--- | :--- |
| `Encoder` | Interrupt-driven counting of wheel ticks. Handles Phase A/B logic. |
| `Motor` | Abstracted H-Bridge interface. Handles PWM frequency and Direction pins. |
| `PIDController` | Computes the correction factor to match actual velocity to target velocity. |
| `CommandParser` | State-machine based serial listener for `m v_l v_r\r` packets. |
| `Supervisor` | Monitors system health, battery voltage, and communication timeouts. |

---

## 3. Communication Protocol

Deterministic and lightweight ASCII protocol optimized for the `jetson_bot_diffdrive` Python node.

### 3.1 Downlink (Jetson → ESP32)
*   **Format:** `m <left_v> <right_v>\r`
*   **Units:** Meters per second (float).
*   **Frequency:** 10Hz – 50Hz.
*   **Timeout:** If no `m` command is received for **500ms**, motors are immediately braked (E-Stop).

### 3.2 Uplink (ESP32 → Jetson)
*   **Format:** `e <left_ticks> <right_ticks>\n`
*   **Units:** Cumulative signed integers (int32_t).
*   **Frequency:** 20Hz (Synchronized to control loop).

---

## 4. Control Theory & Kinematics

### 4.1 Velocity Transformation
The ESP32 receives **linear wheel velocities** ($v_L, v_R$) in m/s. It must maintain these speeds regardless of surface friction or battery sag.

### 4.2 The PID Loop
Each wheel runs an independent PID loop at a fixed frequency of **50Hz (20ms interval)**.
*   **Input:** Target m/s.
*   **Feedback:** Calculated m/s from encoder delta.
*   **Output:** 10-bit PWM value (0–1023).

$$Output = K_p e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}$$

---

## 5. Hardware Hardware Configuration (Recommended)

### 5.1 ESP32-S3 Pin Mapping (Example)
| Component | Function | GPIO | Note |
| :--- | :--- | :--- | :--- |
| **Motor L** | PWM / DIR | 17 / 18 | Use LEDC for high-res PWM |
| **Motor R** | PWM / DIR | 15 / 16 | 20kHz PWM recommended |
| **Encoder L** | Phase A / B | 4 / 5 | Use Internal Pullups |
| **Encoder R** | Phase A / B | 6 / 7 | Hardware Interrupts enabled |
| **Serial** | TX / RX | 43 / 44 | Hardware Serial0 (USB) |

---

## 6. Safety & Reliability Standards

1.  **Watchdog Timer:** If the main loop hangs, the ESP32 will auto-reset.
2.  **Soft-Start Ramping:** Prevent sudden current spikes by limiting the maximum acceleration rate within the ESP32.
3.  **Command Validation:** Ignore malformed serial strings to prevent erratic movements.
4.  **Voltage Monitoring:** (Optional) Monitor battery level and send warning to Jetson if below 10.5V (for 3S LiPo).

---

## 7. Development Roadmap

1.  **Phase A (Basic IO):** Implement Encoder interrupts and verify raw counts in Serial Monitor.
2.  **Phase B (Open Loop):** Implement Motor class and verify `m` commands move wheels.
3.  **Phase C (Closed Loop):** Tune PID constants using a bench test (robot on blocks).
4.  **Phase D (ROS Integration):** Connect to Jetson and verify `/odom` in Web UI.

---

## 8. File Structure (Proposed)
```text
src/
├── main.cpp          # Entry point & FreeRTOS Task setup
├── config.h          # Pins, PID constants, wheel specs
├── Motor.h/cpp       # Motor class
├── Encoder.h/cpp     # Encoder class
├── PID.h/cpp         # PID math
└── Parser.h/cpp      # Serial Command Parser
```
