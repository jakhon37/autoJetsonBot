

## Hardware Components

- **Jestson Nano B1**  
  - Runs Ubuntu 22.04 and ROS 2 Humble.
- **TB6612 Dual Motor Driver**  
  - Controls two DC motors via PWM and direction signals.
- **2 × JGA25-370 DC 6V Geared Motors with Encoders and Wheels**  
  - Provide propulsion and motion feedback.
- **7.2V 4500mAh Li-ion Battery**  
  - Powers the motors (via the motor driver's VM and GND outputs).
- **5V 5000mAh Power Bank**  
  - Powers the Jestson Nano B1.
- **RP‑Lidar A1**  
  - Provides 2D LaserScan data on the `/scan` topic for SLAM and obstacle detection.
- **Camera**  
  - For vision-based tasks such as object detection and line tracking.
- **Extra Balance Wheel**  
  - Helps maintain stability.
- **XL6015 Boost Converter**  
  - Used if voltage adaptation is needed (e.g., when using a 12V battery).

---

## Circuit Connection & Wiring Details

### Jestson Nano B1 Connections

- **Motor Driver (TB6612FNG):**
  - **PWMA (Left Motor PWM):** GPIO18 → TB6612FNG PWMA
  - **AI1 (Left Motor Direction):** GPIO23 → TB6612FNG AI1
  - **AI2 (Left Motor Direction):** GPIO24 → TB6612FNG AI2
  - **PWMB (Right Motor PWM):** GPIO13 → TB6612FNG PWMB
  - **BI1 (Right Motor Direction):** GPIO19 → TB6612FNG BI1
  - **BI2 (Right Motor Direction):** GPIO26 → TB6612FNG BI2
  - **STBY & VCC:** 5V → TB6612FNG (enables driver and provides logic power)
  - **GND:** Common ground

- **Motors with Encoders (JGA25-371dc):**
  - **Motor Power:**
    - **Motor Power +:** Connected to TB6612FNG outputs (A01/B01)
    - **Motor Power -:** Connected to TB6612FNG outputs (A02/B02)
  - **Encoder Power:**
    - **Encoder Power +:** 3V3 from Jestson Nano
    - **Encoder Power -:** GND
  - **Encoder Signals:**
    - **Channel 1 / A (C1/A):** Connect to GPIO5 and/or GPIO17
    - **Channel 2 / B (C2/B):** Connect to GPIO6 and/or GPIO27

- **Power:**
  - **Jestson Nano:** Powered via a 5V 5000mAh power bank (Type-C)
  - **Motors:** Powered by a 7.2V 4500mAh Li-ion battery connected to TB6612FNG VM and GND

### TB6612FNG Motor Driver Connections

- **Inputs (from Jestson Nano):**
  - **Left Motor:** PWMA (GPIO18), AI1 (GPIO23), AI2 (GPIO24)
  - **Right Motor:** PWMB (GPIO13), BI1 (GPIO19), BI2 (GPIO26)
- **Power & Enable:**
  - **STBY & VCC:** 5V (from Jestson Nano)
  - **VM:** 7.2V Li-ion battery
  - **GND:** Common ground
- **Outputs (to Motors):**
  - **A01, B01:** Motor Power +
  - **A02, B02:** Motor Power -

### Additional Components

- **RP‑Lidar A1:**  
  - Connect via USB (e.g., `/dev/ttyUSB0`) and configure in its launch file.
- **Jestson Nano Camera:**  
  - Connect to the dedicated camera interface.
- **Balance Wheel:**  
  - Mount mechanically to help stabilize the robot.

---
