# autoJetsonBot — IMU (MPU6050) Wiring Guide

This document defines the physical connection between the **MPU6050 6-Axis IMU** and the **ESP32-S3 Low-Level Controller**. 

## 🔌 Pin Mapping

The firmware on the board uses these pins. That build (`MOTOR-ESP32S3` with IMU code from `imu_viz`) was never committed. The rebuilt firmware on MOTOR-ESP32S3's `quadrature-speed-control` branch uses the same pins. **Ensure the robot is powered off before wiring.**

| MPU6050 Pin | ESP32-S3 Pin | Function | Wire Color (Typical) |
| :--- | :--- | :--- | :--- |
| **VCC** | **3.3V** | Power | Red |
| **GND** | **GND** | Ground | Black |
| **SDA** | **GPIO 1** | I2C Data | Blue / Green |
| **SCL** | **GPIO 21** | I2C Clock | Yellow / White |

> [!CAUTION]
> Most MPU6050 modules (like the GY-521) have an onboard 3.3V regulator but are safer when powered by the ESP32's **3.3V** pin. Using 5V without a level shifter can damage the S3's I2C pins.

---

## 🚦 Visual Status Guide (Onboard WS2812 LED)

The ESP32-S3 firmware uses the onboard RGB LED to provide instant hardware feedback on boot. These states come from June's notes, because the flashed build's source isn't committed:

*   **🟡 Yellow (Solid)**: Booting / Initializing.
*   **🟢 Green (Solid)**: **SUCCESS**. IMU detected on I2C bus and motors initialized.
*   **🔴 Red (Solid)**: **IMU ERROR**. The MPU6050 was not found. Check wiring or I2C address.
*   **🔵 Blue Pulse**: Normal Operation. Actively receiving ROS 2 commands.

---

## 🔍 Troubleshooting

### 1. The LED is Red
*   **Check Continuity**: Use a multimeter to ensure your SDA/SCL wires aren't swapped.
*   **Check Voltage**: Ensure VCC is receiving a steady 3.3V.
*   **Address Check**: Most MPU6050s use address `0x68`. If yours uses `0x69` (AD0 pin tied high), the firmware will need a one-line update.

### 2. No data in ROS 2 (`/imu/data_raw`)
*   Run `./robot.sh shell "ros2 node list | grep imu_bridge"` to check that `imu_bridge_node` is running. `./robot.sh status` shows whether `i` lines arrive from the ESP32.
*   Run `./robot.sh shell "ros2 topic echo /imu/data_raw"` to see if packets are arriving.
*   "Malformed IMU packet" in the logs means an `i` line arrived with fewer than six values, usually because the ESP32 was rebooting. Baud rate doesn't matter on the S3's native USB.

### 3. Jittery Visualization
*   The firmware includes an **Alpha LPF ($\alpha=0.8$)**. If the visualization in the Web UI is too slow or too shaky, adjust `alpha` in whichever firmware you flash. The filter came from `imu_viz` into the uncommitted build on the board.

---
*Reference: [imu_nav2_roadmap_v2.md](../plans/imu_nav2_roadmap_v2.md)*
