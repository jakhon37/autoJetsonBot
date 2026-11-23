# Hardware

A two-wheel differential-drive robot with a front caster, built around a Jetson Xavier NX. An ESP32-S3 handles the motors, encoders and IMU in real time, and the Jetson runs everything else in ROS 2.

<p align="center">
  <img src="../../assets/photos/robot_side.jpg" alt="Side view: RPLidar on standoffs, Wi-Fi antenna, wheel" width="32%">
  <img src="../../assets/photos/robot_front_three_quarter.jpg" alt="Front three-quarter view of the grey enclosure and lidar" width="32%">
  <img src="../../assets/photos/robot_undercarriage.jpg" alt="Undercarriage: two geared motors with encoders and a caster" width="32%">
  <br>
  <em>Side · front three-quarter · undercarriage with the two encoder gear motors and the caster</em>
</p>

## Components

| Part | Model | Connection | Notes |
| :--- | :--- | :--- | :--- |
| **Computer** | NVIDIA Jetson Xavier NX Developer Kit, 8 GB | Wi-Fi to the network | 6-core Carmel CPU, Volta GPU, 256 GB NVMe. L4T R35.6, Ubuntu 20.04, CUDA 11.4, Docker 28. Runs ROS 2 Humble in the `auto_ros:humble` container, in 20 W 6-core mode |
| **Real-time controller** | ESP32-S3 dev board | Native USB to the Jetson (`303a:1001`, `/dev/ttyACM0`) | Runs [MOTOR-ESP32S3](https://github.com/jakhon37/MOTOR-ESP32S3)'s `gyro-offset-learning` branch (`quadrature-speed-control` plus gyro offsets learned at rest) since 2026-11-10: closed-loop wheel speed on quadrature encoders, encoder and IMU lines at 50 Hz, and motors stopped if no command arrives for 500 ms |
| **Motor driver** | TB6612FNG dual H-bridge | PWM and direction pins from the ESP32-S3 | |
| **Motors** | 2 × JGA25-370 geared DC motors (label reads 170 RPM), 34:1 gearbox | Through the driver | Each has an AB Hall encoder with 11 pulses per motor turn, so 1,497 counts per wheel turn on all 4 edges |
| **Wheels** | 68 mm rubber wheels and a front caster | | Wheel separation 0.212 m |
| **IMU** | MPU6050 | I2C to the ESP32-S3 (SDA GPIO 1, SCL GPIO 21) | See [IMU_WIRING.md](IMU_WIRING.md) |
| **Lidar** | Slamtec RPLidar A1 (A1M8-R6) | USB through a CP2102 adapter (`10c4:ea60`) | 360°, about 6.5 scans per second, 360 points per scan |
| **Camera** | 8 MP Sony IMX219 camera module with a wide-angle lens | CSI ribbon to the Xavier NX's CAM0 connector | Published as `/camera/image_raw` at 15 fps, 820×616 over the full field of view (see [Camera](#camera)). Replaced a Wi-Fi ESP32-CAM ([myESP32CAM-BLACKBOX](https://github.com/jakhon37/myESP32CAM-BLACKBOX)) in November 2026 |
| **Power** | Not documented yet | | The Xavier NX dev kit takes 9 to 20 V. The original design (below) used 5 V rails for a Jetson Nano |

## How It Connects

```mermaid
flowchart TB
    op["Laptop or phone<br/>Web UI, SSH, VNC"]
    cam["IMX219 camera<br/>wide-angle lens"]
    jetson["Jetson Xavier NX<br/>ROS 2 Humble in Docker"]
    lidar["RPLidar A1"]
    esp["ESP32-S3<br/>motor and IMU firmware"]
    imu["MPU6050 IMU"]
    driver["TB6612FNG<br/>motor driver"]
    motorL["Left motor<br/>JGA25-370 + encoder"]
    motorR["Right motor<br/>JGA25-370 + encoder"]
    op -.->|"Wi-Fi"| jetson
    cam -->|"CSI ribbon, CAM0"| jetson
    lidar -->|"USB, CP2102"| jetson
    jetson <-->|"USB serial: m, e, i lines"| esp
    imu -->|"I2C"| esp
    esp -->|"PWM + direction"| driver
    driver --> motorL
    driver --> motorR
    motorL -->|"A/B pulses"| esp
    motorR -->|"A/B pulses"| esp
```

## ESP32-S3 Pin Map

From the `MOTOR-ESP32S3` firmware the board runs since 2026-11-10, and checked on the robot: each wheel turned when driven alone, and both encoder channels counted in the hand test.

| Signal | GPIO | Signal | GPIO |
| :--- | :---: | :--- | :---: |
| Left motor PWM (PWMA) | 11 | Right motor PWM (PWMB) | 6 |
| Left direction (AIN1, AIN2) | 3, 10 | Right direction (BIN1, BIN2) | 8, 7 |
| Left encoder A, B | 18, 14 | Right encoder A, B | 4, 5 |
| IMU SDA | 1 | IMU SCL | 21 |
| Driver STBY | tied high | RGB status LED | 48 |

## Camera

Argus, NVIDIA's camera stack, runs on the Jetson host and isn't in the ROS container. So the camera is read in two steps:

1. **On the host,** [`tools/csi_camera_server.py`](../../tools/csi_camera_server.py) reads the camera with `nvarguscamerasrc` in its 1640×1232 mode, which keeps the full field of view. The Jetson's video hardware scales the frames to 820×616 at 15 fps. The script corrects their colour (below), encodes JPEG and serves them as an MJPEG stream on `127.0.0.1:8091`, each frame tagged with its capture time. It uses about 70% of one of the Jetson's six cores, most of it for the JPEG encoding.
2. **In the container,** `jetson_bot_camera`'s `mjpeg_camera_node` publishes each JPEG on `/camera/image_raw/compressed`. While anything subscribes, such as RViz, it also publishes decoded `bgr8` frames on `/camera/image_raw`. Both use the frame `camera_optical_link`. The Web UI shows the compressed topic through `web_video_server`'s `ros_compressed` stream, which passes the JPEGs on as they are, so nothing decodes or re-encodes frames for it.

`./robot.sh robot` starts the server when the Jetson has `/dev/video0`, and `./robot.sh stop` stops it. If the camera is mounted turned, set `CAMERA_FLIP`, for example `CAMERA_FLIP=2 ./robot.sh robot` for upside down. `./robot.sh help` lists the values. The node takes any MJPEG URL through the launch argument `camera_url`, so it could read an ESP32-CAM's stream too.

**Colour.** With the Jetson's default tuning, this camera and lens give everything a magenta cast, which turns pink toward the edges. On white paper over the lens, red came out up to 16% above green overall and nearly 50% above it at the edges. [`tools/camera_shading.py`](../../tools/camera_shading.py) measures this on white paper and corrects red and blue against green in every frame, so white comes out neutral grey all over. Green and brightness are left alone, so uneven light on the paper doesn't matter, and the dark corners don't get noisier. The measurement is in [`tools/camera_color_shading.json`](../../tools/camera_color_shading.json), which the server loads at start. To measure again, tape white paper over the whole lens under the room's usual light, then run `./robot.sh camera-color`. It refuses a picture that isn't plain or that is clipped to white.

## Physical Parameters

The dimensions were measured on 2026-06-07. These are the values the code uses.

| Parameter | Value | Where it's set |
| :--- | :--- | :--- |
| Wheel radius | 0.034 m | URDF (`robot_core.xacro`), sim controller (`unified_robot_config.yaml`), bridge (`main.launch.py`) |
| Wheel separation | 0.212 m, centre to centre | Same three places |
| Chassis | 19.4 × 17.8 × 9.7 cm box, centred 6.1 cm ahead of the axle | `robot_core.xacro` |
| Ground clearance | 0.056 m, floor to chassis floor | `robot_core.xacro` |
| Lidar | 0.064 m ahead of the axle, 0.161 m above the chassis floor (0.217 m above the ground) | `lidar.xacro` |
| IMU frame | 0.1 m above the chassis floor on the axle line, upside down (roll π). Not measured | `imu.xacro` |
| Mass | 1.4 kg recorded in the June audit. The URDF's links add up to 1.7 kg | URDF inertials |
| Encoder counts per wheel turn | 1,497, from a 10-turn hand test on 2026-11-10, see [RESULTS.md](../experiments/RESULTS.md#9-first-drive-with-the-new-motor-firmware) | Bridge parameter `encoder_cpr` in `main.launch.py`, and `ENCODER_CPR` in the motor firmware |

The frames follow REP 120. `base_footprint` sits on the floor, and `base_link` sits on the axle line at chassis-floor height, 0.056 m above it. Parts attached to `base_link` are placed by their height above the chassis floor, so the ground clearance isn't added again.

## Original Wiring Diagram

<p align="center">
  <img src="../../assets/wiring/cirkit_wiring_original.png" alt="Cirkit Designer wiring diagram of the first version: Jetson Nano, ESP32-S3, TB6612FNG, encoder motors, MPU6050, RPLidar, Pi camera, two battery packs with converters" width="70%">
</p>

Drawn in Cirkit Designer for the first version of the robot, which had a Jetson Nano. The Raspberry Pi camera module in the drawing isn't wired. The robot now has the Xavier NX and the CSI camera listed above, so use this diagram for the ESP32-S3 side only, and check pins against the table above.

## Reference Sheets

Vendor material, kept for wiring work:

| Sheet | Shows |
| :--- | :--- |
| [encoder_datasheet.jpg](../../assets/reference/encoder_datasheet.jpg) | Encoder wire colours and parameters (11 PPR, 22-pole magnet, 3.3 to 5 V) |
| [encoder_wiring.png](../../assets/reference/encoder_wiring.png) | Encoder connector order: M1, GND, C1 (A), C2 (B), VCC, M2 |
| [rplidar_a1_pinout.jpg](../../assets/reference/rplidar_a1_pinout.jpg) | RPLidar A1M8-R6 connector pinout and power limits |
| [jetson-nano/](../../assets/reference/jetson-nano/) | Jetson Nano pinout and board layout from the first version, no longer in use |
