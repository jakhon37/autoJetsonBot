# Hardware Bring-up

How to connect, start and check the real robot. The parts and wiring are in [HARDWARE.md](../architecture/HARDWARE.md), and what each check found last time is in [RESULTS.md](../experiments/RESULTS.md).

## 1. Connect

| Device | Plug into | Shows up as |
| :--- | :--- | :--- |
| ESP32-S3 motor board | Jetson USB, using the board's **native USB socket**, the one next to the RGB LED | `303a:1001`, `/dev/ttyACM0` |
| RPLidar A1 | Jetson USB, through its CP2102 adapter | `10c4:ea60`, `/dev/ttyUSB0` |
| ESP32-CAM | Power only, no data cable | Its own address on Wi-Fi |

- **Plug in first, then start the container.** Docker only sees USB devices that exist when the container starts, so restart it after plugging anything in.
- **Keep the camera off the Jetson's data lines.** Its CH340 adapter also appears as `/dev/ttyUSB*`, so launch could take it for the lidar, and opening its port resets the camera.

## 2. Start and Check

```bash
./robot.sh robot    # hardware mode, no RViz or Gazebo
./robot.sh status   # nodes, topics, ESP32 link, ports
```

The ESP32-S3's RGB LED shows yellow while booting, green when ready, red for an IMU error, and a blue pulse for each motor command.

Expected rates, measured on 2026-11-05:

| Topic | Rate |
| :--- | :--- |
| `/scan` | about 6.5 Hz, 360 points |
| `/odom`, `/joint_states`, `/imu/data` | 50 Hz |
| `/odom_filtered` | 30 Hz |

If a topic shows 0 Hz while its node is active, it's almost always a name or interface mismatch: compare `ros2 param dump` against the config.

## 3. Before the First Drive

1. **Lift the wheels off the ground.** The motor firmware starts each wheel with a short kick (PWM 550 for 40 ms), so a wheel can jerk. [MOTOR-ESP32S3](https://github.com/jakhon37/MOTOR-ESP32S3)'s `docs/FIRST_DRIVE.md` is the step-by-step checklist, with `wheel_test.py` for driving and measuring without ROS.
2. **Know the failsafe.** The ESP32-S3 stops the motors when no command arrives for 500 ms. Measured on 2026-11-10: the wheels stopped 0.75 s after the last command.
3. **Check encoder direction.** Turn each wheel forward by hand and watch the counts in `./robot.sh status`. Counts that move the same way in both directions mean the encoder's B channel isn't connected.
4. **Calibrate.** Mark a wheel, turn it 10 times, and set `encoder_cpr` to the count divided by 10. On 2026-11-10 that gave 1,497, which is set. Then measure the gyro's offsets with the robot still and set them as the firmware's `GYRO_BIAS_X/Y/Z`: it learns drifts from there, but only within 0.02 rad/s of them. Check that the front of the scan points to the front of the robot.
5. **Check the IMU's axes.** The URDF mounts `imu_link` upside down (roll π). At rest the real MPU6050 reads about −10 m/s² on z, which agrees. Turn the robot left by hand and check that `/odom_filtered`'s yaw rises, or the EKF sees yaw rate with the wrong sign.

## 4. Stop

`./robot.sh stop` saves the map if the robot was in mapping mode, and so do `up`, `sim`, `robot`, `nav` and `map2nav`, which can overwrite a map in `maps/`. To end a test without saving, stop the container instead:

```bash
docker stop auto_ros_humble
```

## Camera

The ESP32-CAM serves MJPEG at `http://<camera-ip>/stream`, which you can open in any browser. It serves one client at a time, and its frames carry no timestamps. Every `/capture` call also saves a photo to its SD card. The stream rates measured so far are in [RESULTS.md](../experiments/RESULTS.md#7-esp32-cam-video-over-wi-fi).
