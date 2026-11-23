# Hardware Bring-up

How to connect, start and check the real robot. The parts and wiring are in [HARDWARE.md](../architecture/HARDWARE.md), and what each check found last time is in [RESULTS.md](../experiments/RESULTS.md).

## 1. Connect

| Device | Plug into | Shows up as |
| :--- | :--- | :--- |
| ESP32-S3 motor board | Jetson USB, using the board's **native USB socket**, the one next to the RGB LED | `303a:1001`, `/dev/ttyACM0` |
| RPLidar A1 | Jetson USB, through its CP2102 adapter | `10c4:ea60`, `/dev/ttyUSB0` |
| IMX219 camera | The Jetson's CAM0 connector, by CSI ribbon, with the power off | `/dev/video0`, if it was found at boot |

- **Use the ESP32-S3's native socket, not COM.** Its COM socket (a CH343, `1a86:55d3`) also shows up as `/dev/ttyACM0`, but the firmware doesn't talk there, so the bridge gets no wheel or IMU data. When the COM socket is all they find, launch's hardware audit and `./robot.sh status` say so.
- **Plug in first, then start the container.** Docker only sees USB devices that exist when the container starts, so restart it after plugging anything in.
- **Connect the camera with the power off, then boot.** The Jetson looks for a CSI camera only at boot, and plugging one in while powered can damage it. `journalctl -k -b | grep imx219` shows `subdev imx219 9-0010 bound` when it was found.

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

`./robot.sh stop` saves the map if the robot was in mapping mode, and so do `up`, `sim`, `robot`, `nav` and `map2nav`, which can overwrite a map in `maps/`. To end a test without saving, restart the container first, which stops SLAM before anything can save:

```bash
./robot.sh stop f
```

That also stops the camera server, which runs on the host. `docker stop auto_ros_humble` saves nothing either, but leaves the camera server running until the next `./robot.sh` command.

## Camera

`./robot.sh robot` starts the camera server on the Jetson host and the camera node in the container. [HARDWARE.md](../architecture/HARDWARE.md#camera) explains the split. The launch log's hardware audit prints the server's health, and the server's own log is `/tmp/autojetsonbot_camera_server.log` on the Jetson. To check by hand:

```bash
curl -s http://127.0.0.1:8091/health                  # on the Jetson: ok frames=... fps=15.0 ...
curl -s -o frame.jpg http://127.0.0.1:8091/snapshot.jpg
ros2 topic hz /camera/image_raw/compressed            # in the container: about 15
```

The Web UI's camera panel shows `/camera/image_raw/compressed` as it is, through `web_video_server`'s `ros_compressed` stream on port 8080. If no frame arrives within 4 s, the panel falls back to the raw topic re-encoded at 320×240.

- **No `/dev/video0`:** The camera wasn't found at boot. Power off, reseat the ribbon at both ends with the latches closed, and boot. When the sensor doesn't answer, `journalctl -k -b | grep imx219` shows `error -121`.
- **Blurry picture:** The lens focuses by turning. Out focuses nearer and in focuses farther. Set it on a still scene 2 to 3 m away, then lock it with its ring or a dab of glue, since vibration turns it.
- **Picture turned:** Set `CAMERA_FLIP`, listed in `./robot.sh help`.
- **Colour off, or pink toward the edges:** Tape plain white paper over the whole lens, under the room's usual light and not mixed with daylight from one side. Then run `./robot.sh camera-color`. It measures 30 frames, saves `tools/camera_color_shading.json` and uses it at once. `curl -s http://127.0.0.1:8091/shading` then shows how even the paper is. Measure again after changing the lens or its focus.
