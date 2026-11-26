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
- **Power.** The Jetson runs from its mains adapter, through the DC jack and the switch beside it, or from its battery, through the next switch. Have only one of those two switches on at a time. The motors run only with the switch on the right side wall on. "Power off" in this guide means all three switches off. [HARDWARE.md](../architecture/HARDWARE.md#power) lists the controls, numbered as on the rear photo at the top of that page.

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
5. **Check the IMU's axes.** The MPU6050 sits chip side down on the motor board, and the URDF mounts `imu_link` upside down to match (roll π). At rest it should read about −10 m/s² on z, as it did on 2026-11-09, before the rebuild moved it. Turn the robot left by hand and check that `/odom_filtered`'s yaw rises, or the EKF sees yaw rate with the wrong sign. On 2026-11-23, in the rebuilt body, the gyro read a turn on the floor within 1° of the wheels.

## 4. Stop

`./robot.sh stop` saves the map if the robot was in mapping mode, and so do `up`, `sim`, `robot`, `nav` and `map2nav`, which can overwrite a map in `maps/`. Each says whether the save worked. `nav` and `map2nav` stay in mapping mode if it didn't, while the others stop anyway. To keep a map under its own name, save it first with `./robot.sh save-map NAME`. To end a test without saving, restart the container first, which stops SLAM before anything can save:

```bash
./robot.sh stop f
```

That also stops the camera server, which runs on the host. `docker stop auto_ros_humble` saves nothing either, but leaves the camera server running until the next `./robot.sh` command.

## Camera

`./robot.sh robot` starts the camera server on the Jetson host and the camera node in the container. [HARDWARE.md](../architecture/HARDWARE.md#camera) explains the split. The launch log's hardware audit prints the server's health, and the server's own log is `/tmp/autojetsonbot_camera_server.log` on the Jetson. To check by hand:

```bash
curl -s http://127.0.0.1:8091/health                  # on the Jetson: ok frames=... fps=15.0 ... jpeg=hardware ...
curl -s -o frame.jpg http://127.0.0.1:8091/snapshot.jpg
ros2 topic hz /camera/image_raw/compressed            # in the container: about 15
ros2 topic echo /camera/camera_info --once            # the lens calibration, rational_polynomial
```

The Web UI's camera panel shows `/camera/image_raw/compressed` as it is, through `web_video_server`'s `ros_compressed` stream on port 8080. If no frame arrives within 4 s, the panel falls back to the raw topic re-encoded at 320×240.

- **No `/dev/video0`:** The camera wasn't found at boot. Power off, reseat the ribbon at both ends with the latches closed, and boot. When the sensor doesn't answer, `journalctl -k -b | grep imx219` shows `error -121`.
- **Blurry picture:** The lens focuses by turning. Out focuses nearer and in focuses farther. Set it on a still scene 2 to 3 m away, then lock it with its ring or a dab of glue, since vibration turns it. It was locked on 2026-11-24. Turning it changes the calibration, so calibrate again after.
- **No `/camera/camera_info`:** The launch log says why, after `No camera_info:`. The calibration is for 820×616 frames, so other sizes get none. A new file in `src/jetson_bot_camera/config` needs `./robot.sh build` once before the node finds it.
- **Picture turned:** Set `CAMERA_FLIP`, listed in `./robot.sh help`.
- **`jpeg=software` in `/health`:** The Jetson's JPEG engine didn't start or stopped answering, and the server's log says why. The picture is the same, at about a third of a core more. To compare the two by hand, stop the robot and run `python3 tools/csi_camera_server.py --jpeg software`, or `--jpeg hardware`, which refuses to start without the engine.
- **Colour off, or pink toward the edges:** Tape plain white paper over the whole lens, under the room's usual light and not mixed with daylight from one side. Then run `./robot.sh camera-color`. It measures 30 frames, saves `tools/camera_color_shading.json` and uses it at once. `curl -s http://127.0.0.1:8091/shading` then shows how even the paper is. Measure again after changing the lens or its focus.

### Calibrating the Lens

[`tools/camera_intrinsics.py`](../../tools/camera_intrinsics.py) runs in the container, whose OpenCV has the ArUco module, and reads frames from the camera server, so start the robot, or the server alone, first. The board is [`assets/calibration/charuco_8x5_32mm_A4.pdf`](../../assets/calibration/charuco_8x5_32mm_A4.pdf), printed at 100% and taped flat, or [`charuco_8x5_screen_1920x1080.png`](../../assets/calibration/charuco_8x5_screen_1920x1080.png) full screen on a laptop or tablet. It is found even when part of it is out of the picture. Another ChArUco board works too, given its squares, its markers' dictionary and their width as a share of a square's, such as `--board charuco:7x5:DICT_6X6_50:0.5`, the one a tablet showed on 2026-11-24. So does a plain checkerboard, with `--board chessboard:8x6` for 8 × 6 inner corners, but only when wholly in view.

```bash
docker exec -it auto_ros_humble bash
cd /autonomous_ROS/tools
python3 camera_intrinsics.py focus                        # sharpness, before and after touching the lens
python3 camera_intrinsics.py capture /tmp/calib_views --preview-port 8093  # keeps still views that differ, up to 30
python3 camera_intrinsics.py calibrate /tmp/calib_views \
    --out /tmp/imx219_wide_820x616.yaml --report /tmp/calib_report.json
python3 camera_intrinsics.py tilt /tmp/calib_views --camera-info /tmp/imx219_wide_820x616.yaml
```

- **Views:** Hold the board still for about two seconds per view and move it through every part of the picture, the corners most of all, with some views turned 30 to 45° away from facing the camera. Near the edges a board 30 cm or more away is too small to be found, so hold it close and let it run off the picture. The bottom corners look at the floor just in front of the robot, so the board has to lie there. `capture` prints how many corners each part of the picture has. With `--preview-port` it also serves the frames it looks at, red where no view has corners yet and yellow where they are few, for whoever holds the board: on the PC run `ssh -N -L 127.0.0.1:18093:127.0.0.1:8093 jetson` and open `http://localhost:18093/`. The first views, on 2026-11-24, reached only the middle. The second reached all of the picture but its corners.
- **Lens model:** `calibrate` fits OpenCV's rational model and its fisheye model, tests each on views left out of its fit, and writes the better one as a ROS `camera_info` YAML. Copy it to `src/jetson_bot_camera/config/imx219_wide_820x616.yaml`, where the camera node reads it.
- **Tilt:** `tilt` gives the camera's pitch and roll from views of a board standing upright on the floor, a tablet on its stand say, with its rows level, so capture some that way. At least one must be turned well away from facing the camera. A board lying flat in front of the robot doesn't work: the camera is only 10.6 cm up and sees it almost edge on. The position comes from a ruler. Both go into `camera_joint` in `camera.xacro`.

