"""
Publish an MJPEG stream as ROS images.

On the robot the stream comes from tools/csi_camera_server.py on the Jetson host. That script
reads the CSI camera through Argus, which this container doesn't have. Any MJPEG URL works, an
ESP32-CAM's too.

Publishes camera/image_raw/compressed with every JPEG as received, and camera/image_raw decoded
to bgr8 only while something subscribes to it, which saves the CPU. Frames are stamped with the
stream's X-Timestamp when it is close to this clock, else with the time they arrive.

With a lens calibration (the camera_info_file parameter, by default this package's
config/imx219_wide_820x616.yaml), it also publishes camera/camera_info with each frame, with the
frame's own header. Only frames of the calibration's size get one: a calibration holds for one
size and flip only.

One thread reads the stream and keeps only the newest frame; another publishes it. If publishing
falls behind, frames are skipped instead of piling up in the connection and arriving late.
"""

import array
import os
import threading
import time
import urllib.request

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
import cv2
from jetson_bot_camera import camera_info_file
from jetson_bot_camera.mjpeg_stream import capture_time, jpeg_size, MjpegParser
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

CALIBRATION = 'imx219_wide_820x616.yaml'


def to_time_msg(seconds):
    """Turn seconds since the epoch into a builtin_interfaces/Time."""
    sec = int(seconds)
    return Time(sec=sec, nanosec=int((seconds - sec) * 1e9))


def camera_info_msg(calibration):
    """Turn a calibration from camera_info_file.load() into a CameraInfo, header left empty."""
    msg = CameraInfo()
    msg.width, msg.height = calibration['width'], calibration['height']
    msg.distortion_model = calibration['distortion_model']
    msg.d = calibration['d']
    msg.k = calibration['k']
    msg.r = calibration['r']
    msg.p = calibration['p']
    return msg


def as_uint8_array(data):
    """
    Wrap bytes for a uint8[] message field.

    Given plain bytes, the generated setter checks every element in Python, which takes about
    half a second for one 820x616 image. An array('B') is taken as it is.
    """
    return array.array('B', data)


class MjpegCameraNode(Node):
    """Read the stream on one thread and publish the newest frame on another."""

    def __init__(self):
        super().__init__('mjpeg_camera')
        self.url = self.declare_parameter('url', 'http://127.0.0.1:8091/stream').value
        self.frame_id = self.declare_parameter('frame_id', 'camera_optical_link').value
        self.retry_s = float(self.declare_parameter('retry_s', 2.0).value)
        info_file = self.declare_parameter('camera_info_file', '').value
        # Reliable, which suits reliable subscribers such as web_video_server and RViz.
        # Depth 2 keeps no backlog of old frames.
        qos = QoSProfile(depth=2)
        self.pub_jpeg = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', qos)
        self.pub_raw = self.create_publisher(Image, 'camera/image_raw', qos)
        self.info = self.load_calibration(info_file)
        self.pub_info = self.create_publisher(CameraInfo, 'camera/camera_info', qos)
        self.stop = threading.Event()
        self.cond = threading.Condition()
        self.latest = None  # (jpeg, headers, arrival time), replaced by each new frame
        self.announce = False  # log the frame size once per connection
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.sender = threading.Thread(target=self.publish_loop, daemon=True)
        self.reader.start()
        self.sender.start()

    def load_calibration(self, path):
        """Return the lens calibration as a CameraInfo, or None if there's none to use."""
        if not path:
            path = os.path.join(get_package_share_directory('jetson_bot_camera'), 'config',
                                CALIBRATION)
        if path == 'none':
            return None
        try:
            calibration = camera_info_file.load(path)
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f'No camera_info: {path} not used ({exc})')
            return None
        self.get_logger().info(
            f'Publishing camera_info from {path}: {calibration["distortion_model"]}, '
            f'{calibration["width"]}x{calibration["height"]}')
        return camera_info_msg(calibration)

    def shutdown(self):
        """Stop both threads."""
        self.stop.set()
        with self.cond:
            self.cond.notify_all()

    def read_loop(self):
        """Connect, keep the newest frame, and reconnect after any failure until stopped."""
        while not self.stop.is_set():
            try:
                with urllib.request.urlopen(self.url, timeout=5.0) as resp:
                    parser = MjpegParser()
                    self.announce = True
                    while not self.stop.is_set():
                        data = resp.read1(65536)
                        if not data:
                            raise ConnectionError('the stream ended')
                        frames = parser.feed(data)
                        if frames:
                            with self.cond:
                                self.latest = frames[-1] + (time.time(),)
                                self.cond.notify()
            # Any failure (no server yet, a dropped connection, a timeout) means: wait, reconnect.
            except Exception as exc:
                if self.stop.is_set():
                    break
                self.get_logger().warn(
                    f'Camera stream {self.url} unavailable ({exc}); retrying every '
                    f'{self.retry_s:g} s', throttle_duration_sec=30.0)
                self.stop.wait(self.retry_s)

    def publish_loop(self):
        """Publish each newest frame; frames that arrive while publishing are skipped."""
        while not self.stop.is_set():
            with self.cond:
                self.cond.wait_for(lambda: self.latest is not None or self.stop.is_set(), 1.0)
                item, self.latest = self.latest, None
            if item is not None and not self.stop.is_set():
                self.publish(*item)

    def publish(self, jpeg, headers, arrived):
        """Publish one frame as CompressedImage and, if anyone listens, as Image."""
        stamp = capture_time(headers, arrived)
        msg = CompressedImage()
        msg.header.stamp = to_time_msg(stamp if stamp is not None else arrived)
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = as_uint8_array(jpeg)
        self.pub_jpeg.publish(msg)
        if self.info is not None:
            size = jpeg_size(jpeg)
            if size == (self.info.width, self.info.height):
                self.info.header = msg.header
                self.pub_info.publish(self.info)
            else:
                self.get_logger().warn(
                    f'Frames are {size}, the calibration {self.info.width}x{self.info.height}: '
                    'no camera_info for them', throttle_duration_sec=30.0)
        announce = self.announce
        if not announce and self.pub_raw.get_subscription_count() == 0:
            return
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('A frame did not decode as JPEG', throttle_duration_sec=30.0)
            return
        if announce:
            self.announce = False
            self.get_logger().info(
                f'Receiving {frame.shape[1]}x{frame.shape[0]} frames from {self.url}')
        if self.pub_raw.get_subscription_count() == 0:
            return
        img = Image()
        img.header = msg.header
        img.height, img.width = frame.shape[:2]
        img.encoding = 'bgr8'
        img.is_bigendian = 0
        img.step = img.width * 3
        img.data = as_uint8_array(frame.tobytes())
        self.pub_raw.publish(img)


def main(args=None):
    rclpy.init(args=args)
    node = MjpegCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
