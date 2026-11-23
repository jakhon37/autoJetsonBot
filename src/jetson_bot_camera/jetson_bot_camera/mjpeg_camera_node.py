"""
Publish an MJPEG stream as ROS images.

On the robot the stream comes from tools/csi_camera_server.py on the Jetson host. That script
reads the CSI camera through Argus, which this container doesn't have. Any MJPEG URL works, an
ESP32-CAM's too.

Publishes camera/image_raw/compressed with every JPEG as received, and camera/image_raw decoded
to bgr8 only while something subscribes to it, which saves the CPU. Frames are stamped with the
stream's X-Timestamp when it is close to this clock, else with the time they arrive.

One thread reads the stream and keeps only the newest frame; another publishes it. If publishing
falls behind, frames are skipped instead of piling up in the connection and arriving late.
"""

import array
import threading
import time
import urllib.request

from builtin_interfaces.msg import Time
import cv2
from jetson_bot_camera.mjpeg_stream import capture_time, MjpegParser
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage, Image


def to_time_msg(seconds):
    """Turn seconds since the epoch into a builtin_interfaces/Time."""
    sec = int(seconds)
    return Time(sec=sec, nanosec=int((seconds - sec) * 1e9))


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
        # Reliable, which suits reliable subscribers such as web_video_server and RViz.
        # Depth 2 keeps no backlog of old frames.
        qos = QoSProfile(depth=2)
        self.pub_jpeg = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', qos)
        self.pub_raw = self.create_publisher(Image, 'camera/image_raw', qos)
        self.stop = threading.Event()
        self.cond = threading.Condition()
        self.latest = None  # (jpeg, headers, arrival time), replaced by each new frame
        self.announce = False  # log the frame size once per connection
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.sender = threading.Thread(target=self.publish_loop, daemon=True)
        self.reader.start()
        self.sender.start()

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
