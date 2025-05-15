#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # Publisher to send out raw camera images
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Open the camera (adjust the index if needed)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera!")
        else:
            self.get_logger().info("Camera opened successfully.")

        # Create a timer to capture frames at ~30 FPS
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame from camera")
            return

        # Convert the OpenCV frame (BGR format) to a ROS Image message
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting frame: {e}")
            return

        # Publish the image
        self.publisher_.publish(image_msg)
        self.get_logger().debug("Published camera frame")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
