#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from ament_index_python.packages import get_package_share_directory

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', 'jetson_bot_detection/image')
        
        confidence_threshold = self.get_parameter('confidence_threshold').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Publisher for annotated images
        self.publisher_ = self.create_publisher(Image, output_topic, 10)
        
        # Subscriber for input images
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()

        # Load the pre-trained MobileNetSSD model
        try:
            pkg_path = get_package_share_directory('jetson_bot_detection')
            resource_dir = os.path.join(pkg_path, 'resource')
        except Exception:
            # Fallback for local development
            resource_dir = os.path.join(os.path.dirname(__file__), '..', 'resource')
            
        prototxt_path = os.path.join(resource_dir, 'MobileNetSSD_deploy.prototxt.txt')
        model_path = os.path.join(resource_dir, 'MobileNetSSD_deploy.caffemodel')
        
        if not os.path.exists(prototxt_path) or not os.path.exists(model_path):
            self.get_logger().error(f"Model files not found in {resource_dir}")
            raise FileNotFoundError(f"Model files not found in {resource_dir}")
            
        self.net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
        
        # Define the class labels MobileNetSSD was trained to detect
        self.CLASSES = [
            "background", "aeroplane", "bicycle", "bird", "boat",
            "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
            "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
            "sofa", "train", "tvmonitor"
        ]
        
        self.get_logger().info(f"Object Detection Node started. Subscribed to {input_topic}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV frame
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Get frame dimensions
        (h, w) = frame.shape[:2]
        # Prepare the frame for object detection: resize and create a blob
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                     0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        confidence_threshold = self.get_parameter('confidence_threshold').value

        # Loop over the detections and draw bounding boxes for high-confidence detections
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > confidence_threshold:
                idx = int(detections[0, 0, i, 1])
                label = self.CLASSES[idx] if idx < len(self.CLASSES) else "Unknown"
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                cv2.rectangle(frame, (startX, startY), (endX, endY),
                              (0, 255, 0), 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, f"{label}: {confidence:.2f}",
                            (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)

        # Convert the annotated frame to a ROS Image message and publish it
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # Copy header from original message to maintain synchronization/TF
        image_msg.header = msg.header
        self.publisher_.publish(image_msg)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
