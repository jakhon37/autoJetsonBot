#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        # Publisher for annotated images
        self.publisher_ = self.create_publisher(Image, 'object_detection/image', 10)
        self.bridge = CvBridge()

        # Load the pre-trained MobileNetSSD model
        # Adjust these paths to where you have saved your model files.
        prototxt_path = '/home/ubuntu/my_space/autonomous_ROS/src/object_detection/resource/MobileNetSSD_deploy.prototxt.txt'
        model_path = '/home/ubuntu/my_space/autonomous_ROS/src/object_detection/resource/MobileNetSSD_deploy.caffemodel'
        self.net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
        
        # Define the class labels MobileNetSSD was trained to detect
        self.CLASSES = [
            "background", "aeroplane", "bicycle", "bird", "boat",
            "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
            "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
            "sofa", "train", "tvmonitor"
        ]
        
        # Open the Raspberry Pi camera (this may be 0 or another index depending on your setup)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera!")
        
        # Create a timer to run the detection loop at 10 Hz (adjust as needed)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1 sec = 10 fps

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture image from camera")
            return

        # Get frame dimensions
        (h, w) = frame.shape[:2]
        # Prepare the frame for object detection: resize and create a blob
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                     0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        # Loop over the detections and draw bounding boxes for high-confidence detections
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5:
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
        self.publisher_.publish(image_msg)
        self.get_logger().info("Published annotated image")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
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
