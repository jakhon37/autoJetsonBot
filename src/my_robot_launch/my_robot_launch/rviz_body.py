#!/usr/bin/env python

import os
import sys
import xml.dom.minidom

# ROS 1 imports
try:
    import rospy
    from tf import TransformBroadcaster
    ROS_VERSION = 1
except ImportError:
    # ROS 2 imports
    import rclpy
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster, TransformStamped
    ROS_VERSION = 2

def load_urdf(file_path):
    """Load URDF file and return its content as a string."""
    try:
        with open(file_path, 'r') as file:
            urdf_content = xml.dom.minidom.parseString(file.read()).toprettyxml(indent="  ")
        return urdf_content
    except Exception as e:
        print(f"Error loading URDF file {file_path}: {e}")
        return None

def main():
    # Initialize ROS node
    if ROS_VERSION == 1:
        rospy.init_node('robot_tf_publisher', anonymous=True)
        tf_broadcaster = TransformBroadcaster()
    else:
        rclpy.init()
        node = Node('robot_tf_publisher')
        tf_broadcaster = TransformBroadcaster(node)
    
    # Path to the URDF file (adjust this to your URDF location)
    urdf_path = "/home/jetson/myspace/autoJetsonBot/src/my_robot_launch/urdf/robot_core.urdf"
    
    # Load URDF and set robot_description parameter
    urdf_content = load_urdf(urdf_path)
    if urdf_content is None:
        print("Failed to load URDF. Exiting.")
        if ROS_VERSION == 2:
            rclpy.shutdown()
        return
    
    if ROS_VERSION == 1:
        rospy.set_param('robot_description', urdf_content)
    else:
        node.get_logger().info("Setting robot_description parameter")
        node.set_param('robot_description', urdf_content)
    
    # Define fixed transforms based on URDF
    transforms = [
        {
            'parent': 'base_link',
            'child': 'base_footprint',
            'translation': [0.0, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0, 1.0]  # Quaternion (x, y, z, w)
        },
        {
            'parent': 'base_link',
            'child': 'chassis',
            'translation': [-0.035, 0.0, 0.0],
            'rotation': [0.0, 0.0, 0.0, 1.0]
        },
        {
            'parent': 'chassis',
            'child': 'lidar',
            'translation': [0.1, 0.0, 0.09825],
            'rotation': [0.0, 0.0, 0.0, 1.0]
        }
    ]
    
    # Publish transforms at 10 Hz
    rate = 10.0  # Hz
    if ROS_VERSION == 1:
        rate_obj = rospy.Rate(rate)
        while not rospy.is_shutdown():
            for tf in transforms:
                tf_broadcaster.sendTransform(
                    tf['translation'],
                    tf['rotation'],
                    rospy.Time.now(),
                    tf['child'],
                    tf['parent']
                )
            rate_obj.sleep()
    else:
        def timer_callback():
            for tf in transforms:
                t = TransformStamped()
                t.header.stamp = node.get_clock().now().to_msg()
                t.header.frame_id = tf['parent']
                t.child_frame_id = tf['child']
                t.transform.translation.x = tf['translation'][0]
                t.transform.translation.y = tf['translation'][1]
                t.transform.translation.z = tf['translation'][2]
                t.transform.rotation.x = tf['rotation'][0]
                t.transform.rotation.y = tf['rotation'][1]
                t.transform.rotation.z = tf['rotation'][2]
                t.transform.rotation.w = tf['rotation'][3]
                tf_broadcaster.send_transform(t)
        
        timer = node.create_timer(1.0 / rate, timer_callback)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")
        if ROS_VERSION == 2:
            rclpy.shutdown()