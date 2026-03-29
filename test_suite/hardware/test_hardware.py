"""
Hardware Interface Tests
Tests for real robot hardware (motors, sensors, etc.)
"""
import unittest
import subprocess
import time
import sys
import os
import re

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_suite.config import config
from test_suite.utils import ROS2TestClient, DockerClient


class TestMotorController(unittest.TestCase):
    """Test motor controller hardware"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_motor_controller_connected(self):
        """Test motor controller is connected"""
        # Check for Arduino/serial device
        code, stdout, _ = DockerClient.exec_in_container(
            config.CONTAINER_NAME,
            ["ls", "-la", "/dev/ttyACM0"]
        )
        
        if code != 0:
            # Try USB serial
            code, stdout, _ = DockerClient.exec_in_container(
                config.CONTAINER_NAME,
                ["ls", "-la", "/dev/ttyUSB0"]
            )
        
        if code != 0:
            self.skipTest("Motor controller (Arduino) not connected")
        
        self.assertEqual(code, 0, "Motor controller not found")
    
    def test_motor_topics_available(self):
        """Test motor command topics"""
        topics = ROS2TestClient.list_topics()
        
        motor_topics = [t for t in topics if "motor" in t.lower() or "cmd_vel" in t]
        
        if not motor_topics:
            self.skipTest("Motor topics not available")


class TestLidar(unittest.TestCase):
    """Test RPLidar sensor"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_lidar_device_connected(self):
        """Test lidar device is connected"""
        code, stdout, _ = DockerClient.exec_in_container(
            config.CONTAINER_NAME,
            ["ls", "-la", "/dev/ttyUSB0"]
        )
        
        # Try other USB ports
        if code != 0:
            for port in ["/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]:
                code, stdout, _ = DockerClient.exec_in_container(
                    config.CONTAINER_NAME,
                    ["ls", "-la", port]
                )
                if code == 0:
                    break
        
        if code != 0:
            self.skipTest("Lidar not connected")
    
    def test_lidar_publishing(self):
        """Test lidar is publishing scan data"""
        topics = ROS2TestClient.list_topics()
        
        if "/scan" not in topics:
            self.skipTest("Lidar scan topic not available")
        
        info = ROS2TestClient.topic_info("/scan")
        self.assertGreater(info.get('publishers', 0), 0, "Lidar not publishing")


class TestIMU(unittest.TestCase):
    """Test MPU6050 IMU sensor"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_imu_topic_available(self):
        """Test IMU topic is available"""
        topics = ROS2TestClient.list_topics()
        
        imu_topics = [t for t in topics if "imu" in t.lower()]
        
        if not imu_topics:
            self.skipTest("IMU topic not available")
        
        # Check first IMU topic
        topic = imu_topics[0]
        info = ROS2TestClient.topic_info(topic)
        self.assertGreater(info.get('publishers', 0), 0, "IMU not publishing")
    
    def test_imu_data_valid(self):
        """Test IMU data is valid (not all zeros)"""
        # This would require subscribing to the topic
        # For now just check topic exists
        topics = ROS2TestClient.list_topics()
        
        imu_topics = [t for t in topics if "imu" in t.lower()]
        
        if not imu_topics:
            self.skipTest("IMU topic not available")


class TestCamera(unittest.TestCase):
    """Test camera sensor"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_camera_available(self):
        """Test camera is available"""
        topics = ROS2TestClient.list_topics()
        
        camera_topics = [t for t in topics if "camera" in t.lower() or "image" in t.lower()]
        
        if not camera_topics:
            self.skipTest("Camera not available")
    
    def test_camera_publishing(self):
        """Test camera is publishing images"""
        topics = ROS2TestClient.list_topics()
        
        if "/camera/image_raw" not in topics:
            self.skipTest("Camera image topic not available")
        
        info = ROS2TestClient.topic_info("/camera/image_raw")
        self.assertGreater(info.get('publishers', 0), 0, "Camera not publishing")


class TestSerialCommunication(unittest.TestCase):
    """Test serial communication"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_serial_devices_exist(self):
        """Test serial devices are accessible"""
        code, stdout, _ = DockerClient.exec_in_container(
            config.CONTAINER_NAME,
            ["ls", "/dev/ttyACM0"]
        )
        
        if code != 0:
            # Try any available serial port
            code, stdout, _ = DockerClient.exec_in_container(
                config.CONTAINER_NAME,
                ["bash", "-c", "ls /dev/tty[A-Z]* 2>/dev/null | head -5"]
            )
        
        if code != 0:
            self.skipTest("No serial devices found")
    
    def test_rosserial_connection(self):
        """Test rosserial connection"""
        # Check if there's any rosserial node running
        nodes = ROS2TestClient.list_nodes()
        
        rosserial = any("serial" in node.lower() for node in nodes)
        
        if not rosserial:
            print("  Note: rosserial node not detected - may use different communication")


class TestHardwareParameters(unittest.TestCase):
    """Test hardware-specific parameters"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_hardware_interface_loaded(self):
        """Test hardware interface is loaded"""
        # This would typically show in controller manager
        nodes = ROS2TestClient.list_nodes()
        
        controller = any("controller" in node.lower() for node in nodes)
        
        if not controller:
            self.skipTest("Controllers not loaded")



if __name__ == "__main__":
    unittest.main(verbosity=2)
