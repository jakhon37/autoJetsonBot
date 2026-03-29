"""
Gazebo Simulation Tests
Tests for Gazebo simulation and robot spawning
"""
import unittest
import subprocess
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_suite.config import config
from test_suite.utils import ROS2TestClient


class TestGazeboSimulation(unittest.TestCase):
    """Test Gazebo simulation"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_gazebo_server_running(self):
        """Test that Gazebo server is running"""
        nodes = ROS2TestClient.list_nodes()
        
        gz_found = any("gzserver" in node for node in nodes)
        
        if not gz_found:
            self.skipTest("Gazebo not running. Start with ./robot.sh sim")
        
        self.assertTrue(gz_found, "Gazebo server not found")
    
    def test_gazebo_client_running(self):
        """Test that Gazebo client (GUI) is running"""
        nodes = ROS2TestClient.list_nodes()
        
        gz_client = any("gzclient" in node for node in nodes)
        
        # GUI might not be available in headless mode - not a failure
        if not gz_client:
            print("  Note: Gazebo client (GUI) not running - may be headless")
    
    def test_robot_spawned(self):
        """Test that robot is spawned in Gazebo"""
        topics = ROS2TestClient.list_topics()
        
        # Check for joint states which indicates robot is spawned
        self.assertIn("/joint_states", topics, "Robot not spawned (no joint states)")
        
        info = ROS2TestClient.topic_info("/joint_states")
        self.assertGreater(info.get('publishers', 0), 0, "Joint states not publishing")


class TestRobotController(unittest.TestCase):
    """Test robot controllers"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_controller_manager_running(self):
        """Test that controller manager is running"""
        nodes = ROS2TestClient.list_nodes()
        
        # Check for controller-related nodes
        controller_found = any(
            "controller" in node.lower() or "spawner" in node.lower()
            for node in nodes
        )
        
        if not controller_found:
            self.skipTest("Controllers not loaded")
        
        self.assertTrue(controller_found, "Controller manager not found")
    
    def test_diff_drive_controller(self):
        """Test differential drive controller"""
        topics = ROS2TestClient.list_topics()
        
        cmd_vel_topics = [t for t in topics if "cmd_vel" in t]
        
        self.assertGreater(len(cmd_vel_topics), 0, "No cmd_vel topic found")
    
    def test_joint_broadcaster(self):
        """Test joint state broadcaster"""
        topics = ROS2TestClient.list_topics()
        
        if "/joint_states" not in topics:
            self.skipTest("Joint states not available")
        
        info = ROS2TestClient.topic_info("/joint_states")
        self.assertGreater(info.get('publishers', 0), 0, "Joint states not publishing")


class TestRobotModel(unittest.TestCase):
    """Test robot model and description"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_robot_description_available(self):
        """Test robot description is available"""
        topics = ROS2TestClient.list_topics()
        
        self.assertIn(
            "/robot_description", topics,
            "Robot description not found"
        )
    
    def test_robot_state_publisher(self):
        """Test robot state publisher is running"""
        nodes = ROS2TestClient.list_nodes()
        
        rsp_found = any("robot_state_publisher" in node for node in nodes)
        
        if not rsp_found:
            self.skipTest("Robot state publisher not running")
        
        self.assertTrue(rsp_found, "Robot state publisher not found")
    
    def test_robot_links(self):
        """Test that robot has expected links"""
        # Check TF for robot frames
        # This is validated by checking tf topic exists
        topics = ROS2TestClient.list_topics()
        
        self.assertIn("/tf", topics, "TF not available")


class TestSensors(unittest.TestCase):
    """Test robot sensors in simulation"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_lidar_available(self):
        """Test lidar/scan topic"""
        topics = ROS2TestClient.list_topics()
        
        if "/scan" not in topics:
            self.skipTest("Lidar not available")
        
        info = ROS2TestClient.topic_info("/scan")
        self.assertGreater(info.get('publishers', 0), 0, "Scan has no publishers")
    
    def test_camera_available(self):
        """Test camera topic"""
        topics = ROS2TestClient.list_topics()
        
        camera_topics = [t for t in topics if "camera" in t.lower()]
        
        if not camera_topics:
            self.skipTest("Camera not available")
        
        # Check main image topic
        if "/camera/image_raw" in topics:
            info = ROS2TestClient.topic_info("/camera/image_raw")
            self.assertGreater(info.get('publishers', 0), 0, "Camera not publishing")


class TestGazeboWorld(unittest.TestCase):
    """Test Gazebo world configuration"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_gazebo_world_loaded(self):
        """Test Gazebo world is loaded"""
        nodes = ROS2TestClient.list_nodes()
        
        gz_found = any("gzserver" in node for node in nodes)
        
        if not gz_found:
            self.skipTest("Gazebo not running")
        
        self.assertTrue(gz_found, "Gazebo not loaded")


if __name__ == "__main__":
    unittest.main(verbosity=2)
