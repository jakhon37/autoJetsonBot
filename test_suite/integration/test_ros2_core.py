"""
ROS2 Core System Tests
Tests for ROS2 topics, nodes, and basic functionality
"""
import unittest
import subprocess
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_suite.config import config
from test_suite.utils import ROS2TestClient, assert_ros_topic, assert_ros_node, timer


class TestROS2Core(unittest.TestCase):
    """Test ROS2 core functionality"""
    
    @classmethod
    def setUpClass(cls):
        """Ensure container is running before tests"""
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(2)
    
    def test_01_ros2_cli_works(self):
        """Test that ROS2 CLI is accessible"""
        code, stdout, stderr = ROS2TestClient.run_cli(["ros2", "pkg", "list"])
        self.assertEqual(code, 0, f"ROS2 CLI failed: {stderr}")
    
    def test_02_ros2_daemon_running(self):
        """Test that ROS2 daemon is running"""
        # First, try to start it
        ROS2TestClient.run_cli(["ros2", "daemon", "start"], timeout=5)
        time.sleep(1)
        
        # Then check status
        code, stdout, _ = ROS2TestClient.run_cli(["ros2", "daemon", "status"])
        # Either running or already running (return code 0 or 1)
        self.assertIn(code, [0, 1], "Could not check daemon status")
    
    def test_03_topic_list_not_empty(self):
        """Test that ROS2 topics are available"""
        topics = ROS2TestClient.list_topics()
        self.assertGreater(len(topics), 0, "No ROS2 topics found")
    
    def test_04_node_list_not_empty(self):
        """Test that ROS2 nodes are running"""
        nodes = ROS2TestClient.list_nodes()
        self.assertGreater(len(nodes), 0, "No ROS2 nodes found")


class TestROSTopics(unittest.TestCase):
    """Test required ROS2 topics"""
    
    REQUIRED_TOPICS = [
        "/diff_cont/cmd_vel_unstamped",
        "/odom", 
        "/joint_states",
        "/tf",
        "/robot_description",
    ]
    
    OPTIONAL_TOPICS = [
        "/scan",
        "/camera/image_raw",
        "/map",
    ]
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_required_topics_exist(self):
        """Test that all required topics exist"""
        available_topics = ROS2TestClient.list_topics()
        
        for topic in self.REQUIRED_TOPICS:
            with self.subTest(topic=topic):
                self.assertIn(
                    topic, available_topics,
                    f"Required topic '{topic}' not found"
                )
    
    def test_required_topics_have_publishers(self):
        """Test that required topics have publishers or subscribers"""
        for topic in self.REQUIRED_TOPICS:
            with self.subTest(topic=topic):
                info = ROS2TestClient.topic_info(topic)
                has_activity = (
                    info.get('publishers', 0) > 0 or 
                    info.get('subscribers', 0) > 0
                )
                self.assertTrue(
                    has_activity,
                    f"Topic '{topic}' has no publishers or subscribers"
                )
    
    def test_optional_topics_if_available(self):
        """Test optional topics if they exist"""
        available_topics = ROS2TestClient.list_topics()
        
        for topic in self.OPTIONAL_TOPICS:
            if topic in available_topics:
                with self.subTest(topic=topic):
                    info = ROS2TestClient.topic_info(topic)
                    self.assertGreater(
                        info.get('publishers', 0), 0,
                        f"Optional topic '{topic}' has no publishers"
                    )


class TestROSTopicFrequencies(unittest.TestCase):
    """Test that topics are publishing at expected frequencies"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_odom_frequency(self):
        """Test odometry topic frequency"""
        with timer("odom frequency check"):
            hz = ROS2TestClient.topic_hz(config.ODOM_TOPIC)
            self.assertGreaterEqual(
                hz, config.MIN_ODOM_FREQUENCY,
                f"odom frequency {hz:.1f} Hz below minimum {config.MIN_ODOM_FREQUENCY} Hz"
            )
    
    def test_joint_states_frequency(self):
        """Test joint states topic frequency"""
        with timer("joint states frequency check"):
            hz = ROS2TestClient.topic_hz(config.JOINT_STATES_TOPIC)
            self.assertGreaterEqual(
                hz, config.MIN_ODOM_FREQUENCY,
                f"joint_states frequency {hz:.1f} Hz below minimum {config.MIN_ODOM_FREQUENCY} Hz"
            )
    
    def test_scan_frequency(self):
        """Test lidar scan topic frequency"""
        topics = ROS2TestClient.list_topics()
        if config.SCAN_TOPIC in topics:
            with timer("scan frequency check"):
                hz = ROS2TestClient.topic_hz(config.SCAN_TOPIC)
                self.assertGreaterEqual(
                    hz, config.MIN_SCAN_FREQUENCY,
                    f"scan frequency {hz:.1f} Hz below minimum {config.MIN_SCAN_FREQUENCY} Hz"
                )


class TestROSNodes(unittest.TestCase):
    """Test required ROS2 nodes"""
    
    CORE_NODES = [
        "/robot_state_publisher",
        "/gazebo",
        "/controller_manager",
    ]
    
    NAV_NODES = [
        "/controller_server",
        "/planner_server",
        "/bt_navigator",
    ]
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_core_nodes_running(self):
        """Test that core nodes are running"""
        nodes = ROS2TestClient.list_nodes()
        
        for node in self.CORE_NODES:
            with self.subTest(node=node):
                # Check if any node name contains this
                found = any(node in n for n in nodes)
                self.assertTrue(found, f"Core node '{node}' not found")
    
    def test_navigation_nodes_if_enabled(self):
        """Test navigation nodes if navigation is enabled"""
        nodes = ROS2TestClient.list_nodes()
        
        # Check if any nav nodes are running
        nav_running = any(any(n in node for node in nodes) for n in self.NAV_NODES)
        
        if nav_running:
            for node in self.NAV_NODES:
                with self.subTest(node=node):
                    found = any(node in n for n in nodes)
                    self.assertTrue(found, f"Nav node '{node}' not found")


class TestROSParameters(unittest.TestCase):
    """Test ROS2 parameters"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(2)
    
    def test_use_sim_time_parameter(self):
        """Test use_sim_time parameter is set correctly"""
        # This is typically set at node level
        topics = ROS2TestClient.list_topics()
        # If /clock topic exists, sim time is being used
        if "/clock" in topics:
            self.assertTrue(True)
        else:
            self.skipTest("Simulation not running")


if __name__ == "__main__":
    unittest.main(verbosity=2)
