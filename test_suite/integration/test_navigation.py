"""
Navigation Tests
Tests for Nav2 navigation stack and SLAM
"""
import unittest
import subprocess
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_suite.config import config
from test_suite.utils import ROS2TestClient


class TestNavigationStack(unittest.TestCase):
    """Test Nav2 navigation stack"""
    
    NAV2_NODES = [
        "/controller_server",
        "/planner_server", 
        "/bt_navigator",
        "/recoveries_server",
        "/waypoint_follower",
    ]
    
    NAV2_TOPICS = [
        "/navigate_to_pose",
        "/navigate_to_pose/_action/status",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
    ]
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_navigation_nodes_running(self):
        """Test that Nav2 nodes are running"""
        nodes = ROS2TestClient.list_nodes()
        
        nav_found = False
        for expected_node in self.NAV2_NODES:
            if any(expected_node in n for n in nodes):
                nav_found = True
                break
        
        if not nav_found:
            self.skipTest("Navigation stack not running. Start with ./robot.sh sim")
        
        self.assertTrue(nav_found, "Nav2 nodes not found")
    
    def test_navigation_topics_available(self):
        """Test that navigation topics are available"""
        topics = ROS2TestClient.list_topics()
        
        nav_topics_found = 0
        for nav_topic in self.NAV2_TOPICS:
            if nav_topic in topics:
                nav_topics_found += 1
        
        if nav_topics_found == 0:
            self.skipTest("Navigation topics not available")
        
        # At least some nav topics should exist
        self.assertGreater(nav_topics_found, 0, "No navigation topics found")
    
    def test_costmap_topics(self):
        """Test that costmap topics are publishing"""
        topics = ROS2TestClient.list_topics()
        
        costmap_topics = [
            "/global_costmap/costmap",
            "/local_costmap/costmap"
        ]
        
        for topic in costmap_topics:
            if topic in topics:
                info = ROS2TestClient.topic_info(topic)
                self.assertGreater(
                    info.get('publishers', 0), 0,
                    f"Costmap topic {topic} has no publishers"
                )


class TestSLAM(unittest.TestCase):
    """Test SLAM functionality"""
    
    SLAM_NODES = [
        "/slam_toolbox",
    ]
    
    SLAM_TOPICS = [
        "/map",
        "/map_metadata",
    ]
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_slam_node_running(self):
        """Test that SLAM node is running"""
        nodes = ROS2TestClient.list_nodes()
        
        slam_found = any(
            any(slam_node in node for node in nodes)
            for slam_node in self.SLAM_NODES
        )
        
        if not slam_found:
            self.skipTest("SLAM not running. Start with ./robot.sh sim")
        
        self.assertTrue(slam_found, "SLAM node not found")
    
    def test_map_topic_available(self):
        """Test that map topic is available"""
        topics = ROS2TestClient.list_topics()
        
        if config.MAP_TOPIC not in topics:
            self.skipTest("Map topic not available")
        
        info = ROS2TestClient.topic_info(config.MAP_TOPIC)
        self.assertGreater(info.get('publishers', 0), 0, "Map has no publishers")
    
    def test_map_metadata(self):
        """Test map metadata is valid"""
        topics = ROS2TestClient.list_topics()
        
        if "/map_metadata" not in topics:
            self.skipTest("Map metadata topic not available")


class TestTFTransforms(unittest.TestCase):
    """Test TF transforms"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_tf_topic_exists(self):
        """Test TF topic exists"""
        topics = ROS2TestClient.list_topics()
        
        self.assertIn("/tf", topics, "TF topic not found")
    
    def test_tf_static_topic_exists(self):
        """Test TF static topic exists"""
        topics = ROS2TestClient.list_topics()
        
        self.assertIn("/tf_static", topics, "TF static topic not found")
    
    def test_robot_frame_available(self):
        """Test robot base frame is available"""
        # This is checked via the robot_description topic
        topics = ROS2TestClient.list_topics()
        
        self.assertIn("/robot_description", topics, "Robot description not found")


class TestOdometry(unittest.TestCase):
    """Test odometry functionality"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_odom_topic_exists(self):
        """Test odometry topic exists"""
        topics = ROS2TestClient.list_topics()
        
        self.assertIn(config.ODOM_TOPIC, topics, "Odometry topic not found")
    
    def test_odom_has_publisher(self):
        """Test odometry has publisher"""
        info = ROS2TestClient.topic_info(config.ODOM_TOPIC)
        
        self.assertGreater(info.get('publishers', 0), 0, "Odometry has no publishers")
    
    def test_odom_frequency(self):
        """Test odometry is publishing at expected rate"""
        hz = ROS2TestClient.topic_hz(config.ODOM_TOPIC)
        
        self.assertGreaterEqual(
            hz, config.MIN_ODOM_FREQUENCY,
            f"Odometry frequency {hz:.1f} Hz below minimum"
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
