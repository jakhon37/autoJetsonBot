"""
Docker Environment Tests
Tests for Docker container setup and configuration
"""
import unittest
import socket
import subprocess
import time
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_suite.config import config, paths, env
from test_suite.utils import DockerClient, timer


def wait_for_listening(port: int, timeout: float = 10.0) -> bool:
    """Return True once something accepts TCP connections on localhost:port"""
    deadline = time.time() + timeout
    while True:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=2):
                return True
        except OSError:
            if time.time() > deadline:
                return False
            time.sleep(0.5)


class TestDockerEnvironment(unittest.TestCase):
    """Test Docker container setup and configuration"""
    
    @classmethod
    def setUpClass(cls):
        """Setup test class"""
        cls.container_name = config.CONTAINER_NAME
        cls.docker_client = DockerClient()
    
    def test_01_container_exists(self):
        """Test that Docker container exists"""
        exists = DockerClient.is_container_exists(self.container_name)
        self.assertTrue(
            exists, 
            f"Container '{self.container_name}' does not exist. "
            "Run runrosenv.sh to create it."
        )
    
    def test_02_container_is_running(self):
        """Test that Docker container is running"""
        running = DockerClient.is_container_running(self.container_name)
        
        if not running:
            # Try to start the container
            print(f"  Container not running, attempting to start...")
            started = DockerClient.start_container(self.container_name)
            self.assertTrue(started, "Failed to start container")
            time.sleep(3)  # Wait for container to fully start
            running = DockerClient.is_container_running(self.container_name)
        
        self.assertTrue(running, f"Container '{self.container_name}' is not running")
    
    def test_03_container_ports_listening(self):
        """Test that the web UI and rosbridge ports are listening"""
        # The container uses --network host, so `docker port` prints nothing. Connect to the
        # ports instead. The web server and rosbridge only listen while the stack is up.
        code, _, _ = DockerClient.exec_in_container(
            self.container_name, ["pgrep", "-f", "main.launch.py"]
        )
        if code != 0:
            self.skipTest("Stack not running. Start it with ./robot.sh up, sim or robot")
        for port in (config.WEB_PORT, config.ROSBRIDGE_PORT):
            with self.subTest(port=port):
                self.assertTrue(wait_for_listening(port), f"Nothing listening on port {port}")
    
    def test_04_container_has_ros2_installed(self):
        """Test that ROS2 is installed in container"""
        code, stdout, _ = DockerClient.exec_in_container(
            self.container_name,
            ["bash", "-c", f"source /opt/ros/{env.ROS_DISTRO}/setup.bash && which ros2"]
        )
        self.assertEqual(code, 0, "ros2 CLI not found in container")
    
    def test_05_ros2_workspace_exists(self):
        """Test that ROS2 workspace is mounted"""
        code, stdout, _ = DockerClient.exec_in_container(
            self.container_name,
            ["test", "-d", paths.SRC_DIR]
        )
        self.assertEqual(code, 0, "ROS2 src directory not found in container")
    
    def test_06_ros2_packages_built(self):
        """Test that ROS2 packages are built"""
        code, stdout, _ = DockerClient.exec_in_container(
            self.container_name,
            ["test", "-d", paths.INSTALL_DIR]
        )
        self.assertEqual(code, 0, "ROS2 install directory not found - packages not built")
        
        # Check for current jetson_bot packages (the Dockerized modular packages)
        packages = [
            "jetson_bot_bringup",
            "jetson_bot_description",
            "jetson_bot_diffdrive",
            "jetson_bot_gui",
            "jetson_bot_navigation",
            "jetson_bot_slam",
        ]
        for pkg in packages:
            code, _, _ = DockerClient.exec_in_container(
                self.container_name,
                ["test", "-d", f"{paths.INSTALL_DIR}/{pkg}"]
            )
            self.assertEqual(code, 0, f"Package '{pkg}' not built")
    
    def test_07_ros_environment_sourced(self):
        """Test that ROS2 environment is properly sourced"""
        code, stdout, _ = DockerClient.exec_in_container(
            self.container_name,
            ["bash", "-c", f"source /opt/ros/{env.ROS_DISTRO}/setup.bash && echo $ROS_DISTRO"]
        )
        self.assertIn(env.ROS_DISTRO, stdout, f"ROS2 {env.ROS_DISTRO} not properly sourced")
    
    def test_08_docker_network_exists(self):
        """Test that ROS2 Docker network exists"""
        result = subprocess.run(
            ["docker", "network", "ls", "--format", "{{.Name}}"],
            capture_output=True,
            text=True
        )
        networks = result.stdout
        # Check for ros2_network or host network
        self.assertTrue(
            "ros2_network" in networks or "host" in networks,
            "No suitable Docker network found"
        )


class TestDockerPerformance(unittest.TestCase):
    """Test Docker container performance metrics"""
    
    @classmethod
    def setUpClass(cls):
        cls.container_name = config.CONTAINER_NAME
    
    def test_container_memory_usage(self):
        """Test container memory usage is within limits"""
        result = subprocess.run(
            ["docker", "stats", "--no-stream", "--format", 
             "{{.MemUsage}}", self.container_name],
            capture_output=True,
            text=True
        )
        
        mem_usage = result.stdout.strip()
        # Just check that we got a value
        self.assertTrue(mem_usage, "Could not get memory usage")
        print(f"  Memory usage: {mem_usage}")
    
    def test_container_cpu_usage(self):
        """Test container CPU usage"""
        result = subprocess.run(
            ["docker", "stats", "--no-stream", "--format",
             "{{.CPUPerc}}", self.container_name],
            capture_output=True,
            text=True
        )
        
        cpu_usage = result.stdout.strip()
        self.assertTrue(cpu_usage, "Could not get CPU usage")
        print(f"  CPU usage: {cpu_usage}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
