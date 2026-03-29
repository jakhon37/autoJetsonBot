"""
Test utilities and helpers for ROS2 robot testing
"""
import subprocess
import time
import json
from typing import Optional, List, Dict, Any, Tuple
from dataclasses import dataclass
from contextlib import contextmanager

# Optional imports - handle gracefully if not available
try:
    import requests
except ImportError:
    requests = None

try:
    import websocket
except ImportError:
    websocket = None

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    rclpy = None
    Node = object


class ROS2TestClient:
    """ROS2 client for testing topics and services"""
    
    def __init__(self, node_name: str = "test_client"):
        self.node = None
        self.node_name = node_name
        self.subscriptions = {}
        self.messages = {}
        
    def __enter__(self):
        rclpy.init()
        self.node = rclpy.node.Node(self.node_name)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()
    
    def subscribe(self, topic: str, msg_type, callback=None):
        """Subscribe to a topic"""
        if self.node:
            self.subscriptions[topic] = self.node.create_subscription(
                msg_type, topic, callback or self._default_callback(topic)
            )
    
    def _default_callback(self, topic: str):
        def callback(msg):
            self.messages[topic] = msg
        return callback
    
    def spin_once(self, timeout: float = 0.1):
        """Spin the node once"""
        if self.node:
            rclpy.spin_once(self.node, timeout_sec=timeout)
    
    def get_messages(self, topic: str) -> Any:
        """Get latest message from topic"""
        return self.messages.get(topic)
    
    @staticmethod
    def run_cli(command: List[str], timeout: int = 30) -> Tuple[int, str, str]:
        """Run a ROS2 CLI command"""
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            return result.returncode, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return -1, "", "Command timed out"
    
    @staticmethod
    def topic_hz(topic: str) -> float:
        """Get topic publish frequency in Hz"""
        code, stdout, _ = ROS2TestClient.run_cli(
            ["ros2", "topic", "hz", topic], 
            timeout=5
        )
        if code == 0:
            try:
                lines = stdout.split('\n')
                for line in lines:
                    if 'average rate:' in line:
                        rate = float(line.split('average rate:')[1].split()[0])
                        return rate
            except Exception:
                pass
        return 0.0
    
    @staticmethod
    def topic_info(topic: str) -> Dict[str, Any]:
        """Get topic information"""
        code, stdout, _ = ROS2TestClient.run_cli(
            ["ros2", "topic", "info", topic],
            timeout=5
        )
        if code == 0:
            info = {'topic': topic, 'publishers': 0, 'subscribers': 0}
            for line in stdout.split('\n'):
                if 'Publisher count:' in line:
                    info['publishers'] = int(line.split(':')[1].strip())
                elif 'Subscription count:' in line:
                    info['subscribers'] = int(line.split(':')[1].strip())
            return info
        return {}
    
    @staticmethod
    def list_topics() -> List[str]:
        """List all available topics"""
        code, stdout, _ = ROS2TestClient.run_cli(["ros2", "topic", "list"])
        if code == 0:
            return [t.strip() for t in stdout.split('\n') if t.strip()]
        return []
    
    @staticmethod
    def list_nodes() -> List[str]:
        """List all active nodes"""
        code, stdout, _ = ROS2TestClient.run_cli(["ros2", "node", "list"])
        if code == 0:
            return [n.strip() for n in stdout.split('\n') if n.strip()]
        return []
    
    @staticmethod
    def get_param(param_name: str, default: Any = None) -> Any:
        """Get a parameter value"""
        code, stdout, _ = ROS2TestClient.run_cli(
            ["ros2", "param", "get", "/", param_name],
            timeout=5
        )
        if code == 0 and stdout.strip():
            return stdout.strip()
        return default


class WebClient:
    """HTTP and WebSocket client for web interface testing"""
    
    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url
        self.ws = None
        
    def is_web_running(self) -> bool:
        """Check if web interface is running"""
        try:
            response = requests.get(self.base_url, timeout=2)
            return response.status_code == 200
        except Exception:
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """Get web interface status"""
        try:
            response = requests.get(f"{self.base_url}/api/status", timeout=2)
            if response.status_code == 200:
                return response.json()
        except Exception:
            pass
        return {}
    
    def send_cmd_vel(self, linear: float = 0.0, angular: float = 0.0) -> bool:
        """Send cmd_vel via web API"""
        try:
            response = requests.post(
                f"{self.base_url}/api/cmd_vel",
                json={"linear": linear, "angular": angular},
                timeout=2
            )
            return response.status_code == 200
        except Exception:
            return False
    
    def connect_rosbridge(self, rosbridge_url: str = "ws://localhost:9090") -> bool:
        """Test WebSocket connection"""
        try:
            self.ws = websocket.create_connection(rosbridge_url, timeout=5)
            return True
        except Exception:
            return False
    
    def close(self):
        """Close WebSocket connection"""
        if self.ws:
            self.ws.close()


class DockerClient:
    """Docker container management for testing"""
    
    @staticmethod
    def is_container_running(container_name: str) -> bool:
        """Check if container is running"""
        result = subprocess.run(
            ["docker", "ps", "--filter", f"name={container_name}", "--format", "{{.Names}}"],
            capture_output=True,
            text=True
        )
        return container_name in result.stdout
    
    @staticmethod
    def is_container_exists(container_name: str) -> bool:
        """Check if container exists"""
        result = subprocess.run(
            ["docker", "ps", "-a", "--filter", f"name={container_name}", "--format", "{{.Names}}"],
            capture_output=True,
            text=True
        )
        return container_name in result.stdout
    
    @staticmethod
    def start_container(container_name: str) -> bool:
        """Start a container"""
        result = subprocess.run(
            ["docker", "start", container_name],
            capture_output=True
        )
        return result.returncode == 0
    
    @staticmethod
    def exec_in_container(container_name: str, command: List[str]) -> Tuple[int, str, str]:
        """Execute command in container"""
        cmd = ["docker", "exec", container_name] + command
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode, result.stdout, result.stderr
    
    @staticmethod
    def get_container_logs(container_name: str, tail: int = 50) -> str:
        """Get container logs"""
        result = subprocess.run(
            ["docker", "logs", "--tail", str(tail), container_name],
            capture_output=True,
            text=True
        )
        return result.stdout + result.stderr


@contextmanager
def timer(description: str = "Operation"):
    """Context manager for timing operations"""
    start = time.time()
    yield
    elapsed = time.time() - start
    print(f"  {description}: {elapsed:.2f}s")


def assert_ros_topic(topic: str, timeout: int = 10) -> bool:
    """Assert a ROS topic exists and has publishers"""
    time.sleep(1)  # Wait for topics to stabilize
    topics = ROS2TestClient.list_topics()
    if topic not in topics:
        return False
    info = ROS2TestClient.topic_info(topic)
    return info.get('publishers', 0) > 0


def assert_ros_node(node: str, timeout: int = 10) -> bool:
    """Assert a ROS node is running"""
    time.sleep(1)
    nodes = ROS2TestClient.list_nodes()
    return node in nodes
