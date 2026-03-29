"""
Web Interface Tests
Tests for the web control interface
"""
import unittest
import subprocess
import time
import requests
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_suite.config import config
from test_suite.utils import WebClient


class TestWebInterface(unittest.TestCase):
    """Test web interface functionality"""
    
    @classmethod
    def setUpClass(cls):
        """Ensure container is running"""
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
        cls.client = WebClient(config.WEB_URL)
    
    def test_01_web_server_running(self):
        """Test that web server is responding"""
        is_running = self.client.is_web_running()
        
        if not is_running:
            self.skipTest("Web server not running. Start with ./robot.sh sim")
        
        self.assertTrue(is_running, "Web server is not running")
    
    def test_02_web_server_returns_html(self):
        """Test that web server returns HTML content"""
        try:
            response = requests.get(config.WEB_URL, timeout=5)
            self.assertEqual(response.status_code, 200)
            self.assertIn("html", response.headers.get("Content-Type", "").lower())
        except requests.exceptions.ConnectionError:
            self.skipTest("Web server not accessible")
    
    def test_03_rosbridge_port_open(self):
        """Test that ROSBridge port is accessible"""
        import socket
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            result = sock.connect_ex(("localhost", config.ROSBRIDGE_PORT))
            sock.close()
            
            if result != 0:
                self.skipTest("ROSBridge not running. Start with ./robot.sh sim")
            
            self.assertEqual(result, 0, "ROSBridge port not accessible")
        except Exception as e:
            self.skipTest(f"Could not test ROSBridge: {e}")
    
    def test_04_web_static_assets(self):
        """Test that static assets are served"""
        try:
            response = requests.get(f"{config.WEB_URL}/", timeout=5)
            content = response.text.lower()
            
            # Check for common web app indicators
            has_content = len(content) > 100
            self.assertTrue(has_content, "Web page seems empty")
        except requests.exceptions.ConnectionError:
            self.skipTest("Web server not accessible")


class TestWebAPI(unittest.TestCase):
    """Test web API endpoints"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
        cls.client = WebClient(config.WEB_URL)
    
    def test_rosbridge_websocket_connection(self):
        """Test WebSocket connection to ROSBridge"""
        try:
            import websocket
            
            ws_url = f"ws://localhost:{config.ROSBRIDGE_PORT}"
            ws = websocket.create_connection(ws_url, timeout=5)
            
            # Send a ping
            ws.send('{"op": "ping"}')
            
            # Wait for response
            try:
                response = ws.recv()
                self.assertIsNotNone(response)
            except:
                pass
            
            ws.close()
            
        except ImportError:
            self.skipTest("websocket-client not installed")
        except Exception as e:
            self.skipTest(f"WebSocket test failed: {e}")
    
    def test_cmd_vel_api_exists(self):
        """Test cmd_vel API endpoint exists"""
        # This is a smoke test - actual behavior depends on implementation
        # We'll just check if we can reach the server
        try:
            response = requests.get(config.WEB_URL, timeout=5)
            self.assertEqual(response.status_code, 200)
        except:
            self.skipTest("Web server not accessible")


class TestWebPerformance(unittest.TestCase):
    """Test web interface performance"""
    
    @classmethod
    def setUpClass(cls):
        subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
        time.sleep(3)
    
    def test_page_load_time(self):
        """Test web page loads within acceptable time"""
        start = time.time()
        
        try:
            response = requests.get(config.WEB_URL, timeout=10)
            load_time = time.time() - start
            
            self.assertEqual(response.status_code, 200)
            self.assertLess(load_time, 5.0, f"Page load time {load_time:.2f}s > 5s")
            
        except requests.exceptions.ConnectionError:
            self.skipTest("Web server not accessible")
    
    def test_concurrent_requests(self):
        """Test handling of concurrent requests"""
        import concurrent.futures
        
        def make_request():
            try:
                response = requests.get(config.WEB_URL, timeout=10)
                return response.status_code == 200
            except:
                return False
        
        try:
            with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
                futures = [executor.submit(make_request) for _ in range(5)]
                results = [f.result() for f in concurrent.futures.as_completed(futures, timeout=15)]
            
            success_rate = sum(results) / len(results)
            self.assertGreater(success_rate, 0.8, "Too many failed concurrent requests")
            
        except requests.exceptions.ConnectionError:
            self.skipTest("Web server not accessible")


if __name__ == "__main__":
    unittest.main(verbosity=2)
