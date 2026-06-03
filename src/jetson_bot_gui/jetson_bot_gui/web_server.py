import os
import http.server
import socketserver
import threading
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        
        self.declare_parameter('port', 8000)
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Get the path to the web files
        pkg_share = get_package_share_directory('jetson_bot_gui')
        self.web_dir = os.path.join(pkg_share, 'web')
        
        if not os.path.exists(self.web_dir):
            self.get_logger().error(f"Web directory not found: {self.web_dir}")
            return

        self.get_logger().info(f"Serving web files from: {self.web_dir} on port {self.port}")
        
        # Start the server in a separate thread
        self.thread = threading.Thread(target=self.serve_forever)
        self.thread.daemon = True
        self.thread.start()

    def serve_forever(self):
        os.chdir(self.web_dir)
        Handler = http.server.SimpleHTTPRequestHandler
        
        # Allow reusing the port immediately
        socketserver.TCPServer.allow_reuse_address = True
        
        with socketserver.TCPServer(("", self.port), Handler) as httpd:
            self.get_logger().info(f"Web server started on port {self.port}")
            httpd.serve_forever()

def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
