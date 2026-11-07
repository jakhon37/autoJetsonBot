import os
import http.server
import threading
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node


class TimedHandler(http.server.SimpleHTTPRequestHandler):
    """Static file handler with short timeouts so a stuck browser tab cannot block the UI."""

    timeout = 5
    protocol_version = 'HTTP/1.1'


class ThreadedHTTPServer(http.server.ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True
    request_queue_size = 64


class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')

        self.declare_parameter('port', 8000)
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.httpd = None

        pkg_share = get_package_share_directory('jetson_bot_gui')
        self.web_dir = os.path.join(pkg_share, 'web')

        if not os.path.exists(self.web_dir):
            self.get_logger().error(f"Web directory not found: {self.web_dir}")
            return

        self.get_logger().info(f"Serving web files from: {self.web_dir} on port {self.port}")

        self.thread = threading.Thread(target=self.serve_forever, name='web_http', daemon=True)
        self.thread.start()

    def serve_forever(self):
        handler = lambda *args, **kwargs: TimedHandler(
            *args, directory=self.web_dir, **kwargs)

        try:
            self.httpd = ThreadedHTTPServer(('0.0.0.0', self.port), handler)
            self.httpd.timeout = 1.0
            self.get_logger().info(f"Web server started on port {self.port}")
            self.httpd.serve_forever(poll_interval=0.5)
        except Exception as exc:
            self.get_logger().error(f"Web server failed: {exc}")

    def destroy_node(self):
        if self.httpd is not None:
            try:
                self.httpd.shutdown()
                self.httpd.server_close()
            except Exception:
                pass
        super().destroy_node()

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
