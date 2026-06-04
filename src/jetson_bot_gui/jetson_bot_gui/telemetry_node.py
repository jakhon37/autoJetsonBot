#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import psutil
import time
import random

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        
        # In Foxy, use_sim_time might be declared by the base Node class if passed via ros-args
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        
        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        
        # Timer for publishing (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Simulated battery state
        self.battery_level = 100.0
        self.is_sim = True # Could be a parameter
        
        self.get_logger().info("Telemetry Node started")

    def timer_callback(self):
        # 1. Battery State
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.is_sim:
            # Simulate slight drain
            self.battery_level -= 0.01
            if self.battery_level < 20.0:
                self.battery_level = 100.0 # Reset for sim
        
        msg.percentage = self.battery_level / 100.0
        msg.voltage = 12.0 * (self.battery_level / 100.0) # Dummy voltage
        msg.design_capacity = 5000.0 # mAh
        
        self.battery_pub.publish(msg)
        
        # 2. System Stats (could be published to diagnostics or a custom topic)
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        
        # We'll log it for now, but in a real app we might publish a DiagnosticArray
        self.get_logger().debug(f"CPU: {cpu}%, Mem: {mem}%, Battery: {self.battery_level:.1f}%")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
