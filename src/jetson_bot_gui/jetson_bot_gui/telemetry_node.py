#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import psutil

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        # In Foxy, use_sim_time might be declared by the base Node class if passed via ros-args
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.system_pub  = self.create_publisher(String, '/telemetry/system', 10)

        # Timer for publishing (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Simulated battery state
        self.battery_level = 100.0
        self.is_sim = True  # Could be a parameter

        self.get_logger().info("Telemetry Node started")

    # ------------------------------------------------------------------
    def _read_temperature(self):
        """Return CPU temperature in °C, or None if unavailable."""
        try:
            temps = psutil.sensors_temperatures()
            if not temps:
                return None
            # Jetson exposes 'thermal-fan-est' or 'CPU-therm'; try common keys
            for key in ('thermal-fan-est', 'cpu-thermal', 'cpu_thermal',
                        'coretemp', 'k10temp', 'acpitz'):
                if key in temps and temps[key]:
                    return round(temps[key][0].current, 1)
            # Fallback: first available sensor
            first = next(iter(temps.values()))
            if first:
                return round(first[0].current, 1)
        except (AttributeError, StopIteration):
            pass
        return None

    # ------------------------------------------------------------------
    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # 1. Battery State
        msg = BatteryState()
        msg.header.stamp = now

        if self.is_sim:
            self.battery_level -= 0.01
            if self.battery_level < 20.0:
                self.battery_level = 100.0  # Reset for sim

        msg.percentage      = self.battery_level / 100.0
        msg.voltage         = 12.0 * (self.battery_level / 100.0)
        msg.design_capacity = 5000.0  # mAh
        self.battery_pub.publish(msg)

        # 2. System Stats — published as JSON string on /telemetry/system
        cpu  = psutil.cpu_percent()
        mem  = psutil.virtual_memory().percent
        temp = self._read_temperature()  # None when unavailable

        payload = json.dumps({
            'cpu':         round(cpu, 1),
            'memory':      round(mem, 1),
            'temperature': temp,          # float or null
            'battery':     round(self.battery_level, 1),
        })
        self.system_pub.publish(String(data=payload))

        self.get_logger().debug(
            f"CPU: {cpu}%  Mem: {mem}%  Temp: {temp}°C  Bat: {self.battery_level:.1f}%"
        )

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
