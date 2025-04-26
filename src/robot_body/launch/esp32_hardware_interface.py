#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hardware_interface import SystemInterface
from hardware_interface.hardware_info import HardwareInfo
import serial
import time
import threading
import math
from typing import List, Dict

class ESP32HardwareInterface(SystemInterface):
    def __init__(self):
        super().__init__()
        self._serial_port = None
        self._baud_rate = None
        self._serial_conn = None
        self._logger = rclpy.logging.get_logger('ESP32HardwareInterface')
        
        # State and command storage
        self._hw_positions = []
        self._hw_velocities = []
        self._hw_commands = []
        
        # For position integration
        self._last_time = time.time()
        
        # Thread lock for serial communication
        self._lock = threading.Lock()

    def on_init(self, hardware_info: HardwareInfo):
        if super().on_init(hardware_info) != SystemInterface.CallbackReturn.SUCCESS:
            return SystemInterface.CallbackReturn.ERROR
        
        # Get parameters from URDF
        self._serial_port = hardware_info.hardware_parameters.get("serial_port", "/dev/ttyUSB0")
        self._baud_rate = int(hardware_info.hardware_parameters.get("baud_rate", "115200"))
        
        # Initialize storage vectors
        self._hw_positions = [0.0] * len(hardware_info.joints)
        self._hw_velocities = [0.0] * len(hardware_info.joints)
        self._hw_commands = [0.0] * len(hardware_info.joints)
        
        self._logger.info(f"Initialized with serial port: {self._serial_port}, baud rate: {self._baud_rate}")
        
        return SystemInterface.CallbackReturn.SUCCESS

    def on_configure(self, previous_state):
        # Open serial connection to ESP32
        try:
            self._serial_conn = serial.Serial(
                port=self._serial_port,
                baudrate=self._baud_rate,
                timeout=1.0
            )
            
            # Clear any pending data
            self._serial_conn.reset_input_buffer()
            self._serial_conn.reset_output_buffer()
            
            self._logger.info(f"Serial connection established on {self._serial_port} at {self._baud_rate} baud")
            
            # Let the ESP32 boot up
            time.sleep(2.0)
            
        except serial.SerialException as e:
            self._logger.error(f"Failed to open serial port: {e}")
            return SystemInterface.CallbackReturn.ERROR
        
        return SystemInterface.CallbackReturn.SUCCESS

    def on_cleanup(self, previous_state):
        # Close the serial connection
        if self._serial_conn and self._serial_conn.is_open:
            self._serial_conn.close()
            self._logger.info("Serial connection closed")
            
        return SystemInterface.CallbackReturn.SUCCESS

    def on_activate(self, previous_state):
        # Set motors to stop on activation
        if self._serial_conn and self._serial_conn.is_open:
            self._send_command_to_esp32(0.0, 0.0)
        
        # Reset command and state values
        for i in range(len(self._hw_commands)):
            self._hw_commands[i] = 0.0
            self._hw_positions[i] = 0.0
            self._hw_velocities[i] = 0.0
        
        self._logger.info("Hardware interface activated")
        return SystemInterface.CallbackReturn.SUCCESS

    def on_deactivate(self, previous_state):
        # Stop motors on deactivation
        if self._serial_conn and self._serial_conn.is_open:
            self._send_command_to_esp32(0.0, 0.0)
        
        self._logger.info("Hardware interface deactivated")
        return SystemInterface.CallbackReturn.SUCCESS

    def export_state_interfaces(self):
        from hardware_interface import StateInterface
        
        state_interfaces = []
        
        # Export position and velocity state interfaces for each joint
        for i, joint in enumerate(self.info.joints):
            # Position interface
            state_interfaces.append(
                StateInterface(
                    joint.name,
                    "position",
                    self._hw_positions, i
                )
            )
            
            # Velocity interface
            state_interfaces.append(
                StateInterface(
                    joint.name,
                    "velocity",
                    self._hw_velocities, i
                )
            )
        
        return state_interfaces

    def export_command_interfaces(self):
        from hardware_interface import CommandInterface
        
        command_interfaces = []
        
        # Export velocity command interfaces for each joint
        for i, joint in enumerate(self.info.joints):
            command_interfaces.append(
                CommandInterface(
                    joint.name,
                    "velocity",
                    self._hw_commands, i
                )
            )
        
        return command_interfaces

    def read(self, time, period):
        # Read RPM values from ESP32
        if not self._read_rpm_from_esp32():
            return SystemInterface.ReturnType.ERROR
        
        return SystemInterface.ReturnType.OK

    def write(self, time, period):
        # Convert joint commands to linear and angular velocity
        # For differential drive: linear = (right + left)/2, angular = (right - left)/2
        right_command = self._hw_commands[0]  # right_wheel_joint
        left_command = self._hw_commands[1]   # left_wheel_joint
        
        # Assuming standard differential drive kinematics
        # You may need to adjust this based on your robot's configuration
        linear = (right_command + left_command) / 2.0
        angular = (right_command - left_command) / 2.0
        
        # Send commands to ESP32
        if not self._send_command_to_esp32(linear, angular):
            return SystemInterface.ReturnType.ERROR
        
        return SystemInterface.ReturnType.OK

    def _read_rpm_from_esp32(self):
        """Read RPM values from ESP32 over serial"""
        if not self._serial_conn or not self._serial_conn.is_open:
            return False
        
        with self._lock:
            # Clear any old data
            self._serial_conn.reset_input_buffer()
            
            # Read from serial until we get RPM data
            got_rpm = False
            start_time = time.time()
            
            while not got_rpm and (time.time() - start_time) < 0.5:  # 500ms timeout
                if self._serial_conn.in_waiting:
                    try:
                        response = self._serial_conn.readline().decode('utf-8').strip()
                        
                        # Check if it's an RPM message
                        if response.startswith("RPM:"):
                            # Parse RPM values (format: "RPM:<left>,<right>")
                            values = response[4:]
                            comma_pos = values.find(',')
                            
                            if comma_pos != -1:
                                left_rpm = float(values[:comma_pos])
                                right_rpm = float(values[comma_pos+1:])
                                
                                # Convert RPM to rad/s
                                # rad/s = RPM * 2π/60
                                RPM_TO_RAD_S = 0.10472  # (2 * math.pi) / 60.0
                                self._hw_velocities[1] = left_rpm * RPM_TO_RAD_S   # left_wheel_joint
                                self._hw_velocities[0] = right_rpm * RPM_TO_RAD_S  # right_wheel_joint
                                
                                # Integrate position
                                current_time = time.time()
                                dt = current_time - self._last_time
                                self._last_time = current_time
                                
                                self._hw_positions[0] += self._hw_velocities[0] * dt
                                self._hw_positions[1] += self._hw_velocities[1] * dt
                                
                                got_rpm = True
                                
                    except (UnicodeDecodeError, ValueError) as e:
                        self._logger.warning(f"Failed to parse serial data: {e}")
                
                else:
                    # Wait a bit for more data
                    time.sleep(0.01)
        
        return got_rpm

    def _send_command_to_esp32(self, linear, angular):
        """Send velocity commands to ESP32 over serial"""
        if not self._serial_conn or not self._serial_conn.is_open:
            return False
        
        with self._lock:
            # Format command string - match the format expected by your ESP32 code
            # Format: "CMD:<linear>,<angular>"
            cmd = f"CMD:{linear},{angular}\n"
            
            # Send the command
            try:
                bytes_written = self._serial_conn.write(cmd.encode('utf-8'))
                self._serial_conn.flush()
                return bytes_written == len(cmd)
            except serial.SerialException as e:
                self._logger.error(f"Failed to send command: {e}")
                return False


# Entry point for the hardware interface plugin
def get_hardware_interfaces():
    return [ESP32HardwareInterface()]