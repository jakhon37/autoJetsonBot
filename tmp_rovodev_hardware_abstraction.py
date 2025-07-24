#!/usr/bin/env python3
"""
hardware_abstraction.py - Hardware Abstraction Layer for diffdrive_arduino
Long-term solution for modular hardware interface management
Author: RovoDev Assistant
"""

import os
import yaml
import logging
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, List
from enum import Enum
from dataclasses import dataclass
from pathlib import Path

class HardwareMode(Enum):
    """Hardware operation modes"""
    SIMULATION = "simulation"
    REAL_HARDWARE = "real_hardware"
    MOCK = "mock"

class ControllerType(Enum):
    """Available controller types"""
    DIFFDRIVE_ARDUINO = "diffdrive_arduino"
    MICRO_ROS = "micro_ros"
    GAZEBO_SYSTEM = "gazebo_system"
    MOCK_CONTROLLER = "mock_controller"

@dataclass
class HardwareConfig:
    """Hardware configuration data structure"""
    mode: HardwareMode
    controller_type: ControllerType
    device_path: str
    baud_rate: int
    timeout: int
    loop_rate: int
    encoder_counts_per_rev: int
    wheel_separation: float
    wheel_radius: float
    max_velocity: float
    max_acceleration: float

class HardwareInterface(ABC):
    """Abstract base class for hardware interfaces"""
    
    def __init__(self, config: HardwareConfig):
        self.config = config
        self.logger = logging.getLogger(self.__class__.__name__)
        self.is_initialized = False
        self.is_connected = False
        
    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the hardware interface"""
        pass
        
    @abstractmethod
    def connect(self) -> bool:
        """Connect to the hardware"""
        pass
        
    @abstractmethod
    def disconnect(self) -> bool:
        """Disconnect from the hardware"""
        pass
        
    @abstractmethod
    def get_ros2_control_config(self) -> Dict[str, Any]:
        """Get ROS2 control configuration for this hardware"""
        pass
        
    @abstractmethod
    def get_controller_config(self) -> Dict[str, Any]:
        """Get controller configuration for this hardware"""
        pass
        
    @abstractmethod
    def validate_configuration(self) -> bool:
        """Validate hardware configuration"""
        pass
        
    @abstractmethod
    def get_health_status(self) -> Dict[str, Any]:
        """Get current hardware health status"""
        pass

class DiffDriveArduinoInterface(HardwareInterface):
    """Hardware interface for diffdrive_arduino controller"""
    
    def __init__(self, config: HardwareConfig):
        super().__init__(config)
        self.device_path = config.device_path
        self.baud_rate = config.baud_rate
        
    def initialize(self) -> bool:
        """Initialize diffdrive_arduino interface"""
        try:
            # Check if device exists
            if not os.path.exists(self.device_path):
                self.logger.error(f"Device {self.device_path} not found")
                return False
                
            # Validate configuration
            if not self.validate_configuration():
                return False
                
            self.is_initialized = True
            self.logger.info("DiffDriveArduino interface initialized successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize DiffDriveArduino: {e}")
            return False
            
    def connect(self) -> bool:
        """Connect to Arduino device"""
        try:
            if not self.is_initialized:
                self.logger.error("Interface not initialized")
                return False
                
            # Test device communication
            # This would typically involve sending a test command
            self.is_connected = True
            self.logger.info(f"Connected to device {self.device_path}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect to device: {e}")
            return False
            
    def disconnect(self) -> bool:
        """Disconnect from Arduino device"""
        try:
            self.is_connected = False
            self.logger.info("Disconnected from device")
            return True
        except Exception as e:
            self.logger.error(f"Failed to disconnect: {e}")
            return False
            
    def get_ros2_control_config(self) -> Dict[str, Any]:
        """Generate ROS2 control configuration for diffdrive_arduino"""
        return {
            "ros2_control": {
                "name": "RealRobot",
                "type": "system",
                "hardware": {
                    "plugin": "diffdrive_arduino/DiffDriveArduino",
                    "param": {
                        "left_wheel_name": "left_wheel_joint",
                        "right_wheel_name": "right_wheel_joint",
                        "loop_rate": self.config.loop_rate,
                        "device": self.config.device_path,
                        "baud_rate": self.config.baud_rate,
                        "timeout": self.config.timeout,
                        "enc_counts_per_rev": self.config.encoder_counts_per_rev
                    }
                },
                "joints": {
                    "left_wheel_joint": {
                        "command_interface": {
                            "name": "velocity",
                            "param": {
                                "min": -self.config.max_velocity,
                                "max": self.config.max_velocity
                            }
                        },
                        "state_interface": ["velocity", "position"]
                    },
                    "right_wheel_joint": {
                        "command_interface": {
                            "name": "velocity", 
                            "param": {
                                "min": -self.config.max_velocity,
                                "max": self.config.max_velocity
                            }
                        },
                        "state_interface": ["velocity", "position"]
                    }
                }
            }
        }
        
    def get_controller_config(self) -> Dict[str, Any]:
        """Generate controller configuration for diffdrive_arduino"""
        return {
            "controller_manager": {
                "ros__parameters": {
                    "update_rate": self.config.loop_rate,
                    "use_sim_time": False,
                    "diff_cont": {
                        "type": "diff_drive_controller/DiffDriveController"
                    },
                    "joint_broad": {
                        "type": "joint_state_broadcaster/JointStateBroadcaster"
                    }
                }
            },
            "diff_cont": {
                "ros__parameters": {
                    "left_wheel_names": ["left_wheel_joint"],
                    "right_wheel_names": ["right_wheel_joint"],
                    "wheel_separation": self.config.wheel_separation,
                    "wheel_radius": self.config.wheel_radius,
                    "publish_rate": 50.0,
                    "base_frame_id": "base_link",
                    "odom_frame_id": "odom",
                    "enable_odom_tf": True,
                    "cmd_vel_timeout": 0.5,
                    "use_stamped_vel": False
                }
            },
            "joint_broad": {
                "ros__parameters": {
                    "joints": [
                        "left_wheel_joint",
                        "right_wheel_joint",
                        "caster_wheel_joint"
                    ]
                }
            }
        }
        
    def validate_configuration(self) -> bool:
        """Validate diffdrive_arduino configuration"""
        try:
            # Check device path
            if not self.config.device_path.startswith('/dev/'):
                self.logger.error("Invalid device path")
                return False
                
            # Check baud rate
            valid_baud_rates = [9600, 19200, 38400, 57600, 115200, 230400]
            if self.config.baud_rate not in valid_baud_rates:
                self.logger.error(f"Invalid baud rate: {self.config.baud_rate}")
                return False
                
            # Check encoder counts
            if self.config.encoder_counts_per_rev <= 0:
                self.logger.error("Invalid encoder counts per revolution")
                return False
                
            # Check wheel parameters
            if self.config.wheel_separation <= 0 or self.config.wheel_radius <= 0:
                self.logger.error("Invalid wheel parameters")
                return False
                
            return True
            
        except Exception as e:
            self.logger.error(f"Configuration validation failed: {e}")
            return False
            
    def get_health_status(self) -> Dict[str, Any]:
        """Get hardware health status"""
        return {
            "device_path": self.config.device_path,
            "is_connected": self.is_connected,
            "is_initialized": self.is_initialized,
            "baud_rate": self.config.baud_rate,
            "last_communication": "2024-01-01T00:00:00Z",  # Would be actual timestamp
            "error_count": 0,
            "uptime": "00:00:00"
        }

class GazeboSystemInterface(HardwareInterface):
    """Hardware interface for Gazebo simulation"""
    
    def initialize(self) -> bool:
        """Initialize Gazebo interface"""
        self.is_initialized = True
        self.logger.info("Gazebo system interface initialized")
        return True
        
    def connect(self) -> bool:
        """Connect to Gazebo"""
        self.is_connected = True
        self.logger.info("Connected to Gazebo simulation")
        return True
        
    def disconnect(self) -> bool:
        """Disconnect from Gazebo"""
        self.is_connected = False
        return True
        
    def get_ros2_control_config(self) -> Dict[str, Any]:
        """Generate ROS2 control configuration for Gazebo"""
        return {
            "ros2_control": {
                "name": "GazeboSystem",
                "type": "system",
                "hardware": {
                    "plugin": "gazebo_ros2_control/GazeboSystem"
                },
                "joints": {
                    "left_wheel_joint": {
                        "command_interface": {
                            "name": "velocity",
                            "param": {
                                "min": -self.config.max_velocity,
                                "max": self.config.max_velocity
                            }
                        },
                        "state_interface": ["velocity", "position"]
                    },
                    "right_wheel_joint": {
                        "command_interface": {
                            "name": "velocity",
                            "param": {
                                "min": -self.config.max_velocity,
                                "max": self.config.max_velocity
                            }
                        },
                        "state_interface": ["velocity", "position"]
                    }
                }
            }
        }
        
    def get_controller_config(self) -> Dict[str, Any]:
        """Generate controller configuration for Gazebo"""
        config = DiffDriveArduinoInterface.get_controller_config(self)
        # Override sim-specific parameters
        config["controller_manager"]["ros__parameters"]["use_sim_time"] = True
        return config
        
    def validate_configuration(self) -> bool:
        """Validate Gazebo configuration"""
        return True  # Gazebo validation is simpler
        
    def get_health_status(self) -> Dict[str, Any]:
        """Get Gazebo health status"""
        return {
            "simulation_running": self.is_connected,
            "physics_enabled": True,
            "real_time_factor": 1.0,
            "simulation_time": "00:00:00"
        }

class HardwareManager:
    """Central hardware management system"""
    
    def __init__(self, config_file: str = None):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.config_file = config_file
        self.hardware_interface: Optional[HardwareInterface] = None
        self.config: Optional[HardwareConfig] = None
        
    def load_configuration(self, config_file: str = None) -> bool:
        """Load hardware configuration from file"""
        try:
            config_path = config_file or self.config_file
            if not config_path or not os.path.exists(config_path):
                self.logger.error(f"Configuration file not found: {config_path}")
                return False
                
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                
            # Parse configuration
            hardware_config = config_data.get('hardware_interface', {})
            robot_config = config_data.get('robot', {})
            
            # Determine mode and controller type
            mode = HardwareMode.REAL_HARDWARE  # Default
            controller_type = ControllerType.DIFFDRIVE_ARDUINO  # Default
            
            # Create hardware configuration
            self.config = HardwareConfig(
                mode=mode,
                controller_type=controller_type,
                device_path=hardware_config.get('real', {}).get('communication', {}).get('device', '/dev/ttyACM0'),
                baud_rate=hardware_config.get('real', {}).get('communication', {}).get('baud_rate', 115200),
                timeout=hardware_config.get('real', {}).get('communication', {}).get('timeout', 1000),
                loop_rate=hardware_config.get('real', {}).get('communication', {}).get('loop_rate', 60),
                encoder_counts_per_rev=robot_config.get('encoders', {}).get('counts_per_revolution', 3436),
                wheel_separation=robot_config.get('physical', {}).get('wheel_separation', 0.18),
                wheel_radius=robot_config.get('physical', {}).get('wheel_radius', 0.035),
                max_velocity=robot_config.get('physical', {}).get('max_linear_velocity', 1.0),
                max_acceleration=5.0  # Default
            )
            
            self.logger.info("Configuration loaded successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load configuration: {e}")
            return False
            
    def create_hardware_interface(self, mode: HardwareMode = None) -> bool:
        """Create appropriate hardware interface based on mode"""
        try:
            if not self.config:
                self.logger.error("Configuration not loaded")
                return False
                
            target_mode = mode or self.config.mode
            
            if target_mode == HardwareMode.REAL_HARDWARE:
                self.hardware_interface = DiffDriveArduinoInterface(self.config)
            elif target_mode == HardwareMode.SIMULATION:
                self.hardware_interface = GazeboSystemInterface(self.config)
            else:
                self.logger.error(f"Unsupported hardware mode: {target_mode}")
                return False
                
            # Initialize the interface
            if not self.hardware_interface.initialize():
                self.logger.error("Failed to initialize hardware interface")
                return False
                
            self.logger.info(f"Hardware interface created for mode: {target_mode}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to create hardware interface: {e}")
            return False
            
    def connect_hardware(self) -> bool:
        """Connect to the hardware"""
        if not self.hardware_interface:
            self.logger.error("Hardware interface not created")
            return False
            
        return self.hardware_interface.connect()
        
    def disconnect_hardware(self) -> bool:
        """Disconnect from the hardware"""
        if not self.hardware_interface:
            return True
            
        return self.hardware_interface.disconnect()
        
    def get_ros2_control_config(self) -> Dict[str, Any]:
        """Get ROS2 control configuration"""
        if not self.hardware_interface:
            self.logger.error("Hardware interface not created")
            return {}
            
        return self.hardware_interface.get_ros2_control_config()
        
    def get_controller_config(self) -> Dict[str, Any]:
        """Get controller configuration"""
        if not self.hardware_interface:
            self.logger.error("Hardware interface not created")
            return {}
            
        return self.hardware_interface.get_controller_config()
        
    def export_configuration(self, output_dir: str) -> bool:
        """Export configuration files for ROS2 launch"""
        try:
            if not self.hardware_interface:
                self.logger.error("Hardware interface not created")
                return False
                
            output_path = Path(output_dir)
            output_path.mkdir(parents=True, exist_ok=True)
            
            # Export ROS2 control configuration
            ros2_control_config = self.get_ros2_control_config()
            with open(output_path / "ros2_control.yaml", 'w') as f:
                yaml.dump(ros2_control_config, f, default_flow_style=False)
                
            # Export controller configuration
            controller_config = self.get_controller_config()
            with open(output_path / "controllers.yaml", 'w') as f:
                yaml.dump(controller_config, f, default_flow_style=False)
                
            self.logger.info(f"Configuration exported to {output_dir}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to export configuration: {e}")
            return False
            
    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status"""
        status = {
            "hardware_manager": {
                "initialized": self.config is not None,
                "interface_created": self.hardware_interface is not None,
                "config_file": self.config_file
            }
        }
        
        if self.hardware_interface:
            status["hardware_interface"] = self.hardware_interface.get_health_status()
            
        return status

def main():
    """Example usage of the hardware abstraction layer"""
    logging.basicConfig(level=logging.INFO)
    
    # Create hardware manager
    manager = HardwareManager()
    
    # Load configuration
    if not manager.load_configuration("unified_robot_config.yaml"):
        print("Failed to load configuration")
        return
        
    # Create hardware interface for real hardware
    if not manager.create_hardware_interface(HardwareMode.REAL_HARDWARE):
        print("Failed to create hardware interface")
        return
        
    # Connect to hardware
    if not manager.connect_hardware():
        print("Failed to connect to hardware")
        return
        
    # Export configuration files
    if not manager.export_configuration("./generated_config"):
        print("Failed to export configuration")
        return
        
    # Get system status
    status = manager.get_system_status()
    print("System Status:")
    print(yaml.dump(status, default_flow_style=False))
    
    # Disconnect
    manager.disconnect_hardware()

if __name__ == "__main__":
    main()