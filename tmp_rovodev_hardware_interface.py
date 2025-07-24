#!/usr/bin/env python3
"""
hardware_interface.py - Hardware Abstraction Layer for diffdrive_arduino
Long-term solution for modular hardware management
Author: RovoDev Assistant
"""

import os
import yaml
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

class HardwareMode(Enum):
    """Hardware operation modes"""
    SIMULATION = "simulation"
    REAL_HARDWARE = "real_hardware"
    MOCK = "mock"

@dataclass
class HardwareConfig:
    """Hardware configuration data structure"""
    mode: HardwareMode
    device_path: str
    baud_rate: int
    timeout: int
    loop_rate: int
    encoder_counts_per_rev: int
    wheel_separation: float
    wheel_radius: float
    max_velocity: float
    
class BaseHardwareInterface(ABC):
    """Abstract base class for hardware interfaces"""
    
    def __init__(self, config: HardwareConfig):
        self.config = config
        self.is_initialized = False
        
    @abstractmethod
    def initialize(self) -> bool:
        """Initialize hardware interface"""
        pass
        
    @abstractmethod
    def shutdown(self) -> bool:
        """Shutdown hardware interface"""
        pass
        
    @abstractmethod
    def get_controller_config(self) -> Dict[str, Any]:
        """Get controller configuration"""
        pass
        
    @abstractmethod
    def get_hardware_config(self) -> Dict[str, Any]:
        """Get hardware-specific configuration"""
        pass

class DiffDriveArduinoInterface(BaseHardwareInterface):
    """diffdrive_arduino hardware interface implementation"""
    
    def __init__(self, config: HardwareConfig):
        super().__init__(config)
        self.plugin_name = "diffdrive_arduino/DiffDriveArduino"
        
    def initialize(self) -> bool:
        """Initialize diffdrive_arduino interface"""
        try:
            # Validate device path exists
            if not os.path.exists(self.config.device_path):
                raise FileNotFoundError(f"Device {self.config.device_path} not found")
                
            # Test communication (placeholder)
            self._test_communication()
            
            self.is_initialized = True
            return True
            
        except Exception as e:
            print(f"Failed to initialize diffdrive_arduino: {e}")
            return False
            
    def shutdown(self) -> bool:
        """Shutdown diffdrive_arduino interface"""
        try:
            # Cleanup operations
            self.is_initialized = False
            return True
        except Exception as e:
            print(f"Failed to shutdown diffdrive_arduino: {e}")
            return False
            
    def get_controller_config(self) -> Dict[str, Any]:
        """Get controller configuration for diffdrive_arduino"""
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
        
    def get_hardware_config(self) -> Dict[str, Any]:
        """Get hardware-specific configuration for diffdrive_arduino"""
        return {
            "diffdrive_arduino": {
                "ros__parameters": {
                    "left_wheel_name": "left_wheel_joint",
                    "right_wheel_name": "right_wheel_joint",
                    "loop_rate": float(self.config.loop_rate),
                    "device": self.config.device_path,
                    "baud_rate": self.config.baud_rate,
                    "timeout": self.config.timeout,
                    "enc_counts_per_rev": self.config.encoder_counts_per_rev
                }
            }
        }
        
    def _test_communication(self) -> bool:
        """Test communication with Arduino"""
        # Placeholder for actual communication test
        # In real implementation, this would test serial communication
        return True

class GazeboInterface(BaseHardwareInterface):
    """Gazebo simulation hardware interface"""
    
    def __init__(self, config: HardwareConfig):
        super().__init__(config)
        self.plugin_name = "gazebo_ros2_control/GazeboSystem"
        
    def initialize(self) -> bool:
        """Initialize Gazebo interface"""
        try:
            self.is_initialized = True
            return True
        except Exception as e:
            print(f"Failed to initialize Gazebo interface: {e}")
            return False
            
    def shutdown(self) -> bool:
        """Shutdown Gazebo interface"""
        try:
            self.is_initialized = False
            return True
        except Exception as e:
            print(f"Failed to shutdown Gazebo interface: {e}")
            return False
            
    def get_controller_config(self) -> Dict[str, Any]:
        """Get controller configuration for Gazebo"""
        base_config = DiffDriveArduinoInterface(self.config).get_controller_config()
        # Override sim-specific parameters
        base_config["controller_manager"]["ros__parameters"]["use_sim_time"] = True
        return base_config
        
    def get_hardware_config(self) -> Dict[str, Any]:
        """Get hardware configuration for Gazebo"""
        return {
            "gazebo_ros2_control": {
                "ros__parameters": {
                    "use_sim_time": True
                }
            }
        }

class HardwareManager:
    """Central hardware management class"""
    
    def __init__(self, config_file: Optional[str] = None):
        self.config_file = config_file
        self.hardware_interface: Optional[BaseHardwareInterface] = None
        self.config: Optional[HardwareConfig] = None
        
    def load_config(self, mode: HardwareMode) -> HardwareConfig:
        """Load configuration based on mode"""
        if self.config_file and os.path.exists(self.config_file):
            with open(self.config_file, 'r') as f:
                config_data = yaml.safe_load(f)
        else:
            config_data = self._get_default_config()
            
        # Extract mode-specific configuration
        if mode == HardwareMode.SIMULATION:
            hw_config = config_data.get('hardware_interface', {}).get('simulation', {})
            device_path = "/dev/null"  # Not used in simulation
        else:
            hw_config = config_data.get('hardware_interface', {}).get('real', {})
            device_path = hw_config.get('communication', {}).get('device', '/dev/ttyACM0')
            
        robot_config = config_data.get('robot', {})
        physical = robot_config.get('physical', {})
        encoders = robot_config.get('encoders', {})
        
        return HardwareConfig(
            mode=mode,
            device_path=device_path,
            baud_rate=hw_config.get('communication', {}).get('baud_rate', 115200),
            timeout=hw_config.get('communication', {}).get('timeout', 1000),
            loop_rate=hw_config.get('communication', {}).get('loop_rate', 60),
            encoder_counts_per_rev=encoders.get('counts_per_revolution', 3436),
            wheel_separation=physical.get('wheel_separation', 0.18),
            wheel_radius=physical.get('wheel_radius', 0.035),
            max_velocity=physical.get('max_linear_velocity', 1.0)
        )
        
    def initialize_hardware(self, mode: HardwareMode) -> bool:
        """Initialize hardware interface based on mode"""
        try:
            self.config = self.load_config(mode)
            
            if mode == HardwareMode.SIMULATION:
                self.hardware_interface = GazeboInterface(self.config)
            elif mode == HardwareMode.REAL_HARDWARE:
                self.hardware_interface = DiffDriveArduinoInterface(self.config)
            else:
                raise ValueError(f"Unsupported hardware mode: {mode}")
                
            return self.hardware_interface.initialize()
            
        except Exception as e:
            print(f"Failed to initialize hardware: {e}")
            return False
            
    def shutdown_hardware(self) -> bool:
        """Shutdown current hardware interface"""
        if self.hardware_interface:
            return self.hardware_interface.shutdown()
        return True
        
    def get_controller_config_file(self, output_path: str) -> str:
        """Generate controller configuration file"""
        if not self.hardware_interface:
            raise RuntimeError("Hardware interface not initialized")
            
        config = self.hardware_interface.get_controller_config()
        
        with open(output_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
            
        return output_path
        
    def get_hardware_config_file(self, output_path: str) -> str:
        """Generate hardware configuration file"""
        if not self.hardware_interface:
            raise RuntimeError("Hardware interface not initialized")
            
        config = self.hardware_interface.get_hardware_config()
        
        with open(output_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
            
        return output_path
        
    def _get_default_config(self) -> Dict[str, Any]:
        """Get default configuration if no config file provided"""
        return {
            "robot": {
                "physical": {
                    "wheel_separation": 0.18,
                    "wheel_radius": 0.035,
                    "max_linear_velocity": 1.0
                },
                "encoders": {
                    "counts_per_revolution": 3436
                }
            },
            "hardware_interface": {
                "real": {
                    "communication": {
                        "device": "/dev/ttyACM0",
                        "baud_rate": 115200,
                        "timeout": 1000,
                        "loop_rate": 60
                    }
                },
                "simulation": {
                    "communication": {
                        "loop_rate": 100
                    }
                }
            }
        }

# Utility functions for launch file integration
def create_hardware_manager(mode_str: str, config_file: Optional[str] = None) -> HardwareManager:
    """Factory function to create hardware manager"""
    mode = HardwareMode(mode_str)
    manager = HardwareManager(config_file)
    
    if not manager.initialize_hardware(mode):
        raise RuntimeError(f"Failed to initialize hardware in {mode} mode")
        
    return manager

def generate_config_files(mode_str: str, output_dir: str, config_file: Optional[str] = None) -> tuple:
    """Generate configuration files for given mode"""
    manager = create_hardware_manager(mode_str, config_file)
    
    controller_config_path = os.path.join(output_dir, f"controllers_{mode_str}.yaml")
    hardware_config_path = os.path.join(output_dir, f"hardware_{mode_str}.yaml")
    
    manager.get_controller_config_file(controller_config_path)
    manager.get_hardware_config_file(hardware_config_path)
    
    manager.shutdown_hardware()
    
    return controller_config_path, hardware_config_path

if __name__ == "__main__":
    # Example usage
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python hardware_interface.py <mode> [config_file]")
        print("Modes: simulation, real_hardware")
        sys.exit(1)
        
    mode = sys.argv[1]
    config_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    try:
        manager = create_hardware_manager(mode, config_file)
        print(f"Hardware manager initialized successfully in {mode} mode")
        
        # Generate config files
        controller_config, hardware_config = generate_config_files(
            mode, "/tmp", config_file
        )
        
        print(f"Generated configuration files:")
        print(f"  Controller: {controller_config}")
        print(f"  Hardware: {hardware_config}")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)