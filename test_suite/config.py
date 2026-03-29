"""
Test Configuration for Autonomous Robot Project
Production-level test configuration and constants
"""

import os
from dataclasses import dataclass
from typing import Optional


@dataclass
class RobotConfig:
    """Robot hardware and simulation configuration"""
    # Docker
    CONTAINER_NAME: str = "auto_ros_foxy"
    CONTAINER_IMAGE: str = "auto_ros:foxy"
    
    # Network
    WEB_PORT: int = 8000
    ROSBRIDGE_PORT: int = 9090
    WEB_URL: str = "http://localhost:8000"
    ROSBRIDGE_URL: str = "ws://localhost:9090"
    
    # Timeouts (seconds)
    LAUNCH_TIMEOUT: int = 60
    TOPIC_TIMEOUT: int = 10
    HEALTH_CHECK_TIMEOUT: int = 30
    
    # Topics
    CMD_VEL_TOPIC: str = "/cmd_vel"
    ODOM_TOPIC: str = "/odom"
    SCAN_TOPIC: str = "/scan"
    JOINT_STATES_TOPIC: str = "/joint_states"
    TF_TOPIC: str = "/tf"
    MAP_TOPIC: str = "/map"
    CAMERA_TOPIC: str = "/camera/image_raw"
    
    # Navigation
    NAV2_ACTION: str = "/navigate_to_pose"
    INITIAL_POSE_TOPIC: str = "/initialpose"
    
    # Expected values
    MIN_ODOM_FREQUENCY: float = 10.0  # Hz
    MIN_SCAN_FREQUENCY: float = 5.0   # Hz
    MIN_CAMERA_FREQUENCY: float = 5.0  # Hz
    
    # Robot physical parameters (from config)
    WHEEL_SEPARATION: float = 0.18  # meters
    WHEEL_RADIUS: float = 0.035     # meters
    MAX_LINEAR_VEL: float = 1.0     # m/s
    MAX_ANGULAR_VEL: float = 2.0   # rad/s


@dataclass  
class TestPaths:
    """File paths for testing"""
    PROJECT_ROOT: str = "/home/jakhon37/myspace/robotics/autoJetsonBot"
    SRC_DIR: str = "/autonomous_ROS/src"
    INSTALL_DIR: str = "/autonomous_ROS/install"
    LOG_DIR: str = "/autonomous_ROS/log"
    MAP_FILE: str = "/autonomous_ROS/install/my_robot_launch/share/my_robot_launch/lab_map.yaml"


class EnvConfig:
    """Environment configuration"""
    IS_DOCKER: bool = os.path.exists("/.dockerenv")
    IS_SIMULATION: bool = True
    ROS_DISTRO: str = "foxy"
    DOMAIN_ID: int = 0


# Global config instance
config = RobotConfig()
paths = TestPaths()
env = EnvConfig()
