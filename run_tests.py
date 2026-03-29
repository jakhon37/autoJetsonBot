#!/usr/bin/env python3
"""
Robot Test Runner
Comprehensive test suite for Autonomous Jetson Robot
"""
import argparse
import sys
import os
import unittest
import subprocess
import time
from typing import List, Optional

# Add test_suite to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_suite.config import config


class TestRunner:
    """Test runner with reporting and cleanup"""
    
    def __init__(self, verbose: bool = True):
        self.verbose = verbose
        self.results = []
    
    def ensure_container_running(self):
        """Ensure Docker container is running"""
        print("[SETUP] Checking Docker container...")
        
        result = subprocess.run(
            ["docker", "ps", "--filter", f"name={config.CONTAINER_NAME}", "--format", "{{.Names}}"],
            capture_output=True,
            text=True
        )
        
        if config.CONTAINER_NAME not in result.stdout:
            print(f"  Starting container '{config.CONTAINER_NAME}'...")
            subprocess.run(["docker", "start", config.CONTAINER_NAME], check=False)
            time.sleep(3)
        
        print("[SETUP] Container ready\n")
    
    def run_tests(self, test_path: str, description: str) -> unittest.TestResult:
        """Run a specific test module"""
        print(f"\n{'='*60}")
        print(f"Running: {description}")
        print(f"{'='*60}\n")
        
        # Discover tests from the directory containing test files
        test_dir = os.path.dirname(test_path)
        
        loader = unittest.TestLoader()
        suite = loader.discover(test_dir, pattern="test_*.py")
        
        runner = unittest.TextTestRunner(verbosity=2 if self.verbose else 1)
        result = runner.run(suite)
        
        return result
    
    def print_summary(self, results: List[unittest.TestResult]):
        """Print overall test summary"""
        print(f"\n{'='*60}")
        print("OVERALL TEST SUMMARY")
        print(f"{'='*60}\n")
        
        total_tests = 0
        total_failures = 0
        total_errors = 0
        total_skipped = 0
        
        for result in results:
            total_tests += result.testsRun
            total_failures += len(result.failures)
            total_errors += len(result.errors)
            total_skipped += len(result.skipped)
        
        print(f"Tests Run:     {total_tests}")
        print(f"Failures:      {total_failures}")
        print(f"Errors:        {total_errors}")
        print(f"Skipped:       {total_skipped}")
        print(f"Passed:        {total_tests - total_failures - total_errors - total_skipped}")
        
        success = total_failures == 0 and total_errors == 0
        print(f"\n{'✓' if success else '✗'} Overall: {'PASSED' if success else 'FAILED'}")
        
        return 0 if success else 1


def main():
    parser = argparse.ArgumentParser(
        description="Run Autonomous Robot Test Suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_tests.py                    # Run all tests
  python run_tests.py --docker           # Run only Docker tests
  python run_tests.py --ros              # Run only ROS2 tests
  python run_tests.py --web              # Run only web tests
  python run_tests.py --nav              # Run only navigation tests
  python run_tests.py --sim              # Run only simulation tests
  python run_tests.py --hardware         # Run only hardware tests
  python run_tests.py --quick            # Quick smoke test
        """
    )
    
    parser.add_argument("--docker", action="store_true", help="Run Docker environment tests")
    parser.add_argument("--ros", action="store_true", help="Run ROS2 core tests")
    parser.add_argument("--web", action="store_true", help="Run web interface tests")
    parser.add_argument("--nav", action="store_true", help="Run navigation tests")
    parser.add_argument("--sim", action="store_true", help="Run simulation tests")
    parser.add_argument("--hardware", action="store_true", help="Run hardware tests")
    parser.add_argument("--quick", action="store_true", help="Quick smoke test")
    parser.add_argument("--all", action="store_true", help="Run all tests")
    parser.add_argument("--quiet", action="store_true", help="Less verbose output")
    
    args = parser.parse_args()
    
    runner = TestRunner(verbose=not args.quiet)
    
    # If no specific test selected, show help
    if not any([args.docker, args.ros, args.web, args.nav, 
                args.sim, args.hardware, args.quick, args.all]):
        parser.print_help()
        return 0
    
    # Determine which tests to run
    run_docker = args.docker or args.all or args.quick
    run_ros = args.ros or args.all
    run_web = args.web or args.all
    run_nav = args.nav or args.all
    run_sim = args.sim or args.all
    run_hardware = args.hardware or args.all
    
    results = []
    
    # Get test directory
    test_dir = os.path.dirname(os.path.abspath(__file__))
    
    try:
        # Run Docker tests first (always needed)
        if run_docker:
            runner.ensure_container_running()
            result = runner.run_tests(
                os.path.join(test_dir, "test_suite/integration/test_docker.py"),
                "Docker Environment Tests"
            )
            results.append(result)
        
        # Run ROS2 core tests
        if run_ros:
            result = runner.run_tests(
                os.path.join(test_dir, "test_suite/integration/test_ros2_core.py"),
                "ROS2 Core Tests"
            )
            results.append(result)
        
        # Run web interface tests
        if run_web:
            result = runner.run_tests(
                os.path.join(test_dir, "test_suite/integration/test_web_interface.py"),
                "Web Interface Tests"
            )
            results.append(result)
        
        # Run navigation tests
        if run_nav:
            result = runner.run_tests(
                os.path.join(test_dir, "test_suite/integration/test_navigation.py"),
                "Navigation Tests"
            )
            results.append(result)
        
        # Run simulation tests
        if run_sim:
            result = runner.run_tests(
                os.path.join(test_dir, "test_suite/integration/test_simulation.py"),
                "Simulation Tests"
            )
            results.append(result)
        
        # Run hardware tests
        if run_hardware:
            result = runner.run_tests(
                os.path.join(test_dir, "test_suite/hardware/test_hardware.py"),
                "Hardware Tests"
            )
            results.append(result)
        
        # Quick test - just Docker + basic ROS
        if args.quick:
            result = runner.run_tests(
                os.path.join(test_dir, "test_suite/integration/test_ros2_core.py"),
                "Quick Smoke Test"
            )
            results.append(result)
        
        # Print summary
        return runner.print_summary(results)
        
    except KeyboardInterrupt:
        print("\n\nTests interrupted by user")
        return 1
    except Exception as e:
        print(f"\n\nError running tests: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
