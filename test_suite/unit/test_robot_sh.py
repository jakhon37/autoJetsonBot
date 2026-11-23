"""
robot.sh Tests
Checks the command dispatcher with stub commands, so no Docker or ROS is needed
"""
import os
import re
import shutil
import subprocess
import tempfile
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROBOT_SH = os.path.join(REPO_ROOT, "robot.sh")

# Stand-ins for the commands `./robot.sh watch` runs. docker only answers `ps`, so
# show_status finds no launch and returns early. sleep fails on its second call,
# which ends the loop through `set -e`.
STUBS = {
    "docker": '[ "$1" = "ps" ]\n',
    "curl": "exit 7\n",
    "clear": "exit 0\n",
    "sleep": 'echo "$@" >> "$SLEEP_LOG"\ncount=$(wc -l < "$SLEEP_LOG")\n[ $count -lt 2 ]\n',
}


def _dispatcher_labels():
    """Return the command labels of robot.sh's main case statement, in order"""
    with open(ROBOT_SH) as f:
        text = f.read()
    block = text.split('case "${1:-help}" in', 1)[1].split("\nesac", 1)[0]
    return re.findall(r"^ {4}([\w|*-]+)\)", block, re.MULTILINE)


class TestRobotShDispatcher(unittest.TestCase):
    """Test the ./robot.sh command dispatcher"""

    def test_commands_are_defined_once(self):
        """Test that no command has two case arms, since bash only runs the first"""
        labels = _dispatcher_labels()
        self.assertIn("watch", labels)
        duplicates = sorted({label for label in labels if labels.count(label) > 1})
        self.assertEqual(duplicates, [], f"Commands defined more than once: {duplicates}")

    def test_watch_refreshes_status(self):
        """Test that watch reruns show_status every 10 s"""
        stub_dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, stub_dir)
        for name, body in STUBS.items():
            path = os.path.join(stub_dir, name)
            with open(path, "w") as f:
                f.write("#!/bin/sh\n" + body)
            os.chmod(path, 0o755)
        sleep_log = os.path.join(stub_dir, "sleep.log")
        run_env = dict(os.environ, PATH=stub_dir + os.pathsep + os.environ.get("PATH", ""),
                       SLEEP_LOG=sleep_log)

        result = subprocess.run(["bash", ROBOT_SH, "watch"], cwd=REPO_ROOT, env=run_env,
                                capture_output=True, text=True, timeout=30)

        self.assertIn("Watching status", result.stdout)
        self.assertEqual(result.stdout.count("--- ROS Health ---"), 2,
                         f"Expected two status refreshes, got:\n{result.stdout}")
        with open(sleep_log) as f:
            self.assertEqual(f.read().split(), ["10", "10"])

    def test_lidar_motor_is_stopped_after_stop_and_freed_for_a_launch(self):
        """Test that stop holds the lidar's motor off, and that every launch frees its port"""
        # The holder keeps the lidar's port open. If a launch left it running, the lidar driver
        # would share the port with it.
        with open(ROBOT_SH) as f:
            text = f.read()
        stop_robot = text.split("stop_robot() {", 1)[1].split("\n}\n", 1)[0]
        self.assertIn('"lidar_motor_off"', stop_robot)
        dispatcher = text.split('case "${1:-help}" in', 1)[1].split("\nesac", 1)[0]
        stop_arm = dispatcher.split("\n    stop)", 1)[1].split(";;", 1)[0]
        self.assertIn("stop_lidar_motor", stop_arm)
        for launch in ("up", "sim", "robot", "nav", "map2nav"):
            arm = dispatcher.split(f"\n    {launch})", 1)[1].split(";;", 1)[0]
            self.assertIn("stop_robot", arm, launch)
            self.assertNotIn("stop_lidar_motor", arm, launch)


if __name__ == "__main__":
    unittest.main(verbosity=2)
