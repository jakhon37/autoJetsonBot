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


# A docker that says the container runs, answers the slam_toolbox process check with
# $SLAM_RUNNING, gives the launch's map settings, and makes tools/save_map.py succeed when
# $SAVE_OK is 1. It logs every call, and each path save_map.py is given.
DOCKER_STUB = r"""
echo "$*" >> "$DOCKER_LOG"
case "$1" in
    ps) echo auto_ros_humble; exit 0 ;;
    exec) ;;
    *) exit 0 ;;
esac
case "$*" in
    *"pgrep -f lib/slam_toolbox/"*) [ "$SLAM_RUNNING" = 1 ]; exit ;;
    *save_map.py*)
        for last; do :; done
        echo "$last" >> "$SAVE_LOG"
        [ "$SAVE_OK" = 1 ]; exit ;;
    *" python3 -"*)
        script=$(cat)
        case "$script" in
            *unified_robot_config.yaml*) printf 'lab_map_v3\n/autonomous_ROS/maps\n' ;;
        esac ;;
esac
exit 0
"""


@unittest.skipIf(os.path.exists("/etc/nv_tegra_release"),
                 "stop would end a camera server running on this Jetson")
class TestRobotShMapSaving(unittest.TestCase):
    """Test that robot.sh saves SLAM's map through tools/save_map.py and acts on the result"""

    def run_robot_sh(self, *args, slam_running=True, save_ok=True):
        stub_dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, stub_dir)
        for name, body in (("docker", DOCKER_STUB), ("sleep", "exit 0\n")):
            path = os.path.join(stub_dir, name)
            with open(path, "w") as f:
                f.write("#!/bin/sh\n" + body)
            os.chmod(path, 0o755)
        self.docker_log = os.path.join(stub_dir, "docker.log")
        self.save_log = os.path.join(stub_dir, "save.log")
        open(self.save_log, "w").close()
        run_env = dict(os.environ, PATH=stub_dir + os.pathsep + os.environ.get("PATH", ""),
                       DOCKER_LOG=self.docker_log, SAVE_LOG=self.save_log,
                       SLAM_RUNNING="1" if slam_running else "0",
                       SAVE_OK="1" if save_ok else "0", CONTAINER_NAME="auto_ros_humble")
        return subprocess.run(["bash", ROBOT_SH, *args], cwd=REPO_ROOT, env=run_env,
                              capture_output=True, text=True, timeout=30)

    def saves(self):
        with open(self.save_log) as f:
            return f.read().split()

    def docker_calls(self):
        with open(self.docker_log) as f:
            return f.read()

    def test_save_map_uses_the_launch_map_name(self):
        """Test that save-map saves as the map the launch uses, or as the name given"""
        result = self.run_robot_sh("save-map")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Map saved", result.stdout)
        self.assertEqual(self.saves(), ["/autonomous_ROS/maps/lab_map_v3"])

        self.run_robot_sh("save-map", "lab_map_v4.yaml")
        self.assertEqual(self.saves(), ["/autonomous_ROS/maps/lab_map_v4"])

    def test_failed_save_is_reported(self):
        """Test that save-map exits 1 and says so when the save fails"""
        result = self.run_robot_sh("save-map", save_ok=False)
        self.assertEqual(result.returncode, 1)
        self.assertIn("Map NOT saved", result.stdout)
        self.assertNotIn("Map saved", result.stdout)

    def test_nothing_to_save_without_slam(self):
        """Test that save-map exits 2 when slam_toolbox isn't running, and saves nothing"""
        result = self.run_robot_sh("save-map", slam_running=False)
        self.assertEqual(result.returncode, 2)
        self.assertIn("Nothing to save", result.stdout + result.stderr)
        self.assertEqual(self.saves(), [])

    def test_nav_keeps_mapping_when_the_save_fails(self):
        """Test that nav and map2nav stop nothing and launch nothing after a failed save"""
        for command in ("nav", "map2nav"):
            result = self.run_robot_sh(command, "sim:=true", save_ok=False)
            self.assertEqual(result.returncode, 1, command)
            self.assertIn("Still mapping", result.stdout + result.stderr)
            self.assertNotIn("ros2 launch", self.docker_calls())
            self.assertNotIn("Stopping all processes", result.stdout)

    def test_nav_saves_once_then_switches(self):
        """Test that nav saves the map once, not again while stopping, then launches Nav2"""
        result = self.run_robot_sh("nav", "sim:=true")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.saves(), ["/autonomous_ROS/maps/lab_map_v3"])
        self.assertIn("Stopping all processes", result.stdout)
        self.assertRegex(self.docker_calls(), r"ros2 launch jetson_bot_bringup main\.launch\.py "
                                              r"mode:=navigation sim:=true")

    def test_stop_stops_even_when_the_save_fails(self):
        """Test that stop saves first, and still stops when the save fails"""
        result = self.run_robot_sh("stop", save_ok=False)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.saves(), ["/autonomous_ROS/maps/lab_map_v3"])
        self.assertIn("Map NOT saved", result.stdout)
        self.assertIn("All processes terminated", result.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
