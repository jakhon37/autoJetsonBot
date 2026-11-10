# Copyright 2026 Mac37
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ast
import math
import os
import re
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(PKG_DIR, 'config')
URDF_DIR = os.path.join(os.path.dirname(PKG_DIR), 'jetson_bot_description', 'urdf')
LIDAR_JS = os.path.join(os.path.dirname(PKG_DIR), 'jetson_bot_gui', 'web', 'js', 'lidar.js')


def _params(file_name, node):
    with open(os.path.join(CONFIG_DIR, file_name), 'r') as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def _launch_static_transforms():
    """Return main.launch.py's static transforms by child frame, as their old-style arguments."""
    with open(os.path.join(PKG_DIR, 'launch', 'main.launch.py'), 'r') as f:
        tree = ast.parse(f.read())
    statics = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Tuple) and len(node.elts) == 2 and isinstance(node.elts[0], ast.List):
            try:
                args, name = ast.literal_eval(node)
            except ValueError:
                continue
            if isinstance(name, str) and name.endswith('_broadcaster'):
                statics[args[7]] = args
    return statics


def _urdf_fixed_joint(child):
    """Return the parent, xyz and rpy of the URDF's fixed joint to the child link."""
    for file_name in ('robot_core.xacro', 'lidar.xacro'):
        for joint in ET.parse(os.path.join(URDF_DIR, file_name)).getroot().iter('joint'):
            if joint.get('type') == 'fixed' and joint.find('child').get('link') == child:
                origin = joint.find('origin')
                return (joint.find('parent').get('link'),
                        [float(v) for v in origin.get('xyz').split()],
                        [float(v) for v in origin.get('rpy').split()])
    raise AssertionError(f'No fixed joint to {child} in the URDF')


def test_ekf_is_the_only_odom_tf_publisher_in_sim():
    """Verify that diff_cont leaves odom -> base_footprint to the EKF, as on the robot."""
    diff_cont = _params('unified_robot_config.yaml', 'diff_cont')
    ekf = _params('ekf.yaml', 'ekf_filter_node')
    assert diff_cont['enable_odom_tf'] is False
    assert ekf['publish_tf'] is True
    assert ekf['world_frame'] == ekf['odom_frame'] == diff_cont['odom_frame_id'] == 'odom'
    assert ekf['base_link_frame'] == diff_cont['base_frame_id'] == 'base_footprint'



def test_sim_odom_covariance_is_not_zero():
    """Verify that diff_cont gives the EKF real variances, like the bridge on the robot."""
    diff_cont = _params('unified_robot_config.yaml', 'diff_cont')
    for name in ('pose_covariance_diagonal', 'twist_covariance_diagonal'):
        diagonal = diff_cont[name]
        assert len(diagonal) == 6
        assert all(isinstance(v, float) and v > 0.0 for v in diagonal)


def test_ekf_takes_heading_changes_from_the_gyro():
    """Verify that the EKF fuses the IMU's yaw rate but not the Madgwick filter's yaw."""
    # Without a magnetometer Madgwick's yaw holds no heading. Fusing it kept the EKF's heading
    # within 6 degrees through a 90 degree hand turn on the robot.
    ekf = _params('ekf.yaml', 'ekf_filter_node')
    yaw, vyaw = 5, 11  # [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
    assert ekf['imu0_config'][yaw] is False
    assert ekf['imu0_config'][vyaw] is True
    assert ekf['odom0_config'][vyaw] is True


def test_launch_static_transforms_repeat_the_urdf():
    """Verify that main.launch.py's static transforms equal the URDF's fixed joints."""
    # TF keeps whichever copy arrives last. On 2026-11-10 a lidar yaw changed only in lidar.xacro
    # showed on the robot after one start and not after the next.
    statics = _launch_static_transforms()
    assert set(statics) == {'base_link', 'chassis', 'laser_frame'}
    for child, (x, y, z, yaw, pitch, roll, parent, _) in statics.items():
        xyz = [float(x), float(y), float(z)]
        rpy = [float(roll), float(pitch), float(yaw)]
        assert _urdf_fixed_joint(child) == (parent, xyz, rpy)


def test_urdf_has_no_colon_and_space():
    """Verify that the URDF survives gazebo_ros2_control, which passes it as a --param override."""
    # rcl reads that value as YAML, where ': ' starts a key. One in a comment in lidar.xacro kept
    # diff_cont from loading in Gazebo, so the simulated robot had no /odom (2026-11-10).
    for name in sorted(os.listdir(URDF_DIR)):
        with open(os.path.join(URDF_DIR, name), 'r') as f:
            for number, line in enumerate(f, 1):
                assert ': ' not in line, f'{name}:{number}'


def test_lidar_zero_points_to_the_robots_right():
    """Verify that the URDF and the dashboard turn the scan by the lidar's mounting."""
    # On the robot on 2026-11-10 an object straight ahead showed at +101 degrees in the scan, and
    # objects to the front-left at +115 and +145. The exact angle is still to be measured.
    _, _, rpy = _urdf_fixed_joint('laser_frame')
    assert -105 < math.degrees(rpy[2]) < -75
    with open(LIDAR_JS, 'r') as f:
        dashboard = re.search(r'const LIDAR_YAW = (-?[0-9.]+);', f.read())
    assert dashboard is not None, 'lidar.js does not turn the scan'
    assert float(dashboard.group(1)) == rpy[2]


def _fused(config):
    return {i for i, on in enumerate(config) if on}


def test_ekf_fuses_only_wheel_speeds_and_the_gyro_rate():
    """Verify that the EKF takes speeds from the wheels and only the yaw rate from the IMU."""
    # With the wheels' x and y and the uncalibrated accelerometer fused too, the EKF's heading
    # turned -15 and +47 degrees on two 30 cm straight drives on the floor, with the wheels and
    # the gyro both reading about +2.
    ekf = _params('ekf.yaml', 'ekf_filter_node')
    vx, vy, vyaw = 6, 7, 11
    assert _fused(ekf['odom0_config']) == {vx, vy, vyaw}
    assert _fused(ekf['imu0_config']) == {vyaw}


if __name__ == '__main__':
    test_ekf_is_the_only_odom_tf_publisher_in_sim()
    test_sim_odom_covariance_is_not_zero()
    test_ekf_takes_heading_changes_from_the_gyro()
    test_ekf_fuses_only_wheel_speeds_and_the_gyro_rate()
    test_launch_static_transforms_repeat_the_urdf()
    test_urdf_has_no_colon_and_space()
    test_lidar_zero_points_to_the_robots_right()
