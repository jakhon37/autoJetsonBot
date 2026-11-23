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

import math
import os
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(PKG_DIR, 'config')
URDF_DIR = os.path.join(os.path.dirname(PKG_DIR), 'jetson_bot_description', 'urdf')
NAV2_FILES = ('nav2_params.yaml', 'nav2_params_humble.yaml')
COSTMAPS = ('local_costmap', 'global_costmap')


def _nav2(file_name):
    with open(os.path.join(CONFIG_DIR, file_name), 'r') as f:
        return yaml.safe_load(f)


def _costmap(params, name):
    return params[name][name]['ros__parameters']


def _chassis_corner_distance():
    """Return how far the chassis box's farthest corner is from base_footprint, seen from above."""
    root = ET.parse(os.path.join(URDF_DIR, 'robot_core.xacro')).getroot()
    joints = {j.find('child').get('link'): j for j in root.iter('joint')}
    x = y = 0.0
    link = 'chassis'
    while link != 'base_footprint':
        joint = joints[link]
        assert joint.get('type') == 'fixed'
        origin = joint.find('origin')
        assert [float(v) for v in origin.get('rpy').split()] == [0.0, 0.0, 0.0]
        x += float(origin.get('xyz').split()[0])
        y += float(origin.get('xyz').split()[1])
        link = joint.find('parent').get('link')
    chassis = next(lk for lk in root.iter('link') if lk.get('name') == 'chassis')
    collision = chassis.find('collision')
    cx, cy = (float(v) for v in collision.find('origin').get('xyz').split()[:2])
    size_x, size_y = (float(v) for v in collision.find('geometry/box').get('size').split()[:2])
    return max(math.hypot(x + cx + sx * size_x / 2, y + cy + sy * size_y / 2)
               for sx in (-1, 1) for sy in (-1, 1))


def _assert_yaml_valid(yaml_file):
    assert os.path.exists(yaml_file), f"YAML file not found at {yaml_file}"
    with open(yaml_file, 'r') as f:
        try:
            data = yaml.safe_load(f)
            assert data is not None
            print(f"✅ {os.path.basename(yaml_file)} is valid")
        except yaml.YAMLError as e:
            assert False, f"YAML Parsing failed for {yaml_file}: {str(e)}"


def test_nav2_yaml_validity():
    """Verify that the Nav2 YAML configurations are valid YAML."""
    for file_name in NAV2_FILES:
        _assert_yaml_valid(os.path.join(CONFIG_DIR, file_name))


def test_footprint_covers_the_chassis():
    """Verify that both costmaps' circle holds the chassis' corners as the robot turns."""
    corner = _chassis_corner_distance()
    assert 0.17 < corner < 0.19  # 0.181 m with the 2026-06-07 measurements
    for file_name in NAV2_FILES:
        params = _nav2(file_name)
        for name in COSTMAPS:
            costmap = _costmap(params, name)
            assert 'footprint' not in costmap
            assert costmap['robot_radius'] >= corner, (file_name, name)
            assert costmap['inflation_layer']['inflation_radius'] > costmap['robot_radius']


def test_speed_limits_agree():
    """Verify that DWB, the velocity smoother and the behaviours share one set of limits."""
    humble = _nav2('nav2_params_humble.yaml')
    dwb = humble['controller_server']['ros__parameters']['FollowPath']
    smoother = humble['velocity_smoother']['ros__parameters']
    behaviors = humble['behavior_server']['ros__parameters']
    assert dwb['max_vel_x'] == dwb['max_speed_xy'] == smoother['max_velocity'][0]
    assert dwb['max_vel_theta'] == smoother['max_velocity'][2] == behaviors['max_rotational_vel']
    assert smoother['min_velocity'] == [-v for v in smoother['max_velocity']]
    assert [dwb['acc_lim_x'], dwb['acc_lim_y'], dwb['acc_lim_theta']] == smoother['max_accel']
    decel = [dwb['decel_lim_x'], dwb['decel_lim_y'], dwb['decel_lim_theta']]
    assert decel == smoother['max_decel']
    assert dwb['acc_lim_theta'] == behaviors['rotational_acc_lim']
    assert behaviors['min_rotational_vel'] < behaviors['max_rotational_vel']


def test_goal_tolerances_agree():
    """Verify that DWB's RotateToGoal and the goal checker use the same tolerances."""
    for file_name, checker in (('nav2_params.yaml', 'goal_checker'),
                               ('nav2_params_humble.yaml', 'general_goal_checker')):
        controller = _nav2(file_name)['controller_server']['ros__parameters']
        for key in ('xy_goal_tolerance', 'yaw_goal_tolerance'):
            assert controller[checker][key] == controller['FollowPath'][key], (file_name, key)


def test_foxy_and_humble_match():
    """Verify that the robot's own values are the same in the Foxy and Humble files."""
    foxy, humble = (_nav2(f) for f in NAV2_FILES)
    for key in ('max_vel_x', 'max_vel_theta', 'max_speed_xy', 'acc_lim_x', 'acc_lim_theta',
                'decel_lim_x', 'decel_lim_theta', 'xy_goal_tolerance', 'yaw_goal_tolerance'):
        assert (foxy['controller_server']['ros__parameters']['FollowPath'][key]
                == humble['controller_server']['ros__parameters']['FollowPath'][key]), key
    for name in COSTMAPS:
        for key in ('robot_radius', 'resolution', 'robot_base_frame', 'global_frame'):
            assert _costmap(foxy, name)[key] == _costmap(humble, name)[key], (name, key)
        assert _costmap(foxy, name)['inflation_layer'] == _costmap(humble, name)['inflation_layer']


if __name__ == '__main__':
    test_nav2_yaml_validity()
    test_footprint_covers_the_chassis()
    test_speed_limits_agree()
    test_goal_tolerances_agree()
    test_foxy_and_humble_match()
