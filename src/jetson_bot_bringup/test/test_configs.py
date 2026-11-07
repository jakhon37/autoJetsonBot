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

import os

import yaml

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


def _params(file_name, node):
    with open(os.path.join(CONFIG_DIR, file_name), 'r') as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def test_ekf_is_the_only_odom_tf_publisher_in_sim():
    """Verify that diff_cont leaves odom -> base_footprint to the EKF, as on the robot."""
    diff_cont = _params('unified_robot_config.yaml', 'diff_cont')
    ekf = _params('ekf.yaml', 'ekf_filter_node')
    assert diff_cont['enable_odom_tf'] is False
    assert ekf['publish_tf'] is True
    assert ekf['world_frame'] == ekf['odom_frame'] == diff_cont['odom_frame_id'] == 'odom'
    assert ekf['base_link_frame'] == diff_cont['base_frame_id'] == 'base_footprint'


if __name__ == '__main__':
    test_ekf_is_the_only_odom_tf_publisher_in_sim()
