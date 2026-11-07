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
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _assert_yaml_valid(os.path.join(package_path, 'config', 'nav2_params.yaml'))
    _assert_yaml_valid(os.path.join(package_path, 'config', 'nav2_params_humble.yaml'))


if __name__ == '__main__':
    test_nav2_yaml_validity()
