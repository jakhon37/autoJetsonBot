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


def test_launch_file_exists():
    """Verify that the simulation launch file is present."""
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    launch_file = os.path.join(package_path, 'launch', 'sim.launch.py')

    assert os.path.exists(launch_file), f"Launch file not found at {launch_file}"
    print("✅ Bringup Launch File Found")


if __name__ == '__main__':
    test_launch_file_exists()
