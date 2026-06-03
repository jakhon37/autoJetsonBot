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
import xacro


def test_urdf_parsing():
    """Verify that the XACRO can be correctly parsed into URDF."""
    # Note: We use the local path since the package isn't 'installed' yet during the first test run
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xacro_file = os.path.join(package_path, 'urdf', 'robot.xacro')

    assert os.path.exists(xacro_file), f"XACRO file not found at {xacro_file}"

    try:
        # Check if xacro parsing fails
        robot_description_config = xacro.process_file(xacro_file)
        robot_description_xml = robot_description_config.toxml()
        assert len(robot_description_xml) > 0
        print("✅ URDF Parsing Successful")
    except Exception as e:
        assert False, f"URDF Parsing failed: {str(e)}"


if __name__ == '__main__':
    test_urdf_parsing()
