"""Unit tests for reading camera_info YAML. camera_info_file.py imports no ROS."""
import glob
import os
import sys

import pytest

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PACKAGE)

from jetson_bot_camera import camera_info_file  # noqa: E402

SAMPLE = """# a comment, as tools/camera_intrinsics.py writes
image_width: 820
image_height: 616
camera_name: imx219_wide
camera_matrix:
  rows: 3
  cols: 3
  data: [330.1, 0, 410.2, 0, 330.5, 308.3, 0, 0, 1]
distortion_model: equidistant
distortion_coefficients:
  rows: 1
  cols: 4
  data: [0.01, -0.02, 0.003, -0.0004]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [250, 0, 410, 0, 0, 250, 308, 0, 0, 0, 1, 0]
"""


def _write(tmp_path, text):
    path = tmp_path / 'camera.yaml'
    path.write_text(text)
    return str(path)


def test_load(tmp_path):
    """Every field comes out as CameraInfo names it."""
    info = camera_info_file.load(_write(tmp_path, SAMPLE))
    assert (info['width'], info['height']) == (820, 616)
    assert info['distortion_model'] == 'equidistant'
    assert info['d'] == [0.01, -0.02, 0.003, -0.0004]
    assert info['k'][0] == 330.1 and info['k'][2] == 410.2 and len(info['k']) == 9
    assert info['r'] == [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    assert len(info['p']) == 12


def test_wrong_coefficient_count(tmp_path):
    text = SAMPLE.replace('data: [0.01, -0.02, 0.003, -0.0004]', 'data: [0.01, -0.02, 0.003]')
    with pytest.raises(ValueError, match='4 coefficients'):
        camera_info_file.load(_write(tmp_path, text))


def test_not_a_calibration(tmp_path):
    with pytest.raises(ValueError):
        camera_info_file.load(_write(tmp_path, 'hello: 1\n'))
    with pytest.raises(ValueError):
        bad_shape = SAMPLE.replace('rows: 3\n  cols: 4', 'rows: 2\n  cols: 4')
        camera_info_file.load(_write(tmp_path, bad_shape))
    with pytest.raises(OSError):
        camera_info_file.load(str(tmp_path / 'missing.yaml'))


def test_package_calibrations():
    """The calibrations this package installs load, for the stream size the robot uses."""
    for path in glob.glob(os.path.join(PACKAGE, 'config', '*.yaml')):
        info = camera_info_file.load(path)
        assert (info['width'], info['height']) == (820, 616), path
        assert info['distortion_model'] in camera_info_file.COEFFICIENTS
