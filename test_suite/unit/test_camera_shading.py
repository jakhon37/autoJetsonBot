"""
Camera Colour Shading Tests
Checks tools/camera_shading.py on made-up frames, so no camera is needed. Needs numpy and
OpenCV, which the Jetson's Python and the ROS container have; skipped where they're missing.
"""
import os
import sys
import tempfile
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))

try:
    import cv2
    import numpy as np
    import camera_shading
except ImportError:  # this machine has no numpy or OpenCV
    camera_shading = None

W, H = 820, 616


def white_paper(red_rise=0.45, blue_rise=0.12, tint=(1.15, 1.10)):
    """A frame of white paper as the camera gives it: pink toward the edges, a magenta cast
    overall, and darker corners."""
    y, x = np.mgrid[0:H, 0:W].astype(np.float32)
    r2 = ((x - W / 2) / (W / 2)) ** 2 + ((y - H / 2) / (H / 2)) ** 2
    green = 130.0 * (1.0 - 0.3 * r2 / 2)
    red = green * tint[0] * (1.0 + red_rise * r2 / 2)
    blue = green * tint[1] * (1.0 + blue_rise * r2 / 2)
    return np.clip(np.dstack([blue, green, red]), 0, 255)


def mean_ratios(bgr):
    b, g, r = (float(c.mean()) for c in cv2.split(bgr.astype(np.float32)))
    return r / g, b / g


def worst_ratio_error(bgr):
    """Largest red/green or blue/green difference from neutral over a coarse grid."""
    small = cv2.resize(bgr.astype(np.float32), (32, 24), interpolation=cv2.INTER_AREA)
    b, g, r = cv2.split(small)
    return float(max(np.abs(r / g - 1).max(), np.abs(b / g - 1).max()))


@unittest.skipIf(camera_shading is None, "needs numpy and OpenCV")
class TestCameraShading(unittest.TestCase):

    def corrected(self, paper, flip=0, frame=None):
        gain_r, gain_b, _ = camera_shading.measure(paper, flip)
        frame = paper if frame is None else frame
        size = (frame.shape[1], frame.shape[0])
        out = camera_shading.Correction(gain_r, gain_b, 0, size).apply(frame.astype(np.uint8))
        return out

    def test_white_paper_comes_out_neutral_and_even(self):
        """Pink edges and the overall cast both go, and the green channel is left alone."""
        paper = white_paper()
        self.assertGreater(worst_ratio_error(paper), 0.4)
        out = self.corrected(paper)
        self.assertLess(worst_ratio_error(out), 0.04)
        red, blue = mean_ratios(out)
        self.assertAlmostEqual(red, 1.0, delta=0.02)
        self.assertAlmostEqual(blue, 1.0, delta=0.02)
        np.testing.assert_array_equal(out[:, :, 1], paper.astype(np.uint8)[:, :, 1])

    def test_turned_camera(self):
        """Measured on turned frames, the gains still fit the sensor's own orientation."""
        paper = white_paper()
        turned = cv2.rotate(paper, cv2.ROTATE_90_CLOCKWISE)  # what --flip 3 gives
        gain_r, gain_b, _ = camera_shading.measure(turned, 3)
        self.assertEqual(gain_r.shape, (24, 32))
        out = camera_shading.Correction(gain_r, gain_b, 0, (W, H)).apply(paper.astype(np.uint8))
        self.assertLess(worst_ratio_error(out), 0.04)
        out = camera_shading.Correction(gain_r, gain_b, 3, (H, W)).apply(turned.astype(np.uint8))
        self.assertLess(worst_ratio_error(out), 0.04)

    def test_scene_is_refused(self):
        """A room, not paper, is refused rather than turned into a colour map."""
        rng = np.random.default_rng(1)
        blocks = rng.uniform(40, 200, (24, 32, 3)).astype(np.float32)
        scene = cv2.resize(blocks, (W, H), interpolation=cv2.INTER_NEAREST)
        with self.assertRaisesRegex(camera_shading.NotPlainError, "scene"):
            camera_shading.measure(scene, 0)

    def test_clipped_and_dark_are_refused(self):
        with self.assertRaisesRegex(camera_shading.NotPlainError, "full white"):
            camera_shading.measure(np.full((H, W, 3), 255, np.float32), 0)
        with self.assertRaisesRegex(camera_shading.NotPlainError, "dark"):
            camera_shading.measure(np.full((H, W, 3), 5, np.float32), 0)

    def test_save_and_load(self):
        gain_r, gain_b, report = camera_shading.measure(white_paper(), 0)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "shading.json")
            camera_shading.save(path, gain_r, gain_b, 30, report)
            loaded_r, loaded_b = camera_shading.load(path)
        np.testing.assert_allclose(loaded_r, gain_r, atol=0.001)
        np.testing.assert_allclose(loaded_b, gain_b, atol=0.001)


if __name__ == "__main__":
    unittest.main()
