"""
Camera Calibration Tool Tests
Checks tools/camera_intrinsics.py without a camera: the printed board against OpenCV's own
drawing of it, detection, both lens models on simulated views, the floor check's geometry and
the focus measure. Needs OpenCV with its ArUco module, as in the ROS container; skipped elsewhere.
"""
import math
import os
import re
import sys
import tempfile
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))

try:
    import cv2
    import numpy as np
    import yaml
    if not hasattr(cv2, "aruco"):
        raise ImportError("no ArUco module")
    import camera_intrinsics as ci
except ImportError:
    ci = None

SIZE = (820, 616)


def rotation(roll, pitch, yaw):
    """URDF's fixed-axis roll, pitch, yaw as a matrix."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def one_to_one_reach(model, D):
    """The angle off the axis, as tan, out to which the model maps radius one to one."""
    r = np.linspace(1e-4, 5.0, 5000)
    if model == "equidistant":
        t = np.arctan(r)
        rd = t * (1 + D[0] * t ** 2 + D[1] * t ** 4 + D[2] * t ** 6 + D[3] * t ** 8)
    else:
        k1, k2, p1, p2, k3, k4, k5, k6 = D
        r2 = r * r
        rd = r * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) / (1 + k4 * r2 + k5 * r2 ** 2
                                                                + k6 * r2 ** 3)
    falling = np.flatnonzero(np.diff(rd) <= 0)
    return r[falling[0]] if len(falling) else r[-1]


def simulated_views(board, model, K, D, count=30, seed=1, noise=0.1):
    """
    Project the board's corners from varied poses, keeping those inside the picture.

    Corners beyond the angle where the model folds back are left out: a real lens doesn't
    bring them back into the picture.
    """
    rng = np.random.default_rng(seed)
    w, h = SIZE
    reach = 0.95 * one_to_one_reach(model, D)
    centre = board.corners3d.mean(axis=0)
    views = []
    while len(views) < count:
        R = rotation(*rng.uniform(-0.6, 0.6, 3))
        dist = rng.uniform(0.22, 0.5)
        # aim at a random spot in the picture, out to its corners
        target = ci.to_normalized(model, K, D, rng.uniform((20, 20), (w - 20, h - 20))[None])[0]
        t = np.array([target[0], target[1], 1.0]) * dist - R @ centre
        rvec, _ = cv2.Rodrigues(R)
        pts = ci.project(model, K, D, board.corners3d, rvec, t)
        cam = (R @ board.corners3d.T).T + t
        inside = ((pts[:, 0] > 2) & (pts[:, 0] < w - 3) & (pts[:, 1] > 2) & (pts[:, 1] < h - 3)
                  & (cam[:, 2] > 0.05)
                  & (np.hypot(cam[:, 0], cam[:, 1]) < reach * np.maximum(cam[:, 2], 1e-6)))
        if inside.sum() < 12:
            continue
        ids = np.flatnonzero(inside)
        views.append((pts[inside] + rng.normal(0, noise, (len(ids), 2)), ids))
    return views


@unittest.skipIf(ci is None, "needs OpenCV with ArUco, numpy and PyYAML")
class TestTarget(unittest.TestCase):

    def test_printed_board_matches_opencv(self):
        """The PDF's black shapes, drawn at 2 px/mm, are OpenCV's own picture of the board."""
        board = ci.Board()
        bw, bh = board.size_mm()
        scale = 2
        drawn = board.board.draw((int(bw * scale), int(bh * scale)), marginSize=0, borderBits=1)
        ours = np.full_like(drawn, 255)
        for x, y, w, h in board.rectangles_mm():
            x0, x1 = round(x * scale), round((x + w) * scale)
            y0, y1 = round((bh - y - h) * scale), round((bh - y) * scale)  # y up to rows down
            ours[y0:y1, x0:x1] = 0
        self.assertEqual(int(np.count_nonzero(ours != drawn)), 0)

    def test_board_is_found_on_its_own_picture(self):
        board = ci.Board()
        img = board.board.draw((1024, 640), marginSize=40, borderBits=1)
        points, ids = board.detect(img)
        self.assertEqual(sorted(ids.tolist()), list(range(28)))

    def test_pdf(self):
        """A one-page PDF whose cross-reference table points at its objects."""
        board = ci.Board()
        pdf = ci.target_pdf(board)
        self.assertTrue(pdf.startswith(b"%PDF-1.4"))
        start = int(re.search(rb"startxref\n(\d+)", pdf).group(1))
        self.assertTrue(pdf[start:].startswith(b"xref"))
        offsets = [int(m) for m in re.findall(rb"(\d{10}) 00000 n", pdf)]
        for i, offset in enumerate(offsets, 1):
            self.assertTrue(pdf[offset:].startswith(b"%d 0 obj" % i))
        self.assertEqual(pdf.count(b" re\n"), len(board.rectangles_mm()))
        width = float(re.search(rb"/MediaBox \[0 0 ([\d.]+)", pdf).group(1))
        self.assertAlmostEqual(width / ci.PT_PER_MM, 297.0, places=2)


@unittest.skipIf(ci is None, "needs OpenCV with ArUco, numpy and PyYAML")
class TestLensModels(unittest.TestCase):

    K = np.array([[330.0, 0, 405.0], [0, 331.0, 312.0], [0, 0, 1]])

    def test_fisheye_lens(self):
        """On a strong fisheye the equidistant model is found again and chosen."""
        board = ci.Board()
        D = np.array([0.04, -0.02, 0.006, -0.001])
        views = simulated_views(board, "equidistant", self.K, D)
        reports = ci.compare(board, views, SIZE)
        fish = reports["equidistant"]
        np.testing.assert_allclose(fish["fit"]["K"], self.K, rtol=0.01, atol=1.0)
        self.assertLess(fish["held_out_rms"], 0.2)
        self.assertTrue(fish["one_to_one"])
        self.assertLess(fish["bend_median"], 0.2)  # the simulated noise is 0.1 px
        self.assertEqual(ci.choose(reports), "equidistant")

    def test_rational_lens(self):
        """
        On a lens with a rational distortion, the rational model is found again.

        Fitted on all views it matches them to the simulated noise. Left out of a fit, corners
        near the edges come out worse, as the eight terms extrapolate poorly where few views
        reach: that is what the comparison on views left out is there to catch.
        """
        board = ci.Board()
        # reaches the picture's corners about 65 deg off the axis, and stays one to one there
        D = np.array([-0.2, 0.04, 0.0005, -0.0003, -0.002, 0.0, 0.0, 0.0])
        views = simulated_views(board, "rational_polynomial", self.K, D)
        reports = ci.compare(board, views, SIZE)
        rat = reports["rational_polynomial"]
        np.testing.assert_allclose(rat["fit"]["K"], self.K, rtol=0.01, atol=1.0)
        self.assertLess(rat["rms_all_views"], 0.2)
        self.assertLess(rat["held_out_rms"], 1.0)
        self.assertTrue(rat["one_to_one"])

    def test_choice(self):
        """The model with the smaller error on views left out wins, if it stays one to one."""
        def report(err, one_to_one=True):
            return {"held_out_rms": err, "one_to_one": one_to_one}
        self.assertEqual(ci.choose({"rational_polynomial": report(0.3),
                                    "equidistant": report(0.4)}), "rational_polynomial")
        self.assertEqual(ci.choose({"rational_polynomial": report(0.3, False),
                                    "equidistant": report(0.4)}), "equidistant")
        self.assertEqual(ci.choose({"rational_polynomial": {"failed": "no fit"},
                                    "equidistant": report(0.4)}), "equidistant")
        with self.assertRaises(ValueError):
            ci.choose({"rational_polynomial": report(0.3, False),
                       "equidistant": {"failed": "no fit"}})

    def test_camera_info_yaml(self):
        D = np.array([0.04, -0.02, 0.006, -0.001])
        text = ci.camera_info_yaml("equidistant", self.K, D, SIZE, note="a note\nsecond line")
        data = yaml.safe_load(text)
        self.assertEqual((data["image_width"], data["image_height"]), SIZE)
        self.assertEqual(data["distortion_model"], "equidistant")
        self.assertEqual(data["distortion_coefficients"]["data"], [0.04, -0.02, 0.006, -0.001])
        self.assertEqual(len(data["projection_matrix"]["data"]), 12)
        self.assertTrue(text.startswith("# a note\n# second line\n"))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "cam.yaml")
            with open(path, "w") as f:
                f.write(text)
            model, K, D2, size = ci.read_camera_info(path)
        self.assertEqual(model, "equidistant")
        np.testing.assert_allclose(K, self.K)
        np.testing.assert_allclose(D2, D)


@unittest.skipIf(ci is None, "needs OpenCV with ArUco, numpy and PyYAML")
class TestMounting(unittest.TestCase):

    def test_floor_pose(self):
        """A camera 12 cm up, looking down 18 deg, is found from the board on the floor."""
        board = ci.Board()
        K = np.array([[330.0, 0, 410.0], [0, 330.0, 308.0], [0, 0, 1]])
        D = np.array([0.04, -0.02, 0.006, -0.001])
        truth = dict(x=0.142, y=0.004, z=0.121, roll=math.radians(1.0),
                     pitch=math.radians(18.0), yaw=math.radians(-2.0))
        distance, offset = 0.20, 0.010
        R_r_link = rotation(truth["roll"], truth["pitch"], truth["yaw"])
        R_r_opt = R_r_link @ ci.OPTICAL_TO_LINK.T
        cam = np.array([truth["x"], truth["y"], truth["z"]])
        bw = board.squares[0] * board.square_mm / 1000
        R_rb = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float)
        t_rb = np.array([distance, bw / 2 + offset, 0.0])
        corners_r = (R_rb @ board.corners3d.T).T + t_rb
        corners_opt = (R_r_opt.T @ (corners_r - cam).T).T
        pts = ci.project("equidistant", K, D, corners_opt, np.zeros((3, 1)), np.zeros((3, 1)))
        pose = ci.floor_mounting(board, "equidistant", K, D, pts, np.arange(len(pts)),
                                 distance, offset)
        for key in ("x", "y", "z"):
            self.assertAlmostEqual(pose[key], truth[key], delta=0.001, msg=key)
        for key in ("roll", "pitch", "yaw"):
            self.assertAlmostEqual(math.degrees(pose[key]), math.degrees(truth[key]), delta=0.1,
                                   msg=key)
        self.assertLess(pose["rms"], 0.01)
        # the plane alone gives the same height and tilt, from either end of the board
        for order in (np.arange(len(pts)), np.arange(len(pts))[::-1]):
            tilt = ci.floor_tilt(board, "equidistant", K, D, pts[order], np.arange(len(pts)),
                                 0.007)
            self.assertAlmostEqual(tilt["z"], truth["z"] + 0.007, delta=0.001)
            self.assertAlmostEqual(math.degrees(tilt["pitch"]), 18.0, delta=0.1)
            self.assertAlmostEqual(math.degrees(tilt["roll"]), 1.0, delta=0.1)

    def test_level_tilt(self):
        """A board standing level, turned to several angles, gives the camera's pitch and roll."""
        board = ci.make_board("chessboard:8x6", 20)
        K = np.array([[450.0, 0, 410.0], [0, 450.0, 308.0], [0, 0, 1]])
        D = np.zeros(8)
        R_r_opt = rotation(math.radians(2.3), math.radians(7.0), 0.0) @ ci.OPTICAL_TO_LINK.T
        views = []
        for yaw in (-50, -20, 0, 15, 40):
            # the board upright, its rows level and its face leaning back 20 deg, 0.6 m ahead
            yr, lean = math.radians(yaw), math.radians(20)
            upright = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)  # face toward -x
            R_r_board = rotation(0, 0, yr) @ upright @ rotation(lean, 0, 0)
            centre = np.array([0.6, 0.0, 0.05])
            mid = board.corners3d.mean(axis=0)
            pts_r = (R_r_board @ (board.corners3d - mid).T).T + centre
            pts_opt = (R_r_opt.T @ pts_r.T).T
            pts = ci.project("rational_polynomial", K, D, pts_opt, np.zeros((3, 1)),
                             np.zeros((3, 1)))
            views.append((pts, np.arange(len(pts))))
        tilt = ci.level_tilt(board, "rational_polynomial", K, D, views)
        self.assertAlmostEqual(math.degrees(tilt["pitch"]), 7.0, delta=0.05)
        self.assertAlmostEqual(math.degrees(tilt["roll"]), 2.3, delta=0.05)
        self.assertLess(max(tilt["off_level"]), 0.05)
        self.assertGreater(tilt["spread"], 100 * tilt["flat"])


@unittest.skipIf(ci is None, "needs OpenCV with ArUco, numpy and PyYAML")
class TestFocus(unittest.TestCase):

    def test_sharpness_falls_with_blur_and_ignores_brightness(self):
        board = ci.Board()
        img = board.board.draw((820, 616), marginSize=30, borderBits=1).astype(np.float32)
        img = cv2.resize(img, (820, 616), interpolation=cv2.INTER_AREA)
        values = []
        for sigma in (0.0, 1.0, 2.0, 3.0):
            blurred = cv2.GaussianBlur(img, (0, 0), sigma) if sigma else img
            values.append(ci.sharpness(np.clip(blurred, 0, 255).astype(np.uint8))[1])
        self.assertEqual(values, sorted(values, reverse=True))
        self.assertGreater(values[0], 1.5 * values[2])
        dim = ci.sharpness((img * 0.5).astype(np.uint8))[1]
        self.assertAlmostEqual(dim / values[0], 1.0, delta=0.05)

    def test_coverage_and_new_views(self):
        board = ci.Board()
        a = (np.array([[60.0, 60], [120, 60], [60, 120], [120, 120]] * 3), np.arange(12))
        self.assertTrue(ci.is_new_view(a[0], [], SIZE))
        self.assertFalse(ci.is_new_view(a[0] + 2, [a], SIZE))
        self.assertTrue(ci.is_new_view(a[0] + (500, 300), [a], SIZE))
        grid = ci.coverage([a], SIZE)
        self.assertEqual(grid.sum(), 12)
        self.assertEqual(grid[0, 0], 12)
        self.assertEqual(board.cols, 7)


if __name__ == "__main__":
    unittest.main()
