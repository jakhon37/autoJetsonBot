"""
Map Saving Tests
Checks tools/save_map.py with a stand-in for the map saver, so no ROS is needed. Needs PyYAML,
which the Jetson's Python and the ROS container have; skipped where it's missing.
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))

try:
    import save_map
except ImportError:  # this machine has no PyYAML
    save_map = None

YAML = ("image: {name}.pgm\nmode: trinary\nresolution: 0.05\norigin: [-2.96, -3.03, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: {free}\n")


def write_map(path, free_thresh=0.196):
    """Write a 4 x 2 map as map_saver does: 1 occupied (0), 3 unknown (205) and 4 free (254)."""
    with open(path + ".yaml", "w") as f:
        f.write(YAML.format(name=os.path.basename(path), free=free_thresh))
    with open(path + ".pgm", "wb") as f:
        f.write(b"P5\n# CREATOR: test\n4 2\n255\n" + bytes([0, 205, 205, 205, 254, 254, 254, 254]))


@unittest.skipIf(save_map is None, "needs PyYAML")
class TestSaveMap(unittest.TestCase):
    """Test that save_map.py reports a save as saved only when the files are right"""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir)
        self.path = os.path.join(self.dir, "lab_map_test")
        self.real_call_saver = save_map.call_saver
        self.addCleanup(setattr, save_map, "call_saver", self.real_call_saver)

    def run_with_saver(self, answer, writes=True, free_thresh=0.196):
        def fake_call_saver(path):
            if writes:
                write_map(path, free_thresh)
            return answer
        save_map.call_saver = fake_call_saver
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = save_map.main(["save_map.py", self.path])
        return code, out.getvalue() + err.getvalue()

    def test_saved_map_is_counted(self):
        """Test that a save prints the map's size and cells"""
        code, text = self.run_with_saver(None)
        self.assertEqual(code, 0, text)
        self.assertIn("4 x 2 cells of 0.05 m, 1 occupied, 4 free and 3 unknown", text)

    def test_failed_save_is_reported(self):
        """Test that a save the saver failed exits 1"""
        code, text = self.run_with_saver("The map saver failed.", writes=False)
        self.assertEqual(code, 1)
        self.assertIn("Map not saved", text)

    def test_lost_answer_with_files_written_is_a_save(self):
        """Test that a save whose answer was lost counts when both files were written"""
        code, text = self.run_with_saver("no answer")
        self.assertEqual(code, 0, text)
        self.assertIn("but wrote both files", text)

    def test_lost_answer_without_files_is_a_failure(self):
        """Test that no answer and no files exits 1"""
        code, text = self.run_with_saver("no answer", writes=False)
        self.assertEqual(code, 1)
        self.assertIn("no answer", text)

    def test_old_files_are_not_a_save(self):
        """Test that files from an earlier save don't count when the saver claims success"""
        write_map(self.path)
        old = os.path.getmtime(self.path + ".yaml") - 60
        for ext in (".yaml", ".pgm"):
            os.utime(self.path + ext, (old, old))
        code, text = self.run_with_saver(None, writes=False)
        self.assertEqual(code, 1)
        self.assertIn("not both written", text)

    def test_free_thresh_that_loads_unknown_as_free_fails(self):
        """Test that a map saved with map_saver's default free_thresh of 0.25 exits 1"""
        code, text = self.run_with_saver(None, free_thresh=0.25)
        self.assertEqual(code, 1)
        self.assertIn("loads unknown cells as free", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
