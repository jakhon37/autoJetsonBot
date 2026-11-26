"""
Camera JPEG Encoder Tests
Checks tools/camera_jpeg.py on made-up frames, so no camera is needed. Needs numpy and OpenCV.
The GStreamer tests also need GStreamer's Python bindings, with jpegenc standing in for the
Jetson's nvjpegenc; the nvjpegenc test runs only on a Jetson. Each is skipped where it can't run.
"""
import os
import sys
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))

try:
    import cv2
    import numpy as np
    import camera_jpeg
except ImportError:  # this machine has no numpy or OpenCV
    camera_jpeg = None


def gst_elements(*names):
    """Return True if GStreamer's Python bindings and all these elements are here."""
    try:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
    except (ImportError, ValueError):
        return False
    if not Gst.is_initialized():
        Gst.init(None)
    return all(Gst.ElementFactory.find(name) is not None for name in names)


HAVE_GST = camera_jpeg is not None and gst_elements("appsrc", "appsink", "jpegenc")
HAVE_NVJPEG = camera_jpeg is not None and gst_elements("appsrc", "appsink", "nvjpegenc")
W, H = 820, 616


def test_frame():
    """A frame with a black-to-white ramp on top, colour blocks and a smooth gradient."""
    y, x = np.mgrid[0:H, 0:W].astype(np.float32)
    frame = np.dstack([x / W * 255, y / H * 255, (1 - x / W) * 200 + 30]).astype(np.uint8)
    ramp = np.tile(np.linspace(0, 255, W).astype(np.uint8), (40, 1))
    frame[:40] = cv2.merge([ramp, ramp, ramp])
    frame[100:200, 100:300] = (0, 0, 255)
    frame[100:200, 400:600] = (255, 0, 0)
    return frame


def psnr(a, b):
    mse = np.mean((a.astype(np.float64) - b.astype(np.float64)) ** 2)
    return 10 * np.log10(255 ** 2 / mse)


@unittest.skipIf(camera_jpeg is None, "needs numpy and OpenCV")
class TestI420(unittest.TestCase):

    def test_layout_matches_gstreamer(self):
        """The plane layout is GStreamer's: rows padded to 4 bytes (measured on the Jetson)."""
        self.assertEqual(camera_jpeg.i420_layout(820, 616),
                         ((820, 412, 412), (0, 505120, 632016), 758912))
        self.assertEqual(camera_jpeg.i420_layout(616, 820),
                         ((616, 308, 308), (0, 505120, 631400), 757680))

    @unittest.skipUnless(HAVE_GST, "needs GStreamer's Python bindings")
    def test_layout_matches_gstvideo(self):
        import gi
        gi.require_version("GstVideo", "1.0")
        from gi.repository import GstVideo
        for w, h in ((820, 616), (616, 820), (640, 480), (102, 76), (1640, 1232)):
            info = GstVideo.VideoInfo()
            info.set_format(GstVideo.VideoFormat.I420, w, h)
            expected = (tuple(info.stride[:3]), tuple(info.offset[:3]), info.size)
            self.assertEqual(camera_jpeg.i420_layout(w, h), expected, f"{w}x{h}")

    def test_full_range_planes(self):
        """Y is JPEG's luma over the whole frame, Cb and Cr its 2x2 averages, both 0 to 255."""
        frame = test_frame()
        packer = camera_jpeg.FullRangeI420(W, H)
        data = np.frombuffer(packer.pack(frame), np.uint8)
        (sy, su, sv), (oy, ou, ov), size = camera_jpeg.i420_layout(W, H)
        self.assertEqual(data.size, size)
        y = data[oy:oy + sy * H].reshape(H, sy)[:, :W]
        u = data[ou:ou + su * (H // 2)].reshape(H // 2, su)
        v = data[ov:ov + sv * (H // 2)].reshape(H // 2, sv)
        ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb).astype(np.float32)
        # OpenCV's grey and YCrCb's Y use the same weights and round apart in a few pixels
        self.assertLessEqual(np.abs(y - ycrcb[:, :, 0]).max(), 1.0)
        half = ycrcb.reshape(H // 2, 2, W // 2, 2, 3).mean(axis=(1, 3))
        self.assertLessEqual(np.abs(u[:, :W // 2] - half[:, :, 2]).max(), 1.0)
        self.assertLessEqual(np.abs(v[:, :W // 2] - half[:, :, 1]).max(), 1.0)
        self.assertEqual(int(u[:, W // 2:].max()), 0)  # the padding stays empty
        # full range: black is 0 and white 255, not video range's 16 and 235
        self.assertEqual(int(y[:40, 0].max()), 0)
        self.assertEqual(int(y[:40, -1].min()), 255)
        # grey has no colour
        grey = np.full((H, W, 3), 99, np.uint8)
        data = np.frombuffer(packer.pack(grey), np.uint8)
        self.assertEqual(set(data[ou:ou + su * (H // 2)].reshape(H // 2, su)[:, :W // 2].ravel()),
                         {128})

    def test_refuses_odd_size_and_wrong_frame(self):
        with self.assertRaises(camera_jpeg.EncodeError):
            camera_jpeg.FullRangeI420(821, 616)
        packer = camera_jpeg.FullRangeI420(W, H)
        with self.assertRaises(camera_jpeg.EncodeError):
            packer.pack(np.zeros((H, W + 2, 3), np.uint8))


@unittest.skipIf(camera_jpeg is None, "needs numpy and OpenCV")
class TestEncoders(unittest.TestCase):

    def check_jpeg(self, jpeg, frame, min_psnr=30.0):
        out = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(out.shape, frame.shape)
        self.assertGreater(psnr(out, frame), min_psnr)
        ramp = out[5:35, :, 1].astype(np.float32)
        self.assertLess(ramp[:, :4].mean(), 4)      # black stays black ...
        self.assertGreater(ramp[:, -4:].mean(), 251)  # ... and white white

    def test_software(self):
        frame = test_frame()
        enc, note = camera_jpeg.open_encoder("software", (W, H), 80, 15)
        self.assertEqual(enc.kind, "software")
        self.check_jpeg(enc.encode(frame), frame)

    @unittest.skipUnless(HAVE_GST, "needs GStreamer's Python bindings and jpegenc")
    def test_gstreamer_round_trip(self):
        """Frames go through appsrc and jpegenc in order, and decode in full range."""
        enc = camera_jpeg.GstJpeg((W, H), 80, 15, element="jpegenc")
        try:
            for shift in (0, 60, 120):
                frame = np.roll(test_frame(), shift, axis=1)
                frame[:40] = test_frame()[:40]
                self.check_jpeg(enc.encode(frame), frame)
        finally:
            enc.close()

    @unittest.skipUnless(HAVE_NVJPEG, "needs a Jetson's nvjpegenc")
    def test_jetson_hardware(self):
        frame = test_frame()
        enc, note = camera_jpeg.open_encoder("hardware", (W, H), 80, 15)
        try:
            self.assertEqual(enc.kind, "hardware")
            self.check_jpeg(enc.encode(frame), frame)
        finally:
            enc.close()

    def test_auto_falls_back_to_software(self):
        """Without a working hardware encoder, auto takes OpenCV and hardware refuses."""
        enc, note = camera_jpeg.open_encoder("auto", (W, H), 80, 15, element="nosuchjpegenc")
        self.assertEqual(enc.kind, "software")
        self.assertIn("software", note)
        with self.assertRaises(camera_jpeg.EncodeError):
            camera_jpeg.open_encoder("hardware", (W, H), 80, 15, element="nosuchjpegenc")
        # an odd size can't be 4:2:0, so auto takes OpenCV for it too
        enc, note = camera_jpeg.open_encoder("auto", (W + 1, H), 80, 15)
        self.assertEqual(enc.kind, "software")


def server_module():
    """Import tools/csi_camera_server.py, which needs GStreamer's Python bindings, or None."""
    if not HAVE_GST:
        return None
    try:
        import csi_camera_server
    except (ImportError, ValueError):
        return None
    return csi_camera_server


@unittest.skipIf(server_module() is None, "needs GStreamer's Python bindings")
class TestServerFallback(unittest.TestCase):

    class Broken:
        kind = "hardware"
        closed = False

        def encode(self, bgr):
            raise camera_jpeg.EncodeError("the engine stopped answering")

        def close(self):
            self.closed = True

    def camera(self, encoder):
        """A camera server's Camera without its GStreamer pipeline, for _encode alone."""
        camera = server_module().Camera.__new__(server_module().Camera)
        camera.encoder, camera.quality, camera.jpeg_note = encoder, 80, "hardware"
        return camera

    def test_hardware_failure_falls_back_to_software(self):
        """A frame the hardware fails on still goes out, and later frames go to OpenCV."""
        broken = self.Broken()
        camera = self.camera(broken)
        frame = test_frame()
        jpeg = camera._encode(frame)
        self.assertTrue(jpeg.startswith(b"\xff\xd8"))
        self.assertTrue(broken.closed)
        self.assertEqual(camera.encoder.kind, "software")
        self.assertIn("stopped answering", camera.jpeg_note)
        self.assertTrue(camera._encode(frame).startswith(b"\xff\xd8"))

    def test_software_failure_skips_the_frame(self):
        broken = self.Broken()
        broken.kind = "software"
        self.assertIsNone(self.camera(broken)._encode(test_frame()))


if __name__ == "__main__":
    unittest.main()
