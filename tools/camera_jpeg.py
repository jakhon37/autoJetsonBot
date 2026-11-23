"""JPEG encoders for csi_camera_server.py: the Jetson's JPEG engine, or OpenCV on the CPU.

The camera server corrects each frame's colour on the CPU (camera_shading.py), so the frames
can't stay in the video hardware's memory on their way to the encoder. GstJpeg hands them to
nvjpegenc, which encodes on the Jetson's JPEG engine (NVJPG), through a small GStreamer pipeline:
appsrc ! nvjpegenc ! appsink. SoftwareJpeg is OpenCV's libjpeg, the fallback.

nvjpegenc writes the YCbCr samples it is given, and JPEG decoders read them as full range, 0 to
255, as JFIF defines it. nvvidconv's RGB to YUV conversion gives video range, 16 to 235, whatever
colorimetry its caps ask for (tried on the robot), so its frames came out washed out: black at 16
and white at 235. So FullRangeI420 converts each frame here with JPEG's own weights, which
OpenCV's YCrCb uses.

Needs numpy and OpenCV. GstJpeg also needs GStreamer's Python bindings, which JetPack has.
"""

import cv2
import numpy as np

KINDS = ('auto', 'hardware', 'software')


class EncodeError(RuntimeError):
    """An encoder couldn't start or didn't deliver a JPEG."""


def round_up(value, step):
    return (value + step - 1) // step * step


def i420_layout(width, height):
    """
    Return GStreamer's default I420 layout: (strides, offsets, size) of the Y, U and V planes.

    The same rule as gst-plugins-base's video-info.c: rows padded to 4 bytes, chroma at half the
    width and height, rounded up.
    """
    chroma_h = round_up(height, 2) // 2
    strides = (round_up(width, 4), round_up(round_up(width, 2) // 2, 4))
    strides = (strides[0], strides[1], strides[1])
    offsets = (0, strides[0] * round_up(height, 2))
    offsets = offsets + (offsets[1] + strides[1] * chroma_h,)
    return strides, offsets, offsets[2] + strides[2] * chroma_h


class FullRangeI420:
    """Lay BGR frames out as JPEG's full-range YCbCr 4:2:0, in GStreamer's I420 layout."""

    def __init__(self, width, height):
        if width % 2 or height % 2:
            raise EncodeError(f'4:2:0 needs an even width and height, not {width}x{height}')
        self.size = (width, height)
        strides, offsets, total = i420_layout(width, height)
        self.buf = np.zeros(total, np.uint8)
        half = (height // 2, width // 2)

        def plane(i, rows, cols):
            start = offsets[i]
            return self.buf[start:start + strides[i] * rows].reshape(rows, strides[i])[:, :cols]

        self.y = plane(0, height, width)
        self.u = plane(1, *half)
        self.v = plane(2, *half)

    def pack(self, bgr):
        """Return the frame as I420 bytes."""
        w, h = self.size
        if bgr.shape[:2] != (h, w):
            raise EncodeError(f'expected {w}x{h} frames, got {bgr.shape[1]}x{bgr.shape[0]}')
        # Y from the whole frame; Cb and Cr from its 2x2 averages. The conversion is linear, so
        # averaging before it is the same as after, and four times less work.
        self.y[:] = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)  # the same weights as YCrCb's Y
        half = cv2.resize(bgr, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
        ycrcb = cv2.cvtColor(half, cv2.COLOR_BGR2YCrCb)
        self.u[:] = ycrcb[:, :, 2]
        self.v[:] = ycrcb[:, :, 1]
        return self.buf.tobytes()


class SoftwareJpeg:
    """OpenCV's libjpeg, on the CPU."""

    kind = 'software'

    def __init__(self, quality):
        self.params = [cv2.IMWRITE_JPEG_QUALITY, quality]

    def encode(self, bgr):
        ok, jpeg = cv2.imencode('.jpg', bgr, self.params)
        if not ok:
            raise EncodeError('OpenCV could not encode the frame')
        return jpeg.tobytes()

    def close(self):
        pass


class GstJpeg:
    """
    A GStreamer JPEG element fed from Python: nvjpegenc, the Jetson's JPEG engine, by default.

    encode() pushes one frame and waits for its JPEG, so each JPEG belongs to the frame just
    pushed. Any element that takes I420 works; the tests use GStreamer's software jpegenc.
    """

    kind = 'hardware'

    def __init__(self, size, quality, fps, element='nvjpegenc', timeout_s=1.0):
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import GLib, Gst
        if not Gst.is_initialized():
            Gst.init(None)
        self.Gst = Gst
        self.element = element
        self.packer = FullRangeI420(*size)
        self.timeout = int(timeout_s * Gst.SECOND)
        try:
            self.pipeline = Gst.parse_launch(
                f'appsrc name=src format=time is-live=true ! {element} quality={quality} ! '
                'appsink name=sink sync=false')
        except GLib.Error as exc:
            raise EncodeError(f'{element}: {exc.message}') from None
        self.src = self.pipeline.get_by_name('src')
        self.sink = self.pipeline.get_by_name('sink')
        self.src.set_property('caps', Gst.Caps.from_string(
            f'video/x-raw,format=I420,width={size[0]},height={size[1]},framerate={fps}/1'))
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.close()
            raise EncodeError(f'{element} did not start')
        try:
            jpeg = self.encode(np.full((size[1], size[0], 3), 128, np.uint8))
        except EncodeError:
            self.close()
            raise
        if not jpeg.startswith(b'\xff\xd8'):
            self.close()
            raise EncodeError(f'{element} gave something that is not a JPEG')

    def encode(self, bgr):
        Gst = self.Gst
        flow = self.src.emit('push-buffer', Gst.Buffer.new_wrapped(self.packer.pack(bgr)))
        if flow != Gst.FlowReturn.OK:
            raise EncodeError(f'{self.element} refused the frame ({flow.value_nick}){self._bus()}')
        sample = self.sink.emit('try-pull-sample', self.timeout)
        if sample is None:
            raise EncodeError(f'{self.element} gave no JPEG within '
                              f'{self.timeout / Gst.SECOND:g} s{self._bus()}')
        buf = sample.get_buffer()
        return buf.extract_dup(0, buf.get_size())

    def _bus(self):
        msg = self.pipeline.get_bus().pop_filtered(self.Gst.MessageType.ERROR)
        return f': {msg.parse_error()[0].message}' if msg is not None else ''

    def close(self):
        self.pipeline.set_state(self.Gst.State.NULL)


def open_encoder(kind, size, quality, fps, element='nvjpegenc'):
    """
    Return (encoder, note). 'auto' takes the hardware encoder if it starts, else OpenCV's.

    'hardware' raises EncodeError if the element doesn't start; 'software' never tries it.
    """
    if kind == 'software':
        return SoftwareJpeg(quality), 'software, as asked'
    try:
        return GstJpeg(size, quality, fps, element), f'hardware ({element})'
    except (EncodeError, ImportError, ValueError) as exc:
        if kind == 'hardware':
            raise EncodeError(str(exc)) from None
        return SoftwareJpeg(quality), f'software, since {element} did not start: {exc}'
