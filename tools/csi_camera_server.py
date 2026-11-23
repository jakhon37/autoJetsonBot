#!/usr/bin/env python3
"""Serve the robot's CSI camera as an MJPEG stream on the Jetson host.

Argus, NVIDIA's camera stack, runs on the Jetson host but isn't in the ROS container. So this
script reads the camera here, corrects its colour shading (see camera_shading.py) and encodes
JPEG. In the container, jetson_bot_camera's mjpeg_camera_node turns the stream into ROS images.
robot.sh starts this script when the Jetson has /dev/video0, and stops it with the robot.

Endpoints, on 127.0.0.1 unless --host says otherwise:
  /stream             multipart MJPEG; each part has an X-Timestamp header with the capture
                      time in seconds since the epoch
  /snapshot.jpg       the latest frame; ?raw=1 gives it without the colour correction
  /health             one line of text
  /calibrate          measure the colour shading on the next frames, with white paper over the
                      lens, save it to --color-map and use it (./robot.sh camera-color)
  /shading            how far the colour strays from the middle's, before and after correction

By hand: python3 tools/csi_camera_server.py --flip 3, then on the Jetson
curl -o frame.jpg http://127.0.0.1:8091/snapshot.jpg
"""

import argparse
import collections
import os
import signal
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import camera_shading
import cv2
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst  # noqa: E402

# nvvidconv's flip-method values. 1 and 3 swap the frame's width and height.
FLIPS = {
    0: 'none',
    1: 'rotate 90 deg counter-clockwise',
    2: 'rotate 180 deg',
    3: 'rotate 90 deg clockwise',
    4: 'mirror left-right',
    6: 'mirror top-bottom',
}
# The IMX219's 1640x1232 mode keeps the full field of view and runs at up to 30 fps.
SENSOR_W, SENSOR_H = 1640, 1232
FIRST_FRAME_S = 20.0  # Argus can take a few seconds to start
STALL_S = 10.0
CALIBRATION_FRAMES = 30


class Camera:
    """Run the GStreamer pipeline, correct each frame and keep the latest as JPEG."""

    def __init__(self, sensor_id, width, height, fps, flip, quality, color_map):
        out_w, out_h = (height, width) if flip in (1, 3) else (width, height)
        self.size = (out_w, out_h)
        self.flip = flip
        self.quality = quality
        self.color_map = color_map
        # nvvidconv scales and turns the frames on the Jetson's video hardware and hands them
        # over as BGRx, so the colour correction can work on them.
        self.pipeline = Gst.parse_launch(
            f'nvarguscamerasrc sensor-id={sensor_id} ! '
            f'video/x-raw(memory:NVMM),width={SENSOR_W},height={SENSOR_H},framerate={fps}/1 ! '
            f'nvvidconv flip-method={flip} ! '
            f'video/x-raw,format=BGRx,width={out_w},height={out_h} ! '
            'appsink name=sink max-buffers=1 drop=true sync=false')
        self.sink = self.pipeline.get_by_name('sink')
        self.cond = threading.Condition()
        self.jpeg = None
        self.raw = None
        self.stamp = 0.0
        self.count = 0
        self.error = None
        self.times = collections.deque(maxlen=31)
        self.correction = None
        self.correction_note = 'off'
        self.calibration = None  # [frames wanted, frames summed, sum] while measuring
        self.calibrate_lock = threading.Lock()
        if color_map and os.path.exists(color_map):
            try:
                gain_r, gain_b = camera_shading.load(color_map)
                self.correction = camera_shading.Correction(gain_r, gain_b, flip, self.size)
                self.correction_note = f'on, from {os.path.basename(color_map)}'
            except (OSError, ValueError, KeyError) as exc:
                print(f'colour map {color_map} not used: {exc}', file=sys.stderr, flush=True)

    def run(self, stop):
        """Pull frames until stop is set or the pipeline fails, then release the camera."""
        bus = self.pipeline.get_bus()
        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            while not stop.is_set():
                msg = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if msg is not None:
                    if msg.type == Gst.MessageType.ERROR:
                        err, debug = msg.parse_error()
                        self.error = f'{err.message} ({debug})'
                    else:
                        self.error = 'the pipeline ended'
                    break
                sample = self.sink.emit('try-pull-sample', Gst.SECOND // 2)
                if sample is not None:
                    self._keep(sample)
        finally:
            self.pipeline.set_state(Gst.State.NULL)
            stop.set()
            with self.cond:
                self.cond.notify_all()

    def _keep(self, sample):
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return
        try:
            data = np.frombuffer(info.data, np.uint8).copy()
        finally:
            buf.unmap(info)
        now = time.time()
        stamp = now
        # The buffer's time is in pipeline running time. Its age turns it into wall time.
        # Fall back to now if it looks wrong.
        clock = self.pipeline.get_clock()
        if clock is not None and buf.pts != Gst.CLOCK_TIME_NONE:
            age = (clock.get_time() - self.pipeline.get_base_time() - buf.pts) / Gst.SECOND
            if 0.0 <= age < 1.0:
                stamp = now - age
        w, h = self.size
        stride = data.size // h  # rows can be padded
        raw = cv2.cvtColor(data.reshape(h, stride)[:, :w * 4].reshape(h, w, 4), cv2.COLOR_BGRA2BGR)
        calibration = self.calibration
        if calibration is not None and calibration[1] < calibration[0]:
            calibration[2] += raw
            calibration[1] += 1
        correction = self.correction
        frame = correction.apply(raw) if correction is not None else raw
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        with self.cond:
            self.jpeg, self.raw, self.stamp = jpeg.tobytes(), raw, stamp
            self.count += 1
            self.times.append(now)
            self.cond.notify_all()

    def calibrate(self, frames=CALIBRATION_FRAMES):
        """Measure the colour shading on the next frames, save it and use it. Returns a report."""
        if not self.color_map:
            raise camera_shading.NotPlainError('the server was started without --color-map')
        with self.calibrate_lock:
            w, h = self.size
            self.calibration = [frames, 0, np.zeros((h, w, 3), np.float64)]
            deadline = time.time() + 5.0 + frames / 5.0
            while self.calibration[1] < frames:
                if time.time() > deadline or self.error:
                    self.calibration = None
                    raise camera_shading.NotPlainError('the camera sent too few frames')
                time.sleep(0.05)
            total = self.calibration[2]
            self.calibration = None
            avg = (total / frames).astype(np.float32)
            gain_r, gain_b, report = camera_shading.measure(avg, self.flip)
            camera_shading.save(self.color_map, gain_r, gain_b, frames, report)
            self.correction = camera_shading.Correction(gain_r, gain_b, self.flip, self.size)
            self.correction_note = f'on, measured {time.strftime("%Y-%m-%d %H:%M")}'
            return report

    def fps(self):
        with self.cond:
            if len(self.times) < 2 or self.times[-1] == self.times[0]:
                return 0.0
            return (len(self.times) - 1) / (self.times[-1] - self.times[0])

    def health(self):
        if self.error:
            return f'error: {self.error}'
        with self.cond:
            count, stamp = self.count, self.stamp
        if not count:
            return 'waiting for the first frame'
        return (f'ok frames={count} fps={self.fps():.1f} age={time.time() - stamp:.2f}s '
                f'size={self.size[0]}x{self.size[1]} color={self.correction_note}')


def make_handler(camera, stop):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, body, ctype, code=200):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _text(self, text, code=200):
            self._send((text + '\n').encode(), 'text/plain', code)

        def do_GET(self):
            url = urllib.parse.urlsplit(self.path)
            query = urllib.parse.parse_qs(url.query)
            if url.path == '/health':
                self._text(camera.health())
            elif url.path == '/snapshot.jpg':
                with camera.cond:
                    jpeg, raw = camera.jpeg, camera.raw
                if jpeg is None:
                    self._text('no frame yet', 503)
                elif query.get('raw') == ['1']:
                    ok, body = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, 92])
                    self._send(body.tobytes(), 'image/jpeg')
                else:
                    self._send(jpeg, 'image/jpeg')
            elif url.path == '/calibrate':
                try:
                    report = camera.calibrate()
                    self._text(f'saved to {camera.color_map} and in use; {report}')
                except camera_shading.NotPlainError as exc:
                    self._text(f'not measured: {exc}', 409)
            elif url.path == '/shading':
                with camera.cond:
                    raw = camera.raw
                if raw is None:
                    self._text('no frame yet', 503)
                    return
                lines = [f'as the camera gives it: {camera_shading.spread(raw)}']
                if camera.correction is not None:
                    corrected = camera.correction.apply(raw)
                    lines.append(f'corrected: {camera_shading.spread(corrected)}')
                self._text('\n'.join(lines))
            elif url.path == '/stream':
                self._stream()
            else:
                self._text('not found', 404)

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            seen = 0
            try:
                while not stop.is_set():
                    with camera.cond:
                        camera.cond.wait_for(lambda: camera.count != seen or stop.is_set(), 2.0)
                        if camera.count == seen:
                            continue
                        jpeg, stamp, seen = camera.jpeg, camera.stamp, camera.count
                    head = (f'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}'
                            f'\r\nX-Timestamp: {stamp:.6f}\r\n\r\n')
                    self.wfile.write(head.encode() + jpeg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--host', default='127.0.0.1',
                        help='address to listen on (default: this Jetson only)')
    parser.add_argument('--port', type=int, default=8091)
    parser.add_argument('--sensor-id', type=int, default=0, help='0 is the CAM0 connector')
    parser.add_argument('--width', type=int, default=820, help='frame width before any rotation')
    parser.add_argument('--height', type=int, default=616)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--flip', type=int, default=0, choices=sorted(FLIPS),
                        help='; '.join(f'{k}: {v}' for k, v in FLIPS.items()))
    parser.add_argument('--quality', type=int, default=80, help='JPEG quality, 1 to 100')
    parser.add_argument('--color-map', help='colour shading file: used if it exists, '
                        'written by /calibrate')
    parser.add_argument('--pid-file', help='write this process id here while running')
    args = parser.parse_args()

    if args.pid_file:
        with open(args.pid_file, 'w') as f:
            f.write(f'{os.getpid()}\n')
    try:
        serve(args)
    finally:
        if args.pid_file:
            try:
                with open(args.pid_file) as f:
                    if f.read().strip() == str(os.getpid()):
                        os.remove(args.pid_file)
            except OSError:
                pass


def serve(args):
    """Run the camera and the HTTP server until a signal, a camera error or a stall."""
    Gst.init(None)
    camera = Camera(args.sensor_id, args.width, args.height, args.fps, args.flip, args.quality,
                    args.color_map)
    stop = threading.Event()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(camera, stop))
    server.daemon_threads = True

    def request_stop(*_):
        stop.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    worker = threading.Thread(target=camera.run, args=(stop,))
    worker.start()
    w, h = camera.size
    print(f'camera server on http://{args.host}:{args.port}/: {w}x{h} at {args.fps} fps, '
          f'flip {args.flip} ({FLIPS[args.flip]}), colour correction {camera.correction_note}',
          flush=True)

    started = time.time()
    last_count, last_change = 0, started
    while not stop.wait(1.0):
        with camera.cond:
            count = camera.count
        now = time.time()
        if count != last_count:
            if not last_count:
                print(f'first frame after {now - started:.1f} s', flush=True)
            last_count, last_change = count, now
        elif now - last_change > (STALL_S if count else FIRST_FRAME_S):
            camera.error = f'no frame for {now - last_change:.0f} s'
            stop.set()

    worker.join(5.0)
    server.shutdown()
    if camera.error:
        print(f'camera error: {camera.error}', file=sys.stderr, flush=True)
        sys.exit(1)
    print('camera server stopped', flush=True)


if __name__ == '__main__':
    main()
