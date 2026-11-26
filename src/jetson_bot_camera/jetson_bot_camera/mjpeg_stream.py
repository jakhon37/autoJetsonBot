"""Split an MJPEG HTTP stream into JPEG frames. Imports no ROS, so the tests run anywhere."""

SOI = b'\xff\xd8'  # JPEG start of image
EOI = b'\xff\xd9'  # JPEG end of image
# Bytes kept while looking for a frame. A stream that isn't MJPEG gets dropped, not hoarded.
MAX_BUFFER = 8 * 1024 * 1024


def parse_headers(block):
    """Return a part's headers as a dict with lower-case names; lines without ':' are skipped."""
    headers = {}
    for line in block.split(b'\r\n'):
        name, sep, value = line.partition(b':')
        if sep:
            headers[name.strip().lower().decode('ascii', 'replace')] = \
                value.strip().decode('ascii', 'replace')
    return headers


def jpeg_size(jpeg):
    """Return a JPEG's (width, height) from its frame header, without decoding it, or None."""
    i, n = 2, len(jpeg)
    while i + 4 <= n:
        if jpeg[i] != 0xFF:
            return None
        marker = jpeg[i + 1]
        if marker == 0xFF:  # fill byte
            i += 1
            continue
        if marker == 0x01 or 0xD0 <= marker <= 0xD8:  # markers without a length
            i += 2
            continue
        # start of frame: C0 to CF, except C4 (Huffman tables), C8 and CC
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            if i + 9 > n:
                return None
            return (jpeg[i + 7] << 8 | jpeg[i + 8], jpeg[i + 5] << 8 | jpeg[i + 6])
        i += 2 + (jpeg[i + 2] << 8 | jpeg[i + 3])
    return None


def capture_time(headers, now, max_skew=5.0):
    """Return the part's X-Timestamp in seconds, or None if it's missing or far from now."""
    try:
        stamp = float(headers.get('x-timestamp', ''))
    except ValueError:
        return None
    return stamp if abs(now - stamp) <= max_skew else None


class MjpegParser:
    """
    Collect stream bytes as they arrive and hand back the frames that are complete.

    A frame is (jpeg_bytes, headers). A part's Content-Length is used when it has one; without
    it the JPEG runs to its end-of-image marker. That covers tools/csi_camera_server.py and an
    ESP32-CAM's stream. Parts whose body isn't a JPEG are skipped and counted in `dropped`.
    """

    def __init__(self):
        self.buf = b''
        self.dropped = 0

    def feed(self, data):
        """Add bytes and return the list of frames completed by them."""
        self.buf += data
        frames = []
        while True:
            frame = self._next()
            if frame is None:
                break
            frames.append(frame)
        if len(self.buf) > MAX_BUFFER:
            self.dropped += len(self.buf)
            self.buf = b''
        return frames

    def _next(self):
        while True:
            end = self.buf.find(b'\r\n\r\n')
            if end < 0:
                return None
            headers = parse_headers(self.buf[:end])
            start = end + 4
            length = headers.get('content-length', '')
            if length.isdigit():
                stop = start + int(length)
                if len(self.buf) < stop:
                    return None
            else:
                eoi = self.buf.find(EOI, start + 2)
                if eoi < 0:
                    return None
                stop = eoi + 2
            jpeg = self.buf[start:stop]
            self.buf = self.buf[stop:]
            if jpeg.startswith(SOI):
                return jpeg, headers
            self.dropped += stop
