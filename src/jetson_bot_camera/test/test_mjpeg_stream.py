"""Unit tests for the MJPEG parser. mjpeg_stream.py imports no ROS, so they run anywhere."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from jetson_bot_camera.mjpeg_stream import (  # noqa: E402
    capture_time, jpeg_size, MAX_BUFFER, MjpegParser, parse_headers)

# Small stand-ins for JPEGs: start and end markers around bytes that include CRLFs, which a
# parser must not mistake for the end of a part's headers.
FRAMES = [b'\xff\xd8' + bytes([n]) * 40 + b'\r\n\r\n' + bytes([n]) * 9 + b'\xff\xd9'
          for n in range(1, 6)]


def _server_part(jpeg, stamp):
    """Return one part as tools/csi_camera_server.py sends it."""
    return (b'--frame\r\nContent-Type: image/jpeg\r\n'
            + f'Content-Length: {len(jpeg)}\r\nX-Timestamp: {stamp:.6f}\r\n\r\n'.encode()
            + jpeg + b'\r\n')


def _esp32_part(jpeg):
    """Return one part with no Content-Length, which some MJPEG servers leave out."""
    return b'\r\n--123456789000000000000987654321\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg


def _feed_in_chunks(parser, data, size):
    frames = []
    for i in range(0, len(data), size):
        frames += parser.feed(data[i:i + size])
    return frames


def test_server_stream_any_chunk_size():
    """Every frame and its timestamp come out whole, however the bytes arrive."""
    data = b''.join(_server_part(f, 1000.0 + i) for i, f in enumerate(FRAMES))
    for size in (1, 2, 7, 64, len(data)):
        frames = _feed_in_chunks(MjpegParser(), data, size)
        assert [f for f, _ in frames] == FRAMES
        assert [h['x-timestamp'] for _, h in frames] == [f'{1000.0 + i:.6f}' for i in range(5)]


def test_stream_without_content_length():
    """Without Content-Length a frame runs to its end-of-image marker."""
    data = b''.join(_esp32_part(f) for f in FRAMES)
    for size in (1, 5, len(data)):
        frames = _feed_in_chunks(MjpegParser(), data, size)
        assert [f for f, _ in frames] == FRAMES


def test_incomplete_frame_waits():
    """A frame is handed back only once all its bytes have arrived."""
    parser = MjpegParser()
    part = _server_part(FRAMES[0], 5.0)
    assert parser.feed(part[:-10]) == []
    frames = parser.feed(part[-10:])
    assert [f for f, _ in frames] == [FRAMES[0]]


def test_part_that_is_not_a_jpeg_is_skipped():
    """A part whose body isn't a JPEG is dropped and the next frame still comes out."""
    junk = b'--frame\r\nContent-Type: text/plain\r\nContent-Length: 5\r\n\r\nhello\r\n'
    parser = MjpegParser()
    frames = parser.feed(junk + _server_part(FRAMES[1], 1.0))
    assert [f for f, _ in frames] == [FRAMES[1]]
    assert parser.dropped > 0


def test_buffer_is_bounded():
    """A stream with no frame boundary is thrown away rather than kept growing."""
    parser = MjpegParser()
    parser.feed(b'x' * (MAX_BUFFER + 1))
    assert parser.buf == b''
    assert parser.dropped == MAX_BUFFER + 1


def test_parse_headers():
    """Header names are lower-cased, and the boundary line is ignored."""
    headers = parse_headers(b'\r\n--frame\r\nContent-Type: image/jpeg\r\nX-Timestamp:12.5')
    assert headers == {'content-type': 'image/jpeg', 'x-timestamp': '12.5'}


def test_capture_time():
    """A timestamp is used only if it's close to this clock."""
    assert capture_time({'x-timestamp': '100.25'}, now=101.0) == 100.25
    assert capture_time({'x-timestamp': '100.25'}, now=200.0) is None
    assert capture_time({'x-timestamp': 'soon'}, now=100.0) is None
    assert capture_time({}, now=100.0) is None


def _jpeg_head(width, height, sof=0xC0, app0=True):
    """Return a JPEG's segments up to its frame header, as an encoder writes them."""
    head = b'\xff\xd8'
    if app0:  # OpenCV writes JFIF's APP0; nvjpegenc starts with the quantization table
        head += b'\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00'
    head += b'\xff\xdb\x00\x43\x00' + bytes(64)
    head += b'\xff\xc4\x00\x05\x00\x00\x00'  # a Huffman table, whose marker is C4
    return (head + bytes([0xFF, sof, 0x00, 0x11, 0x08]) + height.to_bytes(2, 'big')
            + width.to_bytes(2, 'big') + b'\x03\x01\x22\x00\x02\x11\x01\x03\x11\x01')


def test_jpeg_size():
    """The frame size comes from the frame header, whatever comes before it."""
    assert jpeg_size(_jpeg_head(820, 616)) == (820, 616)
    assert jpeg_size(_jpeg_head(820, 616, app0=False)) == (820, 616)
    assert jpeg_size(_jpeg_head(616, 820, sof=0xC2)) == (616, 820)  # progressive
    padded = _jpeg_head(640, 480).replace(b'\xff\xdb', b'\xff\xff\xdb', 1)  # a fill byte
    assert jpeg_size(padded) == (640, 480)


def test_jpeg_size_of_junk():
    """Anything that isn't a whole JPEG header gives None."""
    assert jpeg_size(_jpeg_head(820, 616)[:-12]) is None
    assert jpeg_size(b'not a jpeg at all') is None
    assert jpeg_size(b'') is None
