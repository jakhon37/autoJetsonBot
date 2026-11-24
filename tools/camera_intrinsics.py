#!/usr/bin/env python3
"""
Calibrate the robot's CSI camera: printable target, focus, views, lens model and mounting.

Run it in the ROS container, whose OpenCV has the ArUco module. It reads frames from
tools/csi_camera_server.py on the Jetson host, which the container reaches on 127.0.0.1:
  docker exec -it auto_ros_humble python3 /autonomous_ROS/tools/camera_intrinsics.py <step> ...

Steps:
  target OUT.pdf      the ChArUco board to print on A4 at actual size
  focus               sharpness across the picture, on an average of frames
  capture DIR         save still, varied views of the board as PNG files
  calibrate DIR       fit OpenCV's rational model and its fisheye model to the views, compare
                      them on views left out of each fit, and write the better one as a ROS
                      camera_info YAML
  floor               with the board flat on the floor in front of the robot, work out where
                      the camera sits and how it is tilted, for the URDF
  tilt DIR            the camera's pitch and roll from views of a board standing upright on
                      the floor, its rows level, turned to different angles across the views

The board has 8 x 5 squares of 32 mm, with 24 mm markers from DICT_4X4_50, in OpenCV 4.5's
layout: its origin is the board's bottom-left corner as printed, x to the right and y up. It
can be shown on a screen instead of printed (assets/calibration). --board chessboard:8x6 takes a
plain checkerboard with 8 x 6 inner corners instead, which must be wholly in view to be found.
The lens model is fitted to frames as the camera server sends them, so it holds for that size
and flip only (820x616 and flip 0 on the robot): calibrate again after changing either.
"""

import argparse
import json
import math
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

SQUARES = (8, 5)
SQUARE_MM = 32.0
MARKER_MM = 24.0
DICTIONARY = 'DICT_4X4_50'
URL = 'http://127.0.0.1:8091'
PAGE_MM = (297.0, 210.0)  # A4, landscape
PT_PER_MM = 72 / 25.4
MIN_CORNERS = 8  # corners a view needs to be used
GRID = (6, 4)    # coverage cells across and down the picture
STILL_PX = 1.5   # a view is kept once its corners move less than this between two looks


class Board:
    """The ChArUco board: its layout for printing, and detection of its inner corners."""

    def __init__(self, squares=SQUARES, square_mm=SQUARE_MM, marker_mm=MARKER_MM,
                 dictionary=DICTIONARY):
        self.squares = tuple(squares)
        self.square_mm = float(square_mm)
        self.marker_mm = float(marker_mm)
        self.dictionary_name = dictionary
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self.board = cv2.aruco.CharucoBoard_create(
            self.squares[0], self.squares[1], self.square_mm / 1000, self.marker_mm / 1000,
            self.dictionary)
        # inner corner i sits at board x, y in metres; z is 0
        self.corners3d = np.array(self.board.chessboardCorners, np.float64).reshape(-1, 3)
        self.cols = self.squares[0] - 1  # inner corners per row

    def describe(self):
        return (f'ChArUco, {self.squares[0]} x {self.squares[1]} squares of '
                f'{self.square_mm:g} mm, markers {self.marker_mm:g} mm, {self.dictionary_name}')

    def size_mm(self):
        return self.squares[0] * self.square_mm, self.squares[1] * self.square_mm

    def rectangles_mm(self):
        """Return the board's black areas as (x, y, w, h) in mm, origin bottom-left, y up."""
        s = self.square_mm
        nx, ny = self.squares
        markers = np.array(self.board.objPoints, np.float64) * 1000  # (n, 4, 3), mm
        white = set()
        for quad in markers:
            centre = quad[:, :2].mean(axis=0)
            white.add((int(centre[0] // s), int(centre[1] // s)))
        rects = [(x * s, y * s, s, s) for y in range(ny) for x in range(nx)
                 if (x, y) not in white]
        side = 6  # 4x4 bits and a one-cell black border
        for quad, marker_id in zip(markers, np.array(self.board.ids).ravel()):
            bits = cv2.aruco.drawMarker(self.dictionary, int(marker_id), side, borderBits=1)
            x0, y0 = quad[0, 0], quad[0, 1]  # top-left corner
            cell = (quad[1, 0] - quad[0, 0]) / side
            for r in range(side):
                c = 0
                while c < side:  # one rectangle per run of black cells
                    if bits[r, c]:
                        c += 1
                        continue
                    start = c
                    while c < side and not bits[r, c]:
                        c += 1
                    rects.append((x0 + start * cell, y0 - (r + 1) * cell,
                                  (c - start) * cell, cell))
        return rects

    def detect(self, gray, min_corners=MIN_CORNERS):
        """Return the inner corners seen in a grey frame, (N x 2 pixels, N ids), or None."""
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary,
                                                         parameters=params)
        if ids is None or len(ids) < 2:
            return None
        corners, ids, _, _ = cv2.aruco.refineDetectedMarkers(gray, self.board, corners, ids,
                                                             rejected)
        if ids is None:
            return None
        count, points, point_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray,
                                                                       self.board)
        if point_ids is None or count < min_corners:
            return None
        return points.reshape(-1, 2).astype(np.float64), point_ids.ravel().astype(int)


class Chessboard:
    """
    A plain checkerboard, given by its inner corners across and down.

    Its corners are found only when all are in view, by cv2.findChessboardCornersSB, numbered
    row by row. A board with an odd number of squares both ways looks the same turned half
    round, so its numbering can start from either end: the lens fit doesn't mind, the floor
    check would.
    """

    def __init__(self, inner=(8, 6), square_mm=SQUARE_MM):
        self.inner = tuple(inner)
        self.squares = (inner[0] + 1, inner[1] + 1)
        self.square_mm = float(square_mm)
        self.cols = inner[0]
        grid = np.mgrid[0:inner[0], 0:inner[1]].T.reshape(-1, 2) + 1
        self.corners3d = np.hstack([grid * self.square_mm / 1000,
                                    np.zeros((len(grid), 1))]).astype(np.float64)

    def describe(self):
        return (f'checkerboard, {self.inner[0]} x {self.inner[1]} inner corners, squares '
                f'{self.square_mm:g} mm')

    def detect(self, gray, min_corners=MIN_CORNERS):
        """Return all inner corners (N x 2 pixels, ids 0..N-1), or None."""
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        ok, corners = cv2.findChessboardCornersSB(gray, self.inner, flags=flags)
        if not ok:
            return None
        points = corners.reshape(-1, 2).astype(np.float64)
        return points, np.arange(len(points))


def plane_candidates(board, gray):
    """
    Yield (board, points, ids): the whole board, then for a plain checkerboard its blocks.

    Seen nearly edge on, as when it lies on the floor in front of the low camera, a screen
    loses contrast and catches reflections, so a whole checkerboard is often not found when a
    block of it is. Any block gives the board's plane, but the detector also reports blocks
    that aren't there, so check each one's fit. Largest blocks first.
    """
    found = board.detect(gray)
    if found is not None:
        yield board, found[0], found[1]
    if not isinstance(board, Chessboard):
        return
    cols, rows = board.inner
    sizes = sorted(((c, r) for c in range(cols, 3, -1) for r in range(rows, 2, -1)
                    if (c, r) != (cols, rows)), key=lambda cr: -cr[0] * cr[1])
    for c, r in sizes:
        part = Chessboard((c, r), board.square_mm)
        found = part.detect(gray)
        if found is not None:
            yield part, found[0], found[1]


def make_board(spec, square_mm):
    """Board from --board: 'charuco' (the printed board) or 'chessboard:COLSxROWS'."""
    if spec == 'charuco':
        return Board(square_mm=square_mm, marker_mm=MARKER_MM * square_mm / SQUARE_MM)
    kind, _, size = spec.partition(':')
    if kind != 'chessboard' or 'x' not in size:
        raise ValueError(f'--board takes charuco or chessboard:COLSxROWS, not {spec}')
    cols, rows = (int(v) for v in size.split('x'))
    return Chessboard((cols, rows), square_mm)


# ---- the printable target -------------------------------------------------------------------

def _pdf_text(text):
    return text.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)')


def target_pdf(board, page_mm=PAGE_MM):
    """Return a one-page PDF with the board at actual size, a 100 mm scale bar and notes."""
    bw, bh = board.size_mm()
    pw, ph = page_mm
    ox, oy = (pw - bw) / 2, (ph - bh) / 2 + 3

    def pt(v):
        return f'{v * PT_PER_MM:.3f}'

    ops = ['0 g']
    for x, y, w, h in board.rectangles_mm():
        ops.append(f'{pt(ox + x)} {pt(oy + y)} {pt(w)} {pt(h)} re')
    ops.append('f')
    # marks on the board's centre line, above and below it
    for y, sign in ((oy - 1.5, -1), (oy + bh + 1.5, 1)):
        x = ox + bw / 2
        ops.append(f'{pt(x)} {pt(y)} m {pt(x - 2.5)} {pt(y + sign * 4)} l '
                   f'{pt(x + 2.5)} {pt(y + sign * 4)} l f')
    # the 100 mm scale bar, with a tick every 10 mm
    bar_y = oy - 13
    ops.append(f'0.6 w {pt(ox)} {pt(bar_y)} m {pt(ox + 100)} {pt(bar_y)} l S')
    for i in range(11):
        tick = 3 if i % 5 == 0 else 1.5
        ops.append(f'{pt(ox + 10 * i)} {pt(bar_y)} m {pt(ox + 10 * i)} {pt(bar_y + tick)} l S')
    lines = [
        (oy + bh + 8, 9, f'Camera calibration board: {board.describe()} (OpenCV 4.5 layout)'),
        (bar_y - 5, 8, 'Print at 100% (actual size, not fit to page). The bar above must be '
                       '100 mm long, and each square 32 mm.'),
        (bar_y - 9.5, 8, 'Tape it flat to something stiff. For the floor check, lay it flat with '
                         'this edge toward the robot and the two arrows on its centre line.'),
    ]
    for y, size, text in lines:
        ops.append(f'BT /F1 {size} Tf {pt(ox)} {pt(y)} Td ({_pdf_text(text)}) Tj ET')
    ops.append(f'BT /F1 8 Tf {pt(ox + 102)} {pt(bar_y - 1)} Td (100 mm) Tj ET')
    content = '\n'.join(ops).encode('ascii')

    objects = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        (f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {pw * PT_PER_MM:.3f} '
         f'{ph * PT_PER_MM:.3f}] /Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>'
         ).encode(),
        b'<< /Length %d >>\nstream\n' % len(content) + content + b'\nendstream',
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>',
    ]
    out = bytearray(b'%PDF-1.4\n')
    offsets = []
    for i, body in enumerate(objects, 1):
        offsets.append(len(out))
        out += b'%d 0 obj\n' % i + body + b'\nendobj\n'
    xref = len(out)
    out += b'xref\n0 %d\n0000000000 65535 f \n' % (len(objects) + 1)
    for offset in offsets:
        out += b'%010d 00000 n \n' % offset
    out += b'trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n' % (
        len(objects) + 1, xref)
    return bytes(out)


# ---- frames from the camera server ----------------------------------------------------------

def fetch(url, raw=True, timeout=3.0):
    """Return the camera server's latest frame as BGR. raw=True skips the colour correction."""
    with urllib.request.urlopen(f'{url}/snapshot.jpg' + ('?raw=1' if raw else ''),
                                timeout=timeout) as resp:
        data = resp.read()
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError('the snapshot is not a JPEG')
    return frame


def average(url, frames, wait=0.08):
    """Average several frames of a still scene, which takes out most of the sensor's noise."""
    total, last = None, None
    for _ in range(frames):
        frame = fetch(url)
        while last is not None and np.array_equal(frame[::16, ::16], last[::16, ::16]):
            time.sleep(wait)  # the same frame again
            frame = fetch(url)
        last = frame
        total = frame.astype(np.float64) if total is None else total + frame
        time.sleep(wait)
    return np.clip(np.round(total / frames), 0, 255).astype(np.uint8)


# ---- focus ----------------------------------------------------------------------------------

def sharpness(gray):
    """
    Return the sharpness of each cell of a 3x3 grid over a grey frame, and of the whole.

    The measure is the root of the Laplacian's energy over the gradient's, taken where the
    picture has edges. Blurring an edge over sigma pixels scales it by about 1/sigma, and
    brightness and contrast cancel out, so it can be compared before and after a change in
    light. Use it on an average of frames: noise would add to it.
    """
    img = cv2.GaussianBlur(gray.astype(np.float32), (0, 0), 0.7)
    lap = cv2.Laplacian(img, cv2.CV_32F, ksize=3)
    gx = cv2.Sobel(img, cv2.CV_32F, 1, 0, ksize=3) / 8
    gy = cv2.Sobel(img, cv2.CV_32F, 0, 1, ksize=3) / 8
    grad2 = gx * gx + gy * gy
    h, w = gray.shape

    def measure(sl):
        g2 = grad2[sl]
        if g2.size == 0:
            return float('nan')
        edges = g2 >= max(np.percentile(g2, 90), 4.0)  # gradients over 2 grey levels a pixel
        edges = cv2.dilate(edges.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        if edges.sum() < 50:
            return float('nan')
        return float(np.sqrt((lap[sl][edges] ** 2).sum() / g2[edges].sum()))

    cells = [[measure((slice(r * h // 3, (r + 1) * h // 3), slice(c * w // 3, (c + 1) * w // 3)))
              for c in range(3)] for r in range(3)]
    return cells, measure((slice(None), slice(None)))


# ---- views ----------------------------------------------------------------------------------

def view_descriptor(points, size):
    """Where the board sits in the picture, how big it looks and how it is tilted."""
    w, h = size
    centre = points.mean(axis=0) / (w, h)
    hull = cv2.convexHull(points.astype(np.float32))
    scale = math.sqrt(max(cv2.contourArea(hull), 1.0)) / w
    spread = np.cov((points - points.mean(axis=0)).T / w)
    evals = np.linalg.eigvalsh(spread)
    shape = math.sqrt(max(evals[0], 1e-9) / max(evals[1], 1e-9))
    return np.array([centre[0], centre[1], 2 * scale, shape])


def coverage(views, size):
    """Count the corners kept in each cell of a 6x4 grid over the picture."""
    w, h = size
    grid = np.zeros(GRID[::-1], int)
    for points, _ in views:
        cols = np.clip((points[:, 0] / w * GRID[0]).astype(int), 0, GRID[0] - 1)
        rows = np.clip((points[:, 1] / h * GRID[1]).astype(int), 0, GRID[1] - 1)
        np.add.at(grid, (rows, cols), 1)
    return grid


def is_new_view(points, kept, size, min_distance=0.12, few=6, same=0.04):
    """
    Keep a view if it differs from every kept one, or fills a sparsely covered cell.

    A view almost the same as one kept is never new: otherwise the same view counted again
    whenever a cell at its edge held fewer than `few` corners.
    """
    if not kept:
        return True
    d = view_descriptor(points, size)
    nearest = min(np.linalg.norm(d - view_descriptor(p, size)) for p, _ in kept)
    if nearest >= min_distance:
        return True
    if nearest < same:
        return False
    before = coverage(kept, size)
    after = coverage(kept + [(points, None)], size)
    return bool(((before < few) & (after > before)).any())


def capture(board, url, out_dir, views=30, minutes=15.0, log=print):
    """Save still views of the board that differ from each other, until enough are kept."""
    os.makedirs(out_dir, exist_ok=True)
    kept, last, size = [], None, None
    deadline = time.time() + minutes * 60
    seen = 0
    while len(kept) < views and time.time() < deadline:
        try:
            frame = fetch(url)
        except (OSError, ValueError) as exc:
            log(f'no frame ({exc}); retrying')
            time.sleep(1.0)
            continue
        size = (frame.shape[1], frame.shape[0])
        found = board.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        seen += 1
        if found is None:
            last = None
            if seen % 20 == 0:
                log(f'{len(kept)} views kept; no board in sight')
            time.sleep(0.15)
            continue
        points, ids = found
        steady = False
        if last is not None:
            common, a, b = np.intersect1d(ids, last[1], return_indices=True)
            if len(common) >= 0.9 * len(ids):
                steady = float(np.abs(points[a] - last[0][b]).max()) < STILL_PX
        last = (points, ids)
        if not steady or not is_new_view(points, kept, size):
            time.sleep(0.1)
            continue
        kept.append((points, ids))
        name = os.path.join(out_dir, f'view_{len(kept):02d}.png')
        cv2.imwrite(name, frame)
        grid = coverage(kept, size)
        log(f'kept {name}: {len(ids)} corners. Corners per part of the picture:\n'
            + '\n'.join('   ' + ' '.join(f'{v:3d}' for v in row) for row in grid))
        last = None  # wait for the board to move on
    return kept


# ---- lens models ----------------------------------------------------------------------------

def to_normalized(model, K, D, points):
    """Undistort pixel points to normalized image coordinates (x/z, y/z)."""
    pts = np.asarray(points, np.float64).reshape(-1, 1, 2)
    if model == 'equidistant':
        out = cv2.fisheye.undistortPoints(pts, K, D)
    else:
        out = cv2.undistortPoints(pts, K, D)
    return out.reshape(-1, 2)


def project(model, K, D, obj, rvec, tvec):
    if model == 'equidistant':
        out, _ = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, K, D)
    else:
        out, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, K, D)
    return out.reshape(-1, 2)


def solve_pose(model, K, D, obj, points):
    """Return (rvec, tvec) of the board in the camera, from points undistorted by the model."""
    normalized = to_normalized(model, K, D, points)
    ok, rvec, tvec = cv2.solvePnP(obj.reshape(-1, 1, 3), normalized.reshape(-1, 1, 2),
                                  np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError('no pose found')
    return rvec, tvec


CRITERIA = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9)


def sane(K, size):
    """A camera matrix a real lens could have: like focal lengths, the centre near the middle."""
    w, h = size
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return (0 < fx < 10 * w and 0 < fy < 10 * w and abs(fx / fy - 1) < 0.2
            and 0.25 * w < cx < 0.75 * w and 0.25 * h < cy < 0.75 * h)


def starts(size, guess):
    """Camera matrices to start a fit from: the guess, or a few focal lengths."""
    if guess is not None:
        return [guess]
    w, h = size
    return [np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]]) for f in (w / 3, w / 2, w / 1.5)]


def _fit_fisheye(obj, img, size, guess):
    """cv2.fisheye.calibrate from each start; returns the best (rms, K, D)."""
    obj = [o.astype(np.float64) for o in obj]
    img = [i.astype(np.float64) for i in img]
    flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
             | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS)
    best, error = None, 'no start'
    for K0 in starts(size, guess):
        try:
            rms, K, D, _, _ = cv2.fisheye.calibrate(obj, img, size, K0.copy(), np.zeros((4, 1)),
                                                    criteria=CRITERIA, flags=flags)
        except cv2.error as exc:  # a start far off can leave a view degenerate
            error = exc.err
            continue
        if sane(K, size) and np.isfinite(rms) and (best is None or rms < best[0]):
            best = (rms, K, D)
    if best is None:
        raise ValueError(f'the fisheye model did not fit: {error}')
    return best


def _fit_rational(obj, img, size, guess):
    """
    calibrateCamera with the rational model. Returns (rms, K, D).

    OpenCV's own start assumes no distortion, and on a wide lens it can end far off. So each
    start first fits k1 to k3 with the principal point held at the start's. The best start then
    frees the principal point and the tangential terms, and last k4 to k6, the denominator,
    which trade against k1 to k3. A step is kept only if it does better and stays sane.
    calibrateCamera's default SVD took 15 s for 30 views, LU half a second.
    """
    flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_USE_LU | cv2.CALIB_RATIONAL_MODEL
    first = (cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5
             | cv2.CALIB_FIX_K6 | cv2.CALIB_FIX_PRINCIPAL_POINT)
    best, error = None, 'no start'
    for K0 in starts(size, guess):
        try:
            rms, K, D, _, _ = cv2.calibrateCamera(obj, img, size, K0.copy(), np.zeros(8),
                                                  criteria=CRITERIA, flags=flags | first)
        except cv2.error as exc:
            error = exc.err
            continue
        if sane(K, size) and np.isfinite(rms) and (best is None or rms < best[0]):
            best = (rms, K, D)
    if best is None:
        raise ValueError(f'the rational model did not fit: {error}')
    rms, K, D = best
    for step in (cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6, 0):
        try:
            rms2, K2, D2, _, _ = cv2.calibrateCamera(obj, img, size, K.copy(), D.copy(),
                                                     criteria=CRITERIA, flags=flags | step)
        except cv2.error:
            continue
        if sane(K2, size) and rms2 <= rms:
            rms, K, D = rms2, K2, D2
    return rms, K, D


def fit(model, board, views, size, guess=None):
    """Fit a lens model to views [(points, ids)]. Returns dict(model, K, D, rms)."""
    obj = [board.corners3d[ids].astype(np.float32).reshape(-1, 1, 3) for _, ids in views]
    img = [points.astype(np.float32).reshape(-1, 1, 2) for points, _ in views]
    if model == 'equidistant':
        rms, K, D = _fit_fisheye(obj, img, size, guess)
        D = D.ravel()[:4]
    elif model == 'rational_polynomial':
        rms, K, D = _fit_rational(obj, img, size, guess)
        D = D.ravel()[:8]
    else:
        raise ValueError(f'unknown model {model}')
    return {'model': model, 'K': K, 'D': np.asarray(D, np.float64), 'rms': float(rms)}


def radius(points, size):
    """Distance from the picture's centre, as a share of its half diagonal."""
    w, h = size
    return np.hypot(points[:, 0] - w / 2, points[:, 1] - h / 2) / math.hypot(w / 2, h / 2)


def line_bend(model, K, D, board, points, ids, size):
    """
    For each row and column of corners, undistorted, how far they stray from a straight line.

    Returns [(RMS distance in pixels at the focal length, the line's farthest radius)].
    """
    normalized = to_normalized(model, K, D, points)
    out = []
    rows, cols = ids // board.cols, ids % board.cols
    for keys in (rows, cols):
        for key in np.unique(keys):
            sel = keys == key
            if sel.sum() < 4:
                continue
            p = normalized[sel]
            centred = p - p.mean(axis=0)
            _, _, vt = np.linalg.svd(centred)
            offsets = centred @ vt[1]
            out.append((float(np.sqrt(np.mean(offsets ** 2)) * K[0, 0]),
                        float(radius(points[sel], size).max())))
    return out


def monotonic(model, K, D, size):
    """True if the model maps radius one to one out to the picture's corners."""
    w, h = size
    corners = np.array([[0, 0], [w - 1, 0], [0, h - 1], [w - 1, h - 1]], np.float64)
    reach = np.linalg.norm(to_normalized(model, K, D, corners), axis=1).max() * 1.05
    r = np.linspace(1e-4, reach, 400)
    if model == 'equidistant':
        theta = np.arctan(r)
        k = D.ravel()
        rd = theta * (1 + k[0] * theta ** 2 + k[1] * theta ** 4 + k[2] * theta ** 6
                      + k[3] * theta ** 8)
    else:
        k1, k2, p1, p2, k3, k4, k5, k6 = D.ravel()[:8]
        r2 = r * r
        rd = r * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) / (1 + k4 * r2 + k5 * r2 ** 2
                                                                + k6 * r2 ** 3)
    return bool(np.all(np.diff(rd) > 0)) and bool(np.isfinite(reach))


def compare(board, views, size, folds=5):
    """
    Fit both models on all views, and on all but a fifth at a time to test on the rest.

    Returns {model: report}. Each report has the fit on all views, the error on views left
    out (pixels, overall and beyond 70% of the half diagonal), how much rows and columns of
    corners still bend once undistorted, and whether the model stays one to one.
    """
    order = np.arange(len(views))
    np.random.default_rng(0).shuffle(order)
    groups = [order[i::folds] for i in range(folds)]
    reports = {}
    for model in ('rational_polynomial', 'equidistant'):
        try:
            reports[model] = _evaluate(model, board, views, size, order, groups)
        except (ValueError, cv2.error) as exc:
            reports[model] = {'failed': str(exc)}
    return reports


def _evaluate(model, board, views, size, order, groups):
    """Fit one model on all views and on each training part; test on the views left out."""
    full = fit(model, board, views, size)
    errors, radii, bends = [], [], []
    for test in groups:
        train = [views[i] for i in order if i not in set(test)]
        part = fit(model, board, train, size, guess=full['K'])
        for i in test:
            points, ids = views[i]
            obj = board.corners3d[ids]
            rvec, tvec = solve_pose(model, part['K'], part['D'], obj, points)
            proj = project(model, part['K'], part['D'], obj, rvec, tvec)
            errors.extend(np.linalg.norm(proj - points, axis=1))
            radii.extend(radius(points, size))
            bends.extend(line_bend(model, part['K'], part['D'], board, points, ids, size))
    errors, radii = np.array(errors), np.array(radii)
    outer = radii > 0.7
    bend = np.array(bends) if bends else np.zeros((0, 2))
    bend_outer = bend[bend[:, 1] > 0.7, 0] if len(bend) else np.zeros(0)
    return {
        'fit': full,
        'rms_all_views': full['rms'],
        'held_out_rms': float(np.sqrt(np.mean(errors ** 2))),
        'held_out_rms_outer': float(np.sqrt(np.mean(errors[outer] ** 2))) if outer.any()
        else float('nan'),
        'outer_corners': int(outer.sum()),
        'bend_median': float(np.median(bend[:, 0])) if len(bend) else float('nan'),
        'bend_outer_median': (float(np.median(bend_outer)) if len(bend_outer)
                              else float('nan')),
        'one_to_one': monotonic(model, full['K'], full['D'], size),
    }


def choose(reports):
    """Pick the model that does best on views left out, among those that stay one to one."""
    usable = {m: r for m, r in reports.items() if 'failed' not in r and r['one_to_one']}
    if not usable:
        raise ValueError('neither model stays one to one out to the corners')
    best = min(usable, key=lambda m: usable[m]['held_out_rms'])
    return best


def camera_info_yaml(model, K, D, size, name='imx219_wide', note=''):
    """Return a camera_info YAML in camera_calibration_parsers' layout."""
    w, h = size
    if model == 'equidistant':
        P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, np.ravel(D).reshape(4, 1), size, np.eye(3), balance=0.0)
    else:
        P, _ = cv2.getOptimalNewCameraMatrix(K, D, size, 0.0)
    proj = np.hstack([P, np.zeros((3, 1))])

    def row(values):
        return '[' + ', '.join(f'{float(v):.8g}' for v in np.ravel(values)) + ']'

    head = ''.join(f'# {line}\n' for line in note.splitlines() if line.strip())
    return (f'{head}image_width: {w}\nimage_height: {h}\ncamera_name: {name}\n'
            f'camera_matrix:\n  rows: 3\n  cols: 3\n  data: {row(K)}\n'
            f'distortion_model: {model}\n'
            f'distortion_coefficients:\n  rows: 1\n  cols: {len(np.ravel(D))}\n'
            f'  data: {row(D)}\n'
            f'rectification_matrix:\n  rows: 3\n  cols: 3\n  data: {row(np.eye(3))}\n'
            f'projection_matrix:\n  rows: 3\n  cols: 4\n  data: {row(proj)}\n')


def read_camera_info(path):
    """Return (model, K, D, size) from a camera_info YAML."""
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data['camera_matrix']['data'], np.float64).reshape(3, 3)
    D = np.array(data['distortion_coefficients']['data'], np.float64)
    return data['distortion_model'], K, D, (data['image_width'], data['image_height'])


# ---- mounting -------------------------------------------------------------------------------

# camera_link's axes (x forward, y left, z up) in the optical frame (z forward, x right, y down)
OPTICAL_TO_LINK = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], np.float64)


def rpy(R):
    """Roll, pitch and yaw (URDF's fixed-axis x, y, z) of a rotation matrix."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def floor_tilt(board, model, K, D, points, ids, thickness_m=0.0):
    """
    Work out the camera's height and tilt from a view of a board lying flat on the floor.

    Needs only the board's plane, so any board will do, a symmetric checkerboard too, and the
    board can lie anywhere in view. thickness_m is the board's surface above the floor, such as
    a tablet's thickness. Returns dict(z, roll, pitch, rms): z is camera_link's height above
    the floor, and roll and pitch are URDF's, so pitch is positive when the camera looks down.
    """
    obj = board.corners3d[ids]
    rvec, tvec = solve_pose(model, K, D, obj, points)
    proj = project(model, K, D, obj, rvec, tvec)
    rms = float(np.sqrt(np.mean(np.sum((proj - points) ** 2, axis=1))))
    R_cb, _ = cv2.Rodrigues(rvec)
    up = R_cb[:, 2]  # the board's normal, in the optical frame
    if up @ -tvec.ravel() < 0:
        up = -up  # make it point from the board toward the camera
    height = float(up @ -tvec.ravel()) + thickness_m
    # the floor's up in camera_link is (-sin pitch, cos pitch sin roll, cos pitch cos roll)
    up_link = OPTICAL_TO_LINK.T @ up
    return {'z': height, 'roll': math.atan2(up_link[1], up_link[2]),
            'pitch': math.asin(max(-1.0, min(1.0, -up_link[0]))), 'rms': rms,
            'corners': int(len(ids))}


def level_tilt(board, model, K, D, views):
    """
    Work out the camera's pitch and roll from views of a board whose rows were level.

    A board standing upright on the floor, a tablet on its stand say, has level rows of corners.
    Each view gives that direction in the camera's frame, and the views together give the up
    direction, the one at right angles to all of them. The board must be turned to different
    angles across the views, or the pitch isn't fixed: check `spread`, the second smallest
    eigenvalue, against the smallest. Returns dict(pitch, roll, off_level, spread, flat).
    """
    rows = []
    for points, ids in views:
        rvec, _ = solve_pose(model, K, D, board.corners3d[ids], points)
        R, _ = cv2.Rodrigues(rvec)
        rows.append(R[:, 0])
    rows = np.array(rows)
    evals, evecs = np.linalg.eigh(rows.T @ rows)
    up = evecs[:, 0]
    if up[1] > 0:
        up = -up  # the optical frame's y points down
    up_link = OPTICAL_TO_LINK.T @ up
    return {'pitch': math.asin(max(-1.0, min(1.0, -up_link[0]))),
            'roll': math.atan2(up_link[1], up_link[2]),
            'off_level': [math.degrees(math.asin(min(1.0, abs(float(r @ up))))) for r in rows],
            'spread': float(evals[1]), 'flat': float(evals[0])}


def floor_mounting(board, model, K, D, points, ids, distance_m, centre_offset_m=0.0):
    """
    Work out camera_link's pose on the robot from a view of the board lying on the floor.

    The board lies flat, its printed bottom edge toward the robot and square to its heading,
    that edge distance_m ahead of the axle, and the arrows centre_offset_m left of the robot's
    centre line. Returns the pose in base_footprint (on the floor under the axle's middle):
    dict(x, y, z, roll, pitch, yaw, rms) in metres and radians.
    """
    obj = board.corners3d[ids]
    rvec, tvec = solve_pose(model, K, D, obj, points)
    proj = project(model, K, D, obj, rvec, tvec)
    rms = float(np.sqrt(np.mean(np.sum((proj - points) ** 2, axis=1))))
    R_cb, _ = cv2.Rodrigues(rvec)  # board to optical frame
    bw = board.squares[0] * board.square_mm / 1000
    # board axes in the robot: x to the robot's right, y forward, z up
    R_rb = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], np.float64)
    t_rb = np.array([distance_m, bw / 2 + centre_offset_m, 0.0])
    cam_in_board = -R_cb.T @ tvec.ravel()
    position = R_rb @ cam_in_board + t_rb
    R_r_optical = R_rb @ R_cb.T
    R_r_link = R_r_optical @ OPTICAL_TO_LINK
    roll, pitch, yaw = rpy(R_r_link)
    return {'x': float(position[0]), 'y': float(position[1]), 'z': float(position[2]),
            'roll': roll, 'pitch': pitch, 'yaw': yaw, 'rms': rms, 'corners': int(len(ids))}


# ---- command line ---------------------------------------------------------------------------

def load_views(board, folder):
    views, names, size = [], [], None
    for name in sorted(os.listdir(folder)):
        if not name.lower().endswith(('.png', '.jpg')):
            continue
        frame = cv2.imread(os.path.join(folder, name), cv2.IMREAD_GRAYSCALE)
        size = (frame.shape[1], frame.shape[0])
        found = board.detect(frame)
        if found is not None:
            views.append(found)
            names.append(name)
    return views, names, size


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--url', default=URL, help='camera server (default %(default)s)')
    parser.add_argument('--square-mm', type=float, default=SQUARE_MM,
                        help='printed square size, if the print came out at another scale')
    parser.add_argument('--board', default='charuco',
                        help='charuco (default), or chessboard:COLSxROWS for a plain '
                        'checkerboard with that many inner corners, such as chessboard:8x6')
    sub = parser.add_subparsers(dest='step', required=True)
    p = sub.add_parser('target', help='write the board as a PDF to print')
    p.add_argument('out')
    p = sub.add_parser('focus', help='measure sharpness across the picture')
    p.add_argument('--frames', type=int, default=15)
    p.add_argument('--save', help='save the averaged frame here')
    p = sub.add_parser('capture', help='save varied, still views of the board')
    p.add_argument('dir')
    p.add_argument('--views', type=int, default=30)
    p.add_argument('--minutes', type=float, default=15.0)
    p = sub.add_parser('calibrate', help='fit and compare the lens models, write camera_info')
    p.add_argument('dir')
    p.add_argument('--out', help='write the chosen model here as camera_info YAML')
    p.add_argument('--report', help='write the comparison here as JSON')
    p.add_argument('--model', choices=('auto', 'rational_polynomial', 'equidistant'),
                   default='auto')
    p = sub.add_parser('floor', help='camera height and tilt, or its whole pose, from the '
                       'board flat on the floor')
    p.add_argument('--camera-info', required=True)
    p.add_argument('--distance-mm', type=float,
                   help='ChArUco board only: from the axle line to the board\'s near edge, '
                   'along the floor, for the whole pose')
    p.add_argument('--thickness-mm', type=float, default=0.0,
                   help='how far the board\'s surface is above the floor, such as a tablet\'s '
                   'thickness')
    p.add_argument('--offset-mm', type=float, default=0.0,
                   help='how far left of the robot\'s centre line the arrows lie')
    p.add_argument('--frames', type=int, default=15)
    p = sub.add_parser('tilt', help='pitch and roll from views of a board standing level')
    p.add_argument('dir')
    p.add_argument('--camera-info', required=True)
    args = parser.parse_args(argv)

    board = make_board(args.board, args.square_mm)

    if args.step == 'target' and not isinstance(board, Board):
        sys.exit('target makes the ChArUco board only')
    if args.step == 'target':
        with open(args.out, 'wb') as f:
            f.write(target_pdf(board))
        print(f'wrote {args.out}: {board.describe()}')
    elif args.step == 'focus':
        frame = average(args.url, args.frames)
        if args.save:
            cv2.imwrite(args.save, frame)
        cells, whole = sharpness(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        print(f'sharpness over {args.frames} averaged frames: {whole:.3f} overall; by part of '
              'the picture (higher is sharper):')
        for row in cells:
            print('   ' + '  '.join(f'{v:6.3f}' for v in row))
    elif args.step == 'capture':
        kept = capture(board, args.url, args.dir, args.views, args.minutes,
                       log=lambda text: print(text, flush=True))
        print(f'{len(kept)} views in {args.dir}')
    elif args.step == 'calibrate':
        views, names, size = load_views(board, args.dir)
        print(f'{len(views)} views with the board, {size[0]}x{size[1]}, '
              f'{sum(len(i) for _, i in views)} corners')
        reports = compare(board, views, size)
        for model, r in reports.items():
            if 'failed' in r:
                print(f'{model}: {r["failed"]}')
                continue
            K, D = r['fit']['K'], r['fit']['D']
            print(f'{model}: fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} '
                  f'cy {K[1, 2]:.1f}; error on all views {r["rms_all_views"]:.3f} px, on views '
                  f'left out {r["held_out_rms"]:.3f} px ({r["held_out_rms_outer"]:.3f} px for '
                  f'{r["outer_corners"]} corners beyond 70% of the half diagonal); rows and '
                  f'columns bend {r["bend_median"]:.3f} px ({r["bend_outer_median"]:.3f} px '
                  f'near the edges); one to one: {r["one_to_one"]}')
        model = choose(reports) if args.model == 'auto' else args.model
        print(f'chosen: {model}')
        if args.report:
            def plain(r):
                if 'failed' in r:
                    return r
                fit_ = r['fit']
                return dict({k: v for k, v in r.items() if k != 'fit'},
                            K=fit_['K'].tolist(), D=fit_['D'].tolist())
            with open(args.report, 'w') as f:
                json.dump({'views': names, 'size': size, 'chosen': model,
                           'models': {m: plain(r) for m, r in reports.items()}}, f, indent=1)
        if args.out:
            r = reports[model]
            note = (f'{r["fit"]["model"]} lens model of the robot\'s IMX219 camera, for frames '
                    f'of {size[0]}x{size[1]} from tools/csi_camera_server.py with flip 0.\n'
                    f'Fitted by tools/camera_intrinsics.py on {time.strftime("%Y-%m-%d")} from '
                    f'{len(views)} views of a {board.describe()}; error '
                    f'{r["rms_all_views"]:.3f} px on those views, {r["held_out_rms"]:.3f} px on '
                    f'views left out of the fit.')
            with open(args.out, 'w') as f:
                f.write(camera_info_yaml(model, r['fit']['K'], r['fit']['D'], size, note=note))
            print(f'wrote {args.out}')
    elif args.step == 'floor':
        model, K, D, size = read_camera_info(args.camera_info)
        frame = average(args.url, args.frames)
        if (frame.shape[1], frame.shape[0]) != tuple(size):
            sys.exit(f'frames are {frame.shape[1]}x{frame.shape[0]}, the calibration '
                     f'{size[0]}x{size[1]}')
        for part, points, ids in plane_candidates(board, cv2.cvtColor(frame,
                                                                      cv2.COLOR_BGR2GRAY)):
            tilt = floor_tilt(part, model, K, D, points, ids, args.thickness_mm / 1000)
            if tilt['rms'] < 1.0:  # a block that isn't really there fits badly
                break
            print(f'a block of {part.inner[0]} x {part.inner[1]} corners fits a flat board '
                  f'only within {tilt["rms"]:.1f} px: not used')
        else:
            sys.exit('the board is not in sight')
        if part is not board:
            print(f'the whole board was not found: using a block of {part.inner[0]} x '
                  f'{part.inner[1]} inner corners')
        print(f'camera_link {tilt["z"]:.4f} m above the floor; roll '
              f'{math.degrees(tilt["roll"]):.2f}, pitch {math.degrees(tilt["pitch"]):.2f} deg '
              f'(pitch is positive when the camera looks down); {tilt["corners"]} corners, '
              f'{tilt["rms"]:.3f} px')
        if args.distance_mm is not None:
            if not isinstance(board, Board):
                sys.exit('the whole pose needs the ChArUco board: a plain checkerboard can be '
                         'read from either end')
            pose = floor_mounting(board, model, K, D, points, ids,
                                  args.distance_mm / 1000, args.offset_mm / 1000)
            print(f'camera_link in base_footprint: x {pose["x"]:.4f} y {pose["y"]:.4f} '
                  f'z {pose["z"]:.4f} m; roll {math.degrees(pose["roll"]):.2f}, pitch '
                  f'{math.degrees(pose["pitch"]):.2f}, yaw {math.degrees(pose["yaw"]):.2f} deg; '
                  f'{pose["corners"]} corners, {pose["rms"]:.3f} px')
    elif args.step == 'tilt':
        model, K, D, size = read_camera_info(args.camera_info)
        views, names, _ = load_views(board, args.dir)
        tilt = level_tilt(board, model, K, D, views)
        print(f'from {len(views)} views: pitch {math.degrees(tilt["pitch"]):.2f} deg (positive '
              f'looks down), roll {math.degrees(tilt["roll"]):.2f} deg; rows off level in each '
              f'view {", ".join(f"{v:.1f}" for v in tilt["off_level"])} deg; eigenvalues '
              f'{tilt["flat"]:.4f} and {tilt["spread"]:.4f}, the second well above the first')
        for leave, name in enumerate(names):
            part = level_tilt(board, model, K, D, views[:leave] + views[leave + 1:])
            print(f'  without {name}: pitch {math.degrees(part["pitch"]):.2f}, roll '
                  f'{math.degrees(part["roll"]):.2f}')


if __name__ == '__main__':
    main()
