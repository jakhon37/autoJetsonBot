"""Colour shading correction for the CSI camera: measure it on white paper, apply it per frame.

With the Jetson's default tuning, the robot's wide-angle IMX219 turns pink toward the edges of
the picture. With white paper over the lens every part of the picture should have the same colour,
so the red/green and blue/green ratios measured across it show the error. Only those ratios are
corrected, not brightness. So light falling unevenly on the paper doesn't spoil the measurement,
and the lens's dark corners aren't brightened, which would bring up their noise.

Used by csi_camera_server.py. Needs numpy and OpenCV, which JetPack's Python has.
"""

import json
import os
import time

import cv2
import numpy as np

GRID = (32, 24)        # cells across and down for a landscape picture
CENTER = 0.2           # the middle fifth of the picture, each way, sets the target colour
MIN_LEVEL = 0.25       # cells darker than this share of the middle lie outside the lens's image
MIN_GAIN, MAX_GAIN = 0.5, 2.0
MAX_TEXTURE = 0.06     # paper over the lens varies less than this between neighbouring cells
MAX_CLIPPED = 0.01     # share of pixels allowed at full white


class NotPlainError(ValueError):
    """The picture doesn't look like white paper over the lens."""


def orient(grid, flip, to_sensor):
    """Turn a grid between the sensor's orientation and that of frames made with this flip."""
    rotate = {1: cv2.ROTATE_90_COUNTERCLOCKWISE, 2: cv2.ROTATE_180, 3: cv2.ROTATE_90_CLOCKWISE}
    undo = {1: cv2.ROTATE_90_CLOCKWISE, 2: cv2.ROTATE_180, 3: cv2.ROTATE_90_COUNTERCLOCKWISE}
    if flip in rotate:
        return cv2.rotate(grid, (undo if to_sensor else rotate)[flip])
    if flip == 4:
        return cv2.flip(grid, 1)
    if flip == 6:
        return cv2.flip(grid, 0)
    return grid


def _cells(bgr):
    """Average a frame over the grid. Returns the cells and the middle's slices."""
    h, w = bgr.shape[:2]
    gw, gh = GRID if w >= h else GRID[::-1]
    small = cv2.resize(bgr.astype(np.float32), (gw, gh), interpolation=cv2.INTER_AREA)
    rows = slice(int(gh * (0.5 - CENTER / 2)), int(gh * (0.5 + CENTER / 2)) + 1)
    cols = slice(int(gw * (0.5 - CENTER / 2)), int(gw * (0.5 + CENTER / 2)) + 1)
    return small, rows, cols


def _ratios(small, rows, cols):
    """Return red/green and blue/green per cell, their middle values, and the lit cells."""
    b, g, r = cv2.split(small)
    g_mid = float(np.median(g[rows, cols]))
    lit = g >= MIN_LEVEL * max(g_mid, 1.0)
    rg = r / np.maximum(g, 1e-3)
    bg = b / np.maximum(g, 1e-3)
    return rg, bg, float(np.median(rg[rows, cols])), float(np.median(bg[rows, cols])), lit, g_mid


def spread(bgr):
    """Describe how far red/green and blue/green stray from the middle's, over the lit cells."""
    small, rows, cols = _cells(bgr)
    rg, bg, rg0, bg0, lit, _ = _ratios(small, rows, cols)
    red = np.abs(rg / rg0 - 1)[lit]
    blue = np.abs(bg / bg0 - 1)[lit]
    return (f'red/green within {red.max():.1%} of the middle (95% of cells within '
            f'{np.percentile(red, 95):.1%}), blue/green within {blue.max():.1%} '
            f'({np.percentile(blue, 95):.1%}), over {lit.sum()} of {lit.size} cells')


def _fill(grid):
    """Give cells marked NaN the mean of their neighbours, working inward."""
    grid = grid.copy()
    kernel = np.ones((3, 3), np.float32)
    for _ in range(max(grid.shape)):
        missing = np.isnan(grid)
        if not missing.any():
            break
        total = cv2.filter2D(np.nan_to_num(grid), -1, kernel, borderType=cv2.BORDER_REPLICATE)
        count = cv2.filter2D((~missing).astype(np.float32), -1, kernel,
                             borderType=cv2.BORDER_REPLICATE)
        fill = missing & (count > 0)
        grid[fill] = total[fill] / count[fill]
    return np.nan_to_num(grid, nan=1.0)


def measure(avg_bgr, flip):
    """
    Work out red and blue gains from an average frame of white paper over the lens.

    Returns (gain_r, gain_b, report) with the gains on the grid in the sensor's orientation.
    Raises NotPlainError if the frame isn't plain enough to be paper, too dark or clipped.
    """
    clipped = float((avg_bgr >= 250).any(axis=2).mean())
    if clipped > MAX_CLIPPED:
        raise NotPlainError(f'{clipped:.0%} of the picture is at full white: too much light')
    small, rows, cols = _cells(avg_bgr)
    rg, bg, _, _, lit, g_mid = _ratios(small, rows, cols)
    if g_mid < 20:
        raise NotPlainError('the middle of the picture is too dark: is the paper lit?')
    g = small[:, :, 1]
    bright = g >= 0.5 * g_mid
    smooth = cv2.GaussianBlur(g, (5, 5), 0)
    texture = (np.abs(g - smooth) / np.maximum(smooth, 1.0))[bright]
    if np.percentile(texture, 90) > MAX_TEXTURE:
        raise NotPlainError(
            f'the picture varies {np.percentile(texture, 90):.0%} between neighbouring cells, so '
            'it shows a scene, not plain paper: cover the whole lens with white paper')
    # The paper is white, so every cell is brought to neutral grey, with equal red, green and
    # blue. That takes out the pink toward the edges and also the magenta cast the camera's own
    # white balance leaves over the whole picture (red 16% and blue 10% above green on white
    # paper under the room's ceiling light, 2026-11-23).
    gain_r = np.where(lit, 1.0 / rg, np.nan).astype(np.float32)
    gain_b = np.where(lit, 1.0 / bg, np.nan).astype(np.float32)
    gain_r = np.clip(cv2.GaussianBlur(_fill(gain_r), (3, 3), 0), MIN_GAIN, MAX_GAIN)
    gain_b = np.clip(cv2.GaussianBlur(_fill(gain_b), (3, 3), 0), MIN_GAIN, MAX_GAIN)
    report = 'before: ' + spread(avg_bgr)
    return orient(gain_r, flip, True), orient(gain_b, flip, True), report


def save(path, gain_r, gain_b, frames, report, measured=None):
    """Write the gains as JSON, one grid row per line, replacing the file only once complete."""
    meta = {
        'about': 'Colour shading correction for tools/csi_camera_server.py, measured with white '
                 'paper over the lens. Red and blue gains in the sensor\'s orientation (flip 0), '
                 'on a grid stretched over the frame. ./robot.sh camera-color measures it again.',
        'measured': measured or time.strftime('%Y-%m-%d %H:%M'),
        'frames': frames,
        'report': report,
    }

    def rows(grid):
        return ',\n'.join('  ' + json.dumps([round(float(v), 3) for v in row]) for row in grid)

    text = ('{\n' + ''.join(f' {json.dumps(k)}: {json.dumps(v)},\n' for k, v in meta.items())
            + f' "gain_r": [\n{rows(gain_r)}\n ],\n "gain_b": [\n{rows(gain_b)}\n ]\n}}\n')
    json.loads(text)  # the file must stay valid JSON
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(text)
    os.replace(tmp, path)


def load(path):
    """Return (gain_r, gain_b) grids from a file written by save()."""
    with open(path) as f:
        data = json.load(f)
    return (np.array(data['gain_r'], np.float32), np.array(data['gain_b'], np.float32))


class Correction:
    """Blue, green and red gains at the frame's size and orientation, in one 8-bit map."""

    def __init__(self, gain_r, gain_b, flip, size):
        w, h = size

        def stretch(grid):
            full = cv2.resize(orient(grid, flip, False), (w, h), interpolation=cv2.INTER_CUBIC)
            return np.clip(np.round(full * 128), 0, 255).astype(np.uint8)  # 128 means a gain of 1

        # One multiply over all three channels is several times faster than splitting them.
        self.gain = cv2.merge((stretch(gain_b), np.full((h, w), 128, np.uint8), stretch(gain_r)))

    def apply(self, bgr):
        """Return a corrected copy of a BGR frame (green keeps a gain of 1)."""
        return cv2.multiply(bgr, self.gain, scale=1 / 128)
