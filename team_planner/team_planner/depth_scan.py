"""Depth image -> the (bearing, range) readings the planner consumes. numpy only, no ROS.

For each image column it keeps the nearest valid depth inside the band of heights the airframe
occupies (plus a margin for altitude error), so the floor and the ceiling drop out and anything
the drone could hit stays in. The camera is assumed level (pitch 0, as in the declared mount).

Geometry, camera optical frame: +z forward, +x right, +y down. A pixel (u, v) with depth z
(metres along the optical axis) is at lateral x = (u - cx) / fx * z and down y = (v - cy) / fy * z.
Bearing is positive to the LEFT (the planner's body-frame convention), so bearing = -atan((u-cx)/fx).
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ScanParams:
    belly: float = 0.095        # airframe below the camera, m (CAD: camera is 95 mm above the bottom plate)
    top: float = 0.03           # airframe above the camera, m
    margin_v: float = 0.15      # extra height margin for altitude error, m
    min_range: float = 0.40     # closest depth trusted, m (depth.mp4 showed valid down to 0.42 m)
    max_range: float = 5.0      # farthest depth used, m
    n_bins: int = 48            # readings handed to the planner across the field of view
    min_pixels: int = 2         # valid in-band pixels a column needs before it counts (drops speckle)
    depth_scale: float = 0.001  # 16UC1 millimetres -> metres


def depth_to_scan(depth_raw, fx, fy, cx, cy, p=None):
    """Return (readings, valid_fraction).

    readings: list of (bearing_rad, range_m), left positive, one per occupied bin.
    valid_fraction: share of pixels with any depth (a health signal for plain-wall trouble).
    """
    p = p or ScanParams()
    z = np.asarray(depth_raw, dtype=np.float64) * p.depth_scale
    h, w = z.shape
    valid_any = np.isfinite(z) & (z > 0)
    valid_fraction = float(valid_any.mean()) if z.size else 0.0

    u = np.arange(w, dtype=np.float64)
    v = np.arange(h, dtype=np.float64)
    x = (u[None, :] - cx) / fx * z
    up = -(v[:, None] - cy) / fy * z                       # height above the camera, m
    rng = np.hypot(x, z)                                   # distance in the horizontal plane
    in_band = (valid_any & (up >= -(p.belly + p.margin_v)) & (up <= p.top + p.margin_v)
               & (rng >= p.min_range) & (rng <= p.max_range))

    count = in_band.sum(axis=0)
    col_range = np.where(in_band, rng, np.inf).min(axis=0)
    col_range[count < p.min_pixels] = np.inf
    col_bearing = -np.arctan((u - cx) / fx)

    lo, hi = col_bearing.min(), col_bearing.max()
    if not np.isfinite(col_range).any() or hi <= lo:
        return [], valid_fraction
    bins = np.minimum(((col_bearing - lo) / (hi - lo) * p.n_bins).astype(int), p.n_bins - 1)
    out = []
    for b in range(p.n_bins):
        sel = bins == b
        if not sel.any():
            continue
        r = col_range[sel].min()
        if np.isfinite(r):
            out.append((float(col_bearing[sel].mean()), float(r)))
    out.sort()
    return out, valid_fraction


def default_intrinsics(width=320, height=200, hfov_deg=78.4):
    """Fallback when no CameraInfo has arrived: same field of view as the left mono camera."""
    fx = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    return fx, fx, width / 2, height / 2
