"""Contract + shared decision rule for the 400-frame depth decision suite.

Your algorithm is any class with predict(sample) -> dict. Return either or both:

    {"depth": HxW float32,   # any resolution covering the LEFT image's full field of view
     "depth_kind": "metric" | "inverse",  # metric metres, or inverse depth / disparity
                                          # (relative scale is fine: only ratios are used)
     "steer_deg": float}     # optional: your own decision, + = turn LEFT

If steer_deg is missing, the suite applies decide_steer() below to your depth map, so
different depth algorithms are judged by one identical decision rule. Invalid pixels:
NaN, inf or <= 0.

Ground truth for the decision: where the person carrying the camera actually turned in
the next 1 s (yaw_1s, from visual odometry). There is no ground-truth depth for this walk.

    class MyMonoDepth:
        def __init__(self, onnx="fastdepth.onnx"): ...       # load once, not per frame
        def predict(self, s: Sample) -> dict:
            small = cv2.resize(s.left, (224, 224))
            inv = self.session.run(None, {...})[0][0, 0]
            return {"depth": inv, "depth_kind": "inverse"}
"""
import math

import cv2
import numpy as np

FX_FULL, CX_FULL, W_FULL = 627.0, 512.0, 1024


class Sample:
    """left/right: uint8 gray 640x1024 OAK mono pair, stamp-matched (<= 1 ms).
    prev_left: left frame ~0.1 s earlier (may itself be a bad frame).
    rectify: H_left/H_right 3x3 at 1024x640, baseline_m, fx_px."""

    def __init__(self, id, t_s, left, right, prev_left, prev_dt_s, rectify=None):
        self.id, self.t_s, self.prev_dt_s = id, t_s, prev_dt_s
        self.left, self.right, self.prev_left = left, right, prev_left
        self.rectify = rectify or {}


def column_angles_deg(n_cols: int) -> np.ndarray:
    """Bearing of each column centre in the left image, + = LEFT of the optical axis."""
    u = (np.arange(n_cols) + 0.5) * (W_FULL / n_cols)
    return -np.degrees(np.arctan((u - CX_FULL) / FX_FULL))


def decide_steer(depth: np.ndarray, kind: str = "metric", band=(0.30, 0.70), n_cols: int = 64,
                 drone_cols: int = 7, straight_bias: float = 0.15):
    """Pick the most open direction in a horizontal band of the image.

    nearness = 1/depth (or the inverse depth itself), normalised by the image median so
    metric and relative maps behave the same. Per column, take the 80th percentile in the
    band (the closest things), widen by the drone's angular width, add a small penalty for
    leaving straight ahead, and steer to the cheapest column.
    Returns (steer_deg, per-column cost).
    """
    d = np.asarray(depth, dtype=np.float32)
    h = d.shape[0]
    d = d[int(band[0] * h):int(band[1] * h)]
    valid = np.isfinite(d) & (d > 0)
    near = np.full(d.shape, np.nan, np.float32)
    near[valid] = (1.0 / d[valid]) if kind == "metric" else d[valid]
    if valid.sum() < 50:
        return 0.0, np.zeros(n_cols)
    near /= np.nanmedian(near)
    # resample columns
    idx = np.linspace(0, d.shape[1], n_cols + 1).astype(int)
    col = np.array([np.nanpercentile(near[:, a:b], 80) if np.isfinite(near[:, a:b]).any() else np.nan
                    for a, b in zip(idx[:-1], idx[1:])])
    unknown = ~np.isfinite(col)
    col[unknown] = np.nanpercentile(col, 75) if (~unknown).any() else 1.0  # unknown = fairly blocked
    k = np.ones(drone_cols) / drone_cols
    cost = np.convolve(np.pad(col, drone_cols // 2, mode="edge"), k, mode="valid")
    ang = column_angles_deg(n_cols)
    cost = cost + straight_bias * np.abs(ang) / np.abs(ang).max()
    return float(ang[int(np.argmin(cost))]), cost


def rectify_pair(left, right, rect: dict, scale: float):
    S = np.diag([scale, scale, 1.0])
    size = (int(round(1024 * scale)), int(round(640 * scale)))
    HL = S @ np.array(rect["H_left"]) @ np.linalg.inv(S)
    HR = S @ np.array(rect["H_right"]) @ np.linalg.inv(S)
    l = cv2.resize(left, size, interpolation=cv2.INTER_AREA)
    r = cv2.resize(right, size, interpolation=cv2.INTER_AREA)
    return cv2.warpPerspective(l, HL, size), cv2.warpPerspective(r, HR, size), HL


def depth_to_left_view(depth_rect: np.ndarray, HL: np.ndarray) -> np.ndarray:
    """Warp a map from rectified-left coordinates back to the original left image."""
    h, w = depth_rect.shape
    return cv2.warpPerspective(depth_rect, np.linalg.inv(HL), (w, h), flags=cv2.INTER_NEAREST,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=float("nan"))


# ---- drone-shaped flight tube ---------------------------------------------------------
# The hazard check is a 3D box ("tube") in front of the drone, not a patch of the image:
# width  = 0.47 m (13 in frame with props, README-course-dimensions.md) + 1.5 cm -> 0.50 m
#          (shrunk from 0.60 m: same 31/32 hazards caught on the hand labels, fewer false stops;
#          see sweep_tube.py)
# height = drone height (UNKNOWN in the repo; ~0.15 m assumed) + margin        -> 0.25 m
# length = range_m along the flight direction
# Every depth pixel is back-projected to 3D (camera x right, y down, z forward) and tested
# against the tube, so a near object off to the side counts exactly when the drone's body
# would actually sweep through it.
TUBE_W_M, TUBE_H_M, TUBE_RANGE_M = 0.50, 0.25, 2.0
FY_FULL, CY_FULL = 627.0, 326.0
H_FULL = 640


def tube_rect(dist_m, width_m=TUBE_W_M, height_m=TUBE_H_M, bearing_deg=0.0, img_w=W_FULL, img_h=H_FULL):
    """Image rectangle (x0, y0, x1, y1) of the tube's cross-section at dist_m along a
    direction bearing_deg (+ = left), in an image img_w x img_h."""
    k = img_w / float(W_FULL)
    b = np.radians(bearing_deg)
    cxs = []
    for lat in (-width_m / 2.0, width_m / 2.0):
        X = lat * np.cos(b) - dist_m * np.sin(b)
        Z = lat * np.sin(b) + dist_m * np.cos(b)
        cxs.append(CX_FULL + FX_FULL * X / Z)
    zc = dist_m * np.cos(b)
    hy = FY_FULL * (height_m / 2.0) / zc
    return (int(min(cxs) * k), int((CY_FULL - hy) * k), int(max(cxs) * k), int((CY_FULL + hy) * k))


def draw_tube(img, range_m=TUBE_RANGE_M, width_m=TUBE_W_M, height_m=TUBE_H_M, near_m=0.6,
              bearing_deg=0.0, color=(0, 255, 0), thick=1):
    """Perspective outline of the tube: near and far cross-sections joined at the corners."""
    h, w = img.shape[:2]
    n = tube_rect(near_m, width_m, height_m, bearing_deg, w, h)
    f = tube_rect(range_m, width_m, height_m, bearing_deg, w, h)
    cv2.rectangle(img, (n[0], n[1]), (n[2], n[3]), color, thick)
    cv2.rectangle(img, (f[0], f[1]), (f[2], f[3]), color, thick)
    for (xa, ya), (xb, yb) in (((n[0], n[1]), (f[0], f[1])), ((n[2], n[1]), (f[2], f[1])),
                               ((n[0], n[3]), (f[0], f[3])), ((n[2], n[3]), (f[2], f[3]))):
        cv2.line(img, (xa, ya), (xb, yb), color, thick)
    return img


def tube_blocked(depth, range_m=TUBE_RANGE_M, width_m=TUBE_W_M, height_m=TUBE_H_M, bearing_deg=0.0,
                 min_frac=0.005, min_px=8, min_valid=0.3, z_min=0.15, y_up_m=0.0):
    """Would flying along bearing_deg (+ = left) hit something within range_m?
    depth: METRIC map (m) covering the left image's full field of view, any resolution.
    Blocked when at least max(min_px, min_frac * far-section pixels) depth points fall
    inside the tube (a few stray stereo speckles are ignored). If less than min_valid of
    the far cross-section has any depth, the tube is UNKNOWN and counts as blocked.
    z_min: depth points closer than this are ignored (stereo noise / lens-edge junk).
    y_up_m: raise the tube's centre this far above the camera axis (+ = up), e.g. to keep a
    low-carried camera's floor out of it.
    Returns (blocked, clearance_m or None, info dict)."""
    d = np.asarray(depth, np.float32)
    h, w = d.shape
    k = w / float(W_FULL)
    fx, fy, cx, cy = FX_FULL * k, FY_FULL * k, CX_FULL * k, CY_FULL * k
    v, u = np.mgrid[0:h, 0:w].astype(np.float32)
    valid = np.isfinite(d) & (d > z_min)
    Z = np.where(valid, d, 0.0)
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    b = np.radians(bearing_deg)
    along = Z * np.cos(b) - X * np.sin(b)
    lat = X * np.cos(b) + Z * np.sin(b)
    inside = valid & (along > 0) & (along < range_m) & (np.abs(lat) < width_m / 2.0) & (np.abs(Y + y_up_m) < height_m / 2.0)

    x0, y0, x1, y1 = tube_rect(range_m, width_m, height_m, bearing_deg, w, h)
    x0, x1, y0, y1 = max(x0, 0), min(max(x1, x0 + 1), w), max(y0, 0), min(max(y1, y0 + 1), h)
    far = valid[y0:y1, x0:x1]
    far_px = max(far.size, 1)
    need = max(min_px, int(min_frac * far_px))
    n_in = int(inside.sum())
    info = {"points_in_tube": n_in, "needed": need, "far_valid": round(float(far.mean()) if far.size else 0.0, 3)}
    if far.size == 0 or far.mean() < min_valid:
        info["unknown"] = True
        return True, None, info
    if n_in >= need:
        return True, float(np.percentile(along[inside], 10)), info
    return False, None, info


def corridor_blocked(depth, range_m=TUBE_RANGE_M, **kw):
    """Back-compatible name: straight-ahead tube check. Returns (blocked, clearance_m)."""
    blocked, clr, _ = tube_blocked(depth, range_m=range_m, **kw)
    return blocked, clr
