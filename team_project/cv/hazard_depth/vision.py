"""Shared camera-motion helpers for the hazard-detection walk dataset.

Used by the offline labeler (prepare_walk.py) and by the vision-only baseline in the
test suite, so labels and baseline measure rotation the same way.

Conventions
-----------
Camera optical frame: x right, y down, z forward.
yaw   > 0  = camera turns LEFT  (counter-clockwise seen from above, ROS/FLU sign)
pitch > 0  = camera tilts UP
Angles are degrees.
"""
import math

import cv2
import numpy as np

# OAK left camera, from the drone's published camera_info at 1024x640
# (loc_stack/CAMERA_ISSUES.md, mapping/FINDINGS.md). cx is not recorded there; we
# assume the image centre. D = 0 was published, so lens distortion is ignored.
FULL_W, FULL_H = 1024, 640
FX_FULL, FY_FULL = 627.0, 627.0
CX_FULL, CY_FULL = FULL_W / 2.0, 326.0

WORK_SCALE = 0.5  # rotation is estimated at 512x320


def camera_matrix(scale: float = WORK_SCALE) -> np.ndarray:
    return np.array([[FX_FULL * scale, 0, CX_FULL * scale],
                     [0, FY_FULL * scale, CY_FULL * scale],
                     [0, 0, 1]], dtype=np.float64)


def to_gray_small(img: np.ndarray, scale: float = WORK_SCALE) -> np.ndarray:
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if scale != 1.0:
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return img


def rotation_between(g0: np.ndarray, g1: np.ndarray, K=None,
                     min_inliers: int = 25):
    """Camera rotation from gray frame g0 to g1.

    Returns (yaw_deg, pitch_deg, n_inliers); yaw/pitch are NaN when the estimate is
    unreliable (too few tracked features, e.g. a hand over the lens).
    """
    if K is None:
        K = camera_matrix(g0.shape[1] / FULL_W)
    p0 = cv2.goodFeaturesToTrack(g0, maxCorners=400, qualityLevel=0.01, minDistance=8)
    if p0 is None or len(p0) < min_inliers:
        return math.nan, math.nan, 0
    p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0, None, winSize=(21, 21), maxLevel=3)
    pb, stb, _ = cv2.calcOpticalFlowPyrLK(g1, g0, p1, None, winSize=(21, 21), maxLevel=3)
    fb = np.linalg.norm((p0 - pb).reshape(-1, 2), axis=1)
    good = (st.ravel() == 1) & (stb.ravel() == 1) & (fb < 1.0)
    a, b = p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]
    if len(a) < min_inliers:
        return math.nan, math.nan, int(len(a))

    # Essential matrix copes with the parallax of nearby nets; if the camera barely
    # translated it degenerates, so fall back to a rotation-only homography.
    R = None
    n_in = 0
    E, mask = cv2.findEssentialMat(a, b, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is not None and E.shape == (3, 3):
        n_in, R_e, _, _ = cv2.recoverPose(E, a, b, K, mask=mask)
        if n_in >= min_inliers:
            R = R_e
    if R is None:
        H, hmask = cv2.findHomography(a, b, cv2.RANSAC, 2.0)
        if H is None or int(hmask.sum()) < min_inliers:
            return math.nan, math.nan, int(n_in)
        M = np.linalg.inv(K) @ H @ K
        u, _, vt = np.linalg.svd(M)
        R = u @ vt
        if np.linalg.det(R) < 0:
            R = -R
        n_in = int(hmask.sum())

    # R maps points from camera-0 coordinates to camera-1 coordinates (x1 = R x0 + t),
    # i.e. it is the inverse of the camera's own rotation. A right turn (about +y,
    # which points down) moves a straight-ahead point to negative x, so R[0,2] > 0
    # means the camera turned LEFT.
    yaw = math.degrees(math.atan2(R[0, 2], R[2, 2]))
    pitch = math.degrees(math.atan2(R[1, 2], R[2, 2]))  # tilt up -> scene moves down (+y)
    return yaw, pitch, int(n_in)
