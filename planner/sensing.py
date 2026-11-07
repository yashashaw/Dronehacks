"""TEST ONLY: imperfect sensing, state estimate and vehicle for closed-loop tests.

Every number is a placeholder to be replaced by measurements (camera calibration from the
sim's CameraInfo, drift from the estimator tests, tracking error from logged flights).
Flight code must never import this module.
"""
import math
from dataclasses import dataclass

import numpy as np

from theta_star import OCC


@dataclass
class SensorParams:
    hfov_deg: float = 78.4         # from the drone's CameraInfo: fx = 627 px on a 1024 px wide image
    n_rays: int = 48               # depth image collapsed to one horizontal scan at flight height
    min_range: float = 0.40        # depth.mp4: valid down to 0.42 m (arithmetic said 0.50 m without extended disparity)
    max_range: float = 5.0         # usable range; error grows with the square of distance
    focal_px: float = 627.0
    baseline: float = 0.0754
    disparity_sigma_px: float = 0.3
    bearing_sigma_deg: float = 0.5
    dropout_base: float = 0.10     # the S2 has no IR projector: plain walls give stereo little texture
    dropout_per_m: float = 0.05
    outlier_rate: float = 0.01     # spurious returns
    vfov_deg: float = 54.1         # 640 px tall at 627 px focal length
    belly: float = 0.095           # airframe below the camera (CAD: camera 95 mm above the bottom plate)
    top: float = 0.03              # airframe above the camera (battery top is 21 mm higher)
    margin_v: float = 0.15         # extra height margin for altitude error
    cam_forward: float = 0.105     # camera is this far ahead of the body origin (TF)


@dataclass
class EstimatorParams:
    pos_drift: float = 0.03        # m per sqrt(metre travelled)
    yaw_drift_deg: float = 0.7     # degrees per sqrt(metre travelled)
    white_sigma: float = 0.01      # m, per-sample jitter
    jump_rate: float = 0.005       # jumps per second
    jump_sigma: float = 0.10       # m
    scale: float = 1.0             # drone 7 hand-carry: outbound read 11.6% short (n=1), i.e. 0.884


@dataclass
class VehicleParams:
    tau: float = 0.3               # velocity lag, s
    disturbance: float = 0.05      # m/s random velocity kick per step
    yaw_rate_deg: float = 45.0     # MPC_YAWRAUTO_MAX
    prop_radius: float = 0.0889    # 7 in props
    crash_speed: float = 0.5       # m/s into an obstacle above which a touch becomes a crash (my assumption)
    # rotor centres in the body frame (forward, left), from the PX4 airframe params / CAD
    rotors: tuple = ((0.08, 0.16), (0.08, -0.16), (-0.15, 0.14), (-0.15, -0.14))
    alt_sigma: float = 0.05        # altitude wander, m per sqrt(s)
    alt_tau: float = 1.0           # altitude hold time constant, s
    below: float = 0.095           # airframe extent below / above the camera, for contact
    above: float = 0.03


@dataclass
class LatencyParams:
    """Camera-to-command delay. Measured chain (image stamp -> VIO published): median 58 ms,
    95th percentile 281 ms, worst 373 ms. Depth and planning time come on top of that."""
    fast_s: tuple = (0.04, 0.10)
    slow_s: tuple = (0.20, 0.35)
    p_slow: float = 0.10


def raycast_many(grid, x, y, angles, max_range, step=0.03):
    """Distance along each angle to the first occupied cell (outside the grid counts as
    occupied); NaN where nothing is hit within max_range."""
    t = np.arange(0.0, max_range + step, step)
    ang = np.asarray(angles, dtype=float)
    cx = np.floor((x + np.cos(ang)[:, None] * t[None, :] - grid.ox) / grid.res).astype(np.int64)
    cy = np.floor((y + np.sin(ang)[:, None] * t[None, :] - grid.oy) / grid.res).astype(np.int64)
    inside = (cx >= 0) & (cx < grid.w) & (cy >= 0) & (cy < grid.h)
    cells = np.frombuffer(grid.cells, dtype=np.uint8)
    hit = ~inside | (cells[np.where(inside, cy * grid.w + cx, 0)] == OCC)
    first = hit.argmax(axis=1)
    return np.where(hit.any(axis=1), t[first], np.nan)


def raycast(grid, x, y, angle, max_range, step=0.03):
    """Single-ray version: distance to the first occupied cell, or None."""
    r = raycast_many(grid, x, y, [angle], max_range, step)[0]
    return None if np.isnan(r) else float(r)


def scan(world, pos, z, yaw, p, rng):
    """List of (bearing, range) valid returns in the body frame, with noise and dropouts.

    The depth image is collapsed to one scan: for each direction, the nearest obstacle that
    lies inside the height band the drone occupies (plus margins) and inside the camera's
    vertical field of view at that range. z is the measured height.
    """
    lo, hi = z - p.belly - p.margin_v, z + p.top + p.margin_v
    t = math.tan(math.radians(p.vfov_deg) / 2)
    half = math.radians(p.hfov_deg) / 2
    bearings = np.array([-half + 2 * half * (i + 0.5) / p.n_rays + rng.gauss(0, math.radians(p.bearing_sigma_deg))
                         for i in range(p.n_rays)])
    best = np.full(p.n_rays, np.inf)
    for k in world.overlapping(lo, hi):
        z_lo, z_hi, grid = world.layers[k]
        r = raycast_many(grid, pos[0], pos[1], yaw + bearings, p.max_range)
        seen = ~np.isnan(r)
        rr = np.where(seen, r, 0.0)
        in_view = (max(z_lo, lo) <= z + rr * t) & (min(z_hi, hi) >= z - rr * t)  # inside the vertical FOV
        best = np.where(seen & in_view & (rr < best), rr, best)
    out = []
    for i in range(p.n_rays):
        b = float(bearings[i])
        if rng.random() < p.outlier_rate:
            out.append((b, rng.uniform(p.min_range, p.max_range)))
            continue
        d = float(best[i])
        if math.isinf(d) or d < p.min_range:
            continue
        if rng.random() < p.dropout_base + p.dropout_per_m * d:
            continue
        d += rng.gauss(0, d * d * p.disparity_sigma_px / (p.focal_px * p.baseline))
        if p.min_range <= d <= p.max_range:
            out.append((b, d))
    return out


def detect_pad(grid, pos, yaw, pad, p, rng, max_dist=3.5, p_detect=0.7):
    """Colour-camera cue: (range, bearing) of the pad centre when visible, else None."""
    dx, dy = pad[0] - pos[0], pad[1] - pos[1]
    dist = math.hypot(dx, dy)
    bearing = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
    if dist > max_dist or abs(bearing) > math.radians(p.hfov_deg) / 2 or rng.random() > p_detect:
        return None
    blocked = raycast(grid, pos[0], pos[1], math.atan2(dy, dx), dist)
    if blocked is not None and blocked < dist - 0.1:
        return None
    return dist + rng.gauss(0, 0.03 * dist + 0.02), bearing + rng.gauss(0, math.radians(1.5))


class Estimator:
    """Drifting rigid transform between the true course frame and the estimate's frame.

    The estimate frame has its origin at the launch point. Position and yaw errors grow as a
    random walk with distance travelled, plus rare position jumps.
    """

    def __init__(self, origin, p, rng):
        self.origin, self.p, self.rng = origin, p, rng
        self.theta, self.tx, self.ty = 0.0, 0.0, 0.0

    def advance(self, moved, dt):
        g = self.rng.gauss
        s = math.sqrt(moved)
        self.tx += g(0, self.p.pos_drift * s)
        self.ty += g(0, self.p.pos_drift * s)
        self.theta += g(0, math.radians(self.p.yaw_drift_deg) * s)
        if self.rng.random() < self.p.jump_rate * dt:
            self.tx += g(0, self.p.jump_sigma)
            self.ty += g(0, self.p.jump_sigma)

    def estimate(self, pos, yaw):
        c, s = math.cos(self.theta), math.sin(self.theta)
        x, y = pos[0] - self.origin[0], pos[1] - self.origin[1]
        g = self.rng.gauss
        k = self.p.scale
        return (k * (c * x - s * y) + self.tx + g(0, self.p.white_sigma),
                k * (s * x + c * y) + self.ty + g(0, self.p.white_sigma),
                yaw + self.theta)

    def to_true_velocity(self, vx, vy):
        c, s = math.cos(-self.theta), math.sin(-self.theta)
        k = self.p.scale
        return (c * vx - s * vy) / k, (s * vx + c * vy) / k


class Vehicle:
    def __init__(self, pos, yaw, p, rng, altitude=0.76):
        self.pos, self.yaw, self.vel, self.p, self.rng = list(pos), yaw, [0.0, 0.0], p, rng
        self.alt_cmd, self.z = altitude, altitude

    def step(self, v_cmd, yaw_cmd, dt):
        a = dt / (self.p.tau + dt)
        for i in range(2):
            self.vel[i] += a * (v_cmd[i] - self.vel[i]) + self.rng.gauss(0, self.p.disturbance) * math.sqrt(dt)
            self.pos[i] += self.vel[i] * dt
        self.z += (self.alt_cmd - self.z) * dt / self.p.alt_tau + self.rng.gauss(0, self.p.alt_sigma) * math.sqrt(dt)
        err = (yaw_cmd - self.yaw + math.pi) % (2 * math.pi) - math.pi
        limit = math.radians(self.p.yaw_rate_deg) * dt
        self.yaw += max(-limit, min(limit, err))
