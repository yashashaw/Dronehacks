"""TEST ONLY: imperfect sensing, state estimate and vehicle for closed-loop tests.

Every number is a placeholder to be replaced by measurements (camera calibration from the
sim's CameraInfo, drift from the estimator tests, tracking error from logged flights).
Flight code must never import this module.
"""
import math
from dataclasses import dataclass

from theta_star import OCC


@dataclass
class SensorParams:
    hfov_deg: float = 72.0         # OAK-D S2 mono pair, standard FOV (wide variant is larger)
    n_rays: int = 48               # depth image collapsed to one horizontal scan at flight height
    min_range: float = 0.35        # stereo blind zone
    max_range: float = 5.0         # usable range; error grows with the square of distance
    focal_px: float = 440.0        # 640 px wide, 72 degree HFOV
    baseline: float = 0.075
    disparity_sigma_px: float = 0.3
    bearing_sigma_deg: float = 0.5
    dropout_base: float = 0.05     # plain walls give stereo no texture, more so at range
    dropout_per_m: float = 0.03
    outlier_rate: float = 0.01     # spurious returns


@dataclass
class EstimatorParams:
    pos_drift: float = 0.03        # m per sqrt(metre travelled)
    yaw_drift_deg: float = 0.7     # degrees per sqrt(metre travelled)
    white_sigma: float = 0.01      # m, per-sample jitter
    jump_rate: float = 0.005       # jumps per second
    jump_sigma: float = 0.10       # m


@dataclass
class VehicleParams:
    tau: float = 0.3               # velocity lag, s
    disturbance: float = 0.05      # m/s random velocity kick per step
    yaw_rate_deg: float = 90.0
    radius: float = 0.234          # half-diagonal of the 13 in frame


def raycast(grid, x, y, angle, max_range, step=0.03):
    """Distance to the first occupied cell (outside the grid counts as occupied), or None."""
    dx, dy = math.cos(angle), math.sin(angle)
    r = 0.0
    while r <= max_range:
        cx, cy = grid.to_cell(x + dx * r, y + dy * r)
        if not grid.inside(cx, cy) or grid.cells[cy * grid.w + cx] == OCC:
            return r
        r += step
    return None


def scan(grid, pos, yaw, p, rng):
    """List of (bearing, range) valid returns in the body frame, with noise and dropouts."""
    out = []
    half = math.radians(p.hfov_deg) / 2
    for i in range(p.n_rays):
        bearing = -half + 2 * half * (i + 0.5) / p.n_rays
        bearing += rng.gauss(0, math.radians(p.bearing_sigma_deg))
        z = raycast(grid, pos[0], pos[1], yaw + bearing, p.max_range)
        if rng.random() < p.outlier_rate:
            out.append((bearing, rng.uniform(p.min_range, p.max_range)))
            continue
        if z is None or z < p.min_range:
            continue
        if rng.random() < p.dropout_base + p.dropout_per_m * z:
            continue
        sigma = z * z * p.disparity_sigma_px / (p.focal_px * p.baseline)
        z += rng.gauss(0, sigma)
        if p.min_range <= z <= p.max_range:
            out.append((bearing, z))
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
        return (c * x - s * y + self.tx + g(0, self.p.white_sigma),
                s * x + c * y + self.ty + g(0, self.p.white_sigma),
                yaw + self.theta)

    def to_true_velocity(self, vx, vy):
        c, s = math.cos(-self.theta), math.sin(-self.theta)
        return c * vx - s * vy, s * vx + c * vy


class Vehicle:
    def __init__(self, pos, yaw, p, rng):
        self.pos, self.yaw, self.vel, self.p, self.rng = list(pos), yaw, [0.0, 0.0], p, rng

    def step(self, v_cmd, yaw_cmd, dt):
        a = dt / (self.p.tau + dt)
        for i in range(2):
            self.vel[i] += a * (v_cmd[i] - self.vel[i]) + self.rng.gauss(0, self.p.disturbance) * math.sqrt(dt)
            self.pos[i] += self.vel[i] * dt
        err = (yaw_cmd - self.yaw + math.pi) % (2 * math.pi) - math.pi
        limit = math.radians(self.p.yaw_rate_deg) * dt
        self.yaw += max(-limit, min(limit, err))
