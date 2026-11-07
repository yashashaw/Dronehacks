"""Pure helpers for the pad detector and the guide. No ROS, no numpy.

Everything is in the ENU ``odom`` frame unless a name says otherwise. Positions
are (x, y, z) tuples in metres.
"""
import math


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def horizontal_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def step_toward(current, goal, step_m):
    """Move from ``current`` toward ``goal`` by at most ``step_m`` (3-D).

    The guide publishes this instead of the goal itself, so a far goal never
    becomes a far pose setpoint (PX4 would fly that at its own speed limit).
    """
    delta = tuple(g - c for g, c in zip(goal, current))
    distance = norm(delta)
    if step_m <= 0.0:
        return tuple(current)
    if distance <= step_m:
        return tuple(goal)
    scale = step_m / distance
    return tuple(c + d * scale for c, d in zip(current, delta))


def blend(old, new, alpha):
    """Exponential smoothing: ``alpha`` = weight of the new sample (0..1)."""
    return tuple(o + alpha * (n - o) for o, n in zip(old, new))


def is_fresh(received_at, now, timeout_s):
    """True when a message received at ``received_at`` is still usable.

    ``None`` (never received) is stale. A negative age (the clock stepped back)
    is also stale rather than trusted.
    """
    if received_at is None:
        return False
    age = now - received_at
    return 0.0 <= age <= timeout_s


def quat_to_matrix(x, y, z, w):
    """Rotation matrix (row-major, 3x3 nested tuples) of an xyzw quaternion."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError('zero quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def pixel_to_ground(u, v, k, rotation, origin, ground_z=0.0, max_range_m=None):
    """Intersect the camera ray through pixel (u, v) with the plane z = ground_z.

    ``k`` is CameraInfo.k (row-major 3x3, length 9; distortion is ignored).
    ``rotation`` and ``origin`` take the optical frame to ``odom``: the optical
    frame is +x right, +y down, +z forward. Returns an ``odom`` position or
    None when the ray does not hit the ground in front of the camera, or the
    hit is farther than ``max_range_m``.
    """
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    if fx == 0.0 or fy == 0.0:
        return None
    ray = ((u - cx) / fx, (v - cy) / fy, 1.0)
    direction = tuple(sum(rotation[i][j] * ray[j] for j in range(3))
                      for i in range(3))
    if direction[2] >= -1e-9:
        return None  # level or pointing up: never reaches the ground
    s = (ground_z - origin[2]) / direction[2]
    if s <= 0.0:
        return None  # the ground plane is above the camera
    if max_range_m is not None and s * norm(ray) > max_range_m:
        return None
    return (origin[0] + s * direction[0], origin[1] + s * direction[1],
            ground_z)


class Confirmer:
    """Several consecutive, consistent detections make a match.

    ``update(position)`` takes the ground position of this frame's detection,
    or None for a frame with no usable detection. A miss resets the count. A
    detection that jumps more than ``radius_m`` (horizontally) from the
    previous one starts a new streak.
    """

    def __init__(self, frames=5, radius_m=0.5):
        if frames < 1:
            raise ValueError('frames must be >= 1')
        self.frames = int(frames)
        self.radius_m = float(radius_m)
        self.count = 0
        self._previous = None

    def reset(self):
        self.count = 0
        self._previous = None

    def update(self, position):
        if position is None:
            self.reset()
            return False
        if (self._previous is not None and
                horizontal_distance(position, self._previous) > self.radius_m):
            self.count = 0
        self._previous = position
        self.count += 1
        return self.count >= self.frames
