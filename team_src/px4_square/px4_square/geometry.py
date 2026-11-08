"""Pure geometry for px4_square (no ROS), so it can be tested on a laptop.

PX4 reports its local position in NED (x North, y East, z Down) and its heading as
the angle from North towards East. flight_interface takes ENU setpoints
(x East, y North, z Up) and, with anchor_setpoints off, converts them with
ned = (enu.y, enu.x, -enu.z). So a PX4 NED point goes out as ENU (y, x, -z).
"""
import math


def ned_to_enu(n, e, d):
    return (float(e), float(n), float(-d))


def square_corners_enu(start_enu, heading_ned, side, height_up):
    """Four corners of a counter-clockwise square, all at height_up (ENU z).

    Leg 1 goes along the nose (heading at the moment Offboard started),
    then each leg turns 90 deg left, ending above the start point.
    """
    x0, y0, _ = start_enu
    # nose direction in ENU: NED (cos h, sin h) -> ENU (sin h, cos h)
    fx, fy = math.sin(heading_ned), math.cos(heading_ned)
    # left of the nose in ENU (forward rotated +90 deg about Up)
    lx, ly = -fy, fx
    s = float(side)
    return [
        (x0 + s * fx, y0 + s * fy, height_up),
        (x0 + s * (fx + lx), y0 + s * (fy + ly), height_up),
        (x0 + s * lx, y0 + s * ly, height_up),
        (x0, y0, height_up),
    ]
