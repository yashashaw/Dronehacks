import math

from px4_square.geometry import ned_to_enu, square_corners_enu


def close(a, b):
    return all(abs(p - q) < 1e-9 for p, q in zip(a, b))


def test_ned_to_enu():
    assert ned_to_enu(1.0, 2.0, -3.0) == (2.0, 1.0, 3.0)


def test_square_facing_north():
    c = square_corners_enu((0.0, 0.0, 0.0), 0.0, 1.0, 1.0)
    assert close(c[0], (0, 1, 1))     # forward = North = ENU +y
    assert close(c[1], (-1, 1, 1))    # then left = West
    assert close(c[2], (-1, 0, 1))
    assert close(c[3], (0, 0, 1))


def test_square_facing_east_offset_start():
    c = square_corners_enu((5.0, 5.0, 0.2), math.pi / 2, 0.5, 1.2)
    assert close(c[0], (5.5, 5.0, 1.2))   # forward = East
    assert close(c[1], (5.5, 5.5, 1.2))   # left of East = North
    assert close(c[3], (5.0, 5.0, 1.2))
