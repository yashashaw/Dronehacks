import math

import pytest

from course_pipeline.pad_math import (
    Confirmer, blend, is_fresh, norm, pixel_to_ground, quat_to_matrix,
    step_toward)

# Pinhole intrinsics, row-major: fx=500, cx=512, fy=500, cy=320.
K = (500.0, 0.0, 512.0, 0.0, 500.0, 320.0, 0.0, 0.0, 1.0)


# -- short steps ---------------------------------------------------------
def test_goal_within_one_step_is_returned_exactly():
    assert step_toward((0, 0, 0), (0.3, 0.0, 0.2), 0.5) == (0.3, 0.0, 0.2)


def test_far_goal_is_cut_to_one_step_along_the_line():
    out = step_toward((1.0, 1.0, 1.0), (101.0, 1.0, 1.0), 0.5)
    assert out == pytest.approx((1.5, 1.0, 1.0))


def test_step_is_three_dimensional():
    out = step_toward((0, 0, 0), (30.0, 40.0, 0.0), 5.0)
    assert norm(out) == pytest.approx(5.0)
    assert out == pytest.approx((3.0, 4.0, 0.0))


def test_zero_step_stays_put():
    assert step_toward((1, 2, 3), (9, 9, 9), 0.0) == (1, 2, 3)


def test_blend_weights_the_new_sample():
    assert blend((0, 0, 0), (10, 0, 0), 0.3) == pytest.approx((3.0, 0.0, 0.0))


# -- stale goals ---------------------------------------------------------
def test_never_received_is_stale():
    assert not is_fresh(None, 10.0, 0.5)


def test_recent_message_is_fresh_and_old_one_is_stale():
    assert is_fresh(9.8, 10.0, 0.5)
    assert is_fresh(9.5, 10.0, 0.5)
    assert not is_fresh(9.4, 10.0, 0.5)


def test_clock_stepping_back_is_stale():
    assert not is_fresh(12.0, 10.0, 0.5)


# -- confirmation --------------------------------------------------------
def test_needs_the_full_streak():
    c = Confirmer(frames=3, radius_m=0.5)
    assert not c.update((1.0, 1.0, 0.0))
    assert not c.update((1.1, 1.0, 0.0))
    assert c.update((1.0, 1.1, 0.0))


def test_a_missed_frame_resets_the_streak():
    c = Confirmer(frames=3, radius_m=0.5)
    c.update((1.0, 1.0, 0.0))
    c.update((1.0, 1.0, 0.0))
    assert not c.update(None)
    assert c.count == 0
    assert not c.update((1.0, 1.0, 0.0))


def test_a_jump_starts_a_new_streak():
    c = Confirmer(frames=3, radius_m=0.5)
    c.update((1.0, 1.0, 0.0))
    c.update((1.0, 1.0, 0.0))
    assert not c.update((5.0, 5.0, 0.0))
    assert c.count == 1


def test_stays_confirmed_while_detections_agree():
    c = Confirmer(frames=2, radius_m=0.5)
    c.update((0.0, 0.0, 0.0))
    assert c.update((0.1, 0.0, 0.0))
    assert c.update((0.2, 0.0, 0.0))


def test_confirmer_needs_at_least_one_frame():
    with pytest.raises(ValueError):
        Confirmer(frames=0)


# -- pixel to ground -----------------------------------------------------
# Optical frame: +x right, +y down, +z forward.
LOOKING_DOWN = ((1, 0, 0), (0, -1, 0), (0, 0, -1))
S = math.sqrt(0.5)
PITCHED_45 = ((0, -S, S), (-1, 0, 0), (0, -S, -S))   # forward = +x odom, 45 deg down


def test_center_pixel_of_a_downward_camera_hits_straight_below():
    out = pixel_to_ground(512, 320, K, LOOKING_DOWN, (0.0, 0.0, 2.0))
    assert out == pytest.approx((0.0, 0.0, 0.0))


def test_pixel_offset_scales_with_height():
    # 250 px right of center at fx=500 is a ray 0.5 m sideways per metre of height.
    out = pixel_to_ground(762, 320, K, LOOKING_DOWN, (0.0, 0.0, 2.0))
    assert out == pytest.approx((1.0, 0.0, 0.0))


def test_forward_pitched_camera_hits_the_floor_ahead():
    out = pixel_to_ground(512, 320, K, PITCHED_45, (0.0, 0.0, 1.5))
    assert out == pytest.approx((1.5, 0.0, 0.0))


def test_ground_z_shifts_the_plane():
    out = pixel_to_ground(512, 320, K, LOOKING_DOWN, (0.0, 0.0, 2.0), ground_z=0.5)
    assert out == pytest.approx((0.0, 0.0, 0.5))


def test_level_camera_never_hits_the_ground():
    level = ((0, 0, 1), (-1, 0, 0), (0, -1, 0))   # forward = +x, horizontal
    assert pixel_to_ground(512, 320, K, level, (0.0, 0.0, 1.5)) is None


def test_ground_above_the_camera_is_rejected():
    assert pixel_to_ground(512, 320, K, LOOKING_DOWN, (0.0, 0.0, -1.0)) is None


def test_hits_beyond_max_range_are_rejected():
    assert pixel_to_ground(512, 320, K, LOOKING_DOWN, (0.0, 0.0, 2.0),
                           max_range_m=1.9) is None
    assert pixel_to_ground(512, 320, K, LOOKING_DOWN, (0.0, 0.0, 2.0),
                           max_range_m=2.1) is not None


def test_identity_quaternion_is_identity_matrix():
    m = quat_to_matrix(0, 0, 0, 1)
    assert [v for row in m for v in row] == pytest.approx(
        [1, 0, 0, 0, 1, 0, 0, 0, 1])


def test_quarter_turn_about_z_takes_x_to_y():
    m = quat_to_matrix(0, 0, S, S)
    assert [m[i][0] for i in range(3)] == pytest.approx([0, 1, 0])


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError):
        quat_to_matrix(0, 0, 0, 0)


def test_yaw_quaternion_round_trip():
    import math
    from course_pipeline.pad_math import quat_is_zero, quat_to_yaw, yaw_to_quat
    for yaw in (-3.0, -1.0, 0.0, 0.7, 2.5):
        assert abs(quat_to_yaw(*yaw_to_quat(yaw)) - yaw) < 1e-9
    assert quat_is_zero(0.0, 0.0, 0.0, 0.0)
    assert not quat_is_zero(*yaw_to_quat(0.0))
