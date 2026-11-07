import pytest

from course_pipeline.guide_machine import GuideConfig, GuideMachine, Stage
from course_pipeline.pad_math import horizontal_distance

PAD = (3.0, 0.0, 0.0)


def make(**config):
    m = GuideMachine(GuideConfig(**config))
    m.on_position((0.0, 0.0, 0.0))
    return m


def to_follow(m, now=0.0):
    """Offboard starts at the origin and the vehicle reaches cruise altitude."""
    m.on_state(True, True, True)
    m.tick(now)
    assert m.stage is Stage.TAKEOFF
    m.on_position((0.0, 0.0, m.cfg.cruise_altitude_m))
    m.on_state(True, True, False)
    m.tick(now + 0.1)
    assert m.stage is Stage.FOLLOW
    return now + 0.1


def confirm_pad(m, now, position=PAD):
    m.on_detection(True, position, now)


def to_approach(m, now, position=PAD):
    confirm_pad(m, now, position)
    m.tick(now)
    assert m.stage is Stage.APPROACH
    return now


def hover_over_pad(m, now, seconds):
    """Tick over the pad, refreshing the detection, for ``seconds``."""
    end = now + seconds
    while now < end:
        now += 0.1
        confirm_pad(m, now)
        m.tick(now)
    return now


def to_land(m, now):
    now = to_approach(m, now)
    m.on_position((PAD[0], PAD[1], 1.0))
    now = hover_over_pad(m, now, m.cfg.align_hold_s + 0.5)
    assert m.stage is Stage.LAND
    return now


def test_no_odometry_commands_nothing():
    m = GuideMachine()
    assert m.tick(0.0) is None


def test_before_offboard_it_holds_the_current_position():
    m = make()
    assert m.tick(0.0) == (0.0, 0.0, 0.0)
    assert m.stage is Stage.HOLD


def test_offboard_start_saves_home_and_climbs_one_step():
    m = make(step_m=0.5)
    m.on_position((2.0, 1.0, 0.1))
    m.on_state(True, True, True)
    target = m.tick(0.0)
    assert m.stage is Stage.TAKEOFF
    assert m.home == (2.0, 1.0, 0.1)
    assert target == pytest.approx((2.0, 1.0, 0.6))


def test_takeoff_ends_at_cruise_altitude():
    m = make(cruise_altitude_m=1.5, arrival_radius_m=0.3)
    m.on_state(True, True, True)
    m.tick(0.0)
    m.on_position((0.0, 0.0, 1.0))
    m.tick(0.1)
    assert m.stage is Stage.TAKEOFF
    m.on_position((0.0, 0.0, 1.25))
    m.tick(0.2)
    assert m.stage is Stage.FOLLOW


def test_follow_steps_toward_a_far_goal_instead_of_jumping():
    m = make(step_m=0.5)
    now = to_follow(m)
    m.on_goal((20.0, 0.0, 1.5), now)
    target = m.tick(now + 0.1)
    assert target == pytest.approx((0.5, 0.0, 1.5))


def test_follow_tracks_a_goal_that_keeps_changing():
    m = make(step_m=0.5)
    now = to_follow(m)
    m.on_goal((20.0, 0.0, 1.5), now)
    assert m.tick(now + 0.1) == pytest.approx((0.5, 0.0, 1.5))
    m.on_goal((0.0, 20.0, 1.5), now + 0.2)
    assert m.tick(now + 0.3) == pytest.approx((0.0, 0.5, 1.5))


def test_stale_goal_holds_the_last_target():
    m = make(step_m=0.5, goal_timeout_s=0.5)
    now = to_follow(m)
    m.on_goal((20.0, 0.0, 1.5), now)
    held = m.tick(now + 0.1)
    m.on_position((5.0, 0.0, 1.5))        # the vehicle moves; the goal does not refresh
    assert m.tick(now + 1.0) == held
    assert m.stage is Stage.FOLLOW


def test_no_goal_ever_holds_at_cruise_altitude():
    m = make()
    now = to_follow(m)
    target = m.tick(now + 0.1)
    assert target[2] == pytest.approx(1.5, abs=0.3)
    assert horizontal_distance(target, (0.0, 0.0, 0.0)) == pytest.approx(0.0)


def test_pad_is_ignored_during_takeoff():
    m = make()
    m.on_state(True, True, True)
    m.tick(0.0)
    confirm_pad(m, 0.1)
    m.tick(0.1)
    assert m.stage is Stage.TAKEOFF


def test_unconfirmed_detection_does_not_start_the_approach():
    m = make()
    now = to_follow(m)
    m.on_detection(False, PAD, now)
    m.tick(now + 0.1)
    assert m.stage is Stage.FOLLOW


def test_pad_near_home_is_the_home_pad_and_is_ignored():
    m = make(home_exclusion_radius_m=1.0)
    now = to_follow(m)
    confirm_pad(m, now + 0.1, (0.4, 0.3, 0.0))
    m.tick(now + 0.1)
    assert m.stage is Stage.FOLLOW


def test_confirmed_pad_starts_the_approach_and_the_goal_is_ignored():
    m = make(step_m=0.5)
    now = to_follow(m)
    now = to_approach(m, now + 0.1)
    m.on_goal((0.0, 20.0, 1.5), now)         # the planner keeps publishing
    m.on_position((1.0, 0.0, 1.0))
    target = m.tick(now + 0.1)
    assert m.stage is Stage.APPROACH
    assert target == pytest.approx((1.5, 0.0, 1.0))   # toward the pad, not the goal


def test_approach_holds_the_last_pad_position_when_detection_drops():
    m = make(step_m=0.5)
    now = to_follow(m)
    now = to_approach(m, now + 0.1)
    m.on_position((1.0, 0.0, 1.0))
    first = m.tick(now + 0.1)
    # no further detections for several seconds
    later = m.tick(now + 5.0)
    assert m.stage is Stage.APPROACH
    assert later == pytest.approx(first)
    assert m.pad == pytest.approx(PAD)


def test_approach_smooths_new_samples_into_the_pad_estimate():
    m = make(pad_smoothing=0.5)
    now = to_follow(m)
    now = to_approach(m, now + 0.1)
    confirm_pad(m, now + 0.1, (4.0, 0.0, 0.0))
    m.tick(now + 0.1)
    assert m.pad == pytest.approx((3.5, 0.0, 0.0))


def test_staying_aligned_long_enough_starts_the_landing():
    m = make(align_radius_m=0.25, align_hold_s=3.0)
    now = to_follow(m)
    to_land(m, now + 0.1)


def test_leaving_alignment_restarts_the_hold_clock():
    m = make(align_radius_m=0.25, align_hold_s=3.0)
    now = to_follow(m)
    now = to_approach(m, now + 0.1)
    m.on_position((PAD[0], PAD[1], 1.0))
    now = hover_over_pad(m, now, 2.0)
    m.on_position((PAD[0] + 1.0, PAD[1], 1.0))     # drifts out of the radius
    now = hover_over_pad(m, now, 0.2)
    m.on_position((PAD[0], PAD[1], 1.0))
    now = hover_over_pad(m, now, 2.0)
    assert m.stage is Stage.APPROACH


def test_landing_lowers_the_pose_and_stops_at_ground_level():
    m = make(descent_rate_mps=0.5)
    now = to_follow(m)
    now = to_land(m, now + 0.1)
    start_z = m.target[2]
    now += 0.1
    confirm_pad(m, now)
    lowered = m.tick(now)
    assert lowered[2] < start_z
    assert (lowered[0], lowered[1]) == pytest.approx(PAD[:2])
    for _ in range(100):
        now += 0.1
        confirm_pad(m, now)
        m.tick(now)
    assert m.target[2] == pytest.approx(PAD[2])
    assert m.stage is Stage.LAND


def test_drifting_off_the_pad_while_landing_goes_back_to_approach():
    m = make(abort_radius_m=0.6)
    now = to_follow(m)
    now = to_land(m, now + 0.1)
    m.on_position((PAD[0] + 1.0, PAD[1], 0.5))
    now += 0.1
    confirm_pad(m, now)
    m.tick(now)
    assert m.stage is Stage.APPROACH


def test_losing_offboard_keeps_the_stage_and_the_last_target():
    m = make(step_m=0.5)
    now = to_follow(m)
    m.on_goal((20.0, 0.0, 1.5), now)
    held = m.tick(now + 0.1)
    m.on_state(False, True, False)       # pilot took over
    m.on_goal((0.0, 20.0, 1.5), now + 0.2)
    confirm_pad(m, now + 0.2)
    assert m.tick(now + 0.3) == held
    assert m.stage is Stage.FOLLOW
    assert any(level == 'warn' for level, _ in m.pop_events())


def test_landed_and_disarmed_after_offboard_ends_the_session():
    m = make()
    now = to_follow(m)
    now = to_land(m, now + 0.1)
    m.on_position((PAD[0], PAD[1], 0.05))
    m.on_state(False, False, True)       # pilot landed and disarmed
    target = m.tick(now + 0.1)
    assert m.stage is Stage.DONE
    assert target == (PAD[0], PAD[1], 0.05)
    # DONE is final: a new confirmed pad does nothing.
    confirm_pad(m, now + 0.2)
    m.tick(now + 0.2)
    assert m.stage is Stage.DONE


def test_landed_and_disarmed_before_any_offboard_is_not_the_end():
    m = make()
    m.on_state(False, False, True)
    m.tick(0.0)
    assert m.stage is Stage.HOLD
