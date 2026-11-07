"""State-machine tests. No ROS needed: python -m unittest discover test"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from team_takeoff_land.logic import (  # noqa: E402
    ABORTED, HOVER, LANDED, LANDING, PASSTHROUGH, TAKEOFF, Config, Inputs,
    TakeoffLand)


def snap(now=0.0, pos=(0.0, 0.0, 0.0), link=True, armed=True, landed=True,
         offboard=True, land_engaged=False):
    return Inputs(now=now, position=pos, link=link, armed=armed,
                  landed=landed, offboard=offboard, land_engaged=land_engaged)


class TakeoffTests(unittest.TestCase):

    def setUp(self):
        self.tl = TakeoffLand(Config(takeoff_altitude_m=1.0, settle_s=1.0))

    def test_passthrough_forwards_planner(self):
        out = self.tl.step(snap())
        self.assertTrue(out.forward_planner)
        self.assertIsNone(out.setpoint)

    def test_takeoff_refused_without_offboard_or_arm(self):
        self.assertFalse(self.tl.request_takeoff(snap(offboard=False))[0])
        self.assertFalse(self.tl.request_takeoff(snap(armed=False))[0])
        self.assertFalse(self.tl.request_takeoff(snap(pos=None))[0])
        self.assertEqual(self.tl.mode, PASSTHROUGH)

    def test_takeoff_uses_ground_height_not_current(self):
        self.tl.step(snap(pos=(0, 0, 0.05), landed=True))  # records ground
        # Flight interface clamp already lifted it to 0.3 m before takeoff.
        ok, _ = self.tl.request_takeoff(snap(pos=(0.2, 0.1, 0.3),
                                             landed=False))
        self.assertTrue(ok)
        self.assertAlmostEqual(self.tl.target[2], 1.05)
        self.assertEqual(self.tl.target[:2], (0.2, 0.1))

    def test_takeoff_cuts_planner_and_settles_into_hover(self):
        self.tl.request_takeoff(snap())
        out = self.tl.step(snap(now=0.5, pos=(0, 0, 0.5), landed=False))
        self.assertFalse(out.forward_planner)
        self.assertEqual(out.setpoint, (0.0, 0.0, 1.0))
        self.tl.step(snap(now=1.0, pos=(0, 0, 0.95), landed=False))
        self.assertEqual(self.tl.mode, TAKEOFF)  # settle timer started
        self.tl.step(snap(now=2.1, pos=(0, 0, 1.0), landed=False))
        self.assertEqual(self.tl.mode, HOVER)
        out = self.tl.step(snap(now=2.2, pos=(0, 0, 1.0), landed=False))
        self.assertFalse(out.forward_planner)
        self.assertEqual(out.setpoint, (0.0, 0.0, 1.0))

    def test_settle_resets_when_leaving_radius(self):
        self.tl.request_takeoff(snap())
        self.tl.step(snap(now=1.0, pos=(0, 0, 1.0), landed=False))
        self.tl.step(snap(now=1.5, pos=(0, 0, 0.5), landed=False))
        self.tl.step(snap(now=2.2, pos=(0, 0, 1.0), landed=False))
        self.assertEqual(self.tl.mode, TAKEOFF)

    def test_release_after_takeoff_option(self):
        tl = TakeoffLand(Config(release_after_takeoff=True, settle_s=0.0))
        tl.request_takeoff(snap())
        tl.step(snap(now=1.0, pos=(0, 0, 1.0), landed=False))
        tl.step(snap(now=1.1, pos=(0, 0, 1.0), landed=False))
        self.assertEqual(tl.mode, PASSTHROUGH)

    def test_release_from_hover(self):
        self.tl.mode = HOVER
        self.assertTrue(self.tl.request_release(snap())[0])
        self.assertEqual(self.tl.mode, PASSTHROUGH)

    def test_offboard_lost_aborts_then_resets_on_disarm(self):
        self.tl.request_takeoff(snap())
        out = self.tl.step(snap(now=1.0, offboard=False, landed=False))
        self.assertEqual(self.tl.mode, ABORTED)
        self.assertIsNone(out.setpoint)
        self.assertFalse(out.forward_planner)
        self.tl.step(snap(now=2.0, armed=False, offboard=False))
        self.assertEqual(self.tl.mode, PASSTHROUGH)


class LandTests(unittest.TestCase):

    def setUp(self):
        self.tl = TakeoffLand(Config(command_retry_s=1.0, land_retries=3,
                                     disarm_after_landed_s=2.0))
        self.air = dict(pos=(1, 2, 1.0), landed=False)

    def test_land_from_passthrough_cuts_planner_and_sends_land(self):
        ok, _ = self.tl.request_land(snap(**self.air))
        self.assertTrue(ok)
        out = self.tl.step(snap(now=0.0, **self.air))
        self.assertTrue(out.send_land)
        self.assertFalse(out.forward_planner)
        self.assertEqual(out.setpoint, (1, 2, 1.0))

    def test_land_retries_then_gives_up_once(self):
        self.tl.request_land(snap(**self.air))
        sent = [self.tl.step(snap(now=t * 0.5, **self.air)).send_land
                for t in range(12)]
        self.assertEqual(sum(sent), 3)
        self.assertEqual(self.tl.mode, LANDING)

    def test_no_resend_after_land_engaged(self):
        self.tl.request_land(snap(**self.air))
        self.tl.step(snap(now=0.0, **self.air))
        out = self.tl.step(snap(now=1.5, land_engaged=True, **self.air))
        self.assertFalse(out.send_land)
        out = self.tl.step(snap(now=3.0, land_engaged=True, **self.air))
        self.assertFalse(out.send_land)

    def test_landed_and_disarmed_is_terminal(self):
        self.tl.request_land(snap(**self.air))
        self.tl.step(snap(now=0.0, land_engaged=True, **self.air))
        self.tl.step(snap(now=5.0, armed=False, landed=True))
        self.assertEqual(self.tl.mode, LANDED)
        out = self.tl.step(snap(now=6.0, armed=True, landed=True))
        self.assertFalse(out.forward_planner)  # planner stays cut off
        self.assertFalse(self.tl.request_takeoff(snap())[0])

    def test_disarm_fallback_when_px4_does_not_disarm(self):
        self.tl.request_land(snap(**self.air))
        self.tl.step(snap(now=0.0, land_engaged=True, **self.air))
        ground = dict(pos=(1, 2, 0.0), landed=True, land_engaged=True)
        self.assertFalse(self.tl.step(snap(now=1.0, **ground)).send_disarm)
        self.assertFalse(self.tl.step(snap(now=2.5, **ground)).send_disarm)
        self.assertTrue(self.tl.step(snap(now=3.1, **ground)).send_disarm)
        self.assertFalse(self.tl.step(snap(now=3.5, **ground)).send_disarm)
        self.assertTrue(self.tl.step(snap(now=4.2, **ground)).send_disarm)

    def test_pilot_takeover_during_land_aborts_without_resending(self):
        self.tl.request_land(snap(**self.air))
        self.tl.step(snap(now=0.0, land_engaged=True, **self.air))
        out = self.tl.step(snap(now=1.0, land_engaged=False, **self.air))
        self.assertEqual(self.tl.mode, ABORTED)
        self.assertFalse(out.send_land)

    def test_land_refused_when_aborted_or_unlinked(self):
        self.assertFalse(self.tl.request_land(snap(link=False))[0])
        self.tl.mode = ABORTED
        self.assertFalse(self.tl.request_land(snap(**self.air))[0])

    def test_release_refused_while_landing(self):
        self.tl.request_land(snap(**self.air))
        self.assertFalse(self.tl.request_release(snap(**self.air))[0])
        self.assertEqual(self.tl.mode, LANDING)

    def test_land_on_ground_disarmed_goes_straight_to_landed(self):
        ok, _ = self.tl.request_land(snap(armed=False, landed=True))
        self.assertTrue(ok)
        self.assertEqual(self.tl.mode, LANDED)


if __name__ == '__main__':
    unittest.main()
