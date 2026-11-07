"""Mission tests. No ROS needed: python -m unittest discover test"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from team_takeoff_land.mission_logic import (  # noqa: E402
    DONE, FAILED, HOVERING, LAND, TAKEOFF, WAIT_HOVER, WAIT_LANDED,
    WAIT_OFFBOARD, Mission, MissionConfig)


def st(mode='passthrough', offboard=True):
    return {'mode': mode, 'offboard': offboard}


class MissionTests(unittest.TestCase):

    def setUp(self):
        self.m = Mission(MissionConfig(hover_s=1.0, max_flight_s=30.0,
                                       retry_s=1.0, takeoff_attempts=3))

    def fly_to_hover(self):
        self.assertEqual(self.m.step(st(), 0.0), TAKEOFF)
        self.m.on_response(TAKEOFF, True, 'ok', 0.1)
        self.m.step(st('takeoff'), 1.0)
        self.m.step(st('hover'), 5.0)
        self.assertEqual(self.m.phase, HOVERING)

    def test_waits_for_state_and_offboard(self):
        self.assertIsNone(self.m.step(None, 0.0))
        self.assertIsNone(self.m.step(st(offboard=False), 0.0))
        self.assertEqual(self.m.phase, WAIT_OFFBOARD)

    def test_full_sequence(self):
        self.fly_to_hover()
        self.assertIsNone(self.m.step(st('hover'), 5.5))       # hovering 0.5 s
        self.assertEqual(self.m.step(st('hover'), 6.0), LAND)  # 1 s reached
        self.m.on_response(LAND, True, 'landing', 6.1)
        self.assertEqual(self.m.phase, WAIT_LANDED)
        self.assertIsNone(self.m.step(st('landing'), 8.0))
        self.m.step(st('landed', offboard=False), 12.0)
        self.assertEqual(self.m.phase, DONE)
        self.assertIsNone(self.m.step(st(), 20.0))  # never flies again

    def test_no_double_call_while_pending(self):
        self.assertEqual(self.m.step(st(), 0.0), TAKEOFF)
        self.assertIsNone(self.m.step(st(), 5.0))

    def test_takeoff_refusal_retries_then_fails(self):
        for t in range(3):
            self.assertEqual(self.m.step(st(), float(t)), TAKEOFF)
            self.m.on_response(TAKEOFF, False, 'refused', float(t))
        self.assertIsNone(self.m.step(st(), 10.0))
        self.assertEqual(self.m.phase, FAILED)

    def test_service_not_up_does_not_use_attempts(self):
        for t in range(10):
            self.assertEqual(self.m.step(st(), float(t)), TAKEOFF)
            self.m.on_no_response(TAKEOFF)
        self.assertEqual(self.m.phase, WAIT_OFFBOARD)

    def test_land_refusal_retries(self):
        self.fly_to_hover()
        self.assertEqual(self.m.step(st('hover'), 6.0), LAND)
        self.m.on_response(LAND, False, 'refused', 6.0)
        self.assertIsNone(self.m.step(st('hover'), 6.5))
        self.assertEqual(self.m.step(st('hover'), 7.0), LAND)

    def test_max_flight_time_forces_land(self):
        self.m.step(st(), 0.0)
        self.m.on_response(TAKEOFF, True, 'ok', 0.0)
        self.assertIsNone(self.m.step(st('takeoff'), 29.0))
        self.assertEqual(self.m.step(st('takeoff'), 30.0), LAND)

    def test_pilot_abort_stops_mission(self):
        self.m.step(st(), 0.0)
        self.m.on_response(TAKEOFF, True, 'ok', 0.0)
        self.assertEqual(self.m.phase, WAIT_HOVER)
        self.assertIsNone(self.m.step(st('aborted', offboard=False), 2.0))
        self.assertEqual(self.m.phase, FAILED)
        self.assertIsNone(self.m.step(st(), 40.0))


if __name__ == '__main__':
    unittest.main()
