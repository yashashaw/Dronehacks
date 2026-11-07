#!/usr/bin/env python3
"""Offline tests for preflight_rules (no ROS):  python test_preflight_rules.py"""
import copy

import preflight_rules as pr

HOST_OK = {'ntp_synced': True, 'has_network': True, 'nproc': 6, 'power_mode': 'MAXN_SUPER',
           'stack_running': True, 'stack_up_s': 600,
           'panes': {'agent': True, 'slam': True, 'bridge': True, 'competitor': True},
           'slam_cmd': '', 'stray_procs': [], 'disk_free_gb': 20.0}
LIVE_OK = {
    'rates': {'left': 30.0, 'right': 30.0, 'imu': 199.0, 'vio': 30.0, 'bridge': 30.0, 'depth': 0},
    'vio': {'n': 600, 'states': [1], 'max_step_m': 0.004, 'wander_m': 0.01,
            'pos_last': (0.1, 0.05, 0.0)},
    'skybeta_state': {'origin_enu': [0, 0, 0], 'odom_enu': [0.1, 0.05, 0.01]},
    'flight_interface': {'present': True, 'target_system': 7, 'auto_offboard_on_arm': True},
    'publishers': {'/fmu/in/vehicle_visual_odometry': 1, '/fmu/in/offboard_control_mode': 1,
                   '/fmu/in/trajectory_setpoint': 1, '/skybeta/setpoint/pose': 1,
                   '/skybeta/setpoint/twist': 0},
    'fc': {'link': True, 'system_id': 7, 'armed': False, 'preflight_ok': True},
    'landed': True, 'failsafe_true': [],
    'battery': {'connected': True, 'voltage': 16.4, 'remaining': 0.95, 'cells': 4},
    'lpos': {'xy_valid_all': True, 'eph_max': 0.04, 'reset_change': 0, 'xy_jump_max': 0.001,
             'dist_bottom_min': 0.098, 'dist_bottom_max': 0.103, 'dist_bottom_valid_all': True},
    'flags': {'cs_ev_pos': 1.0, 'cs_ev_yaw': 1.0, 'cs_mag': 0.0, 'cs_rng_hgt': 1.0},
}


def run(host=None, live=None, **cfg):
    h = copy.deepcopy(HOST_OK)
    h.update(host or {})
    lv = copy.deepcopy(LIVE_OK)
    for k, v in (live or {}).items():
        if isinstance(v, dict) and isinstance(lv.get(k), dict):
            lv[k].update(v)
        else:
            lv[k] = v
    checks = pr.evaluate(h, lv, pr.Config(**cfg))
    return checks, pr.verdict(checks)[1]


def names(checks, status):
    return {c[1] for c in checks if c[0] == status}


def test_all_good_is_go():
    checks, v = run()
    assert v == 'GO', [c for c in checks if c[0] != 'PASS']


def test_slam_only_restart_is_nogo():
    checks, v = run(host={'slam_cmd': '"bash /root/team_ws/loc_stack/depth/restore_slam.sh"'})
    assert v == 'NO-GO' and 'slam pipeline' in names(checks, 'FAIL')


def test_depth_on_is_nogo_for_stock():
    checks, v = run(live={'rates': {'depth': 26.0}})
    assert 'depth off' in names(checks, 'FAIL')


def test_wrong_target_system_is_nogo():
    checks, v = run(live={'flight_interface': {'target_system': 1}})
    assert 'offboard target id' in names(checks, 'FAIL'), checks


def test_young_stack_waits():
    checks, v = run(host={'stack_up_s': 40}, live={'rates': {'vio': None, 'bridge': None}})
    assert v == 'WAIT', checks
    assert {'stack age', 'VIO odometry'} <= names(checks, 'WAIT')


def test_compass_fallback_is_nogo():
    checks, v = run(live={'flags': {'cs_mag': 0.05, 'cs_ev_yaw': 0.95}})
    assert {'compass not used', 'vision yaw fused'} <= names(checks, 'FAIL')


def test_ekf_reset_is_nogo():
    checks, v = run(live={'lpos': {'reset_change': 3, 'xy_jump_max': 14.6, 'eph_max': 14.1}})
    assert {'EKF2 resets', 'EKF2 jumps', 'EKF2 accuracy'} <= names(checks, 'FAIL')


def test_altitude_origin_off_is_nogo():
    checks, v = run(live={'skybeta_state': {'origin_enu': [0, 0, 0.8], 'odom_enu': [0, 0, 0.0]}})
    assert 'altitude origin' in names(checks, 'FAIL')


def test_no_pixhawk_stops_early():
    checks, v = run(live={'fc': {'link': False}})
    assert v == 'NO-GO' and 'pixhawk link' in names(checks, 'FAIL')
    assert not any(c[1] == 'battery' for c in checks)


def test_usb_power_is_nogo():
    checks, v = run(live={'battery': {'connected': False}, 'fc': {'preflight_ok': False},
                          'failsafe_true': ['battery_unhealthy']})
    assert {'battery', 'PX4 arming checks'} <= names(checks, 'FAIL')


def test_duplicate_setpoint_publishers_is_nogo():
    checks, v = run(live={'publishers': {'/skybeta/setpoint/pose': 2}})
    assert 'publishers /skybeta/setpoint/pose' in names(checks, 'FAIL')


def test_ntp_unsynced_with_network_is_nogo_without_network_warn():
    _, v1 = run(host={'ntp_synced': False, 'has_network': True})
    _, v2 = run(host={'ntp_synced': False, 'has_network': False})
    assert v1 == 'NO-GO' and v2.startswith('GO')


def test_depth_profile_thresholds_and_fallback():
    live = {'rates': {'depth': 26.0, 'left': 27.5, 'right': 27.5, 'vio': 25.5}}
    checks, v = run(host={'slam_profile': 'depth'}, live=live, profile='depth')
    assert v == 'GO', [c for c in checks if c[0] != 'PASS']
    checks, v = run(host={'slam_profile': 'depth'}, live={'rates': {'depth': 0}}, profile='depth')
    assert 'depth on' in names(checks, 'FAIL')


def test_profile_mismatch_is_nogo():
    checks, v = run(host={'slam_profile': 'depth'})          # checking as stock
    assert 'camera profile' in names(checks, 'FAIL')


def test_cold_boot_grace_then_compass_hint():
    early = {'flags': {'cs_ev_pos': 0.0, 'cs_ev_yaw': 0.0, 'cs_mag': 1.0},
             'lpos': {'xy_valid_all': False}}
    checks, v = run(host={'uptime_s': 60, 'ntp_synced': False}, live=early)
    assert v == 'WAIT', [c for c in checks if c[0] not in ('PASS', 'WAIT')]
    checks, v = run(host={'uptime_s': 400}, live=early)
    assert v == 'NO-GO'
    hint = [c for c in checks if c[1] == 'vision position fused'][0][3]
    assert 'Reboot the Pixhawk' in hint, hint


def test_lift_report():
    up = [(0.1 + 0.5 * i / 30, True, 0.5 * i / 30) for i in range(31)]
    assert pr.lift_report(up)[0] == 'PASS'
    stuck = [(0.1, True, 0.5 * i / 30) for i in range(31)]
    assert pr.lift_report(stuck)[0] == 'FAIL'
    scaled = [(0.1 + 0.5 * i / 30, True, 0.35 * i / 30) for i in range(31)]
    assert pr.lift_report(scaled)[0] == 'WARN'


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
    print('ALL PASS')
