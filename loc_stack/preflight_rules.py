"""Pre-flight GO / NO-GO rules for drone 7. Pure Python, no ROS: tested offline.

`evaluate(host, live, cfg)` turns two dicts of measurements into a list of checks:
    host  from preflight.sh on the Jetson (clock, power mode, stack, tmux panes)
    live  from preflight_check.py in the container (rates, VIO, PX4 state over DDS)
Each check is (status, name, detail, fix). Status: PASS, WARN, WAIT (not ready yet,
time will fix it), FAIL (a person must act). The verdict is the worst status.

Where a threshold comes from a measurement, the comment says which.
"""
from dataclasses import dataclass

PASS, WARN, WAIT, FAIL = 'PASS', 'WARN', 'WAIT', 'FAIL'
_RANK = {PASS: 0, WARN: 1, WAIT: 2, FAIL: 3}
VERDICT = {PASS: 'GO', WARN: 'GO (read the warnings)', WAIT: 'WAIT', FAIL: 'NO-GO'}


@dataclass
class Config:
    drone_id: int = 7
    profile: str = 'stock'          # 'stock' (no depth) or 'depth'
    min_stack_age_s: float = 150.0  # ground3: odometry flowing ~2.5 min after a full restart
    min_cam_hz: float = 28.0        # stock: 30.0 by header stamp (restore check, 2026-11-06)
    min_vio_hz: float = 28.0
    min_imu_hz: float = 150.0       # ~199 Hz measured
    min_bridge_hz: float = 25.0
    max_still_wander_m: float = 0.05
    max_vio_step_m: float = 0.05    # SUMMARY: largest normal step 4.7 cm; jumps were 0.17-14.6 m
    max_eph_m: float = 0.10         # ground3: 3.8-4.0 cm steady
    min_fuse_frac: float = 0.98     # cs_ev_pos / cs_ev_yaw fused share of the window
    ground_rng_m: tuple = (0.02, 0.30)   # dist_bottom sitting on the floor (~0.10 m seen)
    max_origin_dz_m: float = 0.10   # drone height vs flight_interface's altitude origin
    max_vio_origin_xy_m: float = 2.0
    min_cell_v: float = 3.85        # 4S pack, per cell, at rest
    min_remaining: float = 0.6
    boot_grace_s: float = 240.0     # cold boot: clock, stack and EKF2 settle within this


def _f(v, nd=2):
    return '-' if v is None else (f'{v:.{nd}f}' if isinstance(v, float) else str(v))


def evaluate(host, live, cfg=None):
    cfg = cfg or Config()
    out = []

    def add(status, name, detail, fix=''):
        out.append((status, name, detail, fix))

    # ---------------- Jetson ----------------
    ntp = host.get('ntp_synced')
    if ntp is True:
        add(PASS, 'clock', 'NTP synchronized')
    elif host.get('has_network'):
        add(FAIL, 'clock', 'not NTP-synced, but the Jetson has a network: a sync can step '
            'the clock mid-run, and the stack service then restarts the whole stack',
            'wait for sync (timedatectl), then do a full restart: preflight.sh --fix')
    else:
        add(WARN, 'clock', 'not NTP-synced and no network: the clock will not step '
            'during the run, but log timestamps are wrong')

    up = host.get('uptime_s')
    if up is not None:
        add(PASS, 'jetson uptime', f'{up:.0f} s since boot ({up / 60:.1f} min)')
    cores = host.get('nproc')
    add(PASS if cores == 6 else FAIL, 'cpu cores', f'{cores} online (want 6)',
        '' if cores == 6 else 'old cloned DTB (skybeta bug #16): sudo host/jetson_uart_dma_fix.sh apply, reboot')
    pm = host.get('power_mode') or ''
    add(PASS if 'MAXN' in pm else WARN, 'power mode', pm or 'unknown',
        '' if 'MAXN' in pm else 'host/provision.sh --from power')

    age = host.get('stack_up_s')
    if not host.get('stack_running'):
        add(FAIL, 'stack', 'skybeta-stack container not running',
            'systemctl status skybeta-stack; journalctl -u skybeta-stack -n 50')
    else:
        dead = [p for p, alive in (host.get('panes') or {}).items() if not alive]
        missing = [p for p in ('agent', 'slam', 'bridge', 'competitor')
                   if p not in (host.get('panes') or {})]
        if dead or missing:
            add(FAIL, 'stack windows', f'dead {dead} missing {missing}',
                'preflight.sh --fix (full restart)')
        else:
            add(PASS, 'stack windows', 'agent, slam, bridge, competitor alive')
        if age is not None and age < cfg.min_stack_age_s:
            add(WAIT, 'stack age', f'up {age:.0f} s; give it {cfg.min_stack_age_s:.0f} s '
                '(odometry took ~2.5 min after a full restart)', 'run preflight again shortly')
        else:
            add(PASS, 'stack age', f'up {_f(age, 0)} s')

    slam_cmd = (host.get('slam_cmd') or '').strip().strip('"')
    if not slam_cmd:
        add(PASS, 'slam pipeline', 'started with the stack (no slam-only restart since)')
    else:
        add(FAIL if cfg.profile == 'stock' else WARN, 'slam pipeline',
            f'slam pane was restarted on its own: {slam_cmd[:90]}. Slam-only restarts and '
            'IMU fusion preceded the 14 m startup jumps and EKF2 heading flips (ground1/2); '
            'only a full restart tested clean (ground3)', 'preflight.sh --fix (full restart)')

    stray = host.get('stray_procs') or []
    add(WARN if stray else PASS, 'stray recorders',
        ', '.join(stray) if stray else 'none',
        'preflight.sh --fix stops them' if stray else '')
    disk = host.get('disk_free_gb')
    add(PASS if disk is None or disk >= 2 else WARN, 'disk', f'{_f(disk, 1)} GB free')

    # ---------------- sensors / VIO ----------------
    hp = host.get('slam_profile')
    if hp is not None and hp != cfg.profile:
        add(FAIL, 'camera profile', f'stack is configured for "{hp}" but this check expects '
            f'"{cfg.profile}"', f'run with --profile {hp}, or change ~/team_ws/slam_profile '
            'and do a full restart (--fix)')
    # With depth on, the camera link carries depth too: measured left/right ~27.5 Hz,
    # VIO ~25-26 Hz on drone 7 (2026-11-06), so the depth profile has its own floor.
    cam_lo = cfg.min_cam_hz if cfg.profile == 'stock' else 25.0
    vio_lo = cfg.min_vio_hz if cfg.profile == 'stock' else 24.0
    r = live.get('rates') or {}
    for key, name, lo in (('left', 'left camera', cam_lo),
                          ('right', 'right camera', cam_lo),
                          ('imu', 'camera IMU', cfg.min_imu_hz),
                          ('vio', 'VIO odometry', vio_lo),
                          ('bridge', 'vision -> PX4 bridge', cfg.min_bridge_hz)):
        hz = r.get(key)
        if hz is None or hz == 0:
            young = age is not None and age < cfg.min_stack_age_s
            add(WAIT if young else FAIL, name, 'no messages',
                'still starting up' if young else 'check the stack windows (tmux attach)')
        else:
            add(PASS if hz >= lo else WARN, name, f'{hz:.1f} Hz (want >= {lo:.0f})')
    depth_hz = r.get('depth') or 0
    if cfg.profile == 'stock':
        add(PASS if not depth_hz else FAIL, 'depth off', f'depth {depth_hz:.1f} Hz' if depth_hz
            else 'no depth stream', '' if not depth_hz else 'preflight.sh --fix (full restart)')
    else:
        add(PASS if depth_hz >= 20 else FAIL, 'depth on',
            f'depth {depth_hz:.1f} Hz' if depth_hz else 'NO depth stream: the slam window '
            'fell back to the stock pipeline (see its log), so obstacle avoidance is blind',
            '' if depth_hz >= 20 else 'tmux attach, look at the slam window; then --fix')

    v = live.get('vio') or {}
    if v.get('n'):
        lost = [s for s in v.get('states', []) if s != 1]
        add(PASS if not lost else FAIL, 'VIO tracking',
            'tracking the whole window' if not lost else f'states seen {v.get("states")}',
            '' if not lost else 'point the cameras at texture, keep still, re-run')
        add(PASS if (v.get('max_step_m') or 0) <= cfg.max_vio_step_m else FAIL, 'VIO jumps',
            f'largest frame step {_f(v.get("max_step_m"), 3)} m',
            '' if (v.get('max_step_m') or 0) <= cfg.max_vio_step_m else 'preflight.sh --fix')
        w = v.get('wander_m') or 0
        add(PASS if w <= cfg.max_still_wander_m else WARN, 'VIO still',
            f'moved {w:.3f} m during the window (drone should be still)')
        p = v.get('pos_last')
        if p is not None:
            dxy = (p[0] ** 2 + p[1] ** 2) ** 0.5
            add(PASS if dxy <= cfg.max_vio_origin_xy_m else WARN, 'VIO vs its origin',
                f'{dxy:.2f} m from where SLAM started (z {p[2]:+.2f})',
                '' if dxy <= cfg.max_vio_origin_xy_m else
                'fine if the drone was carried here after start-up; if it never moved, that '
                'is a startup jump: preflight.sh --fix')

    # ---------------- flight interface / graph ----------------
    st = live.get('skybeta_state') or {}
    org, odom = st.get('origin_enu'), st.get('odom_enu')
    if org and odom:
        dz = odom[2] - org[2]
        add(PASS if abs(dz) <= cfg.max_origin_dz_m else FAIL, 'altitude origin',
            f'drone is {dz:+.2f} m from the flight interface\'s altitude zero (its 0.3-3.0 m '
            'band is measured from the first odometry it saw)',
            '' if abs(dz) <= cfg.max_origin_dz_m else
            'the competitor window started at another height: put the drone on the launch '
            'mark and restart it (or preflight.sh --fix)')
    elif live.get('flight_interface', {}).get('present'):
        add(WARN, 'altitude origin', 'no origin in /skybeta/state yet')
    fi = live.get('flight_interface') or {}
    fc = live.get('fc') or {}
    if not fi.get('present'):
        add(FAIL, 'flight interface', '/flight_interface node not running',
            'check the competitor window')
    else:
        ts = fi.get('target_system')
        sid = fc.get('system_id')
        if sid is not None and ts is not None:
            add(PASS if ts == sid else FAIL, 'offboard target id',
                f'flight_interface targets system {ts}, PX4 is system {sid}',
                '' if ts == sid else
                'PX4 ignores commands for another id, so Offboard never engages. Launch '
                f'flight_interface with target_system:={sid} (start_test.sh does this)')
        else:
            add(WARN, 'offboard target id', f'target {ts}, PX4 system id {sid}')
        add(PASS if fi.get('auto_offboard_on_arm') else FAIL, 'auto offboard',
            f'auto_offboard_on_arm = {fi.get("auto_offboard_on_arm")}')
    for topic, n in (live.get('publishers') or {}).items():
        if topic.startswith('/skybeta/setpoint'):
            add(PASS if n <= 1 else FAIL, f'publishers {topic}', f'{n}',
                '' if n <= 1 else 'two nodes send setpoints (competitor_example still running?)')
        else:
            add(PASS if n == 1 else FAIL, f'publishers {topic}', f'{n} (want exactly 1)',
                '' if n == 1 else 'a duplicate flight_interface/bridge, or it is not running')
    sp = sum(n for t, n in (live.get('publishers') or {}).items() if t.startswith('/skybeta/setpoint'))
    if fi.get('present') and sp == 0:
        add(WARN, 'team setpoints', 'nobody publishes /skybeta/setpoint/*: the flight '
            'interface will refuse Offboard ("no team setpoint received")')

    # ---------------- Pixhawk ----------------
    if not fc.get('link'):
        add(FAIL, 'pixhawk link', 'no vehicle_status from PX4',
            'Pixhawk powered (battery)? If it was on before a stack restart, power-cycle it '
            'and wait ~20 s')
        return out
    add(PASS, 'pixhawk link', f'system id {fc.get("system_id")}')
    sid = fc.get('system_id')
    add(PASS if sid == cfg.drone_id else WARN, 'pixhawk id',
        f'MAV_SYS_ID {sid}, DRONE_ID {cfg.drone_id}')
    add(PASS if fc.get('armed') is False else FAIL, 'disarmed',
        'disarmed' if fc.get('armed') is False else 'ARMED: run pre-flight before arming')
    add(PASS if live.get('landed') else WARN, 'landed', f'landed = {live.get("landed")}')
    pf = fc.get('preflight_ok')
    fs = live.get('failsafe_true') or []
    add(PASS if pf else FAIL, 'PX4 arming checks', 'passing' if pf else
        'failing' + (f': {", ".join(fs)}' if fs else ''),
        '' if pf else 'RC transmitter on? battery (not USB) power? Pixhawk USB unplugged?')

    b = live.get('battery') or {}
    if not b.get('connected'):
        add(FAIL, 'battery', 'no battery seen (on USB power?)', 'connect the flight battery')
    else:
        cells = b.get('cells') or 4
        vc = (b.get('voltage') or 0) / cells
        rem = b.get('remaining')
        ok = vc >= cfg.min_cell_v and (rem is None or rem < 0 or rem >= cfg.min_remaining)
        add(PASS if ok else WARN, 'battery', f'{b.get("voltage"):.2f} V ({vc:.2f} V/cell), '
            f'{_f(rem)} remaining', '' if ok else 'swap to a charged pack')

    lp = live.get('lpos') or {}
    add(PASS if lp.get('xy_valid_all') else FAIL, 'EKF2 xy valid',
        'valid the whole window' if lp.get('xy_valid_all') else 'not valid (vision not fused?)')
    eph = lp.get('eph_max')
    add(PASS if eph is not None and eph <= cfg.max_eph_m else FAIL, 'EKF2 accuracy',
        f'eph max {_f(eph, 3)} m')
    rc = lp.get('reset_change') or 0
    add(PASS if rc == 0 else FAIL, 'EKF2 resets', f'{rc} xy resets in the window',
        '' if rc == 0 else 'EKF2 is jumping: preflight.sh --fix')
    jm = lp.get('xy_jump_max') or 0
    add(PASS if jm <= 0.05 else FAIL, 'EKF2 jumps', f'largest step {jm:.3f} m')
    fl = live.get('flags') or {}
    for key, name in (('cs_ev_pos', 'vision position fused'), ('cs_ev_yaw', 'vision yaw fused')):
        frac = fl.get(key)
        add(PASS if frac is not None and frac >= cfg.min_fuse_frac else FAIL, name,
            f'{100 * (frac or 0):.0f} % of the window',
            '' if (frac or 0) >= cfg.min_fuse_frac else 'EKF2_EV_CTRL=9 loaded? bridge running?')
    mag = fl.get('cs_mag') or 0
    add(PASS if mag == 0 else FAIL, 'compass not used', f'cs_mag on {100 * mag:.0f} % of the window',
        '' if mag == 0 else 'compass fallback = the heading-flip glitch (motion1). Do not fly; '
        'ask organizers about EKF2_MAG_TYPE')
    rng = fl.get('cs_rng_hgt')
    add(PASS if rng and rng >= cfg.min_fuse_frac else WARN, 'height from rangefinder',
        f'cs_rng_hgt {100 * (rng or 0):.0f} %')
    db = (lp.get('dist_bottom_min'), lp.get('dist_bottom_max'))
    lo, hi = cfg.ground_rng_m
    ok = lp.get('dist_bottom_valid_all') and db[0] is not None and lo <= db[0] and db[1] <= hi
    add(PASS if ok else WARN, 'rangefinder on ground',
        f'dist_bottom {_f(db[0], 3)}-{_f(db[1], 3)} m, valid {lp.get("dist_bottom_valid_all")}',
        '' if ok else 'run preflight.sh --lift to see whether it tracks height')

    # Cold boot: the clock, the stack and EKF2 need a few minutes. Inside the grace
    # period these are "not ready yet" (WAIT), not failures. After it, a Pixhawk that is
    # still on the compass gets the cold-boot explanation (2026-11-07 cold boot, drone 7).
    up = host.get('uptime_s')
    boot_items = {'clock', 'stack', 'EKF2 xy valid', 'vision position fused',
                  'vision yaw fused', 'compass not used', 'PX4 arming checks'}
    for i, (status, name, detail, fix) in enumerate(out):
        if name not in boot_items or status != FAIL:
            continue
        if up is not None and up < cfg.boot_grace_s:
            out[i] = (WAIT, name, detail, f'{up:.0f} s after boot; normal while starting up')
        elif name in ('vision position fused', 'vision yaw fused', 'compass not used') \
                and (fl.get('cs_mag') or 0) > 0.5:
            out[i] = (status, name, detail,
                      'EKF2 started on the compass before the camera was up (cold boot) '
                      'and stays there. Reboot the Pixhawk now that VIO is running.')
    return out


def verdict(checks):
    worst = max((c[0] for c in checks), key=_RANK.get, default=PASS)
    return worst, VERDICT[worst]


def lift_report(samples, min_rise_m=0.3):
    """samples: [(dist_bottom, valid, vio_z)] while someone lifts the drone ~0.5 m."""
    good = [(d, z) for d, v, z in samples if v and d is not None]
    if len(good) < 10:
        return FAIL, f'only {len(good)} valid rangefinder samples'
    d = [g[0] for g in good]
    z = [g[1] for g in good if g[1] is not None]
    rise = max(d) - min(d)
    if rise < min_rise_m:
        return FAIL, (f'dist_bottom only moved {rise:.2f} m ({min(d):.2f}-{max(d):.2f}); '
                      'it does not track height')
    msg = f'dist_bottom moved {rise:.2f} m ({min(d):.2f}-{max(d):.2f})'
    if len(z) >= 10:
        vrise = max(z) - min(z)
        ratio = vrise / rise
        msg += f'; VIO z moved {vrise:.2f} m (VIO/range {ratio:.2f})'
        if not 0.8 <= ratio <= 1.2:
            return WARN, msg + ': they disagree by more than 20 %'
    return PASS, msg
