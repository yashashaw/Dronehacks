"""Diagnostic: what would the drone do on each of the 400 frames: STRAIGHT, LEFT or RIGHT?

    python -I hazard/depthsuite/dodge_diag.py
    python -I hazard/depthsuite/dodge_diag.py --run-file ../Dronehacks-drone-tests/team_src/dodge_nav/dodge_nav/dodge_nav.py

Uses the FLIGHT code itself (dodge_nav.py: StereoDepth, scan_boxes, decide), so these images
show exactly the decision the drone would make from that frame while flying straight:
  STRAIGHT    nothing within trigger_m (0.5 m), or something RIGHT IN FRONT (ignored by design)
  LEFT/RIGHT  a CLIP within 0.5 m (close points only in one side's prop zone): sidestep 10 deg
              away from it, 0.5 m, then back to the same waypoint
  HOLD        clip too deep for 10 deg;  UNKNOWN  not enough depth to tell
  A clip only counts after clip_confirm (3) frames in a row with something within 0.5 m (this one
  a clip; earlier ones a same-side clip or 'right in front'; no opposite-side clip), checked on
  the frames 1/hz (0.2 s) and 2/hz before this one in the 10 fps footage, like the flight rule.

Consistency rule (checked, and the run FAILS if broken): every frame the earlier straight-ahead
check called clear (hazard/depthsuite common.tube_blocked: 0.5 x 0.25 x 2.0 m box, run on the
same depth map) must still be STRAIGHT.

Writes data/walk/depth_set_400/results/dodge_decisions/:
  images/NNN_<DECISION>.jpg   base image | depth with the boxes and the chosen direction
  per_sample.csv, summary.json, overview.jpg (every LEFT/RIGHT/HOLD frame + a sample of STRAIGHT)
"""
import argparse
import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from common import tube_blocked  # noqa: E402  (the earlier straight-ahead check)

DEFAULT_RUN = HERE.parents[2] / 'Dronehacks-drone-tests' / 'team_src' / 'dodge_nav' / 'dodge_nav' / 'dodge_nav.py'
COLORS = {'STRAIGHT': (0, 220, 0), 'LEFT': (255, 200, 0), 'RIGHT': (255, 200, 0), 'HOLD': (0, 140, 255),
          'STOP': (0, 0, 255), 'UNKNOWN': (160, 160, 160)}


def load_run(path):
    spec = importlib.util.spec_from_file_location('dodge_nav_run', str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def vis_depth(d, size):
    near = np.where(np.isfinite(d) & (d > 0), 1.0 / d, np.nan)
    v = near[np.isfinite(near)]
    lo, hi = (np.percentile(v, 2), np.percentile(v, 98)) if v.size else (0, 1)
    g = np.clip((np.nan_to_num(near, nan=lo) - lo) / max(hi - lo, 1e-6) * 255, 0, 255).astype(np.uint8)
    img = cv2.applyColorMap(cv2.resize(g, size, interpolation=cv2.INTER_NEAREST), cv2.COLORMAP_INFERNO)
    img[cv2.resize((~np.isfinite(near)).astype(np.uint8), size, interpolation=cv2.INTER_NEAREST) > 0] = (60, 60, 60)
    return img


def draw_arrow(img, rel_deg, color, thick=3):
    """rel_deg + = left. Arrow from the bottom centre toward that bearing."""
    h, w = img.shape[:2]
    u = int(w / 2 - math.tan(math.radians(rel_deg)) * 627 * w / 1024)
    cv2.arrowedLine(img, (w // 2, h - 6), (int(np.clip(u, 4, w - 4)), int(h * 0.42)), color, thick, tipLength=0.12)


def draw_box_ticks(img, boxes, trigger):
    """Small markers across the top: one per bearing, green = clear, red = obstacle within
    trigger, orange = obstacle farther, grey = unknown; with the distance."""
    h, w = img.shape[:2]
    for b, x in boxes.items():
        u = int(w / 2 - math.tan(math.radians(b)) * 627 * w / 1024)
        if not x['known']:
            c, t = (150, 150, 150), '?'
        elif x['nearest'] is None:
            c, t = (0, 220, 0), '-'
        else:
            c = (0, 0, 255) if x['nearest'] <= trigger else (0, 165, 255)
            t = '%.1f' % x['nearest']
        cv2.circle(img, (u, 40), 7, c, -1)
        cv2.putText(img, t, (u - 12, 62), 0, 0.42, c, 1)
        cv2.putText(img, '%+d' % b, (u - 12, 28), 0, 0.35, (255, 255, 255), 1)


def write_html(out, res, summary):
    """index.html next to images/: every frame, its command and why. Local file only."""
    import html
    rows = []
    for x in res:
        rows.append('<tr class="%s"><td>%03d</td><td>%s</td><td><b>%s</b>%s</td><td>%s</td><td>%s</td>'
                    '<td>%s</td><td>%s</td><td>%s</td><td><a href="images/%03d_%s.jpg">'
                    '<img loading="lazy" src="images/%03d_%s.jpg"></a></td></tr>' % (
                        x['decision'], x['id'], x['t_s'], x['decision'],
                        (' %.0f&deg;' % abs(x['turn_deg'])) if x['decision'] in ('LEFT', 'RIGHT') else '',
                        '' if x['ahead_m'] in (None, '') else '%.2f' % float(x['ahead_m']), x['floor_ignored_at'] or '',
                        x['before'], x['hand_label'] or '', html.escape(x['why']),
                        x['id'], x['decision'], x['id'], x['decision']))
    counts = ' &nbsp; '.join('<label><input type="checkbox" checked data-k="%s"> %s (%d)</label>' % (k, k, v)
                             for k, v in summary['decisions'].items() if v)
    page = """<!doctype html><html><head><meta charset="utf-8"><title>Dodge decisions</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#fff;--fg:#111;--mut:#666;--line:#ddd}
@media (prefers-color-scheme:dark){:root{--bg:#141414;--fg:#eee;--mut:#999;--line:#333}}
body{background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif;margin:16px}
table{border-collapse:collapse;width:100%%}td,th{border-bottom:1px solid var(--line);padding:4px 6px;
text-align:left;vertical-align:middle}th{position:sticky;top:0;background:var(--bg)}img{width:340px;display:block}
tr.STRAIGHT b{color:#1a9e3a}tr.LEFT b,tr.RIGHT b{color:#0a6fd6}tr.HOLD b{color:#d97a00}tr.STOP b{color:#d33}
.mut{color:var(--mut)}</style></head><body>
<h1>Dodge decisions, 400 frames</h1>
<p class="mut">Run file: %s. Trigger %.1f m, kick %.0f&deg;. Earlier-clear frames now not STRAIGHT: %s.
Hand-labelled CLEAR now STRAIGHT: %s. Hand-labelled BLOCKED now turn/hold/stop: %s.</p>
<p>%s</p>
<table><thead><tr><th>#</th><th>t s</th><th>command</th><th>ahead m</th><th>floor ignored at</th>
<th>before</th><th>hand label</th><th>why</th><th>image</th></tr></thead><tbody>
%s
</tbody></table>
<script>
document.querySelectorAll('input[data-k]').forEach(function(c){c.addEventListener('change',function(){
document.querySelectorAll('tr.'+c.dataset.k).forEach(function(r){r.style.display=c.checked?'':'none'})})});
</script></body></html>""" % (html.escape(summary['run_file']), summary['trigger_m'], summary['kick_deg'],
                              summary['before_clear_now_not_straight'] or 'none',
                              summary['hand_clear_now_straight'], summary['hand_blocked_now_turn_or_stop'],
                              counts, chr(10).join(rows))
    (out / 'index.html').write_text(page, encoding='utf-8')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--set', type=Path, default=Path('data/walk/depth_set_400'))
    ap.add_argument('--run-file', type=Path, default=DEFAULT_RUN)
    ap.add_argument('--out', type=Path, default=None)
    ap.add_argument('--frames', type=Path, default=Path('data/walk/hazard_10fps'),
                    help='the 10 fps frames the set came from (for the earlier frames of each clip)')
    ap.add_argument('--hz', type=float, default=5.0, help='flight vision rate: earlier frames are 1/hz apart')
    a = ap.parse_args()

    R = load_run(a.run_file)
    P = R.Params()
    rect = json.loads((a.set / 'rectify.json').read_text())
    sd = R.StereoDepth(rect)                       # flight settings (64 disparities: sees 0.23 m+)
    sd_old = R.StereoDepth(rect, num_disp=32)     # the earlier check's depth, for the before/after rule
    with open(a.set / 'samples.csv', newline='') as f:
        rows = list(csv.DictReader(f))
    labels = {}
    if (a.set / 'hazard_labels.csv').exists():
        with open(a.set / 'hazard_labels.csv', newline='') as f:
            labels = {int(r['id']): int(r['blocked']) for r in csv.DictReader(f)}
    out = a.out or a.set / 'results' / 'dodge_decisions'
    (out / 'images').mkdir(parents=True, exist_ok=True)
    for old in (out / 'images').glob('*.jpg'):
        old.unlink()

    res, thumbs = [], []
    W, H = 512, 320
    for r in rows:
        sid = int(r['id'])
        L = cv2.imread(str(a.set / 'left' / ('%03d.jpg' % sid)), cv2.IMREAD_GRAYSCALE)
        Rimg = cv2.imread(str(a.set / 'right' / ('%03d.jpg' % sid)), cv2.IMREAD_GRAYSCALE)
        d = sd.depth(L, Rimg)
        boxes = R.scan_boxes(d)
        dec, why = R.decide(boxes, P)
        clip_frames = ''
        if dec in ('LEFT', 'RIGHT', 'HOLD'):
            # flight rule: the same-side clip must show on clip_confirm frames in a row at a.hz
            kind = R.classify_ahead(boxes, P)[0]
            streak, step = 1, max(1, int(round(10.0 / a.hz)))
            src = int(r['src_idx'])
            for back in range(1, int(P.clip_confirm)):
                j = src - back * step
                lp, rp = a.frames / 'left' / ('%05d.jpg' % j), a.frames / 'right' / ('%05d.jpg' % j)
                if not (lp.exists() and rp.exists()):
                    break
                bj = R.scan_boxes(sd.depth(cv2.imread(str(lp), 0), cv2.imread(str(rp), 0)))
                kj = R.classify_ahead(bj, P)[0]
                if kj not in ('front', kind):          # 'right in front' and same-side clips count
                    break
                streak += 1
            clip_frames = '%d/%d' % (streak, P.clip_confirm)
            if streak < P.clip_confirm:
                why = 'not confirmed: %s on %d of %d frames in a row (%s)' % (kind, streak, P.clip_confirm, why)
                dec = 'STRAIGHT'
        before_blocked = bool(tube_blocked(sd_old.depth(L, Rimg), range_m=2.0, width_m=0.5, height_m=0.25)[0])
        rel = 0.0
        if dec in ('LEFT', 'RIGHT'):
            rel = P.kick_deg if dec == 'LEFT' else -P.kick_deg
        lab = labels.get(sid)
        floor_dirs = [b for b, x in boxes.items() if x.get('floor_ignored')]
        a0 = boxes[0]
        zones = 'L%d C%d R%d /%d' % (a0.get('near_left', 0), a0.get('near_center', 0), a0.get('near_right', 0),
                                     a0.get('need', 0))
        res.append({'id': sid, 't_s': r['t_s'], 'decision': dec, 'turn_deg': round(rel, 0),
                    'ahead_m': boxes[0]['nearest'], 'close_points_LCR_need': zones, 'clip_frames': clip_frames,
                    'floor_ignored_at': ' '.join('%+d' % b for b in sorted(floor_dirs)),
                    'before': 'BLOCKED' if before_blocked else 'CLEAR',
                    'hand_label': {1: 'BLOCKED', 0: 'CLEAR', -1: 'UNSURE'}.get(lab, ''), 'why': why})

        base = cv2.cvtColor(cv2.resize(L, (W, H), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
        dep = vis_depth(d, (W, H))
        draw_box_ticks(dep, boxes, P.trigger_m)
        if dec in ('STRAIGHT', 'LEFT', 'RIGHT'):
            for im in (base, dep):
                draw_arrow(im, rel, COLORS[dec])
        head = np.zeros((34, 2 * W, 3), np.uint8)
        txt = '#%03d  t=%.1fs   %s%s' % (sid, float(r['t_s']), dec,
                                         (' %.0f deg' % abs(rel)) if dec in ('LEFT', 'RIGHT') else '')
        cv2.putText(head, txt, (8, 24), 0, 0.75, COLORS[dec], 2)
        cv2.putText(head, 'before: %s   hand label: %s' % (res[-1]['before'], res[-1]['hand_label'] or '-'),
                    (W + 10, 24), 0, 0.55, (220, 220, 220), 1)
        foot = np.zeros((26, 2 * W, 3), np.uint8)
        cv2.putText(foot, why[:150], (8, 18), 0, 0.45, (220, 220, 220), 1)
        cv2.putText(base, 'base image', (6, H - 8), 0, 0.5, (255, 255, 255), 1)
        img = np.vstack([head, np.hstack([base, dep]), foot])
        cv2.imwrite(str(out / 'images' / ('%03d_%s.jpg' % (sid, dec))), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if dec != 'STRAIGHT' or sid % 25 == 0:
            thumbs.append(cv2.resize(img, (512, 186), interpolation=cv2.INTER_AREA))

    with open(out / 'per_sample.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(res[0]))
        w.writeheader()
        w.writerows(res)
    if thumbs:
        while len(thumbs) % 3:
            thumbs.append(np.zeros_like(thumbs[0]))
        cv2.imwrite(str(out / 'overview.jpg'), np.vstack([np.hstack(thumbs[i:i + 3]) for i in range(0, len(thumbs), 3)]),
                    [cv2.IMWRITE_JPEG_QUALITY, 80])

    counts = {k: sum(1 for x in res if x['decision'] == k) for k in COLORS}
    was_clear = [x for x in res if x['before'] == 'CLEAR']
    broken = [x['id'] for x in was_clear if x['decision'] != 'STRAIGHT']
    lab_clear = [x for x in res if x['hand_label'] == 'CLEAR']
    lab_block = [x for x in res if x['hand_label'] == 'BLOCKED']
    summary = {
        'run_file': str(a.run_file), 'frames': len(res), 'decisions': counts,
        'trigger_m': P.trigger_m, 'kick_deg': P.kick_deg,
        'before_clear_frames': len(was_clear), 'before_clear_now_not_straight': broken,
        'consistency_ok': not broken,
        'hand_clear_now_straight': '%d/%d' % (sum(1 for x in lab_clear if x['decision'] == 'STRAIGHT'), len(lab_clear)),
        'hand_blocked_now_turn_or_stop': '%d/%d' % (sum(1 for x in lab_block if x['decision'] != 'STRAIGHT'),
                                                    len(lab_block)),
    }
    (out / 'summary.json').write_text(json.dumps(summary, indent=2))
    write_html(out, res, summary)
    print(json.dumps(summary, indent=2))
    print('images: %s' % (out / 'images'))
    sys.exit(0 if not broken else 1)


if __name__ == '__main__':
    main()
