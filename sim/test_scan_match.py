"""Offline test of scan_match: synthetic scans at known poses, perturbed predictions."""
import math
import random
import sys
import time

from scan_match import MapMatcher

obj = sys.argv[1] if len(sys.argv) > 1 else '../course/dronehacks-course-v3.obj'
t0 = time.time()
m = MapMatcher.from_obj(obj)
print('map: %d wall segments, distance field %dx%d built in %.1f s' % (len(m.segs), m.n, m.n, time.time() - t0))
fx, cx = 421.0, 319.5
bearings = [math.atan((u - cx) / fx) for u in range(0, 640, 16)]
random.seed(1)
cases = [  # (name, true x, y, heading deg, prediction error dx, dy, dhdg deg)
    ('corner 2 facing M2', -1.32, 2.58, 90, 0.20, -0.15, 8),
    ('leg 3 mid-corridor S', -1.37, 1.80, 180, -0.18, 0.20, -6),
    ('corner 3 facing barrier', -1.38, 0.99, 180, 0.15, 0.10, 10),
    ('leg 4 facing M3', -0.60, 0.97, 90, 0.10, 0.25, -8),
    ('leg 5 M2-M3 lane N', -0.26, 1.80, 0, 0.22, 0.0, 5),
    ('tube S', 2.31, 0.50, 180, -0.15, 0.15, 7),
    ('window approach', 0.37, -2.58, -70, 0.15, -0.15, 6),
]
for name, x, y, h, ex, ey, eh in cases:
    scan = [(b, None if r is None else r + random.gauss(0, 0.02)) for b, r in m.raycast(x, y, math.radians(h), bearings)]
    t1 = time.time()
    fix = m.match(x + ex, y + ey, math.radians(h + eh), scan)
    dt = time.time() - t1
    if fix is None:
        print('%-26s no fix (too few wall points)' % name)
        continue
    print('%-26s pts=%2d stiff=%d  err before (%+.2f,%+.2f,%+.0fdeg) -> after (%+.3f,%+.3f,%+.1fdeg)  %.2fs' % (
        name, fix['n'], fix['stiff'], ex, ey, eh, fix['x'] - x, fix['y'] - y, math.degrees(fix['hdg']) - h, dt))
