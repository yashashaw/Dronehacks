"""Hand-label the 400-frame set: would the drone hit something if it flew STRAIGHT ahead?

    python -I hazard/depthsuite/label_tool.py                  # resumes where you stopped
    python -I hazard/depthsuite/label_tool.py --start 120      # jump to sample 120
    python -I hazard/depthsuite/label_tool.py --review         # step through labelled frames
                                                               # only (x keeps, b/c/u change)

Keys
    b  BLOCKED  something would be inside the green tube (the space the drone's body
                sweeps through in the next 2 m of straight flight)
    c  CLEAR    the tube is empty
    u  UNSURE   saved as "can't tell" (hand on lens...), left out of scoring
    s  SKIP     move on WITHOUT saving anything; the frame stays unlabelled and the
                tool comes back to it next time you start it
    z / LEFT    back one          x / RIGHT   forward one (no label)
    q / ESC     save and quit     (every key press is saved immediately anyway)

The green outline is a 3D tube drawn in perspective: 0.6 m wide (0.47 m drone with props +
margin), 0.25 m tall (drone height + margin), from 0.6 m (big near rectangle) to 2 m (small
far rectangle) straight ahead. Judge in 3D: something counts if it is inside that volume,
not just overlapping the outline on screen. Labels go to depth_set_400/hazard_labels.csv,
the tube geometry to hazard_labels_meta.json.
"""
import argparse
import csv
from pathlib import Path

import json
import sys

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import TUBE_H_M, TUBE_W_M, draw_tube  # noqa: E402

KEYS = {ord("b"): 1, ord("c"): 0, ord("u"): -1}
NAMES = {1: "BLOCKED", 0: "CLEAR", -1: "UNSURE"}
COLORS = {1: (0, 0, 255), 0: (0, 200, 0), -1: (0, 200, 255)}


def step(ids, i, d):
    j = ids.index(i) if i in ids else 0
    return ids[max(0, min(len(ids) - 1, j + d))]


def load(path):
    if not path.exists():
        return {}
    with open(path, newline="") as f:
        return {int(r["id"]): int(r["blocked"]) for r in csv.DictReader(f)}


def save(path, labels):
    tmp = path.with_suffix(".tmp")
    with open(tmp, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "blocked"])
        for k in sorted(labels):
            w.writerow([k, labels[k]])
    tmp.replace(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", type=Path, default=Path("data/walk/depth_set_400"))
    ap.add_argument("--start", type=int, default=None)
    ap.add_argument("--tube-width-m", type=float, default=TUBE_W_M, help="drone width with props + margin")
    ap.add_argument("--tube-height-m", type=float, default=TUBE_H_M, help="drone height + margin")
    ap.add_argument("--range-m", type=float, default=2.0, help="tube length")
    ap.add_argument("--review", action="store_true", help="visit only already-labelled frames")
    a = ap.parse_args()

    out = a.set / "hazard_labels.csv"
    labels = load(out)
    ids = sorted(int(p.stem) for p in (a.set / "left").glob("*.jpg"))
    if a.review:
        ids = [k for k in ids if k in labels]
        if not ids:
            raise SystemExit("nothing labelled yet")
    i = a.start if a.start is not None else (ids[0] if a.review else next((k for k in ids if k not in labels), ids[-1]))
    meta = {"shape": "tube", "width_m": a.tube_width_m, "height_m": a.tube_height_m, "range_m": a.range_m}
    (a.set / "hazard_labels_meta.json").write_text(json.dumps(meta, indent=2))
    win = "hazard labels  (b blocked / c clear / u unsure / s skip / z back / q quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1024, 700)
    while True:
        img = cv2.cvtColor(cv2.imread(str(a.set / "left" / "{:03d}.jpg".format(i)), 0), cv2.COLOR_GRAY2BGR)
        draw_tube(img, a.range_m, a.tube_width_m, a.tube_height_m, color=(0, 255, 0), thick=2)
        cv2.putText(img, "tube {:.2f} x {:.2f} m, 0.6 -> {:.1f} m".format(a.tube_width_m, a.tube_height_m, a.range_m),
                    (10, 630), 0, 0.6, (0, 255, 0), 2)
        lab = labels.get(i)
        done = sum(1 for k in ids if k in labels)
        bar = "#{:03d}   {}/{} labelled   current: {}".format(i, done, len(ids), NAMES.get(lab, "-"))
        cv2.rectangle(img, (0, 0), (1024, 34), (0, 0, 0), -1)
        cv2.putText(img, bar, (10, 24), 0, 0.75, COLORS.get(lab, (255, 255, 255)), 2)
        cv2.imshow(win, img)
        k = cv2.waitKeyEx(0)
        if k in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break
        if (k & 0xFF) in KEYS and k < 256:
            labels[i] = KEYS[k & 0xFF]
            save(out, labels)
            i = step(ids, i, +1)
        elif k in (ord("z"), 2424832, 65361):   # z, Windows LEFT, Linux LEFT
            i = step(ids, i, -1)
        elif k in (ord("s"), ord("x"), 2555904, 65363):   # s skip, x, Windows RIGHT, Linux RIGHT
            i = step(ids, i, +1)
    save(out, labels)
    cv2.destroyAllWindows()
    print("{} labels saved to {}".format(len(labels), out))


if __name__ == "__main__":
    main()
