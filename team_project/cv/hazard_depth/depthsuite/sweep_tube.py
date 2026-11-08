"""Sweep the straight-ahead tube size against the hand labels.

    python -I hazard/depthsuite/sweep_tube.py                      # SGBM depth, default grid
    python -I hazard/depthsuite/sweep_tube.py --min-recall 1.0     # must catch every hazard

Computes depth once per labelled frame, then scores every (width, height, min_frac) on:
caught / missed hazards and false stops. Picks the setting with the fewest false stops
among those meeting --min-recall, then checks it is not just fitted to these frames:
split the walk into alternating 10 s blocks, choose on one half, score on the other.
"""
import argparse
import csv
import itertools
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import Sample, tube_blocked  # noqa: E402
from run_depth_suite import kv, load  # noqa: E402


def score(blocked_pred, truth):
    tp = sum(1 for p, t in zip(blocked_pred, truth) if p and t)
    fn = sum(1 for p, t in zip(blocked_pred, truth) if not p and t)
    fp = sum(1 for p, t in zip(blocked_pred, truth) if p and not t)
    tn = sum(1 for p, t in zip(blocked_pred, truth) if not p and not t)
    return tp, fn, fp, tn


def best(rows, idx, min_recall):
    ok = []
    for r in rows:
        tp, fn, fp, tn = score([r["pred"][i] for i in idx], [r["truth"][i] for i in idx])
        rec = tp / float(tp + fn) if tp + fn else 1.0
        if rec >= min_recall:
            ok.append((fp, -r["w"] * r["h"], r, (tp, fn, fp, tn)))
    ok.sort(key=lambda x: (x[0], x[1]))
    return (ok[0][2], ok[0][3]) if ok else (None, None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", type=Path, default=Path("data/walk/depth_set_400"))
    ap.add_argument("--algo", default="hazard/depthsuite/baselines.py:StereoSGBM")
    ap.add_argument("--arg", action="append")
    ap.add_argument("--labels", default="hazard_labels.csv")
    ap.add_argument("--range-m", type=float, default=2.0)
    ap.add_argument("--widths", default="0.15,0.2,0.25,0.3,0.4,0.5,0.6")
    ap.add_argument("--heights", default="0.06,0.08,0.1,0.12,0.16,0.2,0.25")
    ap.add_argument("--min-fracs", default="0.005,0.01,0.03,0.1")
    ap.add_argument("--z-mins", default="0.15", help="ignore depth closer than this (e.g. 0.15,0.4,0.8)")
    ap.add_argument("--y-ups", default="0.0", help="raise the tube centre (m), e.g. 0,0.05,0.1")
    ap.add_argument("--ranges", default=None, help="tube lengths to try (default: --range-m)")
    ap.add_argument("--min-recall", type=float, default=0.96, help="0.96 = allow 1 miss in 32")
    a = ap.parse_args()

    rect = json.loads((a.set / "rectify.json").read_text())
    with open(a.set / a.labels, newline="") as f:
        lab = {int(r["id"]): int(r["blocked"]) for r in csv.DictReader(f) if r["blocked"] in ("0", "1")}
    with open(a.set / "samples.csv", newline="") as f:
        meta = {int(r["id"]): r for r in csv.DictReader(f)}
    ids = sorted(lab)
    algo = load(a.algo, kv(a.arg))
    print("depth for {} labelled frames ({} blocked) ...".format(len(ids), sum(lab.values())))
    depths = []
    for i in ids:
        rd = lambda d: cv2.imread(str(a.set / d / "{:03d}.jpg".format(i)), cv2.IMREAD_GRAYSCALE)  # noqa: E731
        m = meta[i]
        depths.append(algo.predict(Sample(i, float(m["t_s"]), rd("left"), rd("right"), rd("prev_left"),
                                          float(m["prev_dt_s"]), rect))["depth"])
    truth = [lab[i] for i in ids]

    rows = []
    grids = [[float(x) for x in g.split(",")] for g in
             (a.widths, a.heights, a.min_fracs, a.z_mins, a.y_ups, a.ranges or str(a.range_m))]
    for w, h, mf, zm, yu, rg in itertools.product(*grids):
        pred = [int(tube_blocked(d, range_m=rg, width_m=w, height_m=h, min_frac=mf, z_min=zm, y_up_m=yu)[0])
                for d in depths]
        tp, fn, fp, tn = score(pred, truth)
        rows.append({"w": w, "h": h, "min_frac": mf, "z_min": zm, "y_up": yu, "range": rg, "caught": tp, "missed": fn,
                     "false_stops": fp, "clears": fp + tn, "pred": pred, "truth": truth})

    allidx = list(range(len(ids)))
    pick, s = best(rows, allidx, a.min_recall)
    print("\nbest 10 with recall >= {}  (fewest false stops):".format(a.min_recall))
    print("{:>6}{:>7}{:>9}{:>7}{:>6}{:>6}{:>8}{:>8}{:>13}".format("width", "height", "min_frac", "z_min", "y_up",
                                                                   "range", "caught", "missed", "false stops"))
    good = sorted([r for r in rows if r["caught"] / float(r["caught"] + r["missed"]) >= a.min_recall],
                  key=lambda r: (r["false_stops"], -r["w"] * r["h"]))
    for r in good[:10]:
        print("{:>6}{:>7}{:>9}{:>7}{:>6}{:>6}{:>8}{:>8}{:>9}/{}".format(
            r["w"], r["h"], r["min_frac"], r["z_min"], r["y_up"], r["range"], r["caught"], r["missed"],
            r["false_stops"], r["clears"]))
    if pick is None:
        print("no setting reaches that recall")
        return

    # honesty check: choose on even 10 s blocks, test on odd ones, and vice versa
    t = np.array([float(meta[i]["t_s"]) for i in ids])
    half = (np.floor(t / 10.0).astype(int) % 2)
    cv_out = []
    for fit, test in ((0, 1), (1, 0)):
        p, _ = best(rows, [k for k in allidx if half[k] == fit], a.min_recall)
        if p is None:
            continue
        tidx = [k for k in allidx if half[k] == test]
        tp, fn, fp, tn = score([p["pred"][k] for k in tidx], [truth[k] for k in tidx])
        cv_out.append({"chosen": [p["w"], p["h"], p["min_frac"], p["z_min"], p["y_up"], p["range"]], "test_caught": tp, "test_missed": fn,
                       "test_false_stops": fp, "test_clears": fp + tn})
    print("\nheld-out check (choose on half the walk, score on the other half):")
    for c in cv_out:
        print("  chose w={} h={} min_frac={} z_min={} y_up={} range={} -> caught {}/{}  false stops {}/{}".format(
            *c["chosen"], c["test_caught"], c["test_caught"] + c["test_missed"], c["test_false_stops"], c["test_clears"]))

    missed_any = [ids[k] for k in allidx if truth[k] and not pick["pred"][k]]
    out = {"chosen": {"width_m": pick["w"], "height_m": pick["h"], "min_frac": pick["min_frac"],
                      "z_min": pick["z_min"], "y_up_m": pick["y_up"], "range_m": pick["range"]},
           "caught": s[0], "missed": s[1], "false_stops": s[2], "clears": s[2] + s[3], "missed_ids": missed_any,
           "held_out": cv_out, "algo": a.algo, "labels": a.labels}
    (a.set / "tube_sweep.json").write_text(json.dumps(out, indent=2))
    print("\nchosen: " + json.dumps(out["chosen"]) + "  -> {}".format(a.set / "tube_sweep.json"))


if __name__ == "__main__":
    main()
