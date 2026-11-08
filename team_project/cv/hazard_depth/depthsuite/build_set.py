"""Build the fixed 400-frame stereo test set from the filtered 10 fps walk.

    python -I hazard/depthsuite/build_set.py data/walk/hazard_10fps --out data/walk/depth_set_400

Selection
  * candidates: kept by filter_walk.py, left/right stamps within --sync-ms, valid 1 s label
  * spread over the whole course: the course time span is divided into N equal slots and
    the nearest unused candidate is taken for each, so long dense stretches cannot crowd
    out short ones
  * each sample also gets the previous 10 fps left frame (for motion-aware methods)

Stereo rectification
  The drone's camera_info has no distortion or stereo rotation (raw rows are ~28 px
  apart), so this estimates an uncalibrated rectification (fundamental matrix from ORB
  matches pooled over the set). It is good enough for block matching, not for metric
  accuracy. Replace it with the OAK EEPROM calibration when you can read it on the drone.
"""
from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path

import cv2
import numpy as np


def pick_spread(cands: list[dict], n: int) -> list[dict]:
    t = np.array([float(c["t_s"]) for c in cands])
    used = np.zeros(len(cands), bool)
    out = []
    for target in np.linspace(t.min(), t.max(), n):
        order = np.argsort(np.abs(t - target))
        for j in order:
            if not used[j]:
                used[j] = True
                out.append(cands[j])
                break
    return sorted(out, key=lambda c: float(c["t_s"]))


def estimate_rectification(pairs, size):
    orb = cv2.ORB_create(1500)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    P1, P2 = [], []
    for L, R in pairs:
        k1, d1 = orb.detectAndCompute(L, None)
        k2, d2 = orb.detectAndCompute(R, None)
        if d1 is None or d2 is None:
            continue
        for m in bf.knnMatch(d1, d2, k=2):
            if len(m) == 2 and m[0].distance < 0.75 * m[1].distance:
                a, b = k1[m[0].queryIdx].pt, k2[m[0].trainIdx].pt
                # stereo: right-eye match is left of (or at) the left-eye point, rows close
                if -2 < a[0] - b[0] < 160 and abs(a[1] - b[1]) < 60:
                    P1.append(a)
                    P2.append(b)
    P1, P2 = np.float32(P1), np.float32(P2)
    F, mask = cv2.findFundamentalMat(P1, P2, cv2.FM_RANSAC, 1.0, 0.999)
    m = mask.ravel() == 1
    ok, H1, H2 = cv2.stereoRectifyUncalibrated(P1[m], P2[m], F, size)
    if not ok:
        raise RuntimeError("stereoRectifyUncalibrated failed")

    def warp(H, P):
        return cv2.perspectiveTransform(P.reshape(-1, 1, 2), H).reshape(-1, 2)

    dy_raw = P1[m, 1] - P2[m, 1]
    r1, r2 = warp(H1, P1[m]), warp(H2, P2[m])
    # Uncalibrated rectification leaves the disparity of infinity undetermined. Shift the
    # right image so the farthest 2% of matches (ceiling lights, far walls) sit at ~0 px.
    s = -float(np.percentile(r1[:, 0] - r2[:, 0], 2))
    H2 = np.array([[1, 0, -s], [0, 1, 0], [0, 0, 1]]) @ H2
    r2 = warp(H2, P2[m])
    dy = r1[:, 1] - r2[:, 1]
    disp = r1[:, 0] - r2[:, 0]
    stats = {"matches": int(m.sum()), "infinity_shift_px": round(s, 2),
             "dy_raw_median_px": round(float(np.median(dy_raw)), 2),
             "dy_rect_median_px": round(float(np.median(dy)), 2),
             "dy_rect_p90_abs_px": round(float(np.percentile(np.abs(dy), 90)), 2),
             "disparity_p5_p50_p95_px": [round(float(x), 1) for x in np.percentile(disp, [5, 50, 95])]}
    return F, H1, H2, stats


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("-n", type=int, default=400)
    ap.add_argument("--sync-ms", type=float, default=1.0)
    ap.add_argument("--no-turn-label", action="store_true",
                    help="keep frames without a valid 1 s turn label (the dodge does not use it)")
    ap.add_argument("--t-min", type=float, default=None, help="only frames at or after this time (s)")
    ap.add_argument("--t-max", type=float, default=None, help="only frames at or before this time (s)")
    ap.add_argument("--exclude-set", type=Path, default=None,
                    help="another set folder: skip the frames it already uses (new frames only)")
    ap.add_argument("--rectify-from", type=Path, default=None,
                    help="copy rectify.json from this set instead of estimating a new one")
    args = ap.parse_args()
    ds, out = args.dataset, args.out
    for d in ("left", "right", "prev_left"):
        (out / d).mkdir(parents=True, exist_ok=True)

    with open(ds / "manifest.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    by_idx = {int(r["idx"]): r for r in rows}
    cands = [r for r in rows if r["keep"] == "1" and float(r["sync_ms"]) <= args.sync_ms
             and (args.no_turn_label or r["label_ok_1s"] == "1") and int(r["idx"]) - 1 in by_idx]
    if args.t_min is not None:
        cands = [r for r in cands if float(r["t_s"]) >= args.t_min]
    if args.t_max is not None:
        cands = [r for r in cands if float(r["t_s"]) <= args.t_max]
    if args.exclude_set is not None:
        with open(args.exclude_set / "samples.csv", newline="") as f:
            used = {int(r["src_idx"]) for r in csv.DictReader(f)}
        cands = [r for r in cands if int(r["idx"]) not in used]
    if args.n <= 0:
        args.n = len(cands)
    if len(cands) < args.n:
        raise SystemExit(f"only {len(cands)} candidates for {args.n} samples; loosen filter_walk.py")
    sel = pick_spread(cands, args.n)

    samples = []
    for sid, r in enumerate(sel):
        i = int(r["idx"])
        prev = by_idx[i - 1]
        shutil.copyfile(ds / "left" / f"{i:05d}.jpg", out / "left" / f"{sid:03d}.jpg")
        shutil.copyfile(ds / "right" / f"{i:05d}.jpg", out / "right" / f"{sid:03d}.jpg")
        shutil.copyfile(ds / "left" / f"{i - 1:05d}.jpg", out / "prev_left" / f"{sid:03d}.jpg")
        s = {"id": sid, "src_idx": i, "t_s": r["t_s"], "sync_ms": r["sync_ms"],
             "prev_dt_s": round(float(r["t_s"]) - float(prev["t_s"]), 4),
             "prev_flagged": int(prev["keep"] != "1")}
        for h in ("0.5", "1", "2"):
            s[f"yaw_{h}s"] = r[f"yaw_{h}s"]
            s[f"label_ok_{h}s"] = r[f"label_ok_{h}s"]
        samples.append(s)

    with open(out / "samples.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(samples[0]))
        w.writeheader()
        w.writerows(samples)

    if args.rectify_from is not None:
        rect = json.loads((args.rectify_from / "rectify.json").read_text())
        st = rect.get("stats", {})
        (out / "rectify.json").write_text(json.dumps(rect, indent=2))
        print("rectify.json copied from", args.rectify_from)
    else:
        print("estimating stereo rectification from", min(80, len(sel)), "pairs")
        pick = np.linspace(0, len(sel) - 1, min(80, len(sel))).astype(int)
        pairs = [(cv2.imread(str(out / "left" / f"{k:03d}.jpg"), 0), cv2.imread(str(out / "right" / f"{k:03d}.jpg"), 0))
                 for k in pick]
        F, H1, H2, st = estimate_rectification(pairs, (1024, 640))
        rect = {"size": [1024, 640], "F": F.tolist(), "H_left": H1.tolist(), "H_right": H2.tolist(),
                "baseline_m": 0.0754, "fx_px": 627.0, "stats": st,
                "note": "uncalibrated (fundamental-matrix) rectification estimated from this walk"}
        (out / "rectify.json").write_text(json.dumps(rect, indent=2))

    ts = np.array([float(s["t_s"]) for s in samples])
    y1 = np.array([float(s["yaw_1s"]) for s in samples])
    info = {"samples": len(samples), "t_span_s": [round(ts.min(), 1), round(ts.max(), 1)],
            "max_time_gap_s": round(float(np.diff(ts).max()), 2),
            "sync_ms_max": max(float(s["sync_ms"]) for s in samples),
            "yaw_1s_left_straight_right": [int((y1 > 15).sum()), int((np.abs(y1) <= 15).sum()), int((y1 < -15).sum())],
            "rectification": st}
    (out / "set_info.json").write_text(json.dumps(info, indent=2))

    tiles = []
    for s in samples:
        im = cv2.resize(cv2.imread(str(out / "left" / f"{s['id']:03d}.jpg")), (128, 80), interpolation=cv2.INTER_AREA)
        cv2.putText(im, f"{s['id']} {float(s['t_s']):.0f}s", (2, 10), 0, 0.3, (0, 255, 255), 1)
        tiles.append(im)
    cols = 20
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(str(out / "overview.jpg"), np.vstack([np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]))
    print(json.dumps(info, indent=2))


if __name__ == "__main__":
    main()
