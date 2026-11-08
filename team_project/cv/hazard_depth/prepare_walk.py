"""Step 1: turn a raw OAK stereo walk into a 10 fps dataset with turn labels.

    python -I hazard/prepare_walk.py data/walk/stereo-20261108-044200 \
        --out data/walk/hazard_10fps --cut-s 120

What it does
  1. Pairs left/right frames by timestamp (not by index) and reports stamp gaps.
  2. Estimates camera rotation between every consecutive 30 fps left frame, then
     integrates it into a heading track. Labels come from this full-rate track, so
     they are continuous even across frames the filter later rejects.
  3. Picks frames on a 100 ms timestamp grid (10 fps), keeping only t < cut.
  4. Writes the chosen left/right frames plus per-frame quality metrics (brightness,
     blur, low-texture fraction, left/right agreement, people boxes) to metrics.csv.

Step 2 (filter_walk.py) applies thresholds to metrics.csv, so you can re-tune the
filter without re-decoding the video.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vision import (FX_FULL, FY_FULL, CX_FULL, CY_FULL, FULL_W, FULL_H,  # noqa: E402
                    camera_matrix, rotation_between, to_gray_small)

HORIZONS_S = (0.5, 1.0, 2.0)
GRID_S = 0.1
GRID_TOL_S = 0.02
SYNC_TOL_S = 0.005
MAX_FAILED_FRACTION = 0.2  # of 30 fps rotation steps inside a label window


def load_stamps(path: Path):
    st = {"left": {}, "right": {}}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            st[row["eye"]][int(row["frame"])] = float(row["stamp_s"])
    out = {}
    for eye, d in st.items():
        n = max(d) + 1
        arr = np.full(n, np.nan)
        for k, v in d.items():
            arr[k] = v
        out[eye] = arr
    return out


def stamp_report(t: np.ndarray, eye: str) -> dict:
    dt = np.diff(t)
    return {
        "eye": eye, "frames": int(len(t)), "missing_stamps": int(np.isnan(t).sum()),
        "duration_s": round(float(np.nanmax(t) - np.nanmin(t)), 3),
        "median_dt_ms": round(float(np.nanmedian(dt) * 1e3), 2),
        "gaps_over_50ms": int((dt > 0.05).sum()), "max_gap_ms": round(float(np.nanmax(dt) * 1e3), 1),
        "non_increasing": int((dt <= 0).sum()),
    }


def texture_stats(g: np.ndarray, block: int = 32, std_thr: float = 6.0):
    h, w = g.shape
    hb, wb = h // block, w // block
    blocks = g[:hb * block, :wb * block].reshape(hb, block, wb, block).astype(np.float32)
    std = blocks.std(axis=(1, 3))
    return float((std < std_thr).mean())


def lr_agreement(gl: np.ndarray, gr: np.ndarray) -> float:
    """Normalised correlation of tiny blurred thumbnails. A hand or body against one lens
    makes the two eyes disagree; ordinary stereo disparity barely changes a 64x40 thumb."""
    a = cv2.resize(cv2.GaussianBlur(gl, (9, 9), 0), (64, 40), interpolation=cv2.INTER_AREA).astype(np.float32)
    b = cv2.resize(cv2.GaussianBlur(gr, (9, 9), 0), (64, 40), interpolation=cv2.INTER_AREA).astype(np.float32)
    a -= a.mean()
    b -= b.mean()
    d = math.sqrt(float((a * a).sum() * (b * b).sum()))
    return float((a * b).sum() / d) if d > 1e-6 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", type=Path, help="folder with left.avi, right.avi, stamps.csv")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cut-s", type=float, default=120.0, help="drop everything after this many seconds")
    ap.add_argument("--no-people", action="store_true", help="skip the YOLO person detector")
    args = ap.parse_args()

    rec, out = args.recording, args.out
    for sub in ("left", "right"):
        (out / sub).mkdir(parents=True, exist_ok=True)

    stamps = load_stamps(rec / "stamps.csv")
    tL, tR = stamps["left"], stamps["right"]
    t0 = float(min(np.nanmin(tL), np.nanmin(tR)))
    t_cut = t0 + args.cut_s
    report = {"stamps": [stamp_report(tL, "left"), stamp_report(tR, "right")], "t0_unix_s": t0,
              "cut_s": args.cut_s}

    # ---- pass 1: full-rate left stream -> rotation track ------------------------------
    print("pass 1: 30 fps rotation track (left eye)")
    capL = cv2.VideoCapture(str(rec / "left.avi"))
    n_left = int(np.searchsorted(tL, t_cut))  # frames with stamp < cut (stamps increase)
    # Labels need up to max(HORIZONS_S) after the last kept frame; those are cut too,
    # so late frames get NaN labels instead of peeking past the cut.
    K = camera_matrix()
    yaw_step = np.full(n_left, np.nan)
    pitch_step = np.full(n_left, np.nan)
    inl = np.zeros(n_left, dtype=np.int32)

    grid = np.arange(t0, t_cut, GRID_S)
    sel_left = []  # (grid_idx, left_frame)
    for g_i, g in enumerate(grid):
        k = int(np.nanargmin(np.abs(tL[:n_left] - g)))
        if abs(tL[k] - g) <= GRID_TOL_S and (not sel_left or sel_left[-1][1] != k):
            sel_left.append((g_i, k))
    sel_set = {k: i for i, (_, k) in enumerate(sel_left)}

    prev = None
    left_gray_full = {}
    tic = time.time()
    for k in range(n_left):
        ok, frame = capL.read()
        if not ok:
            print(f"  left video ended at frame {k}")
            break
        g = to_gray_small(frame)
        if prev is not None:
            yaw_step[k], pitch_step[k], inl[k] = rotation_between(prev, g, K)
        prev = g
        if k in sel_set:
            full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imwrite(str(out / "left" / f"{sel_set[k]:05d}.jpg"), full, [cv2.IMWRITE_JPEG_QUALITY, 95])
            left_gray_full[k] = g
        if k % 500 == 0:
            print(f"  {k}/{n_left}  {time.time() - tic:.0f}s")
    capL.release()

    failed = np.isnan(yaw_step)
    failed[0] = False
    heading = np.cumsum(np.nan_to_num(yaw_step))
    tilt = np.cumsum(np.nan_to_num(pitch_step))

    # ---- pass 2: right eye frames matched by timestamp --------------------------------
    print("pass 2: right eye")
    n_right = int(np.searchsorted(tR, t_cut + 0.05))
    want_r = {}
    for i, (_, k) in enumerate(sel_left):
        j = int(np.nanargmin(np.abs(tR[:n_right] - tL[k])))
        want_r.setdefault(j, []).append(i)
    capR = cv2.VideoCapture(str(rec / "right.avi"))
    right_gray = {}
    match_r = {}
    for j in range(max(want_r) + 1):
        ok, frame = capR.read()
        if not ok:
            break
        if j in want_r:
            full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            for i in want_r[j]:
                cv2.imwrite(str(out / "right" / f"{i:05d}.jpg"), full, [cv2.IMWRITE_JPEG_QUALITY, 95])
                right_gray[i] = to_gray_small(full)
                match_r[i] = j
    capR.release()

    # ---- people ------------------------------------------------------------------------
    people = {i: [] for i in range(len(sel_left))}
    if not args.no_people:
        print("pass 3: person detector (YOLOv8n, CPU)")
        from ultralytics import YOLO  # imported late: optional dependency
        model = YOLO("yolov8n.pt")
        paths = [str(out / "left" / f"{i:05d}.jpg") for i in range(len(sel_left))]
        for s in range(0, len(paths), 32):
            res = model.predict(paths[s:s + 32], classes=[0], conf=0.35, imgsz=640, device="cpu", verbose=False)
            for off, r in enumerate(res):
                for (x0, y0, x1, y1), c in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist()):
                    people[s + off].append([round(x0 / FULL_W, 4), round(y0 / FULL_H, 4),
                                            round(x1 / FULL_W, 4), round(y1 / FULL_H, 4), round(c, 3)])

    # ---- metrics + labels --------------------------------------------------------------
    print("writing metrics.csv")
    rows = []
    for i, (g_i, k) in enumerate(sel_left):
        gl = left_gray_full[k]
        gr = right_gray.get(i)
        j = match_r.get(i, -1)
        row = {
            "idx": i, "t_s": round(float(tL[k] - t0), 4), "left_frame": k, "right_frame": j,
            "sync_ms": round(abs(float(tR[j] - tL[k])) * 1e3, 3) if j >= 0 else math.nan,
            "brightness_l": round(float(gl.mean()), 1),
            "brightness_r": round(float(gr.mean()), 1) if gr is not None else math.nan,
            "sharpness": round(float(cv2.Laplacian(gl, cv2.CV_64F).var()), 1),
            "lowtex_l": round(texture_stats(gl), 3),
            "lowtex_r": round(texture_stats(gr), 3) if gr is not None else math.nan,
            "lr_ncc": round(lr_agreement(gl, gr), 3) if gr is not None else math.nan,
            "track_inliers": int(inl[k]),
            "people": json.dumps(people[i]),
        }
        # Turn labels: heading change from this frame to t + h on the 30 fps track.
        for h in HORIZONS_S:
            kh = int(np.searchsorted(tL[:n_left], tL[k] + h))
            if 0 < kh < n_left and abs(tL[kh - 1] - (tL[k] + h)) < abs(tL[kh] - (tL[k] + h)):
                kh -= 1  # nearest stamp, not first-at-or-after
            if kh >= n_left or abs(tL[kh] - (tL[k] + h)) > GRID_TOL_S:
                yaw, pit, valid = math.nan, math.nan, 0
            else:
                window_failed = failed[k + 1:kh + 1].mean()
                yaw = float(heading[kh] - heading[k])
                pit = float(tilt[kh] - tilt[k])
                valid = int(window_failed <= MAX_FAILED_FRACTION)
            row[f"yaw_{h:g}s"] = round(yaw, 2)
            row[f"pitch_{h:g}s"] = round(pit, 2)
            row[f"label_ok_{h:g}s"] = valid
        # Past rate (diagnostics only, NOT given to models): yaw over the previous 0.3 s.
        kp = int(np.searchsorted(tL[:n_left], tL[k] - 0.3))
        row["yaw_rate_past_dps"] = round(float(heading[k] - heading[kp]) / max(tL[k] - tL[kp], 1e-3), 1) if k > 0 else 0.0
        rows.append(row)

    with open(out / "metrics.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    np.savez_compressed(out / "track_30fps.npz", t_s=tL[:n_left] - t0, yaw_step_deg=yaw_step,
                        pitch_step_deg=pitch_step, heading_deg=heading, inliers=inl)

    report.update({
        "frames_10fps": len(rows),
        "rotation_steps_failed": int(failed.sum()), "rotation_steps": int(n_left - 1),
        "sync_over_tol": int(sum(1 for r in rows if not r["sync_ms"] <= SYNC_TOL_S * 1e3)),
        "camera": {"model": "OAK left mono", "width": FULL_W, "height": FULL_H, "fx": FX_FULL,
                   "fy": FY_FULL, "cx": CX_FULL, "cy": CY_FULL,
                   "source": "drone camera_info (fx 627, cy 326); fy and cx ASSUMED; D=0 published, "
                             "distortion ignored. Yaw SIGN is reliable, magnitude is approximate."},
        "conventions": {"yaw": "deg, + = camera turns LEFT", "pitch": "deg, + = camera tilts UP",
                        "label": "yaw_<h>s = heading change from t to t+h, from 30 fps visual odometry"},
        "horizons_s": list(HORIZONS_S),
    })
    (out / "prepare_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k: report[k] for k in ("stamps", "frames_10fps", "rotation_steps_failed",
                                              "rotation_steps", "sync_over_tol")}, indent=2))


if __name__ == "__main__":
    main()
