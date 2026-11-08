"""Score a turn predictor on the 10 fps walk dataset.

    python -I hazard/testsuite/run_suite.py --model hazard/testsuite/baselines.py:ConstantRatePredictor
    python -I hazard/testsuite/run_suite.py --model my_model.py:MyModel --split test --budget-ms 40
    python -I hazard/testsuite/run_suite.py --model pkg.module:Class --model-arg weights=best.onnx

Streaming: every frame of the chosen split is fed in time order (flagged ones too, as a
flight would), and only kept frames with a valid label are scored. Use --stream kept to
feed only kept frames.

Metrics
  mae / rmse          per horizon, degrees of heading change
  direction           3 classes (left / straight / right); "straight" = |yaw| < deadband * h
                      accuracy, plus recall for left and right
  turn events         onset = |yaw rate| > --event-dps for >= 0.3 s on the 30 fps track.
                      A turn counts as called when the 1 s prediction first has the correct
                      sign and |pred| >= deadband before onset + 0.5 s. lead_s > 0 means
                      it was called before the turn began.
  false_alarm_per_min frames predicting a turn (1 s horizon) where the label says straight
  latency_ms          predict() wall time, p50/p95/p99/max (first 5 calls excluded)
  gpu_peak_mb         torch.cuda.max_memory_allocated, if your model uses torch+CUDA

Desktop timings do NOT transfer to a Jetson Nano; rerun there with the same command.
"""
import argparse
import csv
import importlib
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
from interface import Frame  # noqa: E402
from vision import CX_FULL, CY_FULL, FX_FULL, FY_FULL  # noqa: E402

BASELINE_M = 0.0754  # mapping/FINDINGS.md (camera_info P[0,3]/fx, matches TF)


def load_model(spec: str, kwargs: dict):
    target, _, cls = spec.rpartition(":")
    if target.endswith(".py"):
        mod_spec = importlib.util.spec_from_file_location(Path(target).stem, target)
        mod = importlib.util.module_from_spec(mod_spec)
        sys.path.insert(0, str(Path(target).resolve().parent))
        mod_spec.loader.exec_module(mod)
    else:
        mod = importlib.import_module(target)
    return getattr(mod, cls)(**kwargs)


def parse_kv(items):
    out = {}
    for it in items or []:
        k, _, v = it.partition("=")
        for cast in (int, float):
            try:
                v = cast(v)
                break
            except ValueError:
                pass
        out[k] = v
    return out


def turn_events(ds: Path, t_lo: float, t_hi: float, event_dps: float, min_dur: float = 0.3):
    z = np.load(ds / "track_30fps.npz")
    t, hd = z["t_s"], z["heading_deg"]
    rate = np.zeros_like(hd)
    w = 5  # +/- 5 frames (~0.33 s) centred difference
    rate[w:-w] = (hd[2 * w:] - hd[:-2 * w]) / np.maximum(t[2 * w:] - t[:-2 * w], 1e-3)
    sign = np.where(rate > event_dps, 1, np.where(rate < -event_dps, -1, 0))
    events, i = [], 0
    while i < len(sign):
        if sign[i] != 0:
            j = i
            while j + 1 < len(sign) and sign[j + 1] == sign[i]:
                j += 1
            if t[j] - t[i] >= min_dur and t_lo + 1.0 <= t[i] <= t_hi:
                events.append({"t_on": float(t[i]), "dir": int(sign[i]), "dur_s": float(t[j] - t[i])})
            i = j + 1
        else:
            i += 1
    return events


def direction(v: float, thr: float) -> int:
    return 1 if v > thr else (-1 if v < -thr else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True, help="path/file.py:Class or package.module:Class")
    ap.add_argument("--model-arg", action="append", help="constructor kwarg k=v (repeatable)")
    ap.add_argument("--dataset", type=Path, default=Path("data/walk/hazard_10fps"))
    ap.add_argument("--split", default="test", choices=["train", "val", "test", "all"])
    ap.add_argument("--stream", default="all", choices=["all", "kept"])
    ap.add_argument("--deadband-dps", type=float, default=15.0, help="straight if |yaw| < this * horizon")
    ap.add_argument("--event-dps", type=float, default=30.0)
    ap.add_argument("--budget-ms", type=float, default=50.0, help="p95 latency budget for PASS/FAIL")
    ap.add_argument("--out", type=Path, default=None, help="report folder (default dataset/results/<model>)")
    args = ap.parse_args()

    ds = args.dataset
    with open(ds / "manifest.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    horizons = tuple(json.loads((ds / "prepare_report.json").read_text())["horizons_s"])
    if args.split != "all":
        rows = [r for r in rows if r["split"] == args.split]
    if args.stream == "kept":
        rows = [r for r in rows if r["keep"] == "1"]
    if not rows:
        sys.exit("no frames in this split")

    model = load_model(args.model, parse_kv(args.model_arg))
    camera = {"fx": FX_FULL, "fy": FY_FULL, "cx": CX_FULL, "cy": CY_FULL, "baseline_m": BASELINE_M,
              "note": "approximate: camera_info had D=0 and no stereo rectification"}
    torch = sys.modules.get("torch")
    if torch is not None and torch.cuda.is_available():
        torch.cuda.reset_peak_memory_stats()

    if hasattr(model, "reset"):
        model.reset()
    preds, lat = [], []
    prev_t = None
    for r in rows:
        i = int(r["idx"])
        L = cv2.imread(str(ds / "left" / f"{i:05d}.jpg"), cv2.IMREAD_GRAYSCALE)
        Rimg = cv2.imread(str(ds / "right" / f"{i:05d}.jpg"), cv2.IMREAD_GRAYSCALE)
        t = float(r["t_s"])
        fr = Frame(idx=i, t_s=t, dt_s=0.0 if prev_t is None else t - prev_t, left=L, right=Rimg,
                   flagged=r["keep"] != "1", camera=camera, horizons=horizons)
        prev_t = t
        tic = time.perf_counter()
        p = model.predict(fr) or {}
        lat.append((time.perf_counter() - tic) * 1e3)
        preds.append(p)

    torch = sys.modules.get("torch")  # the model may have imported it
    gpu_mb = None
    if torch is not None and torch.cuda.is_available():
        gpu_mb = round(torch.cuda.max_memory_allocated() / 2**20, 1)

    # ---- accuracy ---------------------------------------------------------------------
    report = {"model": args.model, "split": args.split, "stream": args.stream,
              "frames_streamed": len(rows), "horizons": {}}
    for h in horizons:
        key = f"yaw_{h:g}s"
        thr = args.deadband_dps * h
        err, hit, n_cls = [], 0, 0
        rec = {1: [0, 0], -1: [0, 0]}
        for r, p in zip(rows, preds):
            if r["keep"] != "1" or r[f"label_ok_{h:g}s"] != "1" or key not in p:
                continue
            y, yh = float(r[key]), float(p[key])
            if math.isnan(y) or math.isnan(yh):
                continue
            err.append(yh - y)
            cy, cp = direction(y, thr), direction(yh, thr)
            hit += cy == cp
            n_cls += 1
            if cy in rec:
                rec[cy][0] += cp == cy
                rec[cy][1] += 1
        e = np.array(err)
        report["horizons"][key] = {
            "scored": int(len(e)),
            "mae_deg": round(float(np.abs(e).mean()), 2) if len(e) else None,
            "rmse_deg": round(float(np.sqrt((e ** 2).mean())), 2) if len(e) else None,
            "direction_acc": round(hit / n_cls, 3) if n_cls else None,
            "recall_left": round(rec[1][0] / rec[1][1], 3) if rec[1][1] else None,
            "recall_right": round(rec[-1][0] / rec[-1][1], 3) if rec[-1][1] else None,
            "n_left": rec[1][1], "n_right": rec[-1][1],
        }

    # ---- turn events / lead time (1 s horizon) ------------------------------------------
    thr1 = args.deadband_dps * 1.0
    ts = np.array([float(r["t_s"]) for r in rows])
    p1 = np.array([float(p.get("yaw_1s", math.nan)) for p in preds])
    events = turn_events(ds, ts[0], ts[-1], args.event_dps)
    leads = []
    for ev in events:
        win = (ts >= ev["t_on"] - 2.0) & (ts <= ev["t_on"] + 0.5)
        called = np.where(win & (np.sign(p1) == ev["dir"]) & (np.abs(p1) >= thr1))[0]
        ev["lead_s"] = round(float(ev["t_on"] - ts[called[0]]), 2) if len(called) else None
        if len(called):
            leads.append(ev["lead_s"])
    fa = sum(1 for r, p in zip(rows, preds)
             if r["keep"] == "1" and r["label_ok_1s"] == "1"
             and abs(float(r["yaw_1s"])) < thr1 and abs(float(p.get("yaw_1s", 0.0))) >= thr1)
    minutes = max((ts[-1] - ts[0]) / 60.0, 1e-6)
    report["turn_events"] = {
        "events": len(events), "called": len(leads),
        "detection_rate": round(len(leads) / len(events), 3) if events else None,
        "median_lead_s": round(float(np.median(leads)), 2) if leads else None,
        "called_before_onset": sum(1 for x in leads if x > 0),
        "false_alarm_per_min": round(fa / minutes, 1),
        "detail": events,
    }

    lat_w = np.array(lat[5:] if len(lat) > 10 else lat)
    report["latency_ms"] = {q: round(float(np.percentile(lat_w, v)), 2)
                            for q, v in (("p50", 50), ("p95", 95), ("p99", 99), ("max", 100))}
    report["latency_budget_ms"] = args.budget_ms
    report["latency_pass"] = bool(report["latency_ms"]["p95"] <= args.budget_ms)
    report["gpu_peak_mb"] = gpu_mb

    name = Path(args.model.rpartition(":")[0]).stem + "." + args.model.rpartition(":")[2]
    out = args.out or ds / "results" / f"{name}_{args.split}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.json").write_text(json.dumps(report, indent=2))
    keys = sorted({k for p in preds for k in p})
    with open(out / "predictions.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "t_s", "keep"] + [f"label_{k}" for k in keys if k.startswith("yaw_")] + [f"pred_{k}" for k in keys])
        for r, p in zip(rows, preds):
            w.writerow([r["idx"], r["t_s"], r["keep"]] + [r.get(k, "") for k in keys if k.startswith("yaw_")]
                       + [p.get(k, "") for k in keys])

    # ---- console table --------------------------------------------------------------------
    print(f"\n{args.model}  split={args.split}  frames={len(rows)}")
    print(f"{'horizon':10}{'scored':>8}{'MAE':>8}{'RMSE':>8}{'dir acc':>9}{'rec L':>7}{'rec R':>7}")
    for k, v in report["horizons"].items():
        f = lambda x: "-" if x is None else x  # noqa: E731
        print(f"{k:10}{v['scored']:>8}{f(v['mae_deg']):>8}{f(v['rmse_deg']):>8}"
              f"{f(v['direction_acc']):>9}{f(v['recall_left']):>7}{f(v['recall_right']):>7}")
    te = report["turn_events"]
    print(f"turns: {te['called']}/{te['events']} called, median lead {te['median_lead_s']} s, "
          f"{te['called_before_onset']} before onset, {te['false_alarm_per_min']} false alarms/min")
    lm = report["latency_ms"]
    print(f"latency p50 {lm['p50']} ms  p95 {lm['p95']} ms  p99 {lm['p99']} ms  "
          f"[{'PASS' if report['latency_pass'] else 'FAIL'} vs {args.budget_ms} ms]  gpu_peak_mb={gpu_mb}")
    print(f"report: {out / 'report.json'}")


if __name__ == "__main__":
    main()
