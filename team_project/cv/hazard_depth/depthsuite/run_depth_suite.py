"""Score a depth algorithm on the 400-frame set.

MAIN QUESTION (needs hazard_labels.csv from label_tool.py): if the drone flies straight
from here, does it hit something within --range-m? The algorithm returns "blocked"
(bool), or a metric depth map that is checked with common.tube_blocked(): a 3D,
drone-shaped tube (0.6 m wide x 0.25 m tall x --range-m long) — the same tube the
labeller saw drawn in perspective.
  hazard_recall     fraction of real hazards flagged (1 - this = crashes)  <- most important
  missed_hazards    count of BLOCKED frames called clear
  false_stop_rate   fraction of CLEAR frames called blocked (costs time, not the drone)
Images are named NNN_CAUGHT / NNN_MISSED_HAZARD / NNN_FALSE_STOP / NNN_CLEAR_OK.

SECONDARY (always available, weaker): steering vs the walker's next-1 s turn. The walk did
not follow the drone's path exactly, so these numbers are only a loose sanity check.

    python -I hazard/depthsuite/run_depth_suite.py --algo hazard/depthsuite/baselines.py:StereoBM
    python -I hazard/depthsuite/run_depth_suite.py --algo my_depth.py:MyNet --arg onnx=net.onnx --budget-ms 30

Ground truth: the carrier's actual heading change over the next 1 s (yaw_1s, + = left),
from visual odometry on the 30 fps video. A good obstacle-aware depth map should point
the same way the walker went through the course.

Reported
  balanced_acc      HEADLINE: mean of left/straight/right recall (always-straight = 0.333;
                    68% of labels are straight, so plain accuracy flatters doing nothing)
  direction_acc     left / straight / right agreement (label straight = |yaw_1s| < 15 deg;
                    decision straight = |steer| < --steer-deadband)
  recall_left/right how often real turns were steered the right way
  sign_agreement    on real turns only, did steer have the right sign
  spearman          rank correlation steer vs yaw_1s (all samples)
  coverage          mean fraction of valid depth pixels (stereo holes, etc.)
  latency_ms        algorithm predict() p50 / p95 / max (decision rule timed separately)
  depth sheet       depth_sheet.jpg: 24 samples, image | depth | chosen direction, to eyeball

There is no ground-truth depth here, so these numbers rank DECISIONS, not depth accuracy.
Desktop timings do not transfer to a Jetson Nano: rerun the same command on the Nano.
"""
import argparse
import csv
import importlib
import importlib.util
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import TUBE_H_M, TUBE_W_M, Sample, decide_steer, draw_tube, tube_blocked  # noqa: E402

TURN_DEG = 15.0


def load(spec: str, kwargs: dict):
    target, _, cls = spec.rpartition(":")
    if target.endswith(".py"):
        sp = importlib.util.spec_from_file_location(Path(target).stem, target)
        mod = importlib.util.module_from_spec(sp)
        sys.path.insert(0, str(Path(target).resolve().parent))
        sp.loader.exec_module(mod)
    else:
        mod = importlib.import_module(target)
    return getattr(mod, cls)(**kwargs)


def kv(items):
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


def cls3(v, thr):
    return 1 if v > thr else (-1 if v < -thr else 0)


def spearman(a, b):
    ra, rb = np.argsort(np.argsort(a)), np.argsort(np.argsort(b))
    return float(np.corrcoef(ra, rb)[0, 1])


def vis_depth(depth, kind, size):
    d = np.asarray(depth, np.float32)
    near = np.where(np.isfinite(d) & (d > 0), (1 / d) if kind == "metric" else d, np.nan)
    v = near[np.isfinite(near)]
    lo, hi = (np.percentile(v, 2), np.percentile(v, 98)) if v.size else (0, 1)
    img = np.clip((np.nan_to_num(near, nan=lo) - lo) / max(hi - lo, 1e-6) * 255, 0, 255).astype(np.uint8)
    img = cv2.applyColorMap(cv2.resize(img, size, interpolation=cv2.INTER_NEAREST), cv2.COLORMAP_INFERNO)
    img[cv2.resize((~np.isfinite(near)).astype(np.uint8), size, interpolation=cv2.INTER_NEAREST) > 0] = (60, 60, 60)
    return img


def arrow(img, steer, label, color):
    h, w = img.shape[:2]
    u = int(w / 2 - np.tan(np.radians(steer)) * 627 * w / 1024)
    cv2.arrowedLine(img, (w // 2, h - 4), (int(np.clip(u, 0, w - 1)), h // 2), color, 2, tipLength=0.15)
    cv2.putText(img, label, (3, 12), 0, 0.38, color, 1)


def balanced(cy, cs):
    return float(np.mean([(cs[cy == c] == c).mean() for c in (1, 0, -1) if (cy == c).any()]))


def block_bootstrap(t, cy, cs, st, y, block_s=5.0, draws=1000, seed=0):
    """95% CI by resampling whole 5 s time blocks: neighbouring samples are not independent,
    so per-sample bootstrap would make the interval look far too tight."""
    blocks = np.floor((t - t.min()) / block_s).astype(int)
    ids = np.unique(blocks)
    members = [np.where(blocks == b)[0] for b in ids]
    rng = np.random.RandomState(seed)
    ba, sp = [], []
    for _ in range(draws):
        idx = np.concatenate([members[k] for k in rng.randint(0, len(ids), len(ids))])
        ba.append(balanced(cy[idx], cs[idx]))
        sp.append(spearman(st[idx], y[idx]) if np.std(st[idx]) > 0 else 0.0)
    ci = lambda v: [round(float(np.percentile(v, 2.5)), 3), round(float(np.percentile(v, 97.5)), 3)]  # noqa: E731
    return {"balanced_acc": ci(ba), "spearman": ci(sp), "blocks": int(len(ids)), "block_s": block_s}


def hazard_metrics(res, block_s=5.0, draws=1000, seed=0):
    sc = [x for x in res if x["blocked_true"] in (0, 1) and x["blocked_pred"] in (0, 1)]
    if not sc:
        labelled = any(x["blocked_true"] in (0, 1) for x in res)
        return {"scored": 0, "note": "algorithm gave neither 'blocked' nor a metric depth map" if labelled
                else "no hazard labels yet: run hazard/depthsuite/label_tool.py"}
    t = np.array([float(x["t_s"]) for x in sc])
    yt = np.array([x["blocked_true"] for x in sc])
    yp = np.array([x["blocked_pred"] for x in sc])

    def m(i):
        tp, fn = int(((yt[i] == 1) & (yp[i] == 1)).sum()), int(((yt[i] == 1) & (yp[i] == 0)).sum())
        fp, tn = int(((yt[i] == 0) & (yp[i] == 1)).sum()), int(((yt[i] == 0) & (yp[i] == 0)).sum())
        return tp, fn, fp, tn

    tp, fn, fp, tn = m(np.arange(len(sc)))
    blocks = np.floor((t - t.min()) / block_s).astype(int)
    members = [np.where(blocks == b)[0] for b in np.unique(blocks)]
    rng = np.random.RandomState(seed)
    rec, fsr = [], []
    for _ in range(draws):
        i = np.concatenate([members[k] for k in rng.randint(0, len(members), len(members))])
        a, b, c, d = m(i)
        if a + b:
            rec.append(a / float(a + b))
        if c + d:
            fsr.append(c / float(c + d))

    def ci(v):
        return [round(float(np.percentile(v, 2.5)), 3), round(float(np.percentile(v, 97.5)), 3)] if v else None

    return {"scored": len(sc), "hazards": tp + fn, "clears": fp + tn, "caught": tp, "missed_hazards": fn,
            "false_stops": fp,
            "hazard_recall": round(tp / float(tp + fn), 3) if tp + fn else None,
            "false_stop_rate": round(fp / float(fp + tn), 3) if fp + tn else None,
            "precision": round(tp / float(tp + fp), 3) if tp + fp else None,
            "ci95_recall": ci(rec), "ci95_false_stop": ci(fsr)}


def word(v, thr):
    c = cls3(v, thr)
    return "LEFT" if c == 1 else ("RIGHT" if c == -1 else "STRAIGHT")


def given_vs_predicted(s, depth, kind, steer, truth, cost, ok, deadband, hz_line=None, range_m=2.0):
    """One test image: GIVEN (left + right input) | PREDICTED (depth map, decision cost,
    steer arrow) with the truth arrow drawn on both so they can be compared directly."""
    W, H = 320, 200
    L = cv2.cvtColor(cv2.resize(s.left, (W, H), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
    R = cv2.cvtColor(cv2.resize(s.right, (W, H), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
    arrow(L, truth, "GIVEN left   truth {:+.0f}".format(truth), (0, 255, 0))
    cv2.putText(R, "GIVEN right", (3, 12), 0, 0.38, (255, 255, 255), 1)
    if depth is not None:
        D = vis_depth(depth, kind, (W, H))
        title = "PREDICTED depth (bright = near)"
    else:
        D = np.zeros_like(L)
        title = "PREDICTED (no depth map)"
    arrow(D, truth, "", (0, 255, 0))
    arrow(D, steer, title, (255, 255, 0))
    strip = np.zeros((24, W, 3), np.uint8)
    if cost is not None and len(cost):
        c = np.asarray(cost, np.float32)
        c = (c - c.min()) / max(float(c.max() - c.min()), 1e-6)
        bar = cv2.applyColorMap((255 - c * 255).astype(np.uint8)[None, :], cv2.COLORMAP_INFERNO)
        strip[4:20] = cv2.resize(bar, (W, 16), interpolation=cv2.INTER_NEAREST)
        cv2.putText(strip, "decision cost (dark = blocked)", (3, 15), 0, 0.33, (255, 255, 255), 1)
    for im in (L, D):
        draw_tube(im, range_m=range_m)
    D = np.vstack([D, strip])
    pad = np.zeros((24, W, 3), np.uint8)
    L, R = np.vstack([L, pad]), np.vstack([R, pad])
    head = np.zeros((26, 3 * W, 3), np.uint8)
    txt = "#{}  t={:.1f}s   truth (next 1 s): {} {:+.0f} deg   predicted: {} {:+.0f} deg   {}".format(
        s.id, s.t_s, word(truth, TURN_DEG), truth, word(steer, deadband), steer, "OK" if ok else "MISS")
    cv2.putText(head, txt, (6, 18), 0, 0.5, (0, 255, 0) if ok else (0, 0, 255), 1)
    cv2.putText(L, "green = where walker went", (3, H + 16), 0, 0.33, (0, 255, 0), 1)
    cv2.putText(R, "cyan = algorithm's choice", (3, H + 16), 0, 0.33, (255, 255, 0), 1)
    img = np.vstack([head, np.hstack([L, R, D])])
    if hz_line:
        hz = np.zeros((28, 3 * W, 3), np.uint8)
        cv2.putText(hz, hz_line[0], (6, 20), 0, 0.6, hz_line[1], 2)
        img = np.vstack([hz, img])
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--algo", required=True, help="file.py:Class or module:Class")
    ap.add_argument("--arg", action="append", help="constructor kwarg k=v")
    ap.add_argument("--set", type=Path, default=Path("data/walk/depth_set_400"))
    ap.add_argument("--steer-deadband", type=float, default=10.0)
    ap.add_argument("--budget-ms", type=float, default=50.0, help="p95 algorithm latency budget")
    ap.add_argument("--limit", type=int, default=0, help="score only the first N samples (quick try)")
    ap.add_argument("--images", type=int, default=-1,
                    help="write given-vs-predicted images for the first N samples (-1 = all, 0 = none)")
    ap.add_argument("--range-m", type=float, default=2.0, help="tube length, must match label_tool.py")
    ap.add_argument("--tube-width-m", type=float, default=TUBE_W_M)
    ap.add_argument("--tube-height-m", type=float, default=TUBE_H_M)
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()

    rect = json.loads((a.set / "rectify.json").read_text())
    with open(a.set / "samples.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    if a.limit:
        rows = rows[:a.limit]
    algo = load(a.algo, kv(a.arg))
    hz_true = {}
    if (a.set / "hazard_labels.csv").exists():
        with open(a.set / "hazard_labels.csv", newline="") as f:
            hz_true = {int(x["id"]): int(x["blocked"]) for x in csv.DictReader(f)}
        meta_p = a.set / "hazard_labels_meta.json"
        want = {"shape": "tube", "width_m": a.tube_width_m, "height_m": a.tube_height_m, "range_m": a.range_m}
        meta = json.loads(meta_p.read_text()) if meta_p.exists() else {}
        if {k: meta.get(k) for k in want} != want:
            print("WARNING: hazard labels were made with a different search box {} than this run {}.\n"
                  "         Review them: python -I hazard/depthsuite/label_tool.py --review".format(
                      meta or "(old image box)", want))

    name = Path(a.algo.rpartition(":")[0]).stem + "." + a.algo.rpartition(":")[2]
    out = a.out or a.set / "results" / name
    img_dir = out / "images"
    img_dir.mkdir(parents=True, exist_ok=True)
    for old in img_dir.glob("*.jpg"):
        old.unlink()

    res, lat, lat_dec, sheet = [], [], [], []
    sheet_ids = set(np.linspace(0, len(rows) - 1, min(24, len(rows))).astype(int).tolist())
    for n, r in enumerate(rows):
        sid = int(r["id"])
        rd = lambda d: cv2.imread(str(a.set / d / f"{sid:03d}.jpg"), cv2.IMREAD_GRAYSCALE)  # noqa: E731
        s = Sample(id=sid, t_s=float(r["t_s"]), left=rd("left"), right=rd("right"), prev_left=rd("prev_left"),
                   prev_dt_s=float(r["prev_dt_s"]), rectify=rect)
        t0 = time.perf_counter()
        pred = algo.predict(s) or {}
        lat.append((time.perf_counter() - t0) * 1e3)
        depth, kind = pred.get("depth"), pred.get("depth_kind", "metric")
        cov = None
        if depth is not None:
            dd = np.asarray(depth, np.float32)
            cov = float((np.isfinite(dd) & (dd > 0)).mean())
        cost = None
        if "steer_deg" in pred:
            steer = float(pred["steer_deg"])
        elif depth is not None:
            t1 = time.perf_counter()
            steer, cost = decide_steer(depth, kind)
            lat_dec.append((time.perf_counter() - t1) * 1e3)
        else:
            raise SystemExit("predict() returned neither depth nor steer_deg")
        y = float(r["yaw_1s"])
        ok = cls3(y, TURN_DEG) == cls3(steer, a.steer_deadband)
        if "blocked" in pred:
            bp = int(bool(pred["blocked"]))
        elif depth is not None and kind == "metric":
            bp = int(tube_blocked(depth, range_m=a.range_m, width_m=a.tube_width_m,
                                  height_m=a.tube_height_m)[0])
        else:
            bp = None
        bt = hz_true.get(sid)
        hz_line, tag = None, ("OK" if ok else "MISS")
        if bt in (0, 1) and bp is not None:
            tag, colr = {(1, 1): ("CAUGHT", (0, 255, 0)), (1, 0): ("MISSED_HAZARD", (0, 0, 255)),
                         (0, 1): ("FALSE_STOP", (0, 200, 255)), (0, 0): ("CLEAR_OK", (0, 255, 0))}[(bt, bp)]
            hz_line = ("straight ahead: truth {}  predicted {}  -> {}".format(
                "BLOCKED" if bt else "CLEAR", "BLOCKED" if bp else "CLEAR", tag), colr)
        elif bp is not None:
            hz_line = ("straight ahead predicted: {}   (no hand label yet)".format(
                "BLOCKED" if bp else "CLEAR"), (255, 255, 255))
        res.append({"id": sid, "t_s": r["t_s"], "yaw_1s": y, "steer_deg": round(steer, 2),
                    "correct": int(ok), "coverage": None if cov is None else round(cov, 3),
                    "action": pred.get("action", ""), "blocked_pred": "" if bp is None else bp,
                    "blocked_true": "" if bt is None else bt})
        if a.images and (a.images < 0 or n < a.images):
            cv2.imwrite(str(img_dir / "{:03d}_{}.jpg".format(sid, tag)),
                        given_vs_predicted(s, depth, kind, steer, y, cost, ok, a.steer_deadband, hz_line,
                                           a.range_m),
                        [cv2.IMWRITE_JPEG_QUALITY, 85])
        if n in sheet_ids:
            L = cv2.cvtColor(cv2.resize(s.left, (256, 160)), cv2.COLOR_GRAY2BGR)
            arrow(L, y, f"{sid} truth {y:+.0f}", (0, 255, 0))
            D = vis_depth(depth, kind, (256, 160)) if depth is not None else np.zeros_like(L)
            arrow(D, steer, f"steer {steer:+.0f}", (255, 255, 0))
            sheet.append(np.hstack([L, D]))

    y = np.array([x["yaw_1s"] for x in res])
    st = np.array([x["steer_deg"] for x in res])
    cy = np.array([cls3(v, TURN_DEG) for v in y])
    cs = np.array([cls3(v, a.steer_deadband) for v in st])
    turn = cy != 0
    rec = lambda c: round(float((cs[cy == c] == c).mean()), 3) if (cy == c).any() else None  # noqa: E731
    covs = [x["coverage"] for x in res if x["coverage"] is not None]
    lw = np.array(lat[3:] if len(lat) > 6 else lat)
    rep = {
        "algo": a.algo, "samples": len(res),
        "labels_left_straight_right": [int((cy == 1).sum()), int((cy == 0).sum()), int((cy == -1).sum())],
        "balanced_acc": round(float(np.mean([(cs[cy == c] == c).mean() for c in (1, 0, -1) if (cy == c).any()])), 3),
        "direction_acc": round(float((cy == cs).mean()), 3),
        "recall_left": rec(1), "recall_straight": rec(0), "recall_right": rec(-1),
        "sign_agreement_on_turns": round(float((np.sign(st[turn]) == cy[turn]).mean()), 3) if turn.any() else None,
        "spearman_steer_vs_yaw1s": round(spearman(st, y), 3) if np.std(st) > 0 else 0.0,
        "coverage_mean": round(float(np.mean(covs)), 3) if covs else None,
        "latency_ms": {k: round(float(np.percentile(lw, q)), 2) for k, q in (("p50", 50), ("p95", 95), ("max", 100))},
        "decision_rule_ms_p50": round(float(np.median(lat_dec)), 2) if lat_dec else None,
        "budget_ms": a.budget_ms,
    }
    rep["latency_pass"] = bool(rep["latency_ms"]["p95"] <= a.budget_ms)
    acts = [x["action"] for x in res if x["action"]]
    if acts:
        rep["actions"] = {k: acts.count(k) for k in sorted(set(acts))}
    rep["ci95"] = block_bootstrap(np.array([float(x["t_s"]) for x in res]), cy, cs, st, y)
    rep["hazard"] = hazard_metrics(res)
    bps = [x["blocked_pred"] for x in res if x["blocked_pred"] in (0, 1)]
    rep["predicted_blocked_fraction"] = round(float(np.mean(bps)), 3) if bps else None

    (out / "report.json").write_text(json.dumps(rep, indent=2))
    with open(out / "per_sample.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(res[0]))
        w.writeheader()
        w.writerows(res)
    if sheet:
        while len(sheet) % 3:
            sheet.append(np.zeros_like(sheet[0]))
        cv2.imwrite(str(out / "depth_sheet.jpg"), np.vstack([np.hstack(sheet[i:i + 3]) for i in range(0, len(sheet), 3)]))

    print(f"\n{a.algo}  samples={len(res)}  predicted blocked fraction {rep['predicted_blocked_fraction']}")
    hz = rep["hazard"]
    if hz.get("scored"):
        print(f"HAZARD (straight ahead, {a.range_m} m): caught {hz['caught']}/{hz['hazards']} "
              f"recall {hz['hazard_recall']} 95%CI {hz['ci95_recall']}  MISSED {hz['missed_hazards']}  "
              f"false stops {hz['false_stops']}/{hz['clears']} rate {hz['false_stop_rate']} 95%CI {hz['ci95_false_stop']}")
    else:
        print(f"HAZARD: {hz.get('note')}")
    print(f"secondary (walker turns) labels L/S/R={rep['labels_left_straight_right']}")
    print(f"BALANCED acc {rep['balanced_acc']} 95%CI {rep['ci95']['balanced_acc']} (chance 0.333)  "
          f"spearman 95%CI {rep['ci95']['spearman']}")
    print(f"direction acc {rep['direction_acc']}  recall L {rep['recall_left']} S {rep['recall_straight']} "
          f"R {rep['recall_right']}  sign-on-turns {rep['sign_agreement_on_turns']}  spearman {rep['spearman_steer_vs_yaw1s']}")
    print(f"coverage {rep['coverage_mean']}  latency p50 {rep['latency_ms']['p50']} p95 {rep['latency_ms']['p95']} ms "
          f"[{'PASS' if rep['latency_pass'] else 'FAIL'} vs {a.budget_ms}]  decision {rep['decision_rule_ms_p50']} ms")
    if "actions" in rep:
        print(f"actions {rep['actions']}")
    print(f"report: {out}")


if __name__ == "__main__":
    main()
