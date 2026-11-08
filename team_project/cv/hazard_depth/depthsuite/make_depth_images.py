"""Side-by-side test images: base (left camera) image on the left, estimated depth on the right.

    python -I hazard/depthsuite/make_depth_images.py --algo hazard/depthsuite/baselines.py:StereoSGBM
    python -I hazard/depthsuite/make_depth_images.py --algo my_algo.py:MyAlgo --limit 50

Writes data/walk/depth_set_400/depth_images/<algo>/NNN.jpg (one per sample, 1024x320 + legend)
and overview.jpg (every 10th sample). Depth colours: bright = near, dark = far, grey = no
estimate. Metric depth maps also get metre labels on the colour bar. Runs on Python 3.6 /
OpenCV 4.1 (stock Jetson Nano JetPack 4).
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import Sample  # noqa: E402
from run_depth_suite import kv, load  # noqa: E402

PW, PH = 512, 320  # panel size


def colourise(depth, kind):
    d = np.asarray(depth, np.float32)
    valid = np.isfinite(d) & (d > 0)
    near = np.full(d.shape, np.nan, np.float32)
    near[valid] = (1.0 / d[valid]) if kind == "metric" else d[valid]
    v = near[valid]
    lo, hi = (float(np.percentile(v, 2)), float(np.percentile(v, 98))) if v.size else (0.0, 1.0)
    hi = max(hi, lo + 1e-6)
    g = np.clip((np.nan_to_num(near, nan=lo) - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
    img = cv2.applyColorMap(cv2.resize(g, (PW, PH), interpolation=cv2.INTER_NEAREST), cv2.COLORMAP_INFERNO)
    hole = cv2.resize((~valid).astype(np.uint8), (PW, PH), interpolation=cv2.INTER_NEAREST) > 0
    img[hole] = (70, 70, 70)
    return img, lo, hi, float(valid.mean())


def legend(kind, lo, hi, coverage):
    bar = np.zeros((36, 2 * PW, 3), np.uint8)
    ramp = cv2.applyColorMap(np.linspace(0, 255, PW - 120).astype(np.uint8)[None, :], cv2.COLORMAP_INFERNO)
    bar[8:22, PW + 60:2 * PW - 60] = cv2.resize(ramp, (PW - 120, 14))
    if kind == "metric":
        far, near = "{:.1f} m".format(1.0 / lo) if lo > 0 else "inf", "{:.2f} m".format(1.0 / hi)
    else:
        far, near = "far", "near"
    cv2.putText(bar, far, (PW + 4, 20), 0, 0.4, (255, 255, 255), 1)
    cv2.putText(bar, near, (2 * PW - 56, 20), 0, 0.4, (255, 255, 255), 1)
    cv2.putText(bar, "grey = no estimate   valid {:.0f}%".format(coverage * 100), (PW + 60, 33), 0, 0.35,
                (200, 200, 200), 1)
    return bar


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--algo", required=True)
    ap.add_argument("--arg", action="append")
    ap.add_argument("--set", type=Path, default=Path("data/walk/depth_set_400"))
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()

    rect = json.loads((a.set / "rectify.json").read_text())
    with open(a.set / "samples.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    if a.limit:
        rows = rows[:a.limit]
    algo = load(a.algo, kv(a.arg))
    name = Path(a.algo.rpartition(":")[0]).stem + "." + a.algo.rpartition(":")[2]
    out = a.out or a.set / "depth_images" / name
    out.mkdir(parents=True, exist_ok=True)

    thumbs = []
    for n, r in enumerate(rows):
        sid = int(r["id"])
        rd = lambda d: cv2.imread(str(a.set / d / "{:03d}.jpg".format(sid)), cv2.IMREAD_GRAYSCALE)  # noqa: E731
        s = Sample(sid, float(r["t_s"]), rd("left"), rd("right"), rd("prev_left"), float(r["prev_dt_s"]), rect)
        pred = algo.predict(s) or {}
        if pred.get("depth") is None:
            raise SystemExit("{} returns no depth map".format(a.algo))
        kind = pred.get("depth_kind", "metric")
        D, lo, hi, cov = colourise(pred["depth"], kind)
        B = cv2.cvtColor(cv2.resize(s.left, (PW, PH), interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
        cv2.putText(B, "base image  #{}  t={:.1f}s".format(sid, s.t_s), (6, 18), 0, 0.5, (0, 255, 255), 1)
        cv2.putText(D, "estimated depth ({})".format(name.split(".")[-1]), (6, 18), 0, 0.5, (255, 255, 255), 1)
        img = np.vstack([np.hstack([B, D]), legend(kind, lo, hi, cov)])
        cv2.imwrite(str(out / "{:03d}.jpg".format(sid)), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if n % 10 == 0:
            thumbs.append(cv2.resize(img[:PH], (PW, PH // 2), interpolation=cv2.INTER_AREA))

    cols = 4
    while len(thumbs) % cols:
        thumbs.append(np.zeros_like(thumbs[0]))
    cv2.imwrite(str(out / "overview.jpg"), np.vstack([np.hstack(thumbs[i:i + cols]) for i in range(0, len(thumbs), cols)]),
                [cv2.IMWRITE_JPEG_QUALITY, 85])
    print("{} images -> {}".format(len(rows), out))


if __name__ == "__main__":
    main()
