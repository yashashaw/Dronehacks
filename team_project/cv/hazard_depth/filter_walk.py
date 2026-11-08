"""Step 2: flag bad frames in metrics.csv and write manifest.csv + review contact sheets.

    python -I hazard/filter_walk.py data/walk/hazard_10fps            # default rules
    python -I hazard/filter_walk.py data/walk/hazard_10fps --blur 40  # re-tune

Nothing is deleted. Every frame stays in manifest.csv with keep=0/1 and a reason, and
overrides.csv (idx,keep) lets a human force a decision after looking at the sheets.

Rules (all configurable)
  sync      left/right stamps differ by more than --sync-ms
  dark      either eye's mean brightness < --dark
  blur      variance of Laplacian (512x320 left) < --blur
  occluded  near-lens hand/body: low-texture fraction > --lowtex in either eye, or the
            two eyes disagree (thumbnail correlation < --lr-ncc)
  person    a person box that is "in the course": box width >= --person-w of the frame,
            or box area >= --person-area. Distant bystanders behind the nets pass.
  untracked fewer than --min-inliers features tracked to the previous frame
Labels are judged separately (label_ok_*): a kept frame can still have no valid label.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np


def rules_from_args(a):
    return {"sync_ms": a.sync_ms, "dark": a.dark, "blur": a.blur, "lowtex": a.lowtex,
            "lr_ncc": a.lr_ncc, "person_w": a.person_w, "person_area": a.person_area,
            "min_inliers": a.min_inliers, "course_start_s": a.course_start_s}


def reasons_for(r: dict, R: dict) -> list[str]:
    out = []
    f = lambda k: float(r[k]) if r[k] not in ("", "nan") else float("nan")  # noqa: E731
    if not f("sync_ms") <= R["sync_ms"]:
        out.append("sync")
    if min(f("brightness_l"), f("brightness_r")) < R["dark"]:
        out.append("dark")
    if f("sharpness") < R["blur"]:
        out.append("blur")
    if max(f("lowtex_l"), f("lowtex_r")) > R["lowtex"] or f("lr_ncc") < R["lr_ncc"]:
        out.append("occluded")
    if float(r["t_s"]) < R["course_start_s"]:
        out.append("off_course")
    # The camera was carried near floor level, so bystanders behind the nets show up as
    # tall, narrow boxes. People actually in the way are WIDE (close to the lens).
    for x0, y0, x1, y1, _c in json.loads(r["people"]):
        if (x1 - x0) >= R["person_w"] or (x1 - x0) * (y1 - y0) >= R["person_area"]:
            out.append("person")
            break
    if int(r["track_inliers"]) < R["min_inliers"] and int(r["idx"]) > 0:
        out.append("untracked")
    return out


def contact_sheet(ds: Path, rows: list[dict], path: Path, cols: int = 10, tw: int = 192):
    if not rows:
        return
    th = tw * 640 // 1024
    tiles = []
    for r in rows:
        img = cv2.imread(str(ds / "left" / f"{int(r['idx']):05d}.jpg"))
        img = cv2.resize(img, (tw, th), interpolation=cv2.INTER_AREA)
        for x0, y0, x1, y1, _c in json.loads(r["people"]):
            cv2.rectangle(img, (int(x0 * tw), int(y0 * th)), (int(x1 * tw), int(y1 * th)), (0, 200, 255), 1)
        label = f"{r['idx']} {float(r['t_s']):.1f}s"
        cv2.putText(img, label, (3, 13), 0, 0.4, (0, 255, 255), 1)
        if r["reason"]:
            cv2.putText(img, r["reason"], (3, th - 5), 0, 0.4, (0, 0, 255), 1)
        tiles.append(img)
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    sheet = np.vstack([np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)])
    cv2.imwrite(str(path), sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--sync-ms", type=float, default=5.0)
    ap.add_argument("--dark", type=float, default=20.0)
    ap.add_argument("--blur", type=float, default=350.0, help="~3rd percentile of this walk")
    ap.add_argument("--lowtex", type=float, default=0.55)
    ap.add_argument("--lr-ncc", type=float, default=0.5)
    ap.add_argument("--person-w", type=float, default=0.22, help="box width fraction; 9 = off")
    ap.add_argument("--person-area", type=float, default=0.12, help="box area fraction; 9 = off")
    ap.add_argument("--course-start-s", type=float, default=10.0, help="walk-in before this is off course")
    ap.add_argument("--min-inliers", type=int, default=25)
    ap.add_argument("--split", default="train,train,train,val,test", help="split of each successive time block")
    ap.add_argument("--block-s", type=float, default=10.0)
    args = ap.parse_args()
    ds = args.dataset
    R = rules_from_args(args)

    with open(ds / "metrics.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    overrides = {}
    if (ds / "overrides.csv").exists():
        with open(ds / "overrides.csv", newline="") as f:
            overrides = {int(r["idx"]): int(r["keep"]) for r in csv.DictReader(f)}

    # Interleaved 10 s time blocks (train,train,train,val,test, repeating): neighbouring
    # 10 fps frames are near-duplicates so a random split leaks, but one contiguous tail
    # block of a 2-minute walk contained no turns at all.
    pattern = args.split.split(",")
    for r in rows:
        why = reasons_for(r, R)
        keep = int(not why)
        if int(r["idx"]) in overrides:
            keep = overrides[int(r["idx"])]
            why.append("override")
        r["reason"] = "+".join(why)
        r["keep"] = keep
        r["split"] = pattern[int(float(r["t_s"]) // args.block_s) % len(pattern)]

    with open(ds / "manifest.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    rejected = [r for r in rows if not r["keep"]]
    kept = [r for r in rows if r["keep"]]
    contact_sheet(ds, rejected, ds / "review_rejected.jpg")
    contact_sheet(ds, kept[::max(1, len(kept) // 120)], ds / "review_kept_sample.jpg")

    counts = {}
    for r in rejected:
        for why in r["reason"].split("+"):
            counts[why] = counts.get(why, 0) + 1
    summary = {"rules": R, "frames": len(rows), "kept": len(kept), "rejected": len(rejected),
               "rejected_by_reason": counts,
               "kept_by_split": {s: sum(1 for r in kept if r["split"] == s) for s in ("train", "val", "test")},
               "kept_with_1s_label": sum(1 for r in kept if r["label_ok_1s"] == "1")}
    (ds / "filter_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
