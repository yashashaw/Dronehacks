# hazard/ — turn prediction and depth-decision test bench

Goal: a light, Jetson-Nano-friendly perception step that says which way the course goes
(where the camera is about to turn) without loading the GPU that VIO and other sensors need.

Data (not in git, see `.gitignore`): `data/walk/stereo-20261108-044200/` from `walk.zip`,
a hand-carried OAK stereo walk through the course, 1024x640 mono at 30 fps, 177 s.

## Pipeline

```bash
pip install opencv-python numpy ultralytics pytest   # ultralytics only for the person flag (CPU)
python -I hazard/prepare_walk.py data/walk/stereo-20261108-044200 --out data/walk/hazard_10fps --cut-s 120
python -I hazard/filter_walk.py  data/walk/hazard_10fps          # re-run freely with new thresholds
python -I hazard/depthsuite/build_set.py data/walk/hazard_10fps --out data/walk/depth_set_400
python -m pytest -q hazard/tests
```

1. **prepare_walk.py**: cuts at 120 s, pairs L/R by timestamp, samples a 100 ms grid
   (1171 frames), and builds turn labels from 30 fps visual odometry. `yaw_<h>s` is the
   heading change over the next h seconds, + = left. Yaw sign is verified with synthetic
   rotations. Magnitude depends on fx = 627 from the drone's camera_info.
2. **filter_walk.py**: flags frames and never deletes them. Reasons: `off_course`
   (walk-in, before 10 s), `person` (box width ≥ 22 % / area ≥ 12 %: close people; bystanders
   behind the nets are tall and narrow and pass), `occluded` (hand/body on a lens:
   low texture or eyes disagree), `blur`, `sync` (> 5 ms), `untracked`. Writes `manifest.csv`,
   `review_rejected.jpg` and `review_kept_sample.jpg`. Overrides go in `overrides.csv` (`idx,keep`).
   Current run: 846 kept / 325 flagged.
3. **build_set.py**: the fixed **400-frame test set**. Kept frames only, L/R within 1 ms (max 0.023 ms),
   spread evenly over 10–118.5 s (max gap 2.5 s), each with the previous frame. It also estimates
   an uncalibrated stereo rectification (rows aligned to 0.04 px median), because the
   drone's camera_info has no distortion or stereo rotation.

## Main question: would flying straight hit something?

The walk did not follow the drone's path (sometimes it pointed straight at a wall on
purpose), so the walker's turns are NOT the ground truth. The ground truth is a hand label
per frame: **is the straight-ahead corridor blocked within 2 m?**

```bash
python -I hazard/depthsuite/label_tool.py      # b blocked / c clear / u unsure / s skip / z back / q quit
python -I hazard/depthsuite/run_depth_suite.py --algo hazard/depthsuite/baselines.py:StereoSGBM
```

The search region is a **3D drone-shaped tube**: 0.6 m wide (0.47 m with props + margin) x
0.25 m tall (drone height is unknown; ~0.15 m assumed + margin) x 2 m long. Every depth pixel is
back-projected to 3D and tested against it (`common.tube_blocked`), so near objects at the
side count exactly when the drone's body would hit them. Less than 30 % depth in the far
cross-section = unknown = blocked. The labeller draws the same tube in perspective.
Headline: **hazard recall** (1 − recall = crashes) and **false-stop rate**, with 95 % CIs.
Images are named `NNN_CAUGHT`, `NNN_MISSED_HAZARD`, `NNN_FALSE_STOP`, `NNN_CLEAR_OK`.

Labels made with the old flat image box are kept in `hazard_labels_tallbox_v1.csv`; review
them against the tube with `label_tool.py --review` (x keeps, b/c/u changes). The suite warns
when `hazard_labels_meta.json` does not match the tube it scores with.

This walk was carried ~10–20 cm above the floor, so the floor enters the tube's lower half
near the camera; at 0.76 m flight height it cannot. StereoBM at these settings leaves too many
speckles in the tube to be usable (blocks every frame); SGBM's speckle filter handles it.

`policies.py:CenterThenRight` is the team rule: centre open → straight, else right if open,
else left, else stop (`--arg open_m=1.5 --arg band_top=0.2 --arg band_bottom=0.5`).

## Depth decision suite (`depthsuite/`)

Your algorithm returns a depth / inverse-depth map, a `steer_deg`, or both (see `common.py`).
Every depth map goes through the **same** decision rule (most open column in the
middle band, drone-width smoothing, slight straight bias). Scoring compares that
decision to where the walker actually turned in the next 1 s.

```bash
python -I hazard/depthsuite/run_depth_suite.py --algo my_algo.py:MyAlgo --budget-ms 30
```

Outputs in `data/walk/depth_set_400/results/<algo>/`:
- `images/NNN_OK.jpg` / `NNN_MISS.jpg`: one test image per sample. The GIVEN left + right
  inputs, the PREDICTED depth map and decision-cost strip, green arrow = where the walker
  went, cyan = the algorithm's choice.
- `make_depth_images.py --algo ...` writes `depth_set_400/depth_images/<algo>/NNN.jpg`: base
  image on the left, estimated depth on the right (bright = near, grey = no estimate), plus
  `overview.jpg`.
- `report.json` (with 95 % block-bootstrap CIs), `per_sample.csv`, `depth_sheet.jpg`.

Baselines on this laptop's CPU (i5-12450H, no GPU), 400 frames, labels L/S/R = 38/274/88:

| algo | balanced acc [95 % CI] (chance .333) | spearman [95 % CI] | coverage | p50 ms |
|---|---|---|---|---|
| StraightAhead | 0.333 | 0 | – | 0 |
| StereoBM 320x200 | 0.400 [0.26, 0.50] | 0.18 [-0.01, 0.35] | 0.62 | 4.5 |
| StereoSGBM 320x200 | 0.388 [0.27, 0.47] | 0.25 [0.11, 0.40] | 0.77 | 7.5 |
| FlowNearness (mono, no model) | 0.380 [0.30, 0.45] | 0.15 [-0.01, 0.28] | 1.00 | 7.6 |

No baseline's balanced accuracy is distinguishable from chance; only SGBM's correlation
clearly excludes zero. Treat these as a floor. "Most open direction now" only weakly
predicts where a person walked next, because course turns are planned, not always
toward free space.

## Jetson Nano

The runtime path (`vision.py`, `depthsuite/common.py`, `depthsuite/baselines.py`,
`depthsuite/run_depth_suite.py`, `testsuite/*`) needs only Python >= 3.6 + OpenCV + NumPy
(checked with `vermin`), so it runs on stock JetPack 4 (Python 3.6, OpenCV 4.1) with no
pip installs and no GPU. Copy `hazard/` and `data/walk/depth_set_400/` to the Nano and run
the same command. Offline tools (`prepare_walk.py`, `filter_walk.py`, `build_set.py`)
run on the laptop.

## Turn-prediction suite (`testsuite/`)

Streams the 10 fps sequence in time order to a stateful model (`interface.py`) and scores
MAE, direction accuracy, turn lead time, false alarms/min and latency. Splits are
interleaved 10 s blocks (train,train,train,val,test), so every split contains turns and
near-duplicate neighbours do not leak across splits. Baselines: Zero, ConstantRate (vision-only), Oracle (harness check).

## Caveats

- The 400-frame set is drawn from the whole walk: if you train on `hazard_10fps`, it is
  contaminated. Use it for evaluation only, or train on another walk.
- No ground-truth depth exists for this walk, so the suites rank **decisions**, not depth accuracy.
- Handheld walking is not drone flight (bob, wrist rotation, near-floor camera height).
- Desktop latency does not transfer to a Jetson Nano; run the same commands there.
