# Landing-pad YOLO training pipeline

Train a one-class (`pad`) YOLO detector from phone videos in which the landing
pad appears part-way through. Label a small baseline by hand, train a first
model, let it pre-label the remaining frames while you correct it, train the
final model, then check it on a video it has never seen.

## Setup

From `CV/`:

```powershell
.venv\Scripts\Activate.ps1
pip install ultralytics opencv-python
cd video_training
```

The YOLOv8n base weights are downloaded to `models/yolov8n.pt` on first use.

## Folders

| Folder | Contents |
| --- | --- |
| `raw_videos/` | Source videos (`.mov`, `.mp4`, `.avi`, `.mkv`) |
| `extracted_frames/train/`, `extracted_frames/val/` | Frames `{video}_frame_{index:05d}.jpg` and their labels |
| `extracted_frames/split.json` | Which videos went to train and which to val |
| `models/` | `initial.pt`, `final.pt` |
| `results/` | Validation comparisons and rendered videos |
| `work/` | Temporary Ultralytics datasets and training runs (safe to delete) |

Labels are YOLO `.txt` files next to each image (`0 cx cy w h`, normalized):

- a `.txt` with lines: the pad's box(es)
- an empty `.txt`: checked, no pad in the frame (used as a negative example)
- no `.txt`: not labeled yet, never used for training

The split is by whole video, so validation frames come from a flight the model
never trained on (currently `IMG_6184.mov`).

## Run order

| Script | Reads | Writes |
| --- | --- | --- |
| `01_dataset_split.py` | `raw_videos/` | `extracted_frames/{train,val}/*.jpg`, `split.json`. Every 5th frame, 20% of videos to val, seed 42. **Re-running deletes all frames and labels** (asks first if labels exist; `--force` skips the prompt). |
| `02_annotate_baseline.py` | `extracted_frames/train/` | Labels for up to 50 train frames by hand. Keeps existing labels; re-run to resume. |
| `03_train_initial.py` | Labeled train frames | `models/initial.pt` (holds out 20% of the baseline for metrics) |
| `04_review_predictions.py` | `models/initial.pt`, unlabeled train + val frames | A label per frame you accept or correct. Re-run to resume. |
| `05_train_final.py` | Labeled train frames (training), labeled val frames (metrics only) | `models/final.pt` |
| `06_compare_validation.py` | `models/final.pt`, labeled val frames | `results/compare/*.jpg` (green = human, orange = model), `summary.txt` with hits, misses and false alarms; then a viewer (any key = next, `q` = stop) |
| `07_render_video.py` | `models/final.pt`, val videos from `split.json` | `results/{video}_inference.mp4` at the source fps. Pass video names to render only some. |

Helpers, not run directly: `paths.py` (folders and label I/O),
`annotation_ui.py` (the drawing window), `training.py` (dataset build, training
and prediction).

## Labeling keys (steps 02 and 04)

| Key | Action |
| --- | --- |
| Drag | Draw a box. In step 04 the first box you draw replaces the model's guess. |
| `n` / Space | Save. With no boxes this writes an empty file ("no pad"). In step 04 this accepts the model's orange boxes as they are. |
| `s` | Skip: the frame stays unlabeled |
| `b` | Back to the previous frame of this session, showing what you saved in green. Save again to overwrite it, or `s` to delete its label. |
| `c` | Clear all boxes |
| `u` | Undo the last box |
| `q` / Esc | Quit (progress so far is saved) |

## Caveats

- The phone videos are color, but the drone's left camera is 1024x640
  grayscale. Before flying the model, check it on grayscale frames (or train
  on grayscale copies).
- Labels made in step 04 are only as good as your review. Correct the
  model's misses and false boxes instead of accepting them all.
