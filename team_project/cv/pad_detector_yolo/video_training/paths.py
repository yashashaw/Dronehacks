"""Shared folders and label I/O for every numbered script. Not run directly.

Labels are YOLO .txt files next to their images, one class (0 = pad):
  - a .txt with lines   = boxes
  - an empty .txt       = checked by a human, no pad in the frame
  - no .txt at all      = not labeled yet (never used for training)
"""
import glob
import json
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VIDEO_DIR = os.path.join(SCRIPT_DIR, "raw_videos")
FRAMES_DIR = os.path.join(SCRIPT_DIR, "extracted_frames")
TRAIN_DIR = os.path.join(FRAMES_DIR, "train")
VAL_DIR = os.path.join(FRAMES_DIR, "val")
SPLIT_FILE = os.path.join(FRAMES_DIR, "split.json")
MODELS_DIR = os.path.join(SCRIPT_DIR, "models")
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")
WORK_DIR = os.path.join(SCRIPT_DIR, "work")          # temporary datasets and training runs

INITIAL_MODEL = os.path.join(MODELS_DIR, "initial.pt")
FINAL_MODEL = os.path.join(MODELS_DIR, "final.pt")
BASE_WEIGHTS = os.path.join(MODELS_DIR, "yolov8n.pt")  # downloaded by Ultralytics on first use

CLASS_NAMES = ["pad"]
IMAGE_EXTENSIONS = ("*.jpg", "*.jpeg", "*.JPG", "*.JPEG")
VIDEO_EXTENSIONS = ("*.mp4", "*.avi", "*.mov", "*.mkv", "*.MP4", "*.MOV")


def list_images(folder):
    images = set()
    for ext in IMAGE_EXTENSIONS:
        images.update(glob.glob(os.path.join(folder, ext)))
    return sorted(images)


def list_videos(folder=VIDEO_DIR):
    videos = set()
    for ext in VIDEO_EXTENSIONS:
        videos.update(glob.glob(os.path.join(folder, ext)))
    return sorted(videos)


def label_path(image_path):
    return os.path.splitext(image_path)[0] + ".txt"


def is_labeled(image_path):
    return os.path.exists(label_path(image_path))


def frame_video_name(image_path):
    """'IMG_6184_frame_00130.jpg' -> 'IMG_6184'."""
    stem = os.path.splitext(os.path.basename(image_path))[0]
    return stem.rsplit("_frame_", 1)[0]


def read_labels(txt_path, img_w, img_h):
    """Return pixel boxes [(x1, y1, x2, y2), ...] from a YOLO .txt file."""
    boxes = []
    with open(txt_path) as f:
        for line in f:
            parts = line.split()
            if len(parts) != 5:
                continue
            _, cx, cy, w, h = (float(p) for p in parts)
            boxes.append((
                (cx - w / 2) * img_w, (cy - h / 2) * img_h,
                (cx + w / 2) * img_w, (cy + h / 2) * img_h))
    return boxes


def write_labels(txt_path, boxes, img_w, img_h):
    """Write pixel boxes as YOLO lines (class 0). No boxes writes an empty file."""
    with open(txt_path, "w") as f:
        for x1, y1, x2, y2 in boxes:
            x1, x2 = sorted((max(0.0, x1), min(float(img_w), x2)))
            y1, y2 = sorted((max(0.0, y1), min(float(img_h), y2)))
            f.write(f"0 {(x1 + x2) / 2 / img_w:.6f} {(y1 + y2) / 2 / img_h:.6f} "
                    f"{(x2 - x1) / img_w:.6f} {(y2 - y1) / img_h:.6f}\n")


def validation_videos():
    """Paths of the validation videos: from split.json, else inferred from val frame names."""
    if os.path.exists(SPLIT_FILE):
        with open(SPLIT_FILE) as f:
            names = json.load(f)["val"]
        return [os.path.join(VIDEO_DIR, n) for n in names]
    stems = {frame_video_name(p) for p in list_images(VAL_DIR)}
    return [v for v in list_videos() if os.path.splitext(os.path.basename(v))[0] in stems]
