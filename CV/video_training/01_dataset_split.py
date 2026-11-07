"""Step 01: split raw videos into train/val by whole video and extract frames.

Reads:  raw_videos/*.mov|mp4|avi|mkv
Writes: extracted_frames/train/*.jpg, extracted_frames/val/*.jpg, extracted_frames/split.json

Re-running clears extracted_frames/ (including any labels) and re-extracts.
Pass --force to skip the confirmation when labels exist.
"""
import json
import os
import random
import shutil
import sys

import cv2

import paths

FRAME_INTERVAL = 5   # keep every 5th frame
VAL_RATIO = 0.2      # fraction of videos held out for validation
SEED = 42


def extract_frames_from_video(video_path, output_folder, frame_interval):
    """Save every frame_interval-th frame as {video}_frame_{index:05d}.jpg."""
    os.makedirs(output_folder, exist_ok=True)
    video_name = os.path.splitext(os.path.basename(video_path))[0]
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"  Could not open {video_path}")
        return 0

    frame_count = saved_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame_count % frame_interval == 0:
            name = f"{video_name}_frame_{frame_count:05d}.jpg"
            cv2.imwrite(os.path.join(output_folder, name), frame)
            saved_count += 1
        frame_count += 1
    cap.release()
    return saved_count


def confirm_clear():
    labels = [p for d in (paths.TRAIN_DIR, paths.VAL_DIR)
              for p in paths.list_images(d) if paths.is_labeled(p)]
    if not labels or "--force" in sys.argv:
        return True
    answer = input(f"{len(labels)} labeled frames will be deleted. Type 'yes' to continue: ")
    return answer.strip().lower() == "yes"


def main():
    videos = paths.list_videos()
    if len(videos) < 2:
        print(f"Need at least 2 videos in {paths.VIDEO_DIR} (found {len(videos)})")
        return
    if not confirm_clear():
        print("Aborted; nothing changed.")
        return

    random.seed(SEED)
    random.shuffle(videos)
    val_count = max(1, int(len(videos) * VAL_RATIO))
    split = {"val": videos[:val_count], "train": videos[val_count:]}

    shutil.rmtree(paths.FRAMES_DIR, ignore_errors=True)
    for name, folder in (("train", paths.TRAIN_DIR), ("val", paths.VAL_DIR)):
        print(f"\n{name}: {len(split[name])} videos")
        for video in split[name]:
            saved = extract_frames_from_video(video, folder, FRAME_INTERVAL)
            print(f"  {os.path.basename(video)}: {saved} frames")

    with open(paths.SPLIT_FILE, "w") as f:
        json.dump({
            "seed": SEED, "val_ratio": VAL_RATIO, "frame_interval": FRAME_INTERVAL,
            "train": sorted(os.path.basename(v) for v in split["train"]),
            "val": sorted(os.path.basename(v) for v in split["val"]),
        }, f, indent=2)
    print(f"\nWrote {paths.SPLIT_FILE}")


if __name__ == "__main__":
    main()
