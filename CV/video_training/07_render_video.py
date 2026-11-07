"""Step 07: run the final model on every frame of each validation video.

Reads:  models/final.pt, the val videos listed in extracted_frames/split.json
Writes: results/{video}_inference.mp4 (same fps as the source)

Usage: python 07_render_video.py [VIDEO_NAME ...]   (default: all val videos)
"""
import os
import sys

import cv2

import paths
import training
from annotation_ui import MODEL_COLOR

CONFIDENCE = 0.25


def render(model, video_path):
    name = os.path.splitext(os.path.basename(video_path))[0]
    out_path = os.path.join(paths.RESULTS_DIR, f"{name}_inference.mp4")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Could not open {video_path}")
        return
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    writer = None
    index = detected = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if writer is None:   # size from the decoded frame, which is already rotated upright
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            thick = max(2, round(max(h, w) / 500))
        boxes = training.predict_boxes(model, frame, conf=CONFIDENCE)
        detected += bool(boxes)
        for x1, y1, x2, y2, conf in boxes:
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), MODEL_COLOR, thick)
            cv2.putText(frame, f"pad {conf:.2f}", (int(x1), max(20, int(y1) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, thick * 0.4, MODEL_COLOR, thick)
        cv2.putText(frame, f"frame {index}", (10, 20 + thick * 12),
                    cv2.FONT_HERSHEY_SIMPLEX, thick * 0.4, (255, 255, 255), thick)
        writer.write(frame)
        index += 1
        if index % 100 == 0:
            print(f"  {name}: {index}/{total} frames")
    cap.release()
    if writer is not None:
        writer.release()
    print(f"Wrote {out_path}: {index} frames at {fps:.2f} fps, pad seen in {detected}")


def main():
    videos = paths.validation_videos()
    if len(sys.argv) > 1:
        wanted = {os.path.splitext(a)[0] for a in sys.argv[1:]}
        videos = [v for v in videos if os.path.splitext(os.path.basename(v))[0] in wanted]
    if not videos:
        print("No validation videos found (check extracted_frames/split.json or the arguments).")
        return

    model = training.load_model(paths.FINAL_MODEL)
    os.makedirs(paths.RESULTS_DIR, exist_ok=True)
    for video in videos:
        render(model, video)


if __name__ == "__main__":
    main()
