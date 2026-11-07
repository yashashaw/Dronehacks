"""Step 04: label every remaining frame by correcting the initial model's guesses.

Reads:  models/initial.pt, unlabeled frames in extracted_frames/train and /val
Writes: a .txt next to each frame you accept or correct

The model's boxes appear in orange. Pressing n / Space accepts them as they are
(an empty file if the model saw no pad). Drawing a box discards the guess and
starts your own boxes. Frames that already have a .txt are skipped, so
quitting and re-running resumes.

Keys: drag = draw (replaces guess) | n / Space = accept/save | s = skip (stays unlabeled)
      b = back one frame (s there removes its label) | c = clear
      u = undo last box | q / Esc = quit
"""
import os

import paths
import training
from annotation_ui import BoxAnnotator, label_frames

CONFIDENCE = 0.25


def main():
    frames = [(split, p) for split, folder in (("train", paths.TRAIN_DIR), ("val", paths.VAL_DIR))
              for p in paths.list_images(folder) if not paths.is_labeled(p)]
    if not frames:
        print("Every frame is labeled. Next: 05_train_final.py")
        return

    model = training.load_model(paths.INITIAL_MODEL)
    print(f"{len(frames)} unlabeled frames to review.")
    print("KEYS: drag = draw | n/Space = accept/save | s = skip | b = back | c = clear | u = undo | q = quit\n")

    def guess(img):
        return [g[:4] for g in training.predict_boxes(model, img, conf=CONFIDENCE)]

    annotator = BoxAnnotator("Review predictions")
    headers = [f"{i + 1}/{len(frames)} {split} {os.path.basename(p)}"
               for i, (split, p) in enumerate(frames)]
    label_frames(annotator, [p for _, p in frames], headers, guess)
    annotator.close()

    remaining = sum(not paths.is_labeled(p) for _, p in frames)
    print(f"\nLabeled {len(frames) - remaining} frames; {remaining} still unlabeled.")
    print("Next: 05_train_final.py" if remaining == 0 else "Re-run to continue (or train with what you have).")


if __name__ == "__main__":
    main()
