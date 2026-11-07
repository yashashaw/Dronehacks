"""Step 02: hand-label a small baseline set of training frames.

Reads:  extracted_frames/train/*.jpg
Writes: extracted_frames/train/*.txt (YOLO labels; empty file = no pad)

Labels until NUM_TO_ANNOTATE train frames have a .txt. Existing labels are
kept, so quitting and re-running continues where you stopped.

Keys: drag = draw | n / Space = save (empty file if no box) | s = skip
      b = back one frame (s there removes its label) | c = clear
      u = undo last box | q / Esc = quit
"""
import random

import paths
from annotation_ui import BoxAnnotator, label_frames

NUM_TO_ANNOTATE = 50
SEED = 42


def main():
    images = paths.list_images(paths.TRAIN_DIR)
    if not images:
        print(f"No images found in {paths.TRAIN_DIR}. Run 01_dataset_split.py first.")
        return

    labeled = [p for p in images if paths.is_labeled(p)]
    unlabeled = [p for p in images if not paths.is_labeled(p)]
    needed = max(0, NUM_TO_ANNOTATE - len(labeled))
    print(f"{len(labeled)} train frames already labeled; target {NUM_TO_ANNOTATE}.")
    if needed == 0 or not unlabeled:
        print("Baseline complete. Next: 03_train_initial.py")
        return

    random.Random(SEED).shuffle(unlabeled)
    selected = unlabeled[:needed]
    print(f"Labeling {len(selected)} more frames.")
    print("KEYS: drag = draw | n/Space = save | s = skip | b = back | c = clear | u = undo | q = quit\n")

    annotator = BoxAnnotator("Baseline annotator")
    headers = [f"{i + 1}/{len(selected)}" for i in range(len(selected))]
    label_frames(annotator, selected, headers)
    annotator.close()

    total = sum(paths.is_labeled(p) for p in images)
    print(f"\n{total} train frames labeled.", "Next: 03_train_initial.py"
          if total >= NUM_TO_ANNOTATE else "Re-run to continue.")


if __name__ == "__main__":
    main()
