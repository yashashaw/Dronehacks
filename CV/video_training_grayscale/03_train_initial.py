"""Step 03: train a first model on the hand-labeled baseline.

Reads:  extracted_frames/train/*.jpg + .txt (labeled frames only; empty .txt = negative)
Writes: models/initial.pt, work/initial_dataset/, work/runs/initial/

20% of the labeled frames are held out so training reports a metric.
"""
import random

import paths
import training

HOLDOUT_RATIO = 0.2
EPOCHS = 50
SEED = 42


def main():
    labeled = [p for p in paths.list_images(paths.TRAIN_DIR) if paths.is_labeled(p)]
    if len(labeled) < 5:
        print(f"Only {len(labeled)} labeled train frames. Run 02_annotate_baseline.py first.")
        return

    random.Random(SEED).shuffle(labeled)
    holdout = max(1, int(len(labeled) * HOLDOUT_RATIO))
    val, train = labeled[:holdout], labeled[holdout:]
    print(f"Training on {len(train)} frames, validating on {len(val)}.")

    data_yaml = training.build_dataset("initial_dataset", train, val)
    training.train("initial", data_yaml, paths.INITIAL_MODEL, epochs=EPOCHS, seed=SEED)
    print("Next: 04_review_predictions.py")


if __name__ == "__main__":
    main()
