"""Step 05: train the final model on every labeled training frame.

Reads:  labeled frames in extracted_frames/train (training) and /val (metrics only)
Writes: models/final.pt, work/final_dataset/, work/runs/final/

Validation frames come from videos the model never trains on, so the reported
mAP measures how well it generalizes to a new flight.
"""
import paths
import training

EPOCHS = 100
SEED = 42


def main():
    train_all = paths.list_images(paths.TRAIN_DIR)
    val_all = paths.list_images(paths.VAL_DIR)
    train = [p for p in train_all if paths.is_labeled(p)]
    val = [p for p in val_all if paths.is_labeled(p)]

    print(f"train: {len(train)}/{len(train_all)} frames labeled")
    print(f"val:   {len(val)}/{len(val_all)} frames labeled")
    if not train or not val:
        print("Need labeled train and val frames. Run 04_review_predictions.py first.")
        return
    if len(train) < len(train_all) or len(val) < len(val_all):
        print("Unlabeled frames are left out.")

    data_yaml = training.build_dataset("final_dataset", train, val)
    training.train("final", data_yaml, paths.FINAL_MODEL, epochs=EPOCHS, seed=SEED)
    print("Next: 06_compare_validation.py and 07_render_video.py")


if __name__ == "__main__":
    main()
