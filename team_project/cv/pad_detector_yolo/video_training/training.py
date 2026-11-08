"""Dataset building, training and prediction shared by steps 03-07. Not run directly."""
import os
import shutil

import paths


def build_dataset(name, train_images, val_images):
    """Copy images and their labels into work/<name>/ in Ultralytics' layout.

    Returns the path of the data.yaml to train on. Every image must already
    have a .txt label (empty = no pad).
    """
    root = os.path.join(paths.WORK_DIR, name)
    shutil.rmtree(root, ignore_errors=True)
    for split, images in (("train", train_images), ("val", val_images)):
        image_dir = os.path.join(root, "images", split)
        label_dir = os.path.join(root, "labels", split)
        os.makedirs(image_dir)
        os.makedirs(label_dir)
        for image in images:
            shutil.copy2(image, image_dir)
            shutil.copy2(paths.label_path(image), label_dir)

    yaml_path = os.path.join(root, "data.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"path: {root.replace(os.sep, '/')}\ntrain: images/train\nval: images/val\nnames:\n")
        for i, class_name in enumerate(paths.CLASS_NAMES):
            f.write(f"  {i}: {class_name}\n")
    return yaml_path


def train(run_name, data_yaml, output_model, epochs, image_size=640, seed=42):
    """Train from the yolov8n base weights and copy the best weights to output_model."""
    from ultralytics import YOLO

    os.makedirs(paths.MODELS_DIR, exist_ok=True)
    model = YOLO(paths.BASE_WEIGHTS)
    model.train(
        data=data_yaml, epochs=epochs, imgsz=image_size, seed=seed,
        project=os.path.join(paths.WORK_DIR, "runs"), name=run_name, exist_ok=True)
    shutil.copy2(model.trainer.best, output_model)
    print(f"\nSaved {output_model}  (run folder: {model.trainer.save_dir})")


def load_model(model_path):
    from ultralytics import YOLO

    if not os.path.exists(model_path):
        raise SystemExit(f"Model not found: {model_path}. Run the training step first.")
    return YOLO(model_path)


def predict_boxes(model, image, conf=0.25):
    """Return [(x1, y1, x2, y2, confidence), ...] in image pixels."""
    result = model.predict(image, conf=conf, verbose=False)[0]
    return [(*(float(v) for v in xyxy), float(c))
            for xyxy, c in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist())]
