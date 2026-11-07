"""Step 06: compare human labels with the final model on validation frames.

Reads:  models/final.pt, labeled frames in extracted_frames/val
Writes: results/compare/*.jpg, results/compare/summary.txt

Human boxes are green, model boxes orange (with confidence). A model box
counts as a hit if it overlaps a human box with IoU >= 0.5. After saving,
a window steps through the images: any key = next, q / Esc = stop.
"""
import os

import cv2

import paths
import training
from annotation_ui import HUMAN_COLOR, MODEL_COLOR

CONFIDENCE = 0.25
IOU_MATCH = 0.5
COMPARE_DIR = os.path.join(paths.RESULTS_DIR, "compare")


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def count_matches(human, model_boxes):
    """Greedy one-to-one matching, highest confidence first. Returns (hits, false alarms, misses)."""
    unmatched = list(human)
    hits = 0
    for box in sorted(model_boxes, key=lambda b: -b[4]):
        best = max(unmatched, key=lambda h: iou(h, box), default=None)
        if best is not None and iou(best, box) >= IOU_MATCH:
            unmatched.remove(best)
            hits += 1
    return hits, len(model_boxes) - hits, len(unmatched)


def draw(img, human, model_boxes):
    thick = max(2, round(max(img.shape[:2]) / 500))
    for x1, y1, x2, y2 in human:
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), HUMAN_COLOR, thick)
    for x1, y1, x2, y2, conf in model_boxes:
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), MODEL_COLOR, thick)
        cv2.putText(img, f"pad {conf:.2f}", (int(x1), max(20, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, thick * 0.4, MODEL_COLOR, thick)
    cv2.putText(img, "green = human   orange = model", (10, 20 + thick * 12),
                cv2.FONT_HERSHEY_SIMPLEX, thick * 0.4, (255, 255, 255), thick)


def main():
    frames = [p for p in paths.list_images(paths.VAL_DIR) if paths.is_labeled(p)]
    if not frames:
        print("No labeled validation frames. Run 04_review_predictions.py first.")
        return
    model = training.load_model(paths.FINAL_MODEL)
    os.makedirs(COMPARE_DIR, exist_ok=True)

    totals = [0, 0, 0]
    outputs = []
    for img_path in frames:
        img = cv2.imread(img_path)
        if img is None:
            continue
        h, w = img.shape[:2]
        human = paths.read_labels(paths.label_path(img_path), w, h)
        predicted = training.predict_boxes(model, img, conf=CONFIDENCE)
        for i, n in enumerate(count_matches(human, predicted)):
            totals[i] += n
        draw(img, human, predicted)
        out_path = os.path.join(COMPARE_DIR, os.path.basename(img_path))
        cv2.imwrite(out_path, img)
        outputs.append(out_path)

    hits, false_alarms, misses = totals
    precision = hits / (hits + false_alarms) if hits + false_alarms else 0.0
    recall = hits / (hits + misses) if hits + misses else 0.0
    summary = (f"{len(outputs)} validation frames, IoU >= {IOU_MATCH}, conf >= {CONFIDENCE}\n"
               f"hits {hits}  false alarms {false_alarms}  misses {misses}\n"
               f"precision {precision:.3f}  recall {recall:.3f}\n")
    with open(os.path.join(COMPARE_DIR, "summary.txt"), "w") as f:
        f.write(summary)
    print(summary + f"Saved images to {COMPARE_DIR}")

    window = "Validation: green = human, orange = model (any key = next, q = stop)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    for idx, out_path in enumerate(outputs):
        cv2.imshow(window, cv2.imread(out_path))
        cv2.setWindowTitle(window, f"{idx + 1}/{len(outputs)} {os.path.basename(out_path)}")
        if cv2.waitKey(0) & 0xFF in (ord("q"), 27):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
