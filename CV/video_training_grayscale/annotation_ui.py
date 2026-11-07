"""The box-drawing window used by steps 02 and 04. Not run directly.

Frames are shown on a screen-sized copy so a thin box stays visible on a large
phone frame; boxes are converted back to full-resolution pixels when returned.

Keys: drag = draw a box | n / Space = save | s = skip (no file written)
      b = back one frame | c = clear boxes | u = undo last box | q / Esc = quit
"""
import os

import cv2

import paths

HUMAN_COLOR = (0, 200, 0)      # green
MODEL_COLOR = (0, 140, 255)    # orange
DRAWING_COLOR = (255, 255, 0)  # cyan while dragging


class BoxAnnotator:
    def __init__(self, window_name, max_size=(1280, 800)):
        self.window_name = window_name
        self.max_w, self.max_h = max_size
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(window_name, self._on_mouse)
        self._drag_start = None
        self._drag_end = None
        self._boxes = []            # display-pixel boxes
        self._from_model = False    # True while the boxes are the model's guess

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._drag_start = self._drag_end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self._drag_start:
            self._drag_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self._drag_start:
            (x0, y0), (x1, y1) = self._drag_start, (x, y)
            self._drag_start = self._drag_end = None
            if abs(x1 - x0) > 5 and abs(y1 - y0) > 5:   # ignore accidental clicks
                if self._from_model:                    # drawing replaces the guess
                    self._boxes, self._from_model = [], False
                self._boxes.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)))

    def annotate(self, image, header, initial_boxes=(), from_model=True):
        """Show one frame. Returns ('save', boxes), ('skip', None), ('back', None) or ('quit', None).

        ``initial_boxes`` (full-resolution pixels) are shown as the model's guess
        in orange, or as your own earlier boxes in green when ``from_model`` is
        False; saving without drawing keeps them.
        """
        h, w = image.shape[:2]
        scale = min(1.0, self.max_w / w, self.max_h / h)
        display = cv2.resize(image, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        self._boxes = [tuple(v * scale for v in b) for b in initial_boxes]
        self._from_model = from_model and bool(initial_boxes)

        while True:
            canvas = display.copy()
            color = MODEL_COLOR if self._from_model else HUMAN_COLOR
            for x1, y1, x2, y2 in self._boxes:
                cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 0), 4)
                cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            if self._drag_start and self._drag_end:
                cv2.rectangle(canvas, self._drag_start, self._drag_end, DRAWING_COLOR, 2)
            source = "model guess" if self._from_model else "yours"
            text = f"{header} | boxes: {len(self._boxes)} ({source})"
            cv2.putText(canvas, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(canvas, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow(self.window_name, canvas)

            key = cv2.waitKey(20) & 0xFF
            if key in (ord("n"), 32):
                return "save", [tuple(v / scale for v in b) for b in self._boxes]
            if key == ord("s"):
                return "skip", None
            if key == ord("b"):
                return "back", None
            if key == ord("c"):
                self._boxes, self._from_model = [], False
            if key == ord("u") and self._boxes:
                self._boxes.pop()
            if key in (ord("q"), 27):
                return "quit", None

    def close(self):
        cv2.destroyWindow(self.window_name)


def label_frames(annotator, image_paths, headers, guess=None):
    """Step through frames and write a label for each one saved.

    ``b`` returns to the previous frame of this session and shows the boxes
    saved for it in green; saving overwrites that label and ``s`` deletes it.
    ``guess(image)`` returns the model's pixel boxes for frames with no label.
    """
    idx, step = 0, 1
    while 0 <= idx < len(image_paths):
        img_path = image_paths[idx]
        img = cv2.imread(img_path)
        if img is None:
            step = 1 if idx == 0 else step
            idx += step
            continue

        h, w = img.shape[:2]
        label = paths.label_path(img_path)
        if os.path.exists(label):
            initial, from_model, note = paths.read_labels(label, w, h), False, " (saved)"
        else:
            initial, from_model, note = (guess(img) if guess else []), True, ""
        action, boxes = annotator.annotate(img, headers[idx] + note, initial, from_model)

        if action == "quit":
            break
        if action == "back":
            if idx == 0:
                print("Already at the first frame of this session.")
            idx, step = max(0, idx - 1), -1
            continue
        if action == "save":
            paths.write_labels(label, boxes, w, h)
            print(f"Saved {len(boxes)} box(es) for {os.path.basename(img_path)}")
        elif os.path.exists(label):
            os.remove(label)
            print(f"Removed label for {os.path.basename(img_path)}")
        idx, step = idx + 1, 1
