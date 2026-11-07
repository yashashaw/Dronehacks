"""THE MODEL SLOT. Swap this file's body, not the node.

``PadModel.detect(gray)`` takes one grayscale frame (H x W uint8 numpy array)
and returns a list of ``Detection`` in that frame's pixel coordinates. The
detector node knows nothing else about the model.

Without weights the model is unavailable and ``detect`` returns an empty list,
so the node runs and simply reports no pad. Ultralytics is imported only when a
weights file is given, so this module imports on a machine without it.
"""
import os
from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    cx: float           # box center, pixels
    cy: float
    width: float        # box size, pixels
    height: float
    confidence: float   # 0..1
    class_id: int = 0


class PadModel:
    def __init__(self, weights_path='', confidence=0.25, image_size=640,
                 device=None):
        self.weights_path = weights_path or ''
        self.confidence = float(confidence)
        self.image_size = int(image_size)
        self.device = device
        self.error = None       # why the model is unavailable, if it is
        self._model = None
        self._load()

    @property
    def available(self):
        return self._model is not None

    def _load(self):
        if not self.weights_path:
            self.error = 'no weights_path set'
            return
        if not os.path.isfile(self.weights_path):
            self.error = f'weights file not found: {self.weights_path}'
            return
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.error = f'ultralytics is not installed ({exc})'
            return
        try:
            self._model = YOLO(self.weights_path)
        except Exception as exc:  # a bad weights file must not kill the node
            self.error = f'could not load {self.weights_path}: {exc}'

    def detect(self, gray):
        """Return a list of Detection for one grayscale frame."""
        if self._model is None:
            return []
        import numpy as np
        # The camera is monochrome; YOLO weights expect three channels.
        frame = np.repeat(gray[:, :, None], 3, axis=2)
        results = self._model.predict(
            frame, imgsz=self.image_size, conf=self.confidence,
            device=self.device, verbose=False)
        detections = []
        for box in results[0].boxes:
            cx, cy, w, h = (float(v) for v in box.xywh[0].tolist())
            detections.append(Detection(
                cx=cx, cy=cy, width=w, height=h,
                confidence=float(box.conf[0]),
                class_id=int(box.cls[0]) if box.cls is not None else 0))
        return detections
