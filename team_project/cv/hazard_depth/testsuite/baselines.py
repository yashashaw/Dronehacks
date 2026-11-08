"""Reference predictors. Your model should beat ConstantRatePredictor; Zero is the floor.

    --model hazard/testsuite/baselines.py:ZeroPredictor
    --model hazard/testsuite/baselines.py:ConstantRatePredictor
    --model hazard/testsuite/baselines.py:OraclePredictor   (harness self-check only)
"""
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from interface import Frame, TurnPredictor  # noqa: E402
from vision import camera_matrix, rotation_between, to_gray_small  # noqa: E402


class ZeroPredictor(TurnPredictor):
    """Always 'straight ahead'."""

    def predict(self, f: Frame) -> dict:
        return {f"yaw_{h:g}s": 0.0 for h in f.horizons}


class ConstantRatePredictor(TurnPredictor):
    """Measures the current yaw rate from the last two frames (vision only, 256x160) and
    assumes it continues. Strong on smooth turns; it can only react, never anticipate."""

    def __init__(self, scale: float = 0.25, smooth: float = 0.5):
        self.scale, self.smooth = float(scale), float(smooth)
        self.K = camera_matrix(self.scale)

    def reset(self):
        self.prev, self.rate = None, 0.0

    def predict(self, f: Frame) -> dict:
        g = to_gray_small(f.left, self.scale)
        if self.prev is not None and 0 < f.dt_s <= 0.35:
            yaw, _, _ = rotation_between(self.prev, g, self.K, min_inliers=15)
            if not math.isnan(yaw):
                self.rate = self.smooth * self.rate + (1 - self.smooth) * yaw / f.dt_s
        elif f.dt_s > 0.35:
            self.rate = 0.0
        self.prev = g
        return {f"yaw_{h:g}s": self.rate * h for h in f.horizons}


class OraclePredictor(TurnPredictor):
    """Reads the answers from manifest.csv. Must score ~0 error; proves the harness works."""

    def __init__(self, dataset: str = "data/walk/hazard_10fps"):
        with open(Path(dataset) / "manifest.csv", newline="") as fh:
            self.rows = {int(r["idx"]): r for r in csv.DictReader(fh)}

    def predict(self, f: Frame) -> dict:
        r = self.rows[f.idx]
        return {f"yaw_{h:g}s": float(r[f"yaw_{h:g}s"]) for h in f.horizons}
